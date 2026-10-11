"""Offline URDF serial-chain geometry; no SDK, ROS or physical-device access.

Joint inputs are URDF coordinates. This module never infers motor zero offsets
or signs, clamps positions, changes limits, or authorizes motion.
"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path


def _numbers(text: str, count: int, label: str) -> tuple[float, ...]:
    try:
        values = tuple(float(value) for value in text.split())
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid {label}") from exc
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} requires {count} finite values")
    return values


def _attribute(element: ET.Element, child: str, name: str) -> str:
    item = element.find(child)
    if item is None or not item.get(name):
        raise ValueError(f"{element.get('name')}: missing {child}.{name}")
    return str(item.get(name))


@dataclass(frozen=True)
class ChainJoint:
    name: str
    parent: str
    child: str
    kind: str
    xyz: tuple[float, ...]
    rpy: tuple[float, ...]
    axis: tuple[float, ...]
    lower_rad: float | None
    upper_rad: float | None


@dataclass(frozen=True)
class ChainGeometry:
    frame: str
    reference_point: str
    joint_names: tuple[str, ...]
    position_m: tuple[float, ...]
    rotation: tuple[tuple[float, ...], ...]
    jacobian: tuple[tuple[float, ...], ...]
    outside_model_limits: tuple[str, ...]


@dataclass(frozen=True)
class SerialChain:
    root: str
    tip: str
    joints: tuple[ChainJoint, ...]
    urdf_sha256: str

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints if joint.kind == "revolute")

    @classmethod
    def from_urdf(
        cls, path: Path, *, root: str, tip: str, expected_joint_names: tuple[str, ...]
    ) -> SerialChain:
        """Read an explicit fixed/revolute chain from an already rendered URDF.

        No xacro execution, mesh loading, or external resource resolution occurs.
        Origins/axes/limits must be explicit. Other branches are not part of the
        chain, so this geometry is not a complete payload/dynamics model.
        """

        payload = path.read_bytes()
        if b"<!DOCTYPE" in payload.upper() or b"<!ENTITY" in payload.upper():
            raise ValueError("URDF document types and entities are unsupported")
        try:
            robot = ET.fromstring(payload)
        except ET.ParseError as exc:
            raise ValueError("invalid rendered URDF XML") from exc
        if robot.tag != "robot":
            raise ValueError("expected a rendered URDF robot")
        links = [element.get("name") for element in robot.findall("link")]
        if any(not name for name in links) or len(set(links)) != len(links):
            raise ValueError("URDF link names must be unique and nonempty")
        if root not in links or tip not in links or root == tip:
            raise ValueError("root and tip must be distinct existing links")
        parents: dict[str, ET.Element] = {}
        names = set()
        for element in robot.findall("joint"):
            name = element.get("name")
            parent = _attribute(element, "parent", "link")
            child = _attribute(element, "child", "link")
            if not name or name in names or child in parents:
                raise ValueError("URDF joints must have unique names and child links")
            if parent not in links or child not in links:
                raise ValueError(f"{name}: joint references an unknown link")
            names.add(name)
            parents[child] = element
        reverse = []
        current = tip
        visited = set()
        while current != root:
            if current in visited or current not in parents:
                raise ValueError("tip does not have an acyclic path to root")
            visited.add(current)
            element = parents[current]
            kind = element.get("type")
            name = str(element.get("name"))
            if kind not in ("fixed", "revolute") or element.find("mimic") is not None:
                raise ValueError(f"{name}: only independent fixed/revolute chain joints supported")
            xyz = _numbers(_attribute(element, "origin", "xyz"), 3, f"{name} origin xyz")
            rpy = _numbers(_attribute(element, "origin", "rpy"), 3, f"{name} origin rpy")
            axis = (0.0, 0.0, 0.0)
            lower = upper = None
            if kind == "revolute":
                axis = _numbers(_attribute(element, "axis", "xyz"), 3, f"{name} axis")
                norm = math.hypot(*axis)
                if norm == 0 or not math.isfinite(norm):
                    raise ValueError(f"{name}: invalid zero/overflow axis")
                axis = tuple(value / norm for value in axis)
                lower = _numbers(_attribute(element, "limit", "lower"), 1, "lower limit")[0]
                upper = _numbers(_attribute(element, "limit", "upper"), 1, "upper limit")[0]
                if lower >= upper:
                    raise ValueError(f"{name}: invalid joint limits")
            parent = _attribute(element, "parent", "link")
            reverse.append(ChainJoint(name, parent, current, kind, xyz, rpy, axis, lower, upper))
            current = parent
        chain = cls(root, tip, tuple(reversed(reverse)), hashlib.sha256(payload).hexdigest())
        if not expected_joint_names or chain.joint_names != expected_joint_names:
            raise ValueError("URDF chain does not match the exact expected joint order")
        return chain

    def evaluate(self, positions_rad: Mapping[str, float]) -> ChainGeometry:
        """Return tool-point geometric Jacobian [linear; angular] in root frame.

        Out-of-model-limit inputs are reported, never clamped. Evaluating these
        inputs is useful for offline coordinate audits, not a motion permission.
        """

        if set(positions_rad) != set(self.joint_names):
            raise ValueError("positions require exactly the named chain joints")
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in positions_rad.values()
        ):
            raise ValueError("joint positions must be finite real numbers in radians")
        import numpy as np

        rotation = np.eye(3)
        point = np.zeros(3)
        axes, origins, violations = [], [], []

        def axis_rotation(axis, angle):
            x, y, z = axis
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)

        with np.errstate(over="raise", invalid="raise"):
            try:
                for joint in self.joints:
                    point = point + rotation @ joint.xyz
                    roll, pitch, yaw = joint.rpy
                    rotation = (
                        rotation
                        @ axis_rotation((0, 0, 1), yaw)
                        @ axis_rotation((0, 1, 0), pitch)
                        @ axis_rotation((1, 0, 0), roll)
                    )
                    if joint.kind == "revolute":
                        angle = positions_rad[joint.name]
                        axes.append(rotation @ joint.axis)
                        origins.append(point.copy())
                        if not joint.lower_rad <= angle <= joint.upper_rad:
                            violations.append(joint.name)
                        rotation = rotation @ axis_rotation(joint.axis, angle)
                jacobian = np.column_stack(
                    [
                        np.concatenate((np.cross(axis, point - origin), axis))
                        for axis, origin in zip(axes, origins, strict=True)
                    ]
                )
            except FloatingPointError as exc:
                raise ValueError("nonfinite kinematic calculation") from exc
        if not all(np.isfinite(value).all() for value in (point, rotation, jacobian)):
            raise ValueError("nonfinite kinematic result")
        return ChainGeometry(
            self.root,
            self.tip,
            self.joint_names,
            tuple(float(value) for value in point),
            tuple(tuple(float(value) for value in row) for row in rotation),
            tuple(tuple(float(value) for value in row) for row in jacobian),
            tuple(violations),
        )
