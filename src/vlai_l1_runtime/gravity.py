"""Offline URDF gravity efforts. No controller, calibration or hardware access.

Only masses and centres of mass are needed for static gravity. Inertia tensors,
friction, inertial/Coriolis forces and motor-to-model mappings are not inferred.
"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .configuration import _read_local_regular_file
from .kinematics import ChainJoint, _attribute, _numbers


@dataclass(frozen=True)
class PointMass:
    name: str
    link: str
    mass_kg: float
    com_m: tuple[float, ...]  # In the attachment link frame.

    def __post_init__(self) -> None:
        if any(not isinstance(v, str) or not v or v != v.strip() for v in (self.name, self.link)):
            raise ValueError("mass name and attachment link must be explicit")
        if isinstance(self.mass_kg, bool) or not isinstance(self.mass_kg, (int, float)):
            raise ValueError("mass must be positive and finite")
        if not math.isfinite(self.mass_kg) or self.mass_kg <= 0:
            raise ValueError("mass must be positive and finite")
        _vector(self.com_m, "centre of mass")


@dataclass(frozen=True)
class GravityJoint:
    geometry: ChainJoint
    mimic: tuple[str, float, float] | None


@dataclass(frozen=True)
class GravityResult:
    frame: str
    joint_names: tuple[str, ...]
    effort_units: tuple[str, ...]
    holding_effort: tuple[float, ...]
    gravity_m_s2: tuple[float, ...]
    total_mass_kg: float
    potential_energy_j: float
    mass_names: tuple[str, ...]
    mass_contributions: tuple[tuple[float, ...], ...]
    outside_model_limits: tuple[str, ...]


def _vector(values, label):
    if (
        not isinstance(values, tuple)
        or len(values) != 3
        or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in values
        )
    ):
        raise ValueError(f"{label} requires a tuple of three finite real numbers")


@dataclass(frozen=True)
class GravityModel:
    root: str
    branch: str
    joints: tuple[GravityJoint, ...]  # Parent before child, including side branches.
    masses: tuple[PointMass, ...]
    massless_links: tuple[str, ...]
    urdf_sha256: str

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(
            j.geometry.name for j in self.joints if j.geometry.kind != "fixed" and j.mimic is None
        )

    @classmethod
    def from_urdf(
        cls,
        path: Path,
        *,
        root: str,
        branch: str,
        expected_joint_names: tuple[str, ...],
        massless_links: tuple[str, ...],
    ) -> GravityModel:
        """Read one complete subtree whose first link is a direct child of root.

        All descendants, including fixed tools and mimic fingers, are included.
        Only explicitly listed empty frame links may omit mass/COM. No meshes,
        xacro, SDK, ROS or external XML resources are loaded. The root link's
        own mass is excluded: it cannot load joints in its selected subtree.
        """
        payload = _read_local_regular_file(path.absolute())
        if b"<!DOCTYPE" in payload.upper() or b"<!ENTITY" in payload.upper():
            raise ValueError("URDF document types and entities are unsupported")
        try:
            robot = ET.fromstring(payload)
        except ET.ParseError as exc:
            raise ValueError("invalid rendered URDF XML") from exc
        if robot.tag != "robot":
            raise ValueError("expected rendered URDF robot")
        links = {}
        for link in robot.findall("link"):
            name = link.get("name")
            if not name or name in links:
                raise ValueError("link names must be unique and nonempty")
            links[name] = link
        if root not in links or branch not in links or root == branch:
            raise ValueError("root and branch must be distinct existing links")
        parents, children, names = {}, {}, set()
        for element in robot.findall("joint"):
            parent, child = (
                _attribute(element, "parent", "link"),
                _attribute(element, "child", "link"),
            )
            name = element.get("name")
            if not name or name in names or child in parents:
                raise ValueError("joint names and child links must be unique")
            if parent not in links or child not in links:
                raise ValueError("joint references an unknown link")
            names.add(name)
            parents[child] = element
            children.setdefault(parent, []).append(child)
        if branch not in parents or _attribute(parents[branch], "parent", "link") != root:
            raise ValueError("branch must be a direct child of root")
        ordered, seen = [], {root}

        def visit(link):
            if link in seen:
                raise ValueError("gravity subtree must be acyclic")
            seen.add(link)
            ordered.append(link)
            for child in children.get(link, []):
                visit(child)

        visit(branch)
        if len(set(massless_links)) != len(massless_links) or not set(massless_links) <= set(
            ordered
        ):
            raise ValueError("massless links must be distinct selected frame links")
        joints, masses = [], []
        for link_name in ordered:
            element = parents[link_name]
            name, kind = str(element.get("name")), element.get("type")
            if kind not in ("fixed", "revolute", "prismatic"):
                raise ValueError(f"{name}: unsupported gravity joint type")
            xyz = _numbers(_attribute(element, "origin", "xyz"), 3, "joint xyz")
            rpy = _numbers(_attribute(element, "origin", "rpy"), 3, "joint rpy")
            axis, lower, upper = (0.0, 0.0, 0.0), None, None
            mimic = None
            if kind != "fixed":
                axis = _numbers(_attribute(element, "axis", "xyz"), 3, "joint axis")
                norm = math.hypot(*axis)
                if norm == 0 or not math.isfinite(norm):
                    raise ValueError(f"{name}: invalid joint axis")
                axis = tuple(v / norm for v in axis)
                lower = _numbers(_attribute(element, "limit", "lower"), 1, "lower limit")[0]
                upper = _numbers(_attribute(element, "limit", "upper"), 1, "upper limit")[0]
                if lower >= upper:
                    raise ValueError(f"{name}: invalid joint limits")
                item = element.find("mimic")
                if item is not None:
                    target = item.get("joint")
                    if not target:
                        raise ValueError(f"{name}: missing mimic target")
                    # URDF-defined defaults, not robot behaviour defaults.
                    multiplier = _numbers(item.get("multiplier", "1"), 1, "mimic multiplier")[0]
                    offset = _numbers(item.get("offset", "0"), 1, "mimic offset")[0]
                    mimic = (target, multiplier, offset)
            elif element.find("mimic") is not None:
                raise ValueError("fixed joint cannot mimic")
            joints.append(
                GravityJoint(
                    ChainJoint(
                        name,
                        _attribute(element, "parent", "link"),
                        link_name,
                        kind,
                        xyz,
                        rpy,
                        axis,
                        lower,
                        upper,
                    ),
                    mimic,
                )
            )
            link = links[link_name]
            inertials = link.findall("inertial")
            if link_name in massless_links:
                if (
                    inertials
                    or link.find("visual") is not None
                    or link.find("collision") is not None
                ):
                    raise ValueError(
                        f"{link_name}: massless declaration requires an empty frame link"
                    )
                continue
            if len(inertials) != 1:
                raise ValueError(f"{link_name}: requires exactly one explicit mass/COM")
            inertial = inertials[0]
            mass = _numbers(_attribute(inertial, "mass", "value"), 1, "mass")[0]
            com = _numbers(_attribute(inertial, "origin", "xyz"), 3, "COM xyz")
            _numbers(_attribute(inertial, "origin", "rpy"), 3, "inertial rpy")
            masses.append(PointMass(link_name, link_name, mass, com))
        model = cls(
            root,
            branch,
            tuple(joints),
            tuple(masses),
            massless_links,
            hashlib.sha256(payload).hexdigest(),
        )
        if not expected_joint_names or model.joint_names != expected_joint_names:
            raise ValueError("gravity model requires exact independent joint order")
        model._coordinates(dict.fromkeys(model.joint_names, 0.0))  # Validate mimic graph.
        return model

    def _coordinates(self, positions):
        if set(positions) != set(self.joint_names) or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in positions.values()
        ):
            raise ValueError("positions require exact independent names and finite real values")
        by_name = {j.geometry.name: j for j in self.joints}
        resolved, visiting = {}, set()

        def resolve(name):
            if name in resolved:
                return resolved[name]
            if name in visiting or name not in by_name:
                raise ValueError("invalid/cyclic mimic dependency")
            visiting.add(name)
            joint = by_name[name]
            if joint.mimic is None:
                value, derivative = (
                    (0.0, {}) if joint.geometry.kind == "fixed" else (positions[name], {name: 1.0})
                )
            else:
                target, scale, offset = joint.mimic
                if target not in by_name or by_name[target].geometry.kind != joint.geometry.kind:
                    raise ValueError("mimic target must have the same moving joint type in subtree")
                source, rates = resolve(target)
                value, derivative = (
                    scale * source + offset,
                    {k: scale * v for k, v in rates.items()},
                )
            if not math.isfinite(value) or not all(math.isfinite(v) for v in derivative.values()):
                raise ValueError("nonfinite mimic coordinate")
            visiting.remove(name)
            resolved[name] = (value, derivative)
            return resolved[name]

        for name in by_name:
            resolve(name)
        return resolved

    def evaluate(
        self,
        positions: Mapping[str, float],
        *,
        gravity_m_s2: tuple[float, ...],
        payloads: tuple[PointMass, ...],
    ) -> GravityResult:
        """Return g(q) = dU/dq, the effort opposing gravity, in model coordinates.

        Gravity is an explicit acceleration vector expressed in root. Revolute
        coordinates/efforts use rad/Nm; prismatic use m/N. Payloads are additional
        masses, so callers must exclude anything already in the URDF. Neither
        raw motor angles nor motor torques are accepted as calibrated inputs.
        """
        _vector(gravity_m_s2, "gravity")
        coordinates = self._coordinates(positions)
        if not isinstance(payloads, tuple) or any(not isinstance(p, PointMass) for p in payloads):
            raise ValueError("payloads must be an explicit tuple of point masses")
        masses = (*self.masses, *payloads)
        if len({m.name for m in masses}) != len(masses):
            raise ValueError("mass names must be unique; do not double count payloads")
        selected_links = {j.geometry.child for j in self.joints}
        if any(m.link not in selected_links for m in masses):
            raise ValueError("payload attachment must be in selected subtree")
        import numpy as np

        def rotate(axis, angle):
            x, y, z = axis
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)

        transforms = {self.root: (np.eye(3), np.zeros(3), ())}
        columns = {name: i for i, name in enumerate(self.joint_names)}
        violations, contributions = [], []
        potential, total_mass = 0.0, 0.0
        with np.errstate(over="raise", invalid="raise"):
            try:
                for entry in self.joints:
                    joint = entry.geometry
                    rotation, point, ancestors = transforms[joint.parent]
                    point = point + rotation @ joint.xyz
                    roll, pitch, yaw = joint.rpy
                    rotation = (
                        rotation
                        @ rotate((0, 0, 1), yaw)
                        @ rotate((0, 1, 0), pitch)
                        @ rotate((1, 0, 0), roll)
                    )
                    q, rates = coordinates[joint.name]
                    if joint.kind != "fixed":
                        axis = rotation @ joint.axis
                        ancestors = (*ancestors, (joint.kind, axis, point.copy(), rates))
                        if not joint.lower_rad <= q <= joint.upper_rad:
                            violations.append(joint.name)
                        if joint.kind == "revolute":
                            rotation = rotation @ rotate(joint.axis, q)
                        else:
                            point = point + axis * q
                    transforms[joint.child] = (rotation, point, ancestors)
                gravity = np.array(gravity_m_s2)
                for mass in masses:
                    rotation, point, ancestors = transforms[mass.link]
                    com = point + rotation @ mass.com_m
                    force = mass.mass_kg * gravity
                    potential -= float(force @ com)
                    total_mass += mass.mass_kg
                    effort = np.zeros(len(columns))
                    for kind, axis, origin, rates in ancestors:
                        value = -float(
                            axis @ (np.cross(com - origin, force) if kind == "revolute" else force)
                        )
                        for name, scale in rates.items():
                            effort[columns[name]] += scale * value
                    contributions.append(tuple(float(v) for v in effort))
                holding = np.sum(contributions, axis=0) if contributions else np.zeros(len(columns))
                if (
                    not np.isfinite(holding).all()
                    or not math.isfinite(potential)
                    or not math.isfinite(total_mass)
                ):
                    raise ValueError("nonfinite gravity result")
            except (FloatingPointError, OverflowError) as exc:
                raise ValueError("nonfinite gravity calculation") from exc
        return GravityResult(
            self.root,
            self.joint_names,
            tuple(
                "Nm" if j.geometry.kind == "revolute" else "N"
                for j in self.joints
                if j.geometry.name in columns
            ),
            tuple(float(v) for v in holding),
            tuple(gravity_m_s2),
            total_mass,
            potential,
            tuple(m.name for m in masses),
            tuple(contributions),
            tuple(violations),
        )
