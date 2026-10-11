"""Offline static-wrench candidates with explicit assumptions and rejection.

Never opens hardware or supplies a calibrated/live FT stream. Every output
remains hypothesis-only, even if numerical and stationarity checks pass.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from .configuration import MOTOR_NAMES, SIDES, SystemConfig, _is_loader_validated
from .feedback import MotorFeedback
from .feedback_decode import decode_follower_feedback
from .gravity import GravityModel, PointMass
from .kinematics import SerialChain
from .wrench import ARM_JOINT_NAMES, WrenchEstimate, estimate_wrench, propagated_torque_error


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be finite real numbers")


def _vector(values, size, label, *, nonnegative=False):
    if not isinstance(values, tuple) or len(values) != size:
        raise ValueError(f"{label} requires exactly {size} ordered values")
    for value in values:
        _finite(value, label)
        if nonnegative and value < 0:
            raise ValueError(f"{label} must be nonnegative")


@dataclass(frozen=True)
class JointCoordinateMap:
    motor_name: str
    model_name: str
    position_scale: float
    position_offset_rad: float
    torque_gain: float
    torque_bias_nm: float

    def __post_init__(self):
        if (
            self.motor_name not in ARM_JOINT_NAMES
            or not isinstance(self.model_name, str)
            or not self.model_name
        ):
            raise ValueError("coordinate mapping requires explicit arm motor/model names")
        for value in (
            self.position_scale,
            self.position_offset_rad,
            self.torque_gain,
            self.torque_bias_nm,
        ):
            _finite(value, "coordinate mapping")
        if self.position_scale == 0 or self.torque_gain <= 0:
            raise ValueError("position scale must be nonzero and torque gain positive")

    def transform(self, q, dq, torque):
        for value in (q, dq, torque):
            _finite(value, "motor coordinates/torque")
        # q_model = a*q_motor+b. Power consistency requires tau_model=tau_motor/a.
        result = (
            self.position_scale * q + self.position_offset_rad,
            self.position_scale * dq,
            self.torque_gain * (torque - self.torque_bias_nm) / self.position_scale,
        )
        if not all(math.isfinite(v) for v in result):
            raise ValueError("nonfinite coordinate conversion")
        return result


@dataclass(frozen=True)
class StaticWrenchPolicy:
    minimum_stationary_ns: int
    maximum_sample_gap_ns: int
    maximum_pair_age_ns: int
    maximum_motor_skew_ns: int
    maximum_velocity_rad_s: tuple[float, ...]  # All eight raw motor coordinates.
    maximum_position_span_rad: tuple[float, ...]
    allowed_raw_statuses: tuple[frozenset[int], ...]
    characteristic_length_m: float
    damping_m: float
    minimum_singular_value_m: float
    maximum_condition_number: float
    maximum_residual_nm: float
    unmodeled_torque_bound_nm: tuple[float, ...]  # Seven MODEL joint coordinates.
    maximum_force_error_n: tuple[float, ...]
    maximum_moment_error_nm: tuple[float, ...]

    def __post_init__(self):
        for value in (
            self.minimum_stationary_ns,
            self.maximum_sample_gap_ns,
            self.maximum_pair_age_ns,
            self.maximum_motor_skew_ns,
        ):
            if type(value) is not int or value <= 0:
                raise ValueError("time budgets must be positive integers")
        if self.maximum_motor_skew_ns > self.maximum_pair_age_ns:
            raise ValueError("skew budget must not exceed pair age budget")
        for name, size in (
            ("maximum_velocity_rad_s", 8),
            ("maximum_position_span_rad", 8),
            ("unmodeled_torque_bound_nm", 7),
            ("maximum_force_error_n", 3),
            ("maximum_moment_error_nm", 3),
        ):
            _vector(getattr(self, name), size, name, nonnegative=True)
        if (
            not isinstance(self.allowed_raw_statuses, tuple)
            or len(self.allowed_raw_statuses) != 8
            or any(
                not isinstance(statuses, frozenset)
                or not statuses
                or any(type(s) is not int or not 0 <= s <= 15 for s in statuses)
                for statuses in self.allowed_raw_statuses
            )
        ):
            raise ValueError("explicit raw status sets required for all eight motors")
        for name in (
            "characteristic_length_m",
            "minimum_singular_value_m",
            "maximum_condition_number",
            "maximum_residual_nm",
            "damping_m",
        ):
            value = getattr(self, name)
            _finite(value, name)
            if value < 0 or (name != "damping_m" and value == 0):
                raise ValueError(f"invalid {name}")
        if self.maximum_condition_number < 1:
            raise ValueError("condition limit must be at least one")


@dataclass(frozen=True)
class StaticWrenchResult:
    side: str
    monotonic_ns: int
    status: str
    reasons: tuple[str, ...]
    assumptions: tuple[str, ...]
    model_positions_rad: tuple[float, ...] | None = None
    gravity_torque_nm: tuple[float, ...] | None = None
    motor_torque_nm: tuple[float, ...] | None = None
    external_torque_nm: tuple[float, ...] | None = None
    propagated_error_bound: tuple[float, ...] | None = None
    estimate: WrenchEstimate | None = None
    ft_validated: bool = field(default=False, init=False)


class OfflineStaticWrench:
    """One ordered saved-capture stream, with no IO and no state persistence.

    Input identity/status/bounds/timestamp faults latch for this stream. Motion
    or data gaps clear the stationary window. Rejected records return no
    estimate, never a stale estimate or fabricated zero force. This validates
    offline timestamps only, not wall-clock freshness or hardware health.
    """

    def __init__(
        self,
        *,
        config: SystemConfig,
        side: str,
        chain: SerialChain,
        gravity: GravityModel,
        mapping: tuple[JointCoordinateMap, ...],
        auxiliary_positions: tuple[tuple[str, float], ...],
        gravity_m_s2: tuple[float, ...],
        payloads: tuple[PointMass, ...],
        static_friction_torque_nm: tuple[float, ...],
        policy: StaticWrenchPolicy,
        assumptions: tuple[str, ...],
    ):
        if (
            not _is_loader_validated(config)
            or config.follower_feedback is None
            or side not in SIDES
        ):
            raise ValueError("validated Follower configuration and side required")
        if (
            not isinstance(mapping, tuple)
            or any(not isinstance(m, JointCoordinateMap) for m in mapping)
            or tuple(m.motor_name for m in mapping) != ARM_JOINT_NAMES
        ):
            raise ValueError("coordinate mapping requires exact seven-arm-motor order")
        names = tuple(m.model_name for m in mapping)
        if chain.joint_names != names or len(set(names)) != 7:
            raise ValueError("model chain and coordinate mapping must agree exactly")
        if chain.root != gravity.root or chain.urdf_sha256 != gravity.urdf_sha256:
            raise ValueError("gravity and kinematics must share model hash and root frame")
        if not set(names) <= set(gravity.joint_names):
            raise ValueError("gravity model is missing mapped arm joints")
        if any(j.geometry.kind != "revolute" for j in gravity.joints if j.geometry.name in names):
            raise ValueError("mapped arm joints must be revolute")
        extras = dict(auxiliary_positions)
        if len(extras) != len(auxiliary_positions) or set(extras) != set(gravity.joint_names) - set(
            names
        ):
            raise ValueError("explicit exact auxiliary model coordinates required")
        for v in extras.values():
            _finite(v, "auxiliary positions")
        _vector(gravity_m_s2, 3, "gravity")
        _vector(static_friction_torque_nm, 7, "static friction torque")
        if (
            not isinstance(assumptions, tuple)
            or not assumptions
            or any(not isinstance(a, str) or not a.strip() for a in assumptions)
        ):
            raise ValueError("offline physical assumptions must be explicit")
        # Validate payload attachment/types before accepting any stream input.
        gravity.evaluate(
            dict.fromkeys(names, 0.0) | extras, gravity_m_s2=gravity_m_s2, payloads=payloads
        )
        self.config, self.side, self.chain, self.gravity = config, side, chain, gravity
        self.mapping, self.extras, self.gravity_vector, self.payloads = (
            mapping,
            extras,
            gravity_m_s2,
            payloads,
        )
        self.friction, self.policy, self.assumptions = (
            static_friction_torque_nm,
            policy,
            assumptions,
        )
        self._last = None
        self._window = deque()
        self._fault = None

    def process(self, sdk_feedback: MotorFeedback, *, side: str) -> StaticWrenchResult:
        timestamp = sdk_feedback.monotonic_ns

        def result(status, *reasons, **details):
            return StaticWrenchResult(
                self.side, timestamp, status, tuple(reasons), self.assumptions, **details
            )

        if self._fault is not None:
            return result("invalid", self._fault)
        try:
            if side != self.side:
                raise ValueError("unexpected Follower side")
            decoded = decode_follower_feedback(self.config, side, sdk_feedback)
            feedback = decoded.feedback
            times = tuple(r.host_receive_monotonic_ns for r in feedback.raw_frames)
            if (
                timestamp - min(times) > self.policy.maximum_pair_age_ns
                or max(times) - min(times) > self.policy.maximum_motor_skew_ns
            ):
                raise ValueError("raw feedback age/skew exceeds offline policy")
            if self._last is not None and (
                timestamp <= self._last.monotonic_ns
                or any(
                    a <= b.host_receive_monotonic_ns
                    for a, b in zip(times, self._last.raw_frames, strict=True)
                )
            ):
                raise ValueError("non-monotonic feedback timestamps")
            for name, status, allowed, endpoint in zip(
                MOTOR_NAMES,
                decoded.status_nibbles,
                self.policy.allowed_raw_statuses,
                decoded.at_protocol_endpoint,
                strict=True,
            ):
                if status not in allowed or endpoint:
                    raise ValueError(f"unaccepted raw status/protocol endpoint: {name}")
            limits = self.config.teleoperation.joint_safety.for_side(side)
            for name, q, lo, hi in zip(
                ARM_JOINT_NAMES,
                feedback.position_rad[:7],
                limits.min_deg,
                limits.max_deg,
                strict=True,
            ):
                if not lo <= math.degrees(q) <= hi:
                    raise ValueError(f"configured motor bounds violated: {name}")
            converted = tuple(
                m.transform(q, dq, tau)
                for m, q, dq, tau in zip(
                    self.mapping,
                    feedback.position_rad[:7],
                    feedback.velocity_rad_s[:7],
                    feedback.torque_nm[:7],
                    strict=True,
                )
            )
            positions = {
                m.model_name: row[0] for m, row in zip(self.mapping, converted, strict=True)
            }
            geometry = self.chain.evaluate(positions)
            g = self.gravity.evaluate(
                positions | self.extras, gravity_m_s2=self.gravity_vector, payloads=self.payloads
            )
            if geometry.outside_model_limits or g.outside_model_limits:
                raise ValueError(
                    f"model bounds violated: {geometry.outside_model_limits or g.outside_model_limits}"
                )
        except ValueError as exc:
            self._fault = str(exc)
            self._window.clear()
            return result("invalid", self._fault)

        previous = self._last
        self._last = feedback
        if any(
            abs(v) > limit
            for v, limit in zip(
                feedback.velocity_rad_s, self.policy.maximum_velocity_rad_s, strict=True
            )
        ):
            self._window.clear()
            return result("not_stationary", "motor velocity exceeds offline threshold")
        if (
            previous is not None
            and timestamp - previous.monotonic_ns > self.policy.maximum_sample_gap_ns
        ):
            self._window.clear()
        self._window.append(feedback)
        while (
            len(self._window) > 1
            and timestamp - self._window[1].monotonic_ns >= self.policy.minimum_stationary_ns
        ):
            self._window.popleft()
        for i, threshold in enumerate(self.policy.maximum_position_span_rad):
            values = [sample.position_rad[i] for sample in self._window]
            if max(values) - min(values) > threshold:
                self._window.clear()
                self._window.append(feedback)
                return result("not_stationary", f"position span exceeded: {MOTOR_NAMES[i]}")
        if timestamp - self._window[0].monotonic_ns < self.policy.minimum_stationary_ns:
            return result("warming_up", "stationary observation window incomplete")
        gravity_torque = tuple(
            g.holding_effort[g.joint_names.index(m.model_name)] for m in self.mapping
        )
        motor_torque = tuple(row[2] for row in converted)
        external = tuple(
            a + f - b for a, f, b in zip(gravity_torque, self.friction, motor_torque, strict=True)
        )
        details = {
            "model_positions_rad": tuple(row[0] for row in converted),
            "gravity_torque_nm": gravity_torque,
            "motor_torque_nm": motor_torque,
            "external_torque_nm": external,
        }
        try:
            estimate = estimate_wrench(
                joint_names=ARM_JOINT_NAMES,
                jacobian=geometry.jacobian,
                external_joint_torque_nm=dict(zip(ARM_JOINT_NAMES, external, strict=True)),
                frame=geometry.frame,
                reference_point=geometry.reference_point,
                monotonic_ns=timestamp,
                **{
                    key: getattr(self.policy, key)
                    for key in (
                        "characteristic_length_m",
                        "damping_m",
                        "minimum_singular_value_m",
                        "maximum_condition_number",
                        "maximum_residual_nm",
                    )
                },
            )
            # One full protocol step: installed firmware rounding is unverified.
            ranges = self.config.follower_feedback.for_side(side)
            bounds = {
                m.motor_name: 2
                * ranges[i].torque_max_nm
                / 4095
                * m.torque_gain
                / abs(m.position_scale)
                + self.policy.unmodeled_torque_bound_nm[i]
                for i, m in enumerate(self.mapping)
            }
            error = propagated_torque_error(estimate, bounds)
            details["propagated_error_bound"] = error
            if any(
                a > b
                for a, b in zip(
                    error,
                    (*self.policy.maximum_force_error_n, *self.policy.maximum_moment_error_nm),
                    strict=True,
                )
            ):
                return result(
                    "rejected", "propagated torque error exceeds offline limits", **details
                )
        except ValueError as exc:
            return result("rejected", str(exc), **details)
        return result(
            "numerical_candidate",
            "hypothesis only; physical FT not validated",
            estimate=estimate,
            **details,
        )
