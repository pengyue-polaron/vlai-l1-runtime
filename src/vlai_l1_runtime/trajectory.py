"""Pure trajectory mathematics and fault rehearsal; never a hardware adapter.

No clocks, threads, files, sockets, SDK handles or command transport are used.
Rehearsal release acknowledgements model lifecycle evidence; they do not disable
motors. Production CommandSession readiness remains independent and unchanged.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType

from .configuration import SystemConfig, _is_loader_validated
from .contracts import NamedJointVector, feature_names_for_sides, validate_named_values


def _positive(value, label):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be positive and finite")


def _ns(value, label, *, positive=False):
    if type(value) is not int or value < (1 if positive else 0):
        raise ValueError(f"{label} must be {'positive' if positive else 'non-negative'} integer ns")


@dataclass(frozen=True)
class SweepSpec:
    """Explicit experimental parameters, not commissioned physical limits."""

    amplitude_deg: float
    transition_ns: int
    hold_ns: int
    cycles: int
    maximum_speed_deg_s: float
    maximum_acceleration_deg_s2: float

    def __post_init__(self):
        for label in ("amplitude_deg", "maximum_speed_deg_s", "maximum_acceleration_deg_s2"):
            _positive(getattr(self, label), label)
        _ns(self.transition_ns, "transition", positive=True)
        _ns(self.hold_ns, "hold", positive=True)
        if type(self.cycles) is not int or self.cycles <= 0:
            raise ValueError("cycles must be a positive integer")
        seconds = self.transition_ns / 1e9
        if 1.875 * self.amplitude_deg / seconds > self.maximum_speed_deg_s:
            raise ValueError("continuous trajectory exceeds speed budget")
        if (
            10 * math.sqrt(3) / 3
        ) * self.amplitude_deg / seconds**2 > self.maximum_acceleration_deg_s2:
            raise ValueError("continuous trajectory exceeds acceleration budget")


@dataclass(frozen=True)
class TrajectoryPoint:
    relative_ns: int
    phase: str
    position_deg: Mapping[str, float]
    velocity_deg_s: Mapping[str, float]
    acceleration_deg_s2: Mapping[str, float]


@dataclass(frozen=True, init=False)
class WristSweep:
    """Analytic J7 +/- excursion with holds; other joints and gripper fixed.

    Constructor bounds cover the complete monotonic interpolation, not only
    sampled points. This establishes no Cartesian clearance or effort limit.
    """

    features: tuple[str, ...]
    reference: Mapping[str, float]
    active_feature: str
    bounds: tuple[tuple[str, float, float], ...]
    spec: SweepSpec
    duration_ns: int
    state_timeout_ns: int
    first_hold_tolerance_deg: float
    command_blockers: tuple[str, ...]

    def __init__(self, config: SystemConfig, *, side: str, reference_deg, spec: SweepSpec):
        if not isinstance(config, SystemConfig) or not _is_loader_validated(config):
            raise ValueError("loader-validated unmodified configuration required")
        if not isinstance(spec, SweepSpec):
            raise ValueError("SweepSpec required")
        # Revalidate copied inputs, even if a frozen dataclass was tampered with.
        spec = SweepSpec(**vars(spec))
        features = feature_names_for_sides((side,))
        object.__setattr__(self, "features", features)
        object.__setattr__(
            self,
            "reference",
            MappingProxyType(validate_named_values(reference_deg, feature_names=features)),
        )
        object.__setattr__(self, "active_feature", f"{side}_joint_7.pos")
        limits = config.teleoperation.joint_safety.for_side(side)
        object.__setattr__(
            self,
            "bounds",
            tuple(zip(self.features[:7], limits.min_deg, limits.max_deg, strict=True)),
        )
        self.check_position(self.reference)
        for offset in (-spec.amplitude_deg, spec.amplitude_deg):
            q = dict(self.reference)
            q[self.active_feature] += offset
            self.check_position(q)
        object.__setattr__(self, "spec", spec)
        object.__setattr__(
            self,
            "duration_ns",
            spec.hold_ns + 4 * spec.cycles * (spec.transition_ns + spec.hold_ns),
        )
        object.__setattr__(
            self, "state_timeout_ns", math.ceil(config.teleoperation.state_timeout_s * 1e9)
        )
        object.__setattr__(
            self, "first_hold_tolerance_deg", config.safety.first_command_hold_tolerance_deg
        )
        object.__setattr__(self, "command_blockers", config.command_blockers)

    def check_position(self, values):
        result = validate_named_values(values, feature_names=self.features)
        for name, lo, hi in self.bounds:
            if not lo <= result[name] <= hi:
                raise ValueError(f"joint bounds: {name}")
        return result

    def at(self, relative_ns: int) -> TrajectoryPoint:
        _ns(relative_ns, "relative time")
        if relative_ns > self.duration_ns:
            raise ValueError("time is past trajectory end")
        spec = self.spec
        q = dict(self.reference)
        velocity = dict.fromkeys(self.features, 0.0)
        acceleration = dict(velocity)
        phase = "initial_hold"
        if relative_ns == self.duration_ns:
            phase = "complete"
        elif relative_ns >= spec.hold_ns:
            segment, local = divmod(relative_ns - spec.hold_ns, spec.transition_ns + spec.hold_ns)
            leg = segment % 4
            starts = (0.0, spec.amplitude_deg, 0.0, -spec.amplitude_deg)
            ends = (spec.amplitude_deg, 0.0, -spec.amplitude_deg, 0.0)
            suffix = ("positive", "return_from_positive", "negative", "return_from_negative")[leg]
            phase = f"cycle_{segment // 4 + 1}_{suffix}"
            offset = ends[leg]
            if local < spec.transition_ns:
                u = local / spec.transition_ns
                delta = ends[leg] - starts[leg]
                seconds = spec.transition_ns / 1e9
                offset = starts[leg] + delta * u**3 * (10 - 15 * u + 6 * u**2)
                velocity[self.active_feature] = delta * 30 * u**2 * (1 - u) ** 2 / seconds
                acceleration[self.active_feature] = (
                    delta * 60 * u * (1 - u) * (1 - 2 * u) / seconds**2
                )
            else:
                phase += "_hold"
            q[self.active_feature] += offset
        return TrajectoryPoint(
            relative_ns,
            phase,
            MappingProxyType(q),
            MappingProxyType(velocity),
            MappingProxyType(acceleration),
        )


class RehearsalState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    RELEASE_REQUIRED = "release_required"
    CLOSED = "closed"
    RELEASE_FAILED = "release_failed"


@dataclass(frozen=True)
class RehearsalDecision:
    state: RehearsalState
    point: TrajectoryPoint | None
    reason: str | None


class TrajectoryRehearsal:
    """Deterministic offline fault model. No output is a motor command.

    Every step requires a new feedback sample. poll() models an independent
    watchdog and does not renew feedback freshness. Transport health is an
    explicit simulated input, not an actual CAN-health observation.
    """

    def __init__(self, plan: WristSweep, *, tracking_error_deg, tracking_persistence_ns: int):
        if not isinstance(plan, WristSweep):
            raise ValueError("WristSweep required")
        self.plan = plan
        self.tolerance = MappingProxyType(
            validate_named_values(tracking_error_deg, feature_names=plan.features)
        )
        if any(v <= 0 for v in self.tolerance.values()):
            raise ValueError("tracking tolerances must be positive")
        _ns(tracking_persistence_ns, "tracking persistence", positive=True)
        self.persistence_ns = tracking_persistence_ns
        self.state = RehearsalState.IDLE
        self.reason = None
        self._start_ns = None
        self._last_now = None
        self._last_source = None
        self._last_sequence = None
        self._error_since = {}

    def _decision(self, point=None):
        return RehearsalDecision(self.state, point, self.reason)

    def _release(self, reason):
        self.state = RehearsalState.RELEASE_REQUIRED
        self.reason = reason
        return self._decision()

    def _clock(self, now_ns):
        _ns(now_ns, "host time")
        if self._last_now is not None and now_ns <= self._last_now:
            raise ValueError("non_monotonic_host_time")
        self._last_now = now_ns
        if (
            self._last_source is not None
            and now_ns - self._last_source > self.plan.state_timeout_ns
        ):
            raise ValueError("feedback_timeout")

    def step(self, *, now_ns: int, feedback: NamedJointVector, transport_healthy: bool):
        if self.state not in (RehearsalState.IDLE, RehearsalState.RUNNING):
            return self._decision()
        try:
            self._clock(now_ns)
            if transport_healthy is not True:
                raise ValueError("transport_fault")
            if (
                not isinstance(feedback, NamedJointVector)
                or feedback.feature_names != self.plan.features
            ):
                raise ValueError("wrong_feedback_identity")
            sample = NamedJointVector(feedback.values, feedback.metadata, self.plan.features)
            q = self.plan.check_position(sample.values)
            source = sample.metadata.monotonic_ns
            sequence = sample.metadata.source_sequence
            if not 0 <= now_ns - source <= self.plan.state_timeout_ns:
                raise ValueError("stale_or_future_feedback")
            if self._last_sequence is not None and (
                sequence <= self._last_sequence or source <= self._last_source
            ):
                raise ValueError("non_monotonic_feedback")
            if self.state is RehearsalState.IDLE:
                if any(
                    abs(q[n] - self.plan.reference[n]) > self.plan.first_hold_tolerance_deg
                    for n in self.plan.features
                ):
                    raise ValueError("initial_reference_mismatch")
                self._start_ns = now_ns
                self.state = RehearsalState.RUNNING
            relative = now_ns - self._start_ns
            expected = self.plan.at(min(max(0, source - self._start_ns), self.plan.duration_ns))
            for name in self.plan.features:
                if abs(q[name] - expected.position_deg[name]) > self.tolerance[name]:
                    since = self._error_since.setdefault(name, source)
                    if source - since >= self.persistence_ns:
                        raise ValueError(f"persistent_tracking_error:{name}")
                else:
                    self._error_since.pop(name, None)
            self._last_sequence, self._last_source = sequence, source
            if relative >= self.plan.duration_ns:
                return self._release("final_tracking_error" if self._error_since else "completed")
            return self._decision(self.plan.at(relative))
        except (ValueError, TypeError, AttributeError) as exc:
            return self._release(str(exc))

    def poll(self, *, now_ns: int):
        if self.state is RehearsalState.RUNNING:
            try:
                self._clock(now_ns)
            except (ValueError, TypeError) as exc:
                return self._release(str(exc))
        return self._decision()

    def stop(self):
        if self.state in (RehearsalState.IDLE, RehearsalState.RUNNING):
            return self._release("operator_stop")
        return self._decision()

    def acknowledge_release(self, *, disabled: bool, resources_released: bool):
        if self.state is not RehearsalState.RELEASE_REQUIRED:
            raise ValueError("release acknowledgement is not expected")
        self.state = (
            RehearsalState.CLOSED
            if disabled is True and resources_released is True
            else RehearsalState.RELEASE_FAILED
        )
        return self._decision()
