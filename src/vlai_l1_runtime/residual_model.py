"""Offline local posture/history residual prototype; never a live calibration.

Callers supply validated, consistently mapped joint data. Residual means
model gravity minus measured motor torque. Fit only explicitly unloaded runs;
freeze the result before evaluating loaded runs. No hardware or file access.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .wrench import ARM_JOINT_NAMES


def _vector(values, label, *, positive=False):
    if not isinstance(values, tuple) or len(values) != len(ARM_JOINT_NAMES):
        raise ValueError(f"{label} requires seven ordered values")
    if any(
        isinstance(v, bool)
        or not isinstance(v, (float, int))
        or not math.isfinite(v)
        or (positive and v <= 0)
        for v in values
    ):
        raise ValueError(f"invalid {label}")


class ApproachHistory:
    """Offline per-joint last resolved direction, retained through a stop.

    Reversal requires an excursion from the last extremum, not a one-frame
    velocity sign. Thresholds must exceed measured position quantization/noise.
    A sample gap clears history; malformed or regressed input latches failure.
    """

    def __init__(self, *, reversal_excursion_rad: tuple[float, ...], maximum_gap_ns: int):
        _vector(reversal_excursion_rad, "reversal excursion", positive=True)
        if type(maximum_gap_ns) is not int or maximum_gap_ns <= 0:
            raise ValueError("positive history gap budget required")
        self._excursion = reversal_excursion_rad
        self._gap = maximum_gap_ns
        self._timestamp = None
        self._extrema = None
        self._direction = [0] * 7
        self._failed = False

    def update(self, *, joint_names, monotonic_ns, position_rad) -> tuple[int, ...]:
        if self._failed:
            raise ValueError("history input failure latched")
        try:
            if joint_names != ARM_JOINT_NAMES:
                raise ValueError("exact named arm vector required")
            _vector(position_rad, "history position")
            if type(monotonic_ns) is not int or monotonic_ns <= 0:
                raise ValueError("positive history timestamp required")
            if self._timestamp is not None and monotonic_ns <= self._timestamp:
                raise ValueError("non-monotonic history timestamp")
        except ValueError:
            self._failed = True
            raise
        if self._timestamp is None or monotonic_ns - self._timestamp > self._gap:
            self._direction = [0] * 7
            self._extrema = list(position_rad)
        else:
            for j, (q, threshold) in enumerate(zip(position_rad, self._excursion, strict=True)):
                delta = q - self._extrema[j]
                direction = self._direction[j]
                if (direction <= 0 and delta >= threshold) or (
                    direction >= 0 and delta <= -threshold
                ):
                    self._direction[j] = 1 if delta > 0 else -1
                    self._extrema[j] = q
                elif (direction > 0 and delta > 0) or (direction < 0 and delta < 0):
                    self._extrema[j] = q
        self._timestamp = monotonic_ns
        return tuple(self._direction)


@dataclass(frozen=True)
class ResidualSample:
    joint_names: tuple[str, ...]
    context_key: str  # Hash of side, mapping, model, tool, opening and controller contract.
    recording_id: str
    monotonic_ns: int
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    approach_direction: tuple[int, ...]  # Last resolved motion sign; 0 means unknown.
    model_minus_motor_nm: tuple[float, ...]
    external_load_present: bool

    def __post_init__(self):
        if self.joint_names != ARM_JOINT_NAMES:
            raise ValueError("exact named arm vector required")
        if any(
            not isinstance(s, str) or not s.strip() for s in (self.context_key, self.recording_id)
        ):
            raise ValueError("explicit context and recording identity required")
        if type(self.monotonic_ns) is not int or self.monotonic_ns <= 0:
            raise ValueError("positive monotonic timestamp required")
        for label in ("position_rad", "velocity_rad_s", "model_minus_motor_nm"):
            _vector(getattr(self, label), label)
        if (
            not isinstance(self.approach_direction, tuple)
            or len(self.approach_direction) != 7
            or any(type(v) is not int or v not in (-1, 0, 1) for v in self.approach_direction)
        ):
            raise ValueError("approach direction requires seven signs")
        if type(self.external_load_present) is not bool:
            raise ValueError("explicit external load label required")


@dataclass(frozen=True)
class ResidualPolicy:
    position_scale_rad: tuple[float, ...]
    velocity_scale_rad_s: tuple[float, ...]
    minimum_position_span_rad: tuple[float, ...]
    maximum_speed_rad_s: tuple[float, ...]
    maximum_validation_error_nm: tuple[float, ...]
    minimum_samples_per_split: int
    maximum_feature_condition: float
    support_radius: float

    def __post_init__(self):
        for label in (
            "position_scale_rad",
            "velocity_scale_rad_s",
            "minimum_position_span_rad",
            "maximum_speed_rad_s",
            "maximum_validation_error_nm",
        ):
            _vector(getattr(self, label), label, positive=True)
        if type(self.minimum_samples_per_split) is not int or self.minimum_samples_per_split < 10:
            raise ValueError("at least ten samples per split required")
        for label in ("maximum_feature_condition", "support_radius"):
            value = getattr(self, label)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"invalid {label}")
            if value <= 0 or (label == "maximum_feature_condition" and value < 1):
                raise ValueError(f"invalid {label}")


@dataclass(frozen=True)
class ResidualPrediction:
    monotonic_ns: int
    baseline_nm: tuple[float, ...]
    corrected_external_torque_nm: tuple[float, ...]
    held_out_max_error_nm: tuple[float, ...]  # Empirical error, NOT a guaranteed bound.
    ft_validated: bool = field(default=False, init=False)


@dataclass(frozen=True)
class OfflineResidualModel:
    """Local affine bias: seven q features + own velocity + own approach sign.

    Cross-joint history, acceleration, thermal drift and general hysteresis are
    not identified by this prototype. Unknown history and unsupported queries
    are rejected. No update-on-contact/adaptation method exists.
    """

    context_key: str
    policy: ResidualPolicy
    center_rad: tuple[float, ...]
    coefficients: tuple[tuple[float, ...], ...]
    training: tuple[ResidualSample, ...]
    held_out_max_error_nm: tuple[float, ...]
    feature_condition: tuple[float, ...]
    ft_validated: bool = field(default=False, init=False)

    def predict(self, sample: ResidualSample) -> ResidualPrediction:
        import numpy as np

        _check_sample(sample, self.context_key, self.policy)
        q = np.array(sample.position_rad)
        train_q = np.array([s.position_rad for s in self.training])
        if np.any(q < train_q.min(axis=0)) or np.any(q > train_q.max(axis=0)):
            raise ValueError("outside observed posture range")
        pose_distance = np.max(np.abs((train_q - q) / self.policy.position_scale_rad), axis=1)
        baseline = []
        for j in range(7):
            supported = any(
                pose_distance[i] <= self.policy.support_radius
                and s.approach_direction[j] == sample.approach_direction[j]
                and abs(s.velocity_rad_s[j] - sample.velocity_rad_s[j])
                <= self.policy.support_radius * self.policy.velocity_scale_rad_s[j]
                for i, s in enumerate(self.training)
            )
            if not supported:
                raise ValueError(f"joint_{j + 1}: outside posture/history support")
            features = _features((sample,), self.center_rad, self.policy, j)
            baseline.append(float(features[0] @ np.array(self.coefficients[j])))
        corrected = np.array(sample.model_minus_motor_nm) - baseline
        if not np.isfinite((*baseline, *corrected)).all():
            raise ValueError("nonfinite residual prediction")
        return ResidualPrediction(
            sample.monotonic_ns,
            tuple(baseline),
            tuple(float(v) for v in corrected),
            self.held_out_max_error_nm,
        )


def _check_sample(sample, context, policy):
    if not isinstance(sample, ResidualSample) or sample.context_key != context:
        raise ValueError("residual context mismatch")
    if 0 in sample.approach_direction:
        raise ValueError("unknown approach history; do not infer it from zero velocity")
    if any(
        abs(v) > limit
        for v, limit in zip(sample.velocity_rad_s, policy.maximum_speed_rad_s, strict=True)
    ):
        raise ValueError("outside calibrated speed domain")


def _features(samples, center, policy, joint):
    import numpy as np

    return np.column_stack(
        (
            np.ones(len(samples)),
            (np.array([s.position_rad for s in samples]) - center) / policy.position_scale_rad,
            np.array([s.velocity_rad_s[joint] for s in samples])
            / policy.velocity_scale_rad_s[joint],
            np.array([s.approach_direction[joint] for s in samples]),
        )
    )


def fit_residual_model(
    *,
    training: tuple[ResidualSample, ...],
    validation: tuple[ResidualSample, ...],
    policy: ResidualPolicy,
) -> OfflineResidualModel:
    """Fit on training runs; require independent unloaded validation runs.

    All tolerances are explicit offline policy inputs, not physical defaults.
    Caller must enforce raw identity, bounds, timing, mapping and history validity
    before constructing samples. This function does not replace those checks.
    """
    import numpy as np

    if not isinstance(policy, ResidualPolicy):
        raise ValueError("explicit residual policy required")
    for samples in (training, validation):
        if not isinstance(samples, tuple) or len(samples) < policy.minimum_samples_per_split:
            raise ValueError("insufficient samples per independent split")
        if any(not isinstance(s, ResidualSample) for s in samples):
            raise ValueError("validated residual samples required")
    context = training[0].context_key
    if {s.recording_id for s in training} & {s.recording_id for s in validation}:
        raise ValueError("training/validation recording overlap")
    for samples in (training, validation):
        clocks = {}
        for sample in samples:
            _check_sample(sample, context, policy)
            if sample.external_load_present:
                raise ValueError("fit/validation require explicit unloaded samples")
            if sample.monotonic_ns <= clocks.get(sample.recording_id, 0):
                raise ValueError("non-monotonic recording timestamp")
            clocks[sample.recording_id] = sample.monotonic_ns
    q = np.array([s.position_rad for s in training])
    if np.any(np.ptp(q, axis=0) < policy.minimum_position_span_rad):
        raise ValueError("insufficient posture excitation")
    center = tuple(float(v) for v in q.mean(axis=0))
    coefficients, conditions = [], []
    for joint in range(7):
        matrix = _features(training, center, policy, joint)
        target = np.array([s.model_minus_motor_nm[joint] for s in training])
        weights, _, rank, singular = np.linalg.lstsq(matrix, target, rcond=None)
        if rank != matrix.shape[1]:
            raise ValueError(f"joint_{joint + 1}: unidentifiable posture/velocity/history features")
        condition = float(singular[0] / singular[-1])
        if not math.isfinite(condition) or condition > policy.maximum_feature_condition:
            raise ValueError(f"joint_{joint + 1}: ill-conditioned residual fit")
        coefficients.append(tuple(float(v) for v in weights))
        conditions.append(condition)
    candidate = OfflineResidualModel(
        context, policy, center, tuple(coefficients), training, (0.0,) * 7, tuple(conditions)
    )
    errors = np.array([candidate.predict(s).corrected_external_torque_nm for s in validation])
    maximum = np.max(np.abs(errors), axis=0)
    if np.any(maximum > policy.maximum_validation_error_nm):
        raise ValueError("independent unloaded validation error exceeded")
    return OfflineResidualModel(
        context,
        policy,
        center,
        tuple(coefficients),
        training,
        tuple(float(v) for v in maximum),
        tuple(conditions),
    )
