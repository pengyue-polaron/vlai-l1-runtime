"""Hardware-free wrench reconstruction from already compensated joint torques.

This module does not interpret raw SDK torque, infer a coordinate mapping, or
provide gravity/friction compensation. Those require independently verified
inputs. No estimate from this module is a safety or force-control measurement.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from .configuration import MOTOR_NAMES

ARM_JOINT_NAMES = MOTOR_NAMES[:-1]
WRENCH_NAMES = ("fx", "fy", "fz", "mx", "my", "mz")


@dataclass(frozen=True)
class WrenchEstimate:
    """Environment-on-robot wrench at the explicitly named point and frame."""

    frame: str
    reference_point: str
    monotonic_ns: int
    force_n: tuple[float, ...]
    moment_nm: tuple[float, ...]
    residual_joint_torque_nm: tuple[float, ...]
    scaled_singular_values: tuple[float, ...]
    condition_number: float
    torque_to_wrench: tuple[tuple[float, ...], ...]


def estimate_wrench(
    *,
    joint_names: tuple[str, ...],
    jacobian: Sequence[Sequence[float]],
    external_joint_torque_nm: Mapping[str, float],
    frame: str,
    reference_point: str,
    monotonic_ns: int,
    characteristic_length_m: float,
    damping_m: float,
    minimum_singular_value_m: float,
    maximum_condition_number: float,
    maximum_residual_nm: float,
) -> WrenchEstimate:
    """Solve tau_ext = J.T @ [force_N, moment_Nm] using damped SVD.

    J rows must be [linear velocity of reference point; angular velocity],
    both expressed in ``frame``; columns follow the exact seven-joint order
    in radians. A spatial twist Jacobian about another origin is not suitable.
    ``tau_ext`` is environment-on-robot torque, after dynamics/friction/bias
    compensation and verified motor-to-joint conversion. Never pass raw motor
    torque directly. Under M*qdd + C + g + friction = tau_motor + tau_ext,
    tau_ext = model - tau_motor, not the opposite sign.

    All numerical acceptance parameters are explicit. Moment/L is scaled into
    force units before solving; the six singular values therefore have units
    of metres. Rank/conditioning and residual failures reject the estimate.
    Damping cannot restore an unobservable wrench direction.
    """

    if joint_names != ARM_JOINT_NAMES:
        raise ValueError("Jacobian columns require the exact seven-joint order; exclude gripper")
    if set(external_joint_torque_nm) != set(joint_names):
        raise ValueError("external torque requires exactly the seven named arm joints")
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError("wrench frame must be explicit")
    if not isinstance(reference_point, str) or not reference_point.strip():
        raise ValueError("wrench reference point must be explicit")
    if isinstance(monotonic_ns, bool) or not isinstance(monotonic_ns, int) or monotonic_ns <= 0:
        raise ValueError("monotonic_ns must be a positive integer")
    for name, value in (
        ("characteristic_length_m", characteristic_length_m),
        ("minimum_singular_value_m", minimum_singular_value_m),
        ("maximum_condition_number", maximum_condition_number),
        ("maximum_residual_nm", maximum_residual_nm),
    ):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be a finite positive number")
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be a finite positive number")
    if maximum_condition_number < 1:
        raise ValueError("maximum_condition_number must be at least one")
    if (
        isinstance(damping_m, bool)
        or not isinstance(damping_m, (int, float))
        or not math.isfinite(damping_m)
        or damping_m < 0
    ):
        raise ValueError("damping_m must be finite and non-negative")

    # Optional numerical dependency stays lazy; static validation needs no NumPy.
    import numpy as np

    matrix = np.asarray(jacobian)
    torque = np.asarray([external_joint_torque_nm[name] for name in joint_names])
    if matrix.dtype.kind not in "fiu" or torque.dtype.kind not in "fiu":
        raise ValueError("Jacobian and torque must contain real numbers, not bool/string/complex")
    matrix = matrix.astype(float)
    torque = torque.astype(float)
    if matrix.shape != (6, 7) or not np.isfinite(matrix).all():
        raise ValueError("Jacobian must be finite with shape (6, 7)")
    if torque.shape != (7,) or not np.isfinite(torque).all():
        raise ValueError("external joint torques must be finite scalars")
    scale = np.array([1.0] * 3 + [characteristic_length_m] * 3)
    with np.errstate(over="raise", invalid="raise", divide="raise"):
        try:
            a = matrix.T * scale
            u, singular, vh = np.linalg.svd(a, full_matrices=False)
            if singular[-1] < minimum_singular_value_m:
                raise ValueError("wrench is unobservable: scaled Jacobian singular value too small")
            condition = float(singular[0] / singular[-1])
            if condition > maximum_condition_number:
                raise ValueError("wrench is unreliable: scaled Jacobian condition limit exceeded")
            gain = scale[:, None] * (
                vh.T @ ((singular / (singular**2 + damping_m**2))[:, None] * u.T)
            )
            solution = gain @ torque
            residual = torque - matrix.T @ solution
            if not np.isfinite(solution).all() or not np.isfinite(residual).all():
                raise ValueError("wrench solution is not finite")
            if np.max(np.abs(residual)) > maximum_residual_nm:
                raise ValueError("joint torque residual exceeds the explicit acceptance limit")
        except (FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
            raise ValueError("wrench numerical solve failed") from exc
    return WrenchEstimate(
        frame=frame,
        reference_point=reference_point,
        monotonic_ns=monotonic_ns,
        force_n=tuple(float(value) for value in solution[:3]),
        moment_nm=tuple(float(value) for value in solution[3:]),
        residual_joint_torque_nm=tuple(float(value) for value in residual),
        scaled_singular_values=tuple(float(value) for value in singular),
        condition_number=condition,
        torque_to_wrench=tuple(tuple(float(value) for value in row) for row in gain),
    )


def propagated_torque_error(
    estimate: WrenchEstimate, error_bound_nm: Mapping[str, float]
) -> tuple[float, ...]:
    """Componentwise worst-case input-error propagation for the fixed solve.

    Returns [N,N,N,Nm,Nm,Nm]. This only bounds supplied joint-torque errors;
    unknown geometry/model error and damping bias are not covered. It is not
    a statistical confidence interval or a calibrated FT accuracy claim.
    """
    if set(error_bound_nm) != set(ARM_JOINT_NAMES) or any(
        isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v < 0
        for v in error_bound_nm.values()
    ):
        raise ValueError("torque error bounds require seven named finite nonnegative values")
    result = tuple(
        sum(
            abs(gain) * error_bound_nm[name]
            for name, gain in zip(ARM_JOINT_NAMES, row, strict=True)
        )
        for row in estimate.torque_to_wrench
    )
    if not all(math.isfinite(v) for v in result):
        raise ValueError("nonfinite propagated torque error")
    return result
