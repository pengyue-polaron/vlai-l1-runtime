from __future__ import annotations

import numpy as np
import pytest

from vlai_l1_runtime.wrench import ARM_JOINT_NAMES, estimate_wrench, propagated_torque_error


def inputs():
    # Synthetic full-rank geometric Jacobian with a seventh redundant joint.
    matrix = np.column_stack((np.eye(6), np.array([0.3, 0.1, 0.2, -0.1, 0.4, 0.2])))
    wrench = np.array([2.0, -3.0, 4.0, 0.2, -0.1, 0.3])
    torque = matrix.T @ wrench
    return {
        "joint_names": ARM_JOINT_NAMES,
        "jacobian": matrix,
        "external_joint_torque_nm": dict(zip(ARM_JOINT_NAMES, torque, strict=True)),
        "frame": "test_tool",
        "reference_point": "test_tcp",
        "monotonic_ns": 100,
        "characteristic_length_m": 0.2,
        "damping_m": 0.0,
        "minimum_singular_value_m": 1e-6,
        "maximum_condition_number": 100.0,
        "maximum_residual_nm": 1e-8,
    }, wrench


def test_reconstructs_known_six_axis_wrench_and_preserves_sign_units():
    args, expected = inputs()
    actual = estimate_wrench(**args)
    np.testing.assert_allclose((*actual.force_n, *actual.moment_nm), expected)
    np.testing.assert_allclose(actual.residual_joint_torque_nm, 0.0, atol=1e-14)
    assert actual.frame == "test_tool"
    assert actual.reference_point == "test_tcp"
    assert actual.monotonic_ns == 100


def test_force_moment_scaling_does_not_change_undamped_solution():
    args, expected = inputs()
    for length in (0.1, 1.0, 2.0):
        actual = estimate_wrench(**(args | {"characteristic_length_m": length}))
        np.testing.assert_allclose((*actual.force_n, *actual.moment_nm), expected)


def test_damped_solver_reports_nonzero_residual():
    args, expected = inputs()
    actual = estimate_wrench(**(args | {"damping_m": 0.01, "maximum_residual_nm": 0.1}))
    assert max(abs(v) for v in actual.residual_joint_torque_nm) > 0
    np.testing.assert_allclose((*actual.force_n, *actual.moment_nm), expected, atol=0.002)


def test_joint_torque_not_explainable_by_endpoint_wrench_is_rejected():
    args, _ = inputs()
    matrix = args["jacobian"]
    u, _, _ = np.linalg.svd(matrix.T, full_matrices=True)
    residual = u[:, -1]  # Orthogonal to every achievable endpoint wrench torque.
    args["external_joint_torque_nm"] = dict(zip(ARM_JOINT_NAMES, residual, strict=True))
    with pytest.raises(ValueError, match="residual"):
        estimate_wrench(**args)


@pytest.mark.parametrize("scale", [0, 1e-9])
def test_singularity_is_rejected_even_with_damping(scale):
    args, _ = inputs()
    args["jacobian"][5] *= scale
    args["damping_m"] = 0.1
    with pytest.raises(ValueError, match="unobservable"):
        estimate_wrench(**args)


def test_condition_limit_rejects_nearly_singular_geometry():
    args, _ = inputs()
    args["jacobian"][5] *= 0.001
    with pytest.raises(ValueError, match="condition"):
        estimate_wrench(**args)


@pytest.mark.parametrize(
    "changes",
    [
        {"joint_names": tuple(reversed(ARM_JOINT_NAMES))},
        {"external_joint_torque_nm": {"joint_1": 0.0}},
        {"jacobian": np.zeros((7, 6))},
        {"jacobian": np.full((6, 7), np.nan)},
        {"jacobian": np.ones((6, 7), dtype=bool)},
        {"jacobian": np.ones((6, 7), dtype=complex)},
        {"external_joint_torque_nm": dict.fromkeys(ARM_JOINT_NAMES, "1.0")},
        {"external_joint_torque_nm": dict.fromkeys(ARM_JOINT_NAMES, float("inf"))},
        {"frame": ""},
        {"reference_point": ""},
        {"monotonic_ns": True},
        {"characteristic_length_m": 0.0},
        {"damping_m": -1.0},
        {"minimum_singular_value_m": 0.0},
        {"maximum_condition_number": 0.5},
        {"maximum_residual_nm": float("inf")},
    ],
)
def test_invalid_estimation_inputs_are_rejected(changes):
    args, _ = inputs()
    with pytest.raises(ValueError, match=r".+"):
        estimate_wrench(**(args | changes))


def test_componentwise_error_bound_is_attained_by_adversarial_torque_errors():
    args, _ = inputs()
    args["maximum_residual_nm"] = 1.0
    original = estimate_wrench(**args)
    bounds = dict(zip(ARM_JOINT_NAMES, (0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07), strict=True))
    propagated = propagated_torque_error(original, bounds)
    for i, row in enumerate(original.torque_to_wrench):
        altered = args | {
            "external_joint_torque_nm": {
                name: args["external_joint_torque_nm"][name] + np.sign(gain) * bounds[name]
                for name, gain in zip(ARM_JOINT_NAMES, row, strict=True)
            }
        }
        shifted = estimate_wrench(**altered)
        difference = np.array((*shifted.force_n, *shifted.moment_nm)) - (
            *original.force_n,
            *original.moment_nm,
        )
        assert difference[i] == pytest.approx(propagated[i], abs=1e-13)


@pytest.mark.parametrize("value", [-0.1, float("nan"), True])
def test_invalid_torque_uncertainty_rejected(value):
    args, _ = inputs()
    with pytest.raises(ValueError, match="nonnegative"):
        propagated_torque_error(estimate_wrench(**args), dict.fromkeys(ARM_JOINT_NAMES, value))
