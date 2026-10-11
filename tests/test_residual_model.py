from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from vlai_l1_runtime.residual_model import (
    ApproachHistory,
    ResidualPolicy,
    ResidualSample,
    fit_residual_model,
)
from vlai_l1_runtime.wrench import ARM_JOINT_NAMES, estimate_wrench


def policy():
    return ResidualPolicy(
        position_scale_rad=(0.03,) * 7,
        velocity_scale_rad_s=(0.005,) * 7,
        minimum_position_span_rad=(0.02,) * 7,
        maximum_speed_rad_s=(0.01,) * 7,
        maximum_validation_error_nm=(0.02,) * 7,
        minimum_samples_per_split=30,
        maximum_feature_condition=100.0,
        support_radius=1.5,
    )


def baseline(q, v, history):
    # Synthetic cross-joint posture effects plus each joint's velocity/history.
    coupling = np.arange(49).reshape(7, 7) / 40 + np.eye(7) * 3
    return 0.4 + coupling @ q + 2 * v + 0.15 * history


def samples(recording, seed, extent, count=160):
    rng = np.random.default_rng(seed)
    result = []
    for i in range(count):
        q = rng.uniform(-extent, extent, 7)
        history = rng.choice([-1, 1], 7)
        v = history * rng.uniform(0, 0.004, 7)
        result.append(
            ResidualSample(
                ARM_JOINT_NAMES,
                "synthetic-contract",
                recording,
                (i + 1) * 10000000,
                tuple(q),
                tuple(v),
                tuple(int(h) for h in history),
                tuple(baseline(q, v, history)),
                False,
            )
        )
    return tuple(result)


@pytest.fixture
def splits():
    return samples("train", 31, 0.03), samples("held-out", 64, 0.022)


def fit(splits):
    train, validation = splits
    return fit_residual_model(training=train, validation=validation, policy=policy())


def test_recovers_injected_wrench_through_pose_changes_and_reversals(splits):
    model = fit(splits)
    wrench = np.array([1.2, -2.0, 3.0, 0.1, -0.2, 0.3])
    old_baseline = np.array(splits[0][0].model_minus_motor_nm)
    fixed_tare_errors = []
    for sample in samples("loaded", 123, 0.02, count=24):
        # Independent synthetic Jacobian changes with each posture.
        jacobian = np.column_stack(
            (np.diag(1 + np.array(sample.position_rad[:6])), np.ones(6) * 0.1)
        )
        external = jacobian.T @ wrench
        loaded = replace(
            sample,
            external_load_present=True,
            model_minus_motor_nm=tuple(np.array(sample.model_minus_motor_nm) + external),
        )
        prediction = model.predict(loaded)
        np.testing.assert_allclose(prediction.corrected_external_torque_nm, external, atol=1e-12)
        estimate = estimate_wrench(
            joint_names=ARM_JOINT_NAMES,
            jacobian=jacobian,
            external_joint_torque_nm=dict(
                zip(ARM_JOINT_NAMES, prediction.corrected_external_torque_nm, strict=True)
            ),
            frame="synthetic_base",
            reference_point="synthetic_tcp",
            monotonic_ns=sample.monotonic_ns,
            characteristic_length_m=0.2,
            damping_m=0.0,
            minimum_singular_value_m=1e-6,
            maximum_condition_number=100.0,
            maximum_residual_nm=1e-8,
        )
        np.testing.assert_allclose((*estimate.force_n, *estimate.moment_nm), wrench, atol=1e-12)
        fixed_tare_errors.append(
            np.linalg.norm(np.array(sample.model_minus_motor_nm) - old_baseline)
        )
        assert prediction.ft_validated is False
    assert max(fixed_tare_errors) > 0.3
    assert max(model.held_out_max_error_nm) < 1e-12
    assert model.ft_validated is False
    with pytest.raises(FrozenInstanceError):
        model.context_key = "changed"


def test_holds_last_direction_at_zero_velocity_instead_of_treating_it_as_zero(splits):
    model = fit(splits)
    sample = splits[1][0]
    outputs = []
    for sign in (-1, 1):
        q = np.zeros(7)
        v = np.zeros(7)
        h = np.ones(7) * sign
        query = replace(
            sample,
            position_rad=tuple(q),
            velocity_rad_s=tuple(v),
            approach_direction=(sign,) * 7,
            model_minus_motor_nm=tuple(baseline(q, v, h)),
        )
        result = model.predict(query)
        np.testing.assert_allclose(result.corrected_external_torque_nm, 0, atol=1e-12)
        outputs.append(result.baseline_nm)
    np.testing.assert_allclose(np.array(outputs[1]) - outputs[0], 0.3, atol=1e-12)


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ({"context_key": "other-tool"}, "context"),
        ({"position_rad": (2.0,) * 7}, "posture range"),
        ({"velocity_rad_s": (1.0,) * 7}, "speed"),
        ({"approach_direction": (0,) * 7}, "unknown approach"),
    ],
)
def test_rejects_unsupported_queries(splits, change, reason):
    with pytest.raises(ValueError, match=reason):
        fit(splits).predict(replace(splits[1][0], **change))


@pytest.mark.parametrize(
    ("fault", "reason"),
    [
        ("loaded", "unloaded"),
        ("overlap", "overlap"),
        ("clock", "monotonic"),
        ("still", "excitation"),
        ("one_direction", "unidentifiable"),
        ("validation_bias", "validation error"),
        ("unknown_history", "unknown approach"),
    ],
)
def test_rejects_unusable_calibration(splits, fault, reason):
    training, validation = splits
    if fault == "loaded":
        training = (replace(training[0], external_load_present=True), *training[1:])
    elif fault == "overlap":
        validation = tuple(replace(s, recording_id="train") for s in validation)
    elif fault == "clock":
        training = (training[1], training[0], *training[2:])
    elif fault == "still":
        training = tuple(replace(s, position_rad=(0.0,) * 7) for s in training)
    elif fault == "one_direction":
        training = tuple(replace(s, approach_direction=(1,) * 7) for s in training)
    elif fault == "unknown_history":
        training = tuple(replace(s, approach_direction=(0,) * 7) for s in training)
    elif fault == "validation_bias":
        validation = tuple(replace(s, model_minus_motor_nm=(10.0,) * 7) for s in validation)
    with pytest.raises(ValueError, match=reason):
        fit_residual_model(training=training, validation=validation, policy=policy())


@pytest.mark.parametrize(
    "change",
    [
        {"joint_names": tuple(reversed(ARM_JOINT_NAMES))},
        {"position_rad": (float("nan"),) * 7},
        {"velocity_rad_s": (True,) * 7},
        {"external_load_present": None},
        {"monotonic_ns": True},
        {"approach_direction": (True,) * 7},
        {"recording_id": ""},
    ],
)
def test_rejects_malformed_samples(splits, change):
    with pytest.raises(ValueError, match=r".+"):
        replace(splits[0][0], **change)


def test_local_support_rejects_unobserved_combinations_inside_global_box(splits):
    training, validation = splits
    tight_policy = replace(policy(), support_radius=1e-10)
    with pytest.raises(ValueError, match="posture/history support"):
        fit_residual_model(training=training, validation=validation, policy=tight_policy)


def test_history_resolves_cumulative_motion_ignores_jitter_and_clears_after_gap():
    history = ApproachHistory(reversal_excursion_rad=(0.01,) * 7, maximum_gap_ns=100)
    path = [0.0, 0.004, 0.008, 0.012, 0.03, 0.029, 0.03, 0.019, 0.0]
    expected = [0, 0, 0, 1, 1, 1, 1, -1, -1]
    for t, (q, sign) in enumerate(zip(path, expected, strict=True), start=1):
        assert (
            history.update(joint_names=ARM_JOINT_NAMES, monotonic_ns=t, position_rad=(q,) * 7)
            == (sign,) * 7
        )
    assert (
        history.update(joint_names=ARM_JOINT_NAMES, monotonic_ns=200, position_rad=(0.0,) * 7)
        == (0,) * 7
    )


def test_history_timestamp_fault_is_latched():
    history = ApproachHistory(reversal_excursion_rad=(0.01,) * 7, maximum_gap_ns=100)
    args = {"joint_names": ARM_JOINT_NAMES, "position_rad": (0.0,) * 7}
    history.update(**args, monotonic_ns=10)
    with pytest.raises(ValueError, match="non-monotonic"):
        history.update(**args, monotonic_ns=9)
    with pytest.raises(ValueError, match="latched"):
        history.update(**args, monotonic_ns=11)


@pytest.mark.parametrize(
    "change",
    [
        {"support_radius": float("nan")},
        {"maximum_feature_condition": 0.5},
        {"minimum_samples_per_split": True},
        {"position_scale_rad": (0.0,) * 7},
    ],
)
def test_policy_requires_explicit_finite_positive_scales(change):
    with pytest.raises(ValueError, match=r".+"):
        replace(policy(), **change)
