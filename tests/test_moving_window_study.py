import copy
import json
from pathlib import Path

import numpy as np
import pytest

from vlai_l1_runtime.moving_window_study import (
    evaluate,
    evaluate_loaded,
    extract_windows,
    fit_study,
    validate_frozen,
)

POLICY = json.loads(
    (
        Path(__file__).resolve().parents[1] / "configs/diagnostics/right_moving_window_study.json"
    ).read_text()
)


def windows():
    result = []
    for cycle in (1, 2):
        for i, angle in enumerate((-0.4, -0.2, 0, 0.1, 0.3, 0.5)):
            angle += (cycle - 1) * 0.01
            direction = -1 if i < 3 else 1
            result.append(
                {
                    "cycle": cycle,
                    "q_deg": [0, 0, 0, angle, 0, 0, 0, 0],
                    "direction": direction,
                    "torque_nm": 0.2 + 0.3 * angle + 0.4 * direction,
                }
            )
    return {"windows": result, "rejected_windows": []}


def test_fit_uses_only_first_cycle_and_measured_position_direction():
    data = windows()
    study = fit_study(data, "joint_4", {"external_load_present": False}, POLICY)
    assert study["temporal_holdout"]["models"]["angle_direction"]["supported_rmse_nm"] < 1e-12
    altered = copy.deepcopy(data)
    for w in altered["windows"][6:]:
        w["torque_nm"] += 10
    other = fit_study(altered, "joint_4", {"external_load_present": False}, POLICY)
    assert study["models"] == other["models"]
    assert other["temporal_holdout"]["models"]["angle_direction"][
        "supported_rmse_nm"
    ] == pytest.approx(10)
    assert study["static_supported"] is False
    assert study["ft_validated"] is False


@pytest.mark.parametrize("load", [True, None, 0])
def test_loaded_or_unknown_training_rejected(load):
    with pytest.raises(ValueError, match="unloaded"):
        fit_study(windows(), "joint_4", {"external_load_present": load}, POLICY)


def test_one_direction_or_insufficient_data_rejects_fit():
    data = windows()
    for w in data["windows"]:
        w["direction"] = 1
    with pytest.raises(ValueError, match="bidirectional"):
        fit_study(data, "joint_4", {"external_load_present": False}, POLICY)


def test_different_other_joint_pose_is_outside_support():
    study = fit_study(windows(), "joint_4", {"external_load_present": False}, POLICY)
    query = windows()["windows"][6:]
    for w in query:
        w["q_deg"][5] = 1
    result = evaluate(study, query)
    assert not any(result["supported_windows"])
    assert result["models"]["angle_direction"]["supported_rmse_nm"] is None
    for diagnostic in result["support_diagnostics"]:
        assert diagnostic["direction_supported"]
        violation = diagnostic["pose_violations"][0]
        assert violation["motor"] == "joint_6"
        assert violation["distance_outside_deg"] == pytest.approx(0.9)


def samples():
    x = np.zeros((650, 17))
    x[:, 0] = np.arange(650) / 100
    return x


MOTION = {"active_motor": "joint_4", "transition_seconds": 1, "hold_seconds": 0.5, "cycles": 1}


def test_static_measured_joint_is_excluded_even_during_commanded_transition():
    result = extract_windows(samples(), MOTION, POLICY)
    assert result["windows"] == []
    assert len(result["rejected_windows"]) == 4
    assert all(w["reason"] == "insufficient_measured_motion" for w in result["rejected_windows"])


def test_direction_is_measured_and_other_joint_movement_is_rejected():
    x = samples()
    x[:, 4] = -0.2 * x[:, 0]
    result = extract_windows(x, MOTION, POLICY)
    assert len(result["windows"]) == 4
    assert all(w["direction"] == -1 for w in result["windows"])
    x[:, 1] = x[:, 0]
    assert extract_windows(x, MOTION, POLICY)["windows"] == []


def test_reversal_inside_window_is_rejected():
    x = samples()
    t = x[:, 0] - 0.5
    x[:, 4] = np.where(t <= 0.5, 1.6 * t, 0.8 - 1.2 * (t - 0.5))
    result = extract_windows(x, MOTION, POLICY)
    assert result["rejected_windows"][0]["reason"] == "reversal_or_oscillation"


def test_invalid_timestamps_and_policy_fail_closed():
    x = samples()
    x[1, 0] = x[0, 0]
    with pytest.raises(ValueError, match="ordered"):
        extract_windows(x, MOTION, POLICY)
    with pytest.raises(ValueError, match="positive finite"):
        extract_windows(samples(), MOTION, {**POLICY, "minimum_displacement_deg": float("nan")})


@pytest.mark.parametrize("bad", [None, "native_sha256", "raw_sha256", "joint", "load", "policy"])
def test_independent_validation_requires_same_context_and_distinct_capture(bad):
    provenance = {
        "external_load_present": False,
        "raw_sha256": "first",
        "native_sha256": "native",
        "config_sha256": "config",
        "sdk_revision": "sdk",
        "tool_context": "tool",
    }
    study = fit_study(windows(), "joint_4", provenance, POLICY)
    other = {**provenance, "raw_sha256": "second"}
    motor, policy = "joint_4", copy.deepcopy(POLICY)
    if bad == "native_sha256":
        other[bad] = "different"
    if bad == "raw_sha256":
        other[bad] = "first"
    if bad == "joint":
        motor = "joint_6"
    if bad == "load":
        other["external_load_present"] = True
    if bad == "policy":
        policy["window_seconds"] = 2
    if bad is None:
        validate_frozen(study, motor, other, policy)
    else:
        with pytest.raises(ValueError, match=r"mismatch|different recording|unloaded"):
            validate_frozen(study, motor, other, policy)


def test_loaded_response_is_preserved_without_refit_or_zero_error_score():
    study = fit_study(windows(), "joint_4", {"external_load_present": False}, POLICY)
    original = copy.deepcopy(study)
    query = windows()["windows"][6:]
    for w in query:
        w["torque_nm"] += 0.12
    query[-1]["q_deg"][3] += 10
    result = evaluate_loaded(study, query)
    model = result["models"]["angle_direction"]
    assert model["supported_motor_torque_residual_nm"][:-1] == pytest.approx([0.12] * 5)
    assert model["supported_motor_torque_residual_nm"][-1] is None
    assert model["outside_support_residual_diagnostic_only_nm"][-1] is not None
    assert "supported_rmse_nm" not in model
    assert result["ground_truth_joint_torque_nm"] is None
    assert study == original


def test_loaded_provenance_requires_explicit_validation_mode():
    provenance = {
        "external_load_present": False,
        "raw_sha256": "first",
        "native_sha256": "native",
        "config_sha256": "config",
        "sdk_revision": "sdk",
        "tool_context": "tool",
    }
    study = fit_study(windows(), "joint_4", provenance, POLICY)
    loaded = {**provenance, "raw_sha256": "second", "external_load_present": True}
    validate_frozen(study, "joint_4", loaded, POLICY, loaded=True)
    with pytest.raises(ValueError, match="load label"):
        validate_frozen(study, "joint_4", loaded, POLICY)
