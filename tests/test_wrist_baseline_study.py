from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from vlai_l1_runtime.wrist_baseline_study import (
    evaluate,
    evaluate_loaded,
    fit_study,
    load_holds,
    validate_context,
    validate_policy,
)

ROOT = Path(__file__).resolve().parents[1]
POLICY = json.loads((ROOT / "configs/diagnostics/right_wrist_baseline_study.json").read_text())


def holds():
    result = []
    for cycle in (1, 2):
        for leg, (q, direction) in enumerate(zip((0.8, 0, -1, -0.4), (1, -1, -1, 1), strict=True)):
            q += (cycle - 1) * 0.01
            result.append(
                {
                    "cycle": cycle,
                    "leg": leg,
                    "q_deg": [0] * 6 + [q, 1.5],
                    "direction": direction,
                    "torque_nm": 0.1 + 0.02 * q + 0.09 * direction,
                }
            )
    return result


def test_angle_direction_model_recovers_known_synthetic_hold_torque():
    study = fit_study(holds(), {}, POLICY)
    assert study["ft_validated"] is False
    assert study["runtime_eligible"] is False
    validation = study["temporal_holdout"]
    assert validation["supported_holds"] == [True] * 4
    assert validation["models"]["angle_direction"]["supported"]["rmse_nm"] < 1e-12
    assert validation["models"]["constant"]["supported"]["rmse_nm"] > 0.05


def test_validation_torque_never_changes_fitted_coefficients():
    original = fit_study(holds(), {}, POLICY)
    altered = holds()
    for hold in altered[4:]:
        hold["torque_nm"] += 100
    changed = fit_study(altered, {}, POLICY)
    assert original["models"] == changed["models"]
    assert changed["temporal_holdout"]["models"]["angle_direction"]["supported"]["rmse_nm"] > 99


@pytest.mark.parametrize(
    "changed", ["wrist_position", "other_joint", "gripper", "unknown_direction"]
)
def test_unsupported_queries_are_separated_from_supported_error(changed):
    study = fit_study(holds(), {}, POLICY)
    query = copy.deepcopy(holds()[4:5])
    if changed == "wrist_position":
        query[0]["q_deg"][6] += 10
    elif changed == "other_joint":
        query[0]["q_deg"][1] += 1
    elif changed == "gripper":
        query[0]["q_deg"][7] += 1
    else:
        query[0]["direction"] = 0
    result = evaluate(study, query)
    assert result["supported_holds"] == [False]
    for model in result["models"].values():
        assert model["supported"]["hold_count"] == 0
        assert model["supported"]["rmse_nm"] is None
        assert model["outside_support_diagnostic_only"]["hold_count"] == 1


def test_rank_deficient_fit_is_rejected():
    samples = holds()
    for hold in samples:
        hold["q_deg"][6] = 0
    with pytest.raises(ValueError, match="unidentifiable"):
        fit_study(samples, {}, POLICY)


def test_incomplete_cycle_split_is_rejected():
    with pytest.raises(ValueError, match="separated"):
        fit_study(holds()[:5], {}, POLICY)


def test_unsafe_or_nonfinite_offline_policy_rejected():
    for value in (float("nan"), float("inf"), -1, True):
        with pytest.raises(ValueError, match="positive finite"):
            validate_policy({**POLICY, "reversal_excursion_deg": value})


@pytest.mark.parametrize("load", [None, True, 0])
def test_unconfirmed_unloaded_context_is_rejected_before_reading_capture(tmp_path, load):
    path = tmp_path / "context.json"
    path.write_text(json.dumps({"side": "right", "external_load_present": load}))
    with pytest.raises(ValueError, match="explicit unloaded"):
        load_holds(None, tmp_path / "missing_capture", path, POLICY)


def test_loaded_response_is_preserved_without_refitting_or_zero_error_claim():
    study = fit_study(holds(), {}, POLICY)
    original = copy.deepcopy(study)
    loaded = copy.deepcopy(holds()[4:])
    for hold in loaded:
        hold["torque_nm"] += 0.12
    loaded[-1]["q_deg"][6] += 10
    result = evaluate_loaded(study, loaded)
    model = result["models"]["angle_direction"]
    assert model["supported_motor_torque_residual_nm"][:3] == pytest.approx([0.12] * 3)
    assert model["supported_motor_torque_residual_nm"][-1] is None
    assert model["outside_support_residual_diagnostic_only_nm"][-1] is not None
    assert result["ground_truth_joint_torque_nm"] is None
    assert "error_nm" not in model
    assert "supported" not in model
    assert study == original


def test_j6_sweep_cannot_be_silently_used_as_j7_baseline(tmp_path, monkeypatch):
    from vlai_l1_runtime import wrist_baseline_study as module

    context = tmp_path / "context.json"
    context.write_text(
        json.dumps(
            {
                "side": "right",
                "external_load_present": False,
                "follower_contact": False,
                "tool_context": "original_gripper_and_camera",
                "applies_to_stages": ["sweep"],
                "operator_statement": "confirmed empty",
            }
        )
    )
    (tmp_path / "report.json").write_text(json.dumps({"policy": {"active_motor": "joint_6"}}))
    monkeypatch.setattr(module, "review_automatic", lambda *_: {})
    with pytest.raises(ValueError, match="requires a J7"):
        load_holds(None, tmp_path, context, POLICY)


def test_loaded_capture_cannot_train_baseline():
    with pytest.raises(ValueError, match="cannot train"):
        fit_study(holds(), {"external_load_present": True}, POLICY)


def loaded_context():
    return {
        "side": "right",
        "external_load_present": True,
        "follower_contact": False,
        "tool_context": "original_gripper_and_camera",
        "applies_to_stages": ["sweep"],
        "operator_statement": "remote tied securely; hands clear",
        "payload": {
            "mass_g": 300,
            "mass_source": "operator_nominal",
            "attachment_description": "remote tied to gripper; moment arm unknown",
            "securely_attached": True,
        },
    }


def test_nominal_payload_is_labelled_and_rejected_as_unloaded():
    context = loaded_context()
    validate_context(context, loaded=True)
    with pytest.raises(ValueError, match="explicit unloaded"):
        validate_context(context)


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("mass_g", float("nan")),
        ("mass_g", True),
        ("mass_g", 0),
        ("mass_source", "unknown"),
        ("attachment_description", ""),
        ("securely_attached", False),
    ],
)
def test_loaded_capture_requires_payload_and_attachment_context(key, value):
    context = loaded_context()
    context["payload"][key] = value
    with pytest.raises(ValueError, match="payload mass"):
        validate_context(context, loaded=True)
