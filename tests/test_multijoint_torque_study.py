import copy
import json
from pathlib import Path

import numpy as np
import pytest

from vlai_l1_runtime.automatic_trajectory import sweep_duration, target_offsets
from vlai_l1_runtime.multijoint_torque_study import (
    build_study,
    evaluate,
    extract_windows,
    features,
    fit,
    group_geometric_segments,
    validate_frozen,
    validate_policy,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def policy():
    return json.loads((ROOT / "configs/diagnostics/right_multijoint_torque_study.json").read_text())


@pytest.fixture
def motion():
    return json.loads(
        (ROOT / "configs/diagnostics/right_multijoint_unloaded_single.json").read_text()
    )


def samples(motion):
    times = np.arange(0, sweep_duration(motion), 0.01)
    q = np.array([target_offsets(motion, t) for t in times])
    torque = q * 0.02
    return np.column_stack((times, q, torque))


def independent_windows():
    rng = np.random.default_rng(427)
    result = []
    # Independent measured features, known seven-output affine response.
    for i in range(160):
        q, v = rng.normal(size=8), rng.normal(size=7)
        h = rng.choice([-1, 1], size=7)
        torque = 0.3 * q[1] + 0.2 * v[3] - 0.1 * h[5] + np.arange(7)
        result.append(
            {
                "q_deg": q.tolist(),
                "velocity_deg_s": v.tolist(),
                "history": h.tolist(),
                "torque_nm": torque.tolist(),
                "group": i // 20,
                "phase": "transition",
            }
        )
    return result


def test_window_history_is_causal_and_torque_never_a_feature(policy, motion):
    original = samples(motion)
    before = extract_windows(original, motion, policy)
    changed = original.copy()
    changed[changed[:, 0] >= 100, 1:9] *= -1
    changed[:, 9:] += 100
    after = extract_windows(changed, motion, policy)
    early_a = [w for w in before["windows"] if w["end_seconds"] <= 100]
    early_b = [w for w in after["windows"] if w["end_seconds"] <= 100]
    np.testing.assert_array_equal(
        features(early_a, "pose_velocity_history"), features(early_b, "pose_velocity_history")
    )
    assert len(before["windows"]) == 296
    assert not before["rejected_windows"]
    assert sum(w["phase"] == "hold" for w in before["windows"]) == 16
    assert before["windows"][0]["history"] == [0] * 7


@pytest.mark.parametrize("bad", ["nonmonotonic", "gap", "nan", "shape"])
def test_malformed_samples_fail_closed(policy, motion, bad):
    x = samples(motion)
    if bad == "nonmonotonic":
        x[1, 0] = x[0, 0]
    elif bad == "gap":
        x = np.delete(x, np.arange(20, 40), axis=0)
    elif bad == "nan":
        x[3, 5] = np.nan
    else:
        x = x[:, :-1]
    with pytest.raises(ValueError, match="finite ordered"):
        extract_windows(x, motion, policy)


def test_known_model_generalizes_without_training_on_heldout_targets(policy):
    windows = independent_windows()
    frozen = fit(windows[:120], policy)
    original = copy.deepcopy(frozen)
    validation = windows[120:]
    result = evaluate(frozen, validation)
    good = result["models"]["pose_velocity_history"]["all_windows_including_extrapolation"]
    assert max(good["rmse_nm"]) < 0.02
    poor = result["models"]["pose"]["all_windows_including_extrapolation"]
    assert min(poor["rmse_nm"]) > 0.15
    changed = copy.deepcopy(validation)
    for w in changed:
        w["torque_nm"] = [100] * 7
    new = evaluate(frozen, changed)
    assert (
        new["models"]["pose_velocity_history"]["prediction_nm"]
        == result["models"]["pose_velocity_history"]["prediction_nm"]
    )
    assert frozen == original


def test_grouped_validation_does_not_randomly_split_neighbor_windows(policy):
    windows = independent_windows()
    study = build_study(
        {"windows": windows, "rejected_windows": []}, {"external_load_present": False}, policy
    )
    assert len(study["grouped_folds"]) == 8
    for fold in study["grouped_folds"]:
        group = fold["held_out_group"]
        assert {w["group"] for w in fold["windows"]} == {group}
        train = [w for w in windows if w["group"] != group]
        expected = evaluate(fit(train, policy), fold["windows"])
        assert expected == fold["evaluation"]
    assert study["ft_validated"] is False
    assert study["runtime_eligible"] is False
    assert study["independent_validation"] is False


def test_outside_pose_support_is_not_counted(policy):
    windows = independent_windows()
    frozen = fit(windows, policy)
    far = copy.deepcopy(windows[:1])
    far[0]["q_deg"][2] += 100
    result = evaluate(frozen, far)
    assert result["pose_supported"] == [False]
    assert result["models"]["pose"]["pose_supported_windows"] is None


def test_geometric_partition_keeps_both_directions_and_cycles_together(motion):
    motion["waypoints_deg"] += list(reversed(motion["waypoints_deg"]))[1:]
    motion["cycles"] = 2
    windows = [{"group": i, "q_deg": [i] * 8} for i in range(-1, 28)]
    original = copy.deepcopy(windows)
    grouped = group_geometric_segments({"windows": windows, "rejected_windows": []}, motion)
    assert windows == original
    mapping = grouped["temporal_to_geometric_group"]
    assert mapping[-1] == -1
    for i in range(7):
        assert mapping[i] == mapping[13 - i] == mapping[i + 14] == mapping[27 - i]
    assert len(set(mapping.values())) == 8
    assert [w["q_deg"] for w in grouped["windows"]] == [w["q_deg"] for w in windows]
    with pytest.raises(ValueError, match="outside scheduled"):
        group_geometric_segments({"windows": [{"group": 28}], "rejected_windows": []}, motion)


def test_frozen_validation_rejects_same_recording_loaded_and_drift(policy):
    source = {
        k: k
        for k in (
            "raw_sha256",
            "samples_sha256",
            "config_sha256",
            "native_sha256",
            "sdk_revision",
            "tool_context",
        )
    }
    source["external_load_present"] = False
    frozen = {"kind": "offline_multijoint_torque_study_v1", "policy": policy, "provenance": source}
    with pytest.raises(ValueError, match="different recording"):
        validate_frozen(frozen, source, policy)
    independent = {**source, "raw_sha256": "other-raw", "samples_sha256": "other-samples"}
    validate_frozen(frozen, independent, policy)
    for key, value in (("native_sha256", "drift"), ("external_load_present", True)):
        with pytest.raises(ValueError, match=r"context mismatch|explicitly unloaded"):
            validate_frozen(frozen, {**independent, key: value}, policy)


@pytest.mark.parametrize("value", [True, 0, float("nan"), -1])
def test_invalid_policy_rejected(policy, value):
    policy["ridge_penalty"] = value
    with pytest.raises(ValueError, match="positive finite"):
        validate_policy(policy)
