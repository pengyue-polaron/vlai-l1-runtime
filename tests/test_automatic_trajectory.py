from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np
import pytest

from vlai_l1_runtime.automatic_trajectory import path_offsets, sweep_duration, target_offsets
from vlai_l1_runtime.configuration import MOTOR_NAMES, load_system_config
from vlai_l1_runtime.teleoperation.automatic import (
    render_native_spec,
    run_automatic,
    validate_policy,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")
PATH = ROOT / "configs/diagnostics/right_multijoint_unloaded_single.json"


def policy():
    return json.loads(PATH.read_text())


def test_reverse_validation_preserves_every_guard_and_reverses_same_path():
    forward = policy()
    reverse = json.loads(
        (ROOT / "configs/diagnostics/right_multijoint_unloaded_reverse.json").read_text()
    )
    validate_policy(CONFIG, reverse)
    assert reverse["waypoints_deg"] == list(reversed(forward["waypoints_deg"]))
    for key in set(forward) - {"waypoints_deg", "description"}:
        assert reverse[key] == forward[key]
    assert sweep_duration(reverse) == sweep_duration(forward) == 320
    assert (
        render_native_spec(CONFIG, reverse).splitlines()[:13]
        == render_native_spec(CONFIG, forward).splitlines()[:13]
    )
    # Exclude holds: each reversed transition traverses the same geometric line.
    for leg in range(7):
        for local in (0, 10, 20, 30, 40):
            a = target_offsets(forward, 5 + leg * 45 + local)
            b = target_offsets(reverse, 5 + (6 - leg) * 45 + 40 - local)
            assert a == pytest.approx(b)


def test_tracked_multi_path_limits_and_return():
    p = policy()
    validate_policy(CONFIG, p)
    assert sweep_duration(p) == 320
    assert target_offsets(p, 25) == (0, 5, -5, 8, -5, 5, 5, 0)
    assert target_offsets(p, 320) == (0,) * 8
    assert target_offsets(p, 25, hold_only=True) == (0,) * 8
    q = np.array([target_offsets(p, float(t)) for t in np.arange(0, 320.01, 0.01)])
    assert np.ptp(q, axis=0) == pytest.approx([0, 10, 20, 16, 20, 20, 20, 0])
    assert np.max(np.abs(np.diff(q, axis=0))) / 0.01 <= 1
    assert np.max(np.abs(np.diff(q, n=2, axis=0))) / 0.01**2 <= 0.8
    rows = render_native_spec(CONFIG, p).splitlines()
    assert rows[0] == "VLAI_RIGHT_FOLLOWER_DIAGNOSTIC_V4"
    assert rows[2].split()[-1] == "8"
    assert rows[11:13] == ["0", "8"]
    assert [float(row.split()[13]) for row in rows[3:11]] == [45] * 7 + [15]
    assert [float(row.split()[12]) for row in rows[3:11]] == [2] * 8
    assert [list(map(float, r.split())) for r in rows[13:]] == [list(r) for r in path_offsets(p)]
    # JSON key order cannot silently change physical motor assignment.
    reordered = copy.deepcopy(p)
    reordered["waypoints_deg"] = [dict(reversed(list(v.items()))) for v in p["waypoints_deg"]]
    assert render_native_spec(CONFIG, reordered) == render_native_spec(CONFIG, p)
    old = json.loads(
        (ROOT / "configs/diagnostics/right_joint4_threequarter_single.json").read_text()
    )
    allowed = {
        "kind",
        "active_motor",
        "amplitude_deg",
        "maximum_amplitude_deg",
        "transition_seconds",
        "maximum_relative_excursion_deg",
        "maximum_observed_speed_deg_s",
        "maximum_tracking_error_deg",
        "description",
        "waypoints_deg",
    }
    assert all(p[k] == old[k] for k in set(p) - allowed)


def test_balanced_training_has_one_reference_and_preserves_both_paths():
    forward = policy()
    balanced = json.loads(
        (ROOT / "configs/diagnostics/right_multijoint_unloaded_balanced.json").read_text()
    )
    reverse = json.loads(
        (ROOT / "configs/diagnostics/right_multijoint_unloaded_reverse.json").read_text()
    )
    validate_policy(CONFIG, balanced)
    assert sweep_duration(balanced) == 635
    assert len(balanced["waypoints_deg"]) == 15
    for key in set(forward) - {"waypoints_deg", "description"}:
        assert balanced[key] == forward[key]
    # One initial hold, a retained middle hold, and no second initial hold.
    for t in np.arange(0, 320.01, 0.25):
        assert target_offsets(balanced, float(t)) == pytest.approx(
            target_offsets(forward, float(t))
        )
        assert target_offsets(balanced, float(t + 315)) == pytest.approx(
            target_offsets(reverse, float(t))
        )
    rows = render_native_spec(CONFIG, balanced).splitlines()
    assert rows[12] == "15"
    assert rows[3:12] == render_native_spec(CONFIG, forward).splitlines()[3:12]
    # Includes the launcher's ten-second retention reserve, without raising capacity.
    assert (635 + 10) * CONFIG.teleoperation.publish_hz * 8 <= balanced["raw_log_capacity"]


def test_independent_inner_path_scales_offsets_only():
    original = policy()
    validation = json.loads(
        (ROOT / "configs/diagnostics/right_multijoint_unloaded_validation75.json").read_text()
    )
    validate_policy(CONFIG, validation)
    for key in set(original) - {"waypoints_deg", "description"}:
        assert validation[key] == original[key]
    for t in np.arange(0, 320.01, 0.25):
        assert target_offsets(validation, float(t)) == pytest.approx(
            np.array(target_offsets(original, float(t))) * 0.75
        )
    assert (
        render_native_spec(CONFIG, original).splitlines()[:13]
        == render_native_spec(CONFIG, validation).splitlines()[:13]
    )


@pytest.mark.parametrize("fault", range(15))
def test_invalid_multi_path_rejected_before_hardware(fault):
    p = policy()
    if fault == 0:
        p["waypoints_deg"][1]["gripper"] = 1
    elif fault == 1:
        p["waypoints_deg"][0]["joint_2"] = 1
    elif fault == 2:
        p["waypoints_deg"][-1]["joint_2"] = 1
    elif fault == 3:
        p["waypoints_deg"][1]["joint_2"] = float("nan")
    elif fault == 4:
        p["transition_seconds"] = 32  # amplitude fits, 20-degree inter-point delta fails
    elif fault == 5:
        p["maximum_relative_excursion_deg"][1] = 5
    elif fault == 6:
        p["raw_log_capacity"] = 10000
    elif fault == 7:
        p["active_motor"] = "joint_4"
    elif fault == 8:
        p["waypoints_deg"] = p["waypoints_deg"][:2]
    elif fault == 9:
        p["manual_handling"] = {}
    elif fault == 10:
        for point in p["waypoints_deg"]:
            for name in MOTOR_NAMES[2:]:
                point[name] = 0
    elif fault == 11:
        p["waypoints_deg"] = p["waypoints_deg"] * 9
    elif fault == 12:
        del p["waypoints_deg"][1]["joint_2"]
    elif fault == 13:
        p["waypoints_deg"][1]["joint_2"] = True
    elif fault == 14:
        p["waypoints_deg"][1]["j2"] = 1
    with pytest.raises(ValueError, match=r"waypoint|automatic|capacity|only"):
        validate_policy(CONFIG, p)


def test_multi_paired_rejected_before_output_or_hardware(tmp_path):
    with pytest.raises(ValueError, match="excludes paired"):
        run_automatic(CONFIG, PATH, tmp_path / "out", mode="paired")
    assert not (tmp_path / "out").exists()


def test_multi_cycles_have_identical_commands_but_separate_time():
    p = policy()
    p["cycles"] = 2
    validate_policy(CONFIG, p)
    assert sweep_duration(p) == 635
    assert target_offsets(p, 25) == target_offsets(p, 340)
    for t in (float("nan"), float("inf"), -1):
        with pytest.raises(ValueError, match="trajectory time"):
            target_offsets(p, t)
