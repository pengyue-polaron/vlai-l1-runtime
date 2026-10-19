import copy
import json
from pathlib import Path

import pytest

from vlai_l1_runtime.joint_repeatability import compare_holds

POLICY = json.loads(
    (
        Path(__file__).resolve().parents[1]
        / "configs/diagnostics/right_joint_repeatability_review.json"
    ).read_text()
)


def example():
    holds = []
    for i in range(9):
        leg = (i - 1) % 4
        q = 0 if i == 0 else (0.2, 0.18, -0.02, 0)[leg]
        torque = 0 if i == 0 else (0.2, -0.05, -0.25, 0)[leg] + (0.001 if i > 4 else 0)
        holds.append(
            {
                "kind": "initial" if i == 0 else ("positive", "return", "negative", "return")[leg],
                "samples": 250,
                "mean_q_deg": [0] * 5 + [q, 0, 0],
                "mean_torque_nm": [0] * 5 + [torque, 0, 0],
                "q_span_deg": [0.02] * 8,
            }
        )
    return {"active_motor": "joint_6", "hold_windows": holds}


def test_distinguishes_repeatability_from_near_pose_path_spread():
    result = compare_holds(example(), POLICY)
    assert result["maximum_repeat_torque_delta_nm"] == pytest.approx(0.001)
    assert result["maximum_near_pose_torque_delta_nm"] == pytest.approx(0.251)
    assert result["ft_validated"] is False
    assert len(result["same_scheduled_path_repeat_pairs"]) == 4


def test_other_joint_shift_is_not_a_near_pose_match():
    data = example()
    for i, h in enumerate(data["hold_windows"]):
        h["mean_q_deg"][0] = i
    result = compare_holds(data, POLICY)
    assert result["near_pose_different_path_pairs"] == []
    assert not any(p["within_pose_tolerance"] for p in result["same_scheduled_path_repeat_pairs"])


@pytest.mark.parametrize("bad", ["nan", "unstable", "few", "order", "incomplete"])
def test_invalid_holds_fail_closed(bad):
    data = copy.deepcopy(example())
    h = data["hold_windows"][1]
    if bad == "nan":
        h["mean_torque_nm"][5] = float("nan")
    if bad == "unstable":
        h["q_span_deg"][5] = 1
    if bad == "few":
        h["samples"] = 2
    if bad == "order":
        h["kind"] = "return"
    if bad == "incomplete":
        data["hold_windows"].pop()
    with pytest.raises(ValueError, match=r"finite|stable|sampled|cycles"):
        compare_holds(data, POLICY)
