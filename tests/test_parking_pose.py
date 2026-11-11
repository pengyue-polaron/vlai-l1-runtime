"""Parking targets must retain their physical identity and stationary evidence."""

import json
from pathlib import Path

import pytest

from vlai_l1_runtime import parking_pose
from vlai_l1_runtime.configuration import load_system_config

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")


def test_saved_parking_pose_is_valid():
    pose = parking_pose.load_parking_pose(CONFIG)
    assert pose["source"] == "idle_right_follower_readonly_feedback"


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("side", "left"),
        ("role", "leader"),
        ("position_unit", "radian"),
        ("validation_config_sha256", "wrong"),
        ("max_span_deg", 2),
        ("follower_deg", {"right_joint_1.pos": 0}),
    ],
)
def test_parking_rejects_incompatible_or_unsettled_pose(tmp_path, monkeypatch, key, value):
    pose = parking_pose.load_parking_pose(CONFIG)
    pose[key] = value
    path = tmp_path / "parking.json"
    path.write_text(json.dumps(pose))
    monkeypatch.setattr(parking_pose, "parking_path", lambda _: path)
    with pytest.raises(ValueError, match=r"mismatch|stationary limits|eight-joint order"):
        parking_pose.load_parking_pose(CONFIG)
