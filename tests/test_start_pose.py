from __future__ import annotations

from pathlib import Path

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.contracts import NamedJointVector, SampleMetadata
from vlai_l1_runtime.start_pose import (
    CaptureSpec,
    PoseCaptureError,
    StationaryPoseWindow,
    capture_pose,
)
from vlai_l1_runtime.task_definition import FEATURES

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")
SPEC = CaptureSpec()
NS = 1_000_000_000


def sample(sequence, stamp, *, follower=None, leader=None):
    base = dict.fromkeys(FEATURES, 0.0)
    follower_values = {**base, **(follower or {})}
    leader_values = {**base, **(leader or {})}
    metadata = SampleMetadata(sequence, stamp)
    return (
        NamedJointVector(follower_values, metadata, FEATURES),
        NamedJointVector(leader_values, metadata, FEATURES),
    )


def test_stationary_right_pair_yields_named_pose_only_after_window():
    window = StationaryPoseWindow(CONFIG, SPEC)
    for index in range(20):
        stamp = NS + index * 10_000_000
        assert window.accept(*sample(index, stamp), now_ns=stamp) is None
    result = window.accept(*sample(20, NS + SPEC.settle_ns), now_ns=NS + SPEC.settle_ns)
    assert result["stationary_samples"] == 21
    assert result["stationary_window_ns"] == SPEC.settle_ns
    assert result["follower_deg"] == dict.fromkeys(FEATURES, 0.0)
    assert result["leader_deg"] == dict.fromkeys(FEATURES, 0.0)


def test_follower_motion_restarts_stationary_window():
    window = StationaryPoseWindow(CONFIG, SPEC)
    assert window.accept(*sample(0, NS), now_ns=NS) is None
    halfway = NS + SPEC.settle_ns // 2
    assert window.accept(*sample(1, halfway, follower={FEATURES[6]: 1.0}), now_ns=halfway) is None
    at_old_deadline = NS + SPEC.settle_ns
    assert (
        window.accept(
            *sample(2, at_old_deadline, follower={FEATURES[6]: 1.0}),
            now_ns=at_old_deadline,
        )
        is None
    )
    at_new_deadline = halfway + SPEC.settle_ns
    result = window.accept(
        *sample(3, at_new_deadline, follower={FEATURES[6]: 1.0}), now_ns=at_new_deadline
    )
    assert result["follower_deg"][FEATURES[6]] == 1.0


def test_stationary_follower_is_recordable_while_leader_moves_or_is_absent():
    for with_leader in (False, True):
        window = StationaryPoseWindow(CONFIG, SPEC)
        for i in range(21):
            stamp = NS + i * 10_000_000
            follower, leader = sample(i, stamp, leader={FEATURES[6]: float(i)})
            result = window.accept(follower, leader if with_leader else None, now_ns=stamp)
        assert result["max_span_deg"] == result["max_drift_deg_s"] == 0
        assert result["leader_deg"] == (dict(leader.values) if with_leader else None)


def test_stale_bounds_and_nonmonotonic_state_fail_closed():
    stale = sample(0, NS)
    with pytest.raises(PoseCaptureError, match="stale"):
        StationaryPoseWindow(CONFIG, SPEC).accept(
            *stale, now_ns=NS + round(CONFIG.teleoperation.state_timeout_s * 1e9) + 1
        )
    out_of_bounds = sample(0, NS, follower={FEATURES[4]: 96.0})
    with pytest.raises(PoseCaptureError, match="tracked bound"):
        StationaryPoseWindow(CONFIG, SPEC).accept(*out_of_bounds, now_ns=NS)
    window = StationaryPoseWindow(CONFIG, SPEC)
    assert window.accept(*sample(1, NS), now_ns=NS) is None
    with pytest.raises(PoseCaptureError, match="sequence or timestamp"):
        window.accept(*sample(1, NS + 10_000_000), now_ns=NS + 10_000_000)


def test_leader_and_follower_must_be_same_sample():
    follower, _ = sample(1, NS)
    _, leader = sample(2, NS)
    with pytest.raises(PoseCaptureError, match="share one"):
        StationaryPoseWindow(CONFIG, SPEC).accept(follower, leader, now_ns=NS)


def test_existing_pose_refuses_before_opening_state_receiver(monkeypatch, tmp_path):
    from vlai_l1_runtime.teleoperation import xair

    config_path = tmp_path / "configs/system/vlai_l1.toml"
    config_path.parent.mkdir(parents=True)
    config_path.write_bytes(CONFIG.path.read_bytes())
    config = load_system_config(config_path)
    output = tmp_path / "data/start_poses/push-distance.json"
    output.parent.mkdir(parents=True)
    output.write_text("existing", encoding="utf-8")

    def forbidden(_self):
        pytest.fail("existing output opened the state receiver")

    monkeypatch.setattr(xair.XAirStateReceiver, "__enter__", forbidden)
    with pytest.raises(FileExistsError, match="already exists"):
        capture_pose(config, SPEC, task_id="push-distance", output=output)
    assert output.read_text(encoding="utf-8") == "existing"


def test_full_window_accepts_stationary_encoder_word_jitter_without_looser_limits():
    window = StationaryPoseWindow(CONFIG, SPEC)
    for i in range(21):
        stamp = NS + i * 10_000_000
        q = 0.02185694 if i % 2 else 0.0
        result = window.accept(*sample(i, stamp, follower={FEATURES[6]: q}), now_ns=stamp)
        if i < 20:
            assert result is None
    assert result["stationary_window_ns"] == SPEC.settle_ns
    assert result["max_span_deg"] < SPEC.settle_span_deg
    assert result["max_drift_deg_s"] < SPEC.settle_speed_deg_s


def test_full_window_still_rejects_real_drift_within_span_limit():
    window = StationaryPoseWindow(CONFIG, SPEC)
    for i in range(50):
        stamp = NS + i * 10_000_000
        q = i * 0.002  # 0.2deg/s, though every200ms span is only0.04deg.
        assert window.accept(*sample(i, stamp, follower={FEATURES[6]: q}), now_ns=stamp) is None
    assert window.last_diagnostics["max_drift_deg_s"] > SPEC.settle_speed_deg_s


def test_missing_feedback_does_not_count_as_a_stable_window():
    window = StationaryPoseWindow(CONFIG, SPEC)
    window.accept(*sample(1, NS), now_ns=NS)
    with pytest.raises(PoseCaptureError, match="gap"):
        window.accept(*sample(2, NS + SPEC.settle_ns), now_ns=NS + SPEC.settle_ns)


@pytest.mark.parametrize("replace", [False, True])
def test_capture_persists_canonical_task_pose_with_source_and_config_provenance(
    monkeypatch, tmp_path, replace
):
    from vlai_l1_runtime import start_pose
    from vlai_l1_runtime.task_definition import config_sha256, validate_pose_record
    from vlai_l1_runtime.teleoperation import xair

    config_path = tmp_path / "configs/system/vlai_l1.toml"
    config_path.parent.mkdir(parents=True)
    config_path.write_bytes(CONFIG.path.read_bytes())
    config = load_system_config(config_path)
    output = tmp_path / "data/start_poses/push-distance.json"
    if replace:
        output.parent.mkdir(parents=True)
        output.write_text("old pose")
    clock = [NS]

    class Receiver:
        def __init__(self, config, *, sides):
            self.index = -1
            assert sides == ("right",)

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def receive(self, *, timeout_s):
            self.index += 1
            clock[0] = NS + self.index * 10_000_000
            return sample(self.index, clock[0], leader={FEATURES[6]: float(self.index)})

    monkeypatch.setattr(xair, "XAirStateReceiver", Receiver)
    monkeypatch.setattr(start_pose.time, "monotonic_ns", lambda: clock[0])
    record = capture_pose(config, SPEC, task_id="push-distance", replace=replace)
    assert (tmp_path / "data/start_poses/push-distance.json").is_file()
    assert record["provenance"]["mode"] == "captured"
    assert record["validation_config_sha256"] == config_sha256(config)
    assert record["stationary_samples"] == 21
    assert record["leader_deg"][FEATURES[6]] == 20
    validate_pose_record(config, "push-distance", record)
    assert "old pose" not in output.read_text()
    assert not list(output.parent.glob(".pose-*"))


def test_failed_recapture_keeps_existing_pose(monkeypatch, tmp_path):
    from vlai_l1_runtime.teleoperation import xair

    config_path = tmp_path / "configs/system/vlai_l1.toml"
    config_path.parent.mkdir(parents=True)
    config_path.write_bytes(CONFIG.path.read_bytes())
    config = load_system_config(config_path)
    output = tmp_path / "data/start_poses/push-distance.json"
    output.parent.mkdir(parents=True)
    output.write_text("old pose")

    class Receiver:
        def __init__(self, *_args, **_kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def receive(self, **_kwargs):
            return None

    monkeypatch.setattr(xair, "XAirStateReceiver", Receiver)
    with pytest.raises(PoseCaptureError, match="no fresh"):
        capture_pose(config, SPEC, task_id="push-distance", replace=True)
    assert output.read_text() == "old pose"


def test_failed_atomic_replacement_preserves_old_pose(monkeypatch, tmp_path):
    import json

    from vlai_l1_runtime import start_pose

    output = tmp_path / "pose.json"
    output.write_text("old pose")
    record = json.loads((ROOT / "data/start_poses/push-distance.json").read_text())
    monkeypatch.setattr(start_pose, "_output_path", lambda *args, **kwargs: output)

    def fail_replace(self, target):
        raise OSError("injected replacement failure")

    monkeypatch.setattr(Path, "replace", fail_replace)
    with pytest.raises(OSError, match="injected replacement failure"):
        start_pose._write_record(CONFIG, "push-distance", record, output, replace=True)
    assert output.read_text() == "old pose"
    assert not list(tmp_path.glob(".pose-*"))
