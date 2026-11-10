"""Offline task identity, historical pose import, and containment regressions."""

import hashlib
import json
from pathlib import Path

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.start_pose import import_task_pose
from vlai_l1_runtime.task_definition import COORDINATE_KIND, FEATURES, load_task

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def task_files(tmp_path):
    config_path = tmp_path / "configs/system/vlai_l1.toml"
    config_path.parent.mkdir(parents=True)
    config_path.write_bytes((ROOT / "configs/system/vlai_l1.toml").read_bytes())
    config = load_system_config(config_path)
    directory = tmp_path / "configs/tasks"
    directory.mkdir()
    for name in ("inference.json", "automatic.json"):
        (directory / name).write_text("{}")
    manifest = {
        "kind": "vlai_task_v1",
        "id": "push-distance",
        "prompt": "Push the block",
        "distance_cm": 10,
        "start_pose": "data/start_poses/push-distance.json",
        "expected_checkpoint": {
            "kind": "pi0",
            "path": "/remote/checkpoint",
            "deployment_sha256": "a" * 64,
        },
        "inference_spec": "configs/tasks/inference.json",
        "automatic_policy": "configs/tasks/automatic.json",
        "settings": {
            "observation_max_age_s": 1,
            "producer_timeout_s": 0.25,
        },
    }
    path = directory / "push-distance.json"
    path.write_text(json.dumps(manifest))
    original = {
        "schema_version": 1,
        "kind": "right_teleop_start_pose_candidate",
        "status": "recorded_reference_only",
        "automatic_motion_enabled": False,
        "coordinate_kind": COORDINATE_KIND,
        "position_unit": "degree",
        "feature_names": list(FEATURES),
        "captured_utc": "2026-11-09T15:09:27.952193+00:00",
        "source": "existing_x_air_right_state_ipc",
        "system_config": "/original/config.toml",
        "stationary_spec_kind": "offline_only_control_handoff_preview_v1",
        "follower_deg": dict.fromkeys(FEATURES, 0.0),
        "receiver_sequence": 20,
        "source_monotonic_ns": 123_000_000_000,
        "stationary_window_ns": 200_000_610,
        "stationary_samples": 21,
        "max_span_deg": 0.021857,
        "max_drift_deg_s": 0.03406,
    }
    source = tmp_path / "historical.json"
    source.write_text(json.dumps(original))
    return config, path, manifest, source, original


def test_missing_start_pose_is_explicit_and_import_preserves_history(task_files):
    config, path, _, source, original = task_files
    assert load_task(config, path, require_start_pose=False).pose is None
    with pytest.raises(ValueError, match="not recorded"):
        load_task(config, path)
    record = import_task_pose(config, "push-distance", source)
    task = load_task(config, path)
    assert task.pose == record
    assert task.pose["follower_deg"] == original["follower_deg"]
    assert task.pose["captured_utc"] == original["captured_utc"]
    assert record["provenance"]["mode"] == "historical_import"
    assert record["provenance"]["original_config_sha256"] is None
    assert record["provenance"]["source_sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()
    before = task.start_pose.read_bytes()
    with pytest.raises(FileExistsError, match="already exists"):
        import_task_pose(config, "push-distance", source)
    assert task.start_pose.read_bytes() == before


@pytest.mark.parametrize("fault", ["unit", "order", "frame", "snapshot", "moving", "bounds"])
def test_import_rejects_nonstationary_or_incompatible_historical_record(task_files, fault):
    config, _, _, source, original = task_files
    if fault == "unit":
        original["position_unit"] = "radian"
    elif fault == "order":
        original["follower_deg"] = dict(reversed(list(original["follower_deg"].items())))
    elif fault == "frame":
        original["coordinate_kind"] = "calibrated"
    elif fault == "snapshot":
        original["stationary_samples"] = 1
    elif fault == "moving":
        original["max_drift_deg_s"] = 0.2
    elif fault == "bounds":
        original["follower_deg"][FEATURES[4]] = 96
    source.write_text(json.dumps(original))
    with pytest.raises(ValueError, match=r"pose|stationary|bound|frame|order"):
        import_task_pose(config, "push-distance", source)


@pytest.mark.parametrize("fault", ["task", "unit", "order", "frame", "config"])
def test_loader_rejects_pose_identity_corruption_even_in_validate_mode(task_files, fault):
    config, path, _, source, _ = task_files
    record = import_task_pose(config, "push-distance", source)
    task = load_task(config, path)
    if fault == "task":
        record["task_id"] = "other"
    elif fault == "unit":
        record["position_unit"] = "radian"
    elif fault == "order":
        record["feature_names"].reverse()
    elif fault == "frame":
        record["coordinate_kind"] = "world"
    elif fault == "config":
        record["validation_config_sha256"] = "0" * 64
    task.start_pose.write_text(json.dumps(record))
    with pytest.raises(ValueError, match="pose"):
        load_task(config, path, require_start_pose=False)


@pytest.mark.parametrize(
    "fault", ["absolute", "traversal", "symlink", "canonical", "sha", "boolean", "setting"]
)
def test_manifest_path_checkpoint_and_setting_validation(task_files, fault):
    config, path, manifest, _, _ = task_files
    if fault == "absolute":
        manifest["inference_spec"] = "/tmp/inference.json"
    elif fault == "traversal":
        manifest["automatic_policy"] = "../outside.json"
    elif fault == "symlink":
        target = path.parent / "inference.json"
        target.unlink()
        target.symlink_to(path)
    elif fault == "canonical":
        manifest["start_pose"] = "data/start_poses/other.json"
    elif fault == "sha":
        manifest["expected_checkpoint"]["deployment_sha256"] = "bad"
    elif fault == "boolean":
        manifest["settings"]["observation_max_age_s"] = True
    elif fault == "setting":
        manifest["settings"]["producer_timeout_s"] = float("nan")
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match=r"path|symlink|sha256|setting"):
        load_task(config, path, require_start_pose=False)


def test_optional_arrival_tolerance_is_loaded(task_files):
    config, path, manifest, _, _ = task_files
    manifest["settings"]["arrival_tolerance_deg"] = 2.0
    path.write_text(json.dumps(manifest))
    assert load_task(config, path, require_start_pose=False).settings["arrival_tolerance_deg"] == 2


@pytest.mark.parametrize("value", [0, -1, True, float("nan"), float("inf")])
def test_invalid_arrival_tolerance_is_rejected(task_files, value):
    config, path, manifest, _, _ = task_files
    manifest["settings"]["arrival_tolerance_deg"] = value
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="arrival_tolerance_deg"):
        load_task(config, path, require_start_pose=False)
