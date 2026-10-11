from dataclasses import replace
from pathlib import Path

import pytest

from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.managed import ManagedXAirRuntimes
from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.teleoperation.lifecycle import validate_feedback_log_options

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")


def test_passive_log_is_opt_in_and_ids_come_from_config(tmp_path):
    assert validate_feedback_log_options(CONFIG, "right", None, None) == ()
    args = validate_feedback_log_options(CONFIG, "right", tmp_path / "raw.csv", 1000)
    assert args == (
        "--feedback-log",
        str(tmp_path / "raw.csv"),
        "--feedback-max-frames",
        "1000",
        "--feedback-response-ids",
        ",".join(str(m.receive_id) for m in CONFIG.motors),
    )
    assert not (tmp_path / "raw.csv").exists()


@pytest.mark.parametrize("capacity", [None, True, 0, 7, 2_000_001, 8.0])
def test_passive_log_rejects_missing_or_unbounded_capacity(tmp_path, capacity):
    with pytest.raises(ValueError, match="explicit capacity"):
        validate_feedback_log_options(CONFIG, "right", tmp_path / "raw.csv", capacity)


def test_passive_log_never_overwrites_and_requires_profile(tmp_path):
    target = tmp_path / "raw.csv"
    target.write_text("preserve")
    with pytest.raises(FileExistsError):
        validate_feedback_log_options(CONFIG, "right", target, 8)
    link = tmp_path / "link"
    link.symlink_to(tmp_path / "absent")
    with pytest.raises(FileExistsError):
        validate_feedback_log_options(CONFIG, "right", link, 8)
    with pytest.raises(ValueError, match="non-device directory"):
        validate_feedback_log_options(CONFIG, "right", tmp_path / "absent" / "raw.csv", 8)
    with pytest.raises(ValueError, match="profiles"):
        validate_feedback_log_options(
            replace(CONFIG, follower_feedback=None), "right", tmp_path / "new", 8
        )
    assert target.read_text() == "preserve"


def test_latest_retention_requires_explicit_log_and_capacity(tmp_path):
    with pytest.raises(ValueError, match="explicit capacity"):
        validate_feedback_log_options(CONFIG, "right", None, None, "latest")
    with pytest.raises(ValueError, match="retention"):
        validate_feedback_log_options(CONFIG, "right", tmp_path / "raw.csv", 1000, "unknown")
    args = validate_feedback_log_options(CONFIG, "right", tmp_path / "raw.csv", 1000, "latest")
    assert args[-2:] == ("--feedback-retention", "latest")
    manager = ManagedXAirRuntimes(
        load_collection_config(ROOT / "configs/collection/right_only.toml"),
        feedback_log=tmp_path / "raw.csv",
        feedback_max_frames=1000,
        feedback_retention="latest",
    )
    assert manager._feedback_arguments == args[:4] + args[6:]
    assert "--feedback-response-ids" not in manager._feedback_arguments


def test_managed_logging_requires_isolation_and_forwards_only_diagnostic_options(tmp_path):
    config = load_collection_config(ROOT / "configs/collection/right_only.toml")
    manager = ManagedXAirRuntimes(
        config, feedback_log=tmp_path / "raw.csv", feedback_max_frames=1000
    )
    assert manager._feedback_arguments == (
        "--feedback-log",
        str(tmp_path / "raw.csv"),
        "--feedback-max-frames",
        "1000",
    )
    assert manager._runtimes == []
    with pytest.raises(ValueError, match="isolated"):
        ManagedXAirRuntimes(
            load_collection_config(ROOT / "configs/collection/default.toml"),
            feedback_log=tmp_path / "raw.csv",
            feedback_max_frames=1000,
        )
