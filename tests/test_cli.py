from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from vlai_l1_runtime.cli import build_parser, main
from vlai_l1_runtime.contracts import FEATURE_NAMES, NamedJointVector, SampleMetadata

ROOT = Path(__file__).resolve().parents[1]
SYSTEM_CONFIG = ROOT / "configs/system/vlai_l1.toml"
COLLECTION_CONFIG = ROOT / "configs/collection/default.toml"
RIGHT_COLLECTION_CONFIG = ROOT / "configs/collection/right_only.toml"


def test_hardware_free_cli_validates_and_describes(capsys) -> None:
    assert main(["validate-config", "--config", str(SYSTEM_CONFIG)]) == 0
    assert capsys.readouterr().out.startswith("PASS ")

    assert main(["describe", "--config", str(SYSTEM_CONFIG)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["teleoperation_ready"] is True
    assert payload["command_ready"] is False
    assert payload["collection_ready"] is True
    assert len(payload["action_features"]) == 16

    assert (
        main(
            [
                "describe-xair",
                "--config",
                str(SYSTEM_CONFIG),
                "--side",
                "left",
            ]
        )
        == 0
    )
    xair = json.loads(capsys.readouterr().out)
    assert (xair["leader_can"], xair["follower_can"]) == ("can1", "can3")


def test_hardware_free_cli_validates_and_describes_collection(capsys) -> None:
    assert main(["validate-collection", "--config", str(COLLECTION_CONFIG)]) == 0
    assert capsys.readouterr().out.startswith("PASS ")

    assert main(["describe-collection", "--config", str(COLLECTION_CONFIG)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["dataset_schema"] == "vlai_l1_lerobot_dataset_v3_v3"
    assert payload["teleoperation_sides"] == ["left", "right"]
    assert payload["record_camera_roles"] == ["wrist_left", "wrist_right", "agent"]
    assert payload["collection_ready"] is True
    assert payload["reset"] == {
        "after_discard": True,
        "after_save": True,
        "before_collection": True,
    }
    assert payload["leading_stillness"]["enabled"] is True
    assert set(payload["features"]) == {
        "action",
        "observation.images.agent",
        "observation.images.wrist_left",
        "observation.images.wrist_right",
        "observation.state",
    }

    assert main(["describe-collection", "--config", str(RIGHT_COLLECTION_CONFIG)]) == 0
    right = json.loads(capsys.readouterr().out)
    assert right["teleoperation_sides"] == ["right"]
    assert right["record_camera_roles"] == ["wrist_right", "agent"]
    assert right["features"]["action"]["shape"] == [8]
    assert "observation.images.wrist_left" not in right["features"]


def test_trim_leading_stillness_cli_keeps_the_just_facing_dataset_shape() -> None:
    args = build_parser().parse_args(
        [
            "dataset",
            "trim-leading-stillness",
            "--side",
            "right",
            "block_plate",
            "block_plate_trimmed_v1",
            "--dry-run",
        ]
    )

    assert args.dataset_command == "trim-leading-stillness"
    assert args.source_experiment == "block_plate"
    assert args.target_experiment == "block_plate_trimmed_v1"
    assert args.side == "right"
    assert args.dry_run is True


def test_top_level_help_exposes_only_operator_workflows() -> None:
    help_text = build_parser().format_help()

    assert "{panel,hardware,dataset,collect,reset}" in help_text
    assert "camera-service-run" not in help_text
    assert "dataset-doctor" not in help_text
    assert "tracked VLAI L1 operator workflows" in help_text


def test_dataset_commands_accept_the_standard_json_mode() -> None:
    doctor = build_parser().parse_args(["dataset", "doctor", "blocks_v1", "--json"])
    export = build_parser().parse_args(["dataset", "export-v21", "blocks_v1", "--json"])

    assert doctor.json is True
    assert export.json is True


def test_dataset_doctor_has_shared_human_and_json_output(monkeypatch, capsys) -> None:
    identity = SimpleNamespace(
        target_root=Path("/data/blocks_v1"),
        repo_id="example/vlai-blocks-v1",
    )
    state = SimpleNamespace(total_episodes=3, total_frames=90, task="place the block")
    monkeypatch.setattr(
        "vlai_l1_runtime.cli._load_selected_collection_config", lambda _args: object()
    )
    monkeypatch.setattr("vlai_l1_runtime.cli.identity_from_config", lambda *_args: identity)
    monkeypatch.setattr("vlai_l1_runtime.cli.provenance_from_config", lambda _config: {})
    monkeypatch.setattr(
        "vlai_l1_runtime.cli.inspect_direct_dataset", lambda *_args, **_kwargs: state
    )

    assert main(["dataset", "doctor", "blocks_v1"]) == 0
    assert "[PASS] Dataset doctor passed" in capsys.readouterr().out

    assert main(["dataset", "doctor", "blocks_v1", "--json"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report == {
        "episodes": 3,
        "experiment": "blocks_v1",
        "frames": 90,
        "repo_id": "example/vlai-blocks-v1",
        "robot": "vlai-l1",
        "root": "/data/blocks_v1",
        "status": "PASS",
        "tasks": ["place the block"],
        "workflow": "dataset-doctor",
    }


def test_reset_cli_uses_the_managed_teleoperation_lifecycle(monkeypatch) -> None:
    called = []
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.reset_managed_teleoperation",
        lambda config: called.append(config.path),
    )

    assert main(["reset", "--config", str(COLLECTION_CONFIG)]) == 0
    assert called == [COLLECTION_CONFIG.resolve()]


def test_hardware_cli_selects_the_right_only_contract(monkeypatch) -> None:
    selected = []
    monkeypatch.setattr(
        "vlai_l1_runtime.hardware_check.inspect_hardware",
        lambda config: selected.append(config.teleoperation_sides) or (),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.hardware_check.print_hardware_report",
        lambda checks, *, json_output: 0,
    )

    assert main(["hardware", "--side", "right", "--json"]) == 0
    assert selected == [("right",)]


def test_right_only_cli_selects_the_existing_side_with_isolation(monkeypatch) -> None:
    called = []

    def run(config, side, *, managed_startup_gate, isolated_side):
        called.append((config.path, side, managed_startup_gate, isolated_side))
        return 0

    monkeypatch.setattr("vlai_l1_runtime.cli.run_xair_side", run)

    assert (
        main(
            [
                "run-xair",
                "--config",
                str(SYSTEM_CONFIG),
                "--side",
                "right",
                "--isolated-side",
            ]
        )
        == 0
    )
    assert called == [(SYSTEM_CONFIG.resolve(), "right", False, True)]


def test_xair_observer_reports_paired_bimanual_state(monkeypatch, capsys) -> None:
    values = {name: float(index) for index, name in enumerate(FEATURE_NAMES)}
    sample = NamedJointVector(values, SampleMetadata(7, 123_000))

    class Receiver:
        def __init__(self, config) -> None:
            self.config = config

        def __enter__(self):
            return self

        def receive(self, *, timeout_s: float):
            assert timeout_s == 0.5
            return sample, sample

        def __exit__(self, exc_type, exc, traceback) -> None:
            return None

    monkeypatch.setattr("vlai_l1_runtime.cli.XAirStateReceiver", Receiver)

    assert (
        main(
            [
                "observe-xair",
                "--config",
                str(SYSTEM_CONFIG),
                "--side",
                "bimanual",
                "--samples",
                "1",
                "--timeout",
                "0.5",
            ]
        )
        == 0
    )
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "PASS"
    assert payload["mode"] == "bimanual"
    assert payload["sample_count"] == 1
    assert payload["first_source_sequence"] == 7
    assert payload["last_source_sequence"] == 7
    assert payload["first_monotonic_ns"] == 123_000
    assert payload["last_monotonic_ns"] == 123_000
    assert payload["observation_deg"] == values
    assert payload["action_deg"] == values
