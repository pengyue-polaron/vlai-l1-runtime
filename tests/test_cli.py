from __future__ import annotations

import json
from pathlib import Path

from vlai_l1_runtime.cli import main
from vlai_l1_runtime.contracts import FEATURE_NAMES, NamedJointVector, SampleMetadata

ROOT = Path(__file__).resolve().parents[1]
SYSTEM_CONFIG = ROOT / "configs/system/vlai_l1.toml"
COLLECTION_CONFIG = ROOT / "configs/collection/default.toml"


def test_hardware_free_cli_validates_and_describes(capsys) -> None:
    assert main(["validate-config", "--config", str(SYSTEM_CONFIG)]) == 0
    assert capsys.readouterr().out.startswith("PASS ")

    assert main(["describe", "--config", str(SYSTEM_CONFIG)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["teleoperation_ready"] is False
    assert payload["command_ready"] is False
    assert payload["collection_ready"] is False
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
    assert payload["dataset_schema"] == "vlai_l1_lerobot_dataset_v3_v2"
    assert payload["collection_ready"] is False
    assert set(payload["features"]) == {
        "action",
        "observation.state",
    }


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
