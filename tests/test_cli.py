from __future__ import annotations

import json
from pathlib import Path

from vlai_l1_runtime.cli import main

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
