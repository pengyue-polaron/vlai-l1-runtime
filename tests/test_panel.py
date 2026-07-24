from __future__ import annotations

import shutil
import sys
from pathlib import Path

import pytest
from embodied_ops.operator_panel import InputAction, OperatorPanelApplication

from vlai_l1_runtime.panel import L1OperatorPanelAdapter

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/collection/default.toml"


def test_panel_exposes_the_commissioned_collection_stack() -> None:
    adapter = L1OperatorPanelAdapter(ROOT, CONFIG)
    catalog = adapter.catalog()

    assert catalog["product"] == {"brand": "VLAI L1", "title": "Operations"}
    assert catalog["readiness"]["collection"] is True
    assert {(camera["id"], camera["port"], camera["path"]) for camera in catalog["cameras"]} == {
        ("wrist_left", 8088, "/wrist_left.mjpg"),
        ("wrist_right", 8088, "/wrist_right.mjpg"),
        ("agent", 8088, "/agent.mjpg"),
    }
    assert [registration["id"] for registration in catalog["registrations"]] == ["prompt"]
    assert [
        (control["label"], control["values"]["action"]) for control in catalog["camera_controls"]
    ] == [("Start cameras", "start"), ("Stop cameras", "stop")]
    assert {workflow["id"] for workflow in catalog["workflows"]} == {
        "validate-collection",
        "collect",
        "reset",
        "dataset-doctor",
        "export-v21",
    }
    launch = adapter.build_launch("dataset-doctor", {"experiment": "pick_v1"})
    assert launch.command[:3] == (sys.executable, "-m", "vlai_l1_runtime.cli")
    assert launch.command[-2:] == ("--experiment", "pick_v1")
    camera = adapter.build_launch("camera", {"action": "start"})
    assert camera.command == (
        str(ROOT / "scripts/camera_service.sh"),
        "start",
    )
    reset = adapter.build_launch("reset", {})
    assert reset.command[-3:] == (
        "reset",
        "--config",
        str(CONFIG),
    )


def test_panel_builds_live_collection_from_tracked_contract() -> None:
    adapter = L1OperatorPanelAdapter(ROOT, CONFIG)
    launch = adapter.build_launch(
        "collect",
        {
            "experiment": "pick_v1",
            "task": "pick up the object",
        },
    )
    assert launch.command[0] == str(ROOT / "scripts/collect.sh")
    assert launch.command[-4:] == (
        "--experiment",
        "pick_v1",
        "--task",
        "pick up the object",
    )
    assert launch.input_actions == (
        InputAction("enter", "Next / Save", "\n", "primary"),
        InputAction("reset", "Reset", "r\n", "quiet"),
        InputAction("discard", "Discard", "d\n", "danger"),
        InputAction("quit", "Quit", "q\n", "quiet"),
    )


def test_panel_uses_shared_collection_camera_health(monkeypatch) -> None:
    health = {
        "available": True,
        "ok": True,
        "streams": {
            "agent": {
                "ready": True,
                "fresh": True,
                "preview_fps": 9.8,
                "age_s": 0.03,
                "error": None,
            }
        },
    }

    monkeypatch.setattr(
        "vlai_l1_runtime.panel.fetch_camera_health",
        lambda port: health if port == 8088 else None,
    )
    assert L1OperatorPanelAdapter(ROOT, CONFIG).camera_health() == health


def test_panel_registers_a_create_only_collection_prompt(tmp_path: Path) -> None:
    shutil.copytree(ROOT / "configs", tmp_path / "configs")
    adapter = L1OperatorPanelAdapter(
        tmp_path,
        tmp_path / "configs/collection/default.toml",
    )
    application = OperatorPanelApplication(adapter)
    payload = {
        "registration": "prompt",
        "values": {
            "catalog": "configs/tasks/fruit_placement/catalog.json",
            "task_id": "place_red_apple_in_bowl",
            "prompt": "place the red apple in the bowl",
            "distribution": "ood",
        },
    }
    result = application.register(payload)

    assert result["created"] == (
        "configs/tasks/fruit_placement/prompts/place_red_apple_in_bowl.json"
    )
    assert result["activate"] == {
        "panel": "collect",
        "values": {"task": "place the red apple in the bowl"},
    }
    with pytest.raises(FileExistsError, match="already registered"):
        application.register(payload)
