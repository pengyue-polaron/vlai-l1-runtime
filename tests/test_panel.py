from __future__ import annotations

import shutil
import sys
from pathlib import Path

import pytest
from embodied_ops.operator_panel import (
    PANEL_CATALOG_SCHEMA_VERSION,
    OperatorPanelApplication,
    validate_panel_catalog,
)

from vlai_l1_runtime.collection.interaction import L1_COLLECTION_INTERACTION
from vlai_l1_runtime.panel import L1OperatorPanelAdapter

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/collection/default.toml"
RIGHT_CONFIG = ROOT / "configs/collection/right_only.toml"


def test_panel_exposes_the_commissioned_collection_stack() -> None:
    adapter = L1OperatorPanelAdapter(ROOT, CONFIG)
    catalog = adapter.catalog()

    assert catalog["schema_version"] == PANEL_CATALOG_SCHEMA_VERSION
    assert validate_panel_catalog(catalog) is catalog
    assert catalog["product"] == {"brand": "VLAI L1", "title": "Operations"}
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
    assert launch.command[-2:] == (
        str(CONFIG),
        "pick_v1",
    )
    camera = adapter.build_launch("camera", {"action": "start"})
    assert camera.command == (
        str(ROOT / "scripts/camera_service.sh"),
        "start",
    )
    reset = adapter.build_launch("reset", {"config": "configs/collection/default.toml"})
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
    assert launch.command[-3:] == (
        "--task",
        "pick up the object",
        "pick_v1",
    )
    assert launch.input_actions == L1_COLLECTION_INTERACTION.input_actions


def test_right_only_panel_exposes_only_recorded_camera_roles() -> None:
    adapter = L1OperatorPanelAdapter(ROOT, RIGHT_CONFIG)
    catalog = adapter.catalog()

    assert [camera["id"] for camera in catalog["cameras"]] == ["wrist_right", "agent"]
    collect = next(workflow for workflow in catalog["workflows"] if workflow["id"] == "collect")
    assert "right" in collect["description"]
    assert "wrist_right, agent" in collect["description"]
    task_field = next(field for field in collect["fields"] if field["name"] == "task")
    assert task_field["type"] == "combobox"
    assert "put the blue block into the red plate" in {
        option["value"] for option in task_field["options"]
    }
    reset = next(workflow for workflow in catalog["workflows"] if workflow["id"] == "reset")
    assert "right leader/follower" in reset["confirm"]
    prompts = {
        item["value"]
        for group in catalog["configuration_groups"]
        if group["label"] == "Registered prompts"
        for item in group["items"]
    }
    assert "put the blue block into the red plate" in prompts


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
