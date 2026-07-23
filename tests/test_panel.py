from __future__ import annotations

import json
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
    assert {workflow["id"] for workflow in catalog["workflows"]} == {
        "validate-collection",
        "collect",
        "dataset-doctor",
        "export-v21",
    }
    launch = adapter.build_launch("dataset-doctor", {"experiment": "pick_v1"})
    assert launch.command[:3] == (sys.executable, "-m", "vlai_l1_runtime.cli")
    assert launch.command[-2:] == ("--experiment", "pick_v1")


def test_panel_builds_live_collection_from_tracked_contract() -> None:
    adapter = L1OperatorPanelAdapter(ROOT, CONFIG)
    launch = adapter.build_launch(
        "collect",
        {
            "experiment": "pick_v1",
            "task": "pick up the object",
            "frames": "300",
            "decision": "discard",
        },
    )
    assert launch.command[-8:] == (
        "--experiment",
        "pick_v1",
        "--task",
        "pick up the object",
        "--frames",
        "300",
        "--decision",
        "discard",
    )
    assert launch.input_actions == (InputAction("enter", "Start recording", "\n", "primary"),)


def test_panel_normalizes_collection_camera_health(monkeypatch) -> None:
    payload = {
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

    class Response:
        status = 200

        def read(self, _limit):
            return json.dumps(payload).encode()

    class Connection:
        def __init__(self, host, port, *, timeout):
            assert (host, port) == ("127.0.0.1", 8088)
            assert timeout > 0

        def request(self, method, path, *, headers):
            assert (method, path) == ("GET", "/healthz")
            assert headers == {"Cache-Control": "no-store"}

        def getresponse(self):
            return Response()

        def close(self):
            return None

    monkeypatch.setattr("vlai_l1_runtime.panel.HTTPConnection", Connection)
    assert L1OperatorPanelAdapter(ROOT, CONFIG).camera_health() == {
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
