from __future__ import annotations

import time
from dataclasses import replace
from http.client import HTTPConnection
from pathlib import Path

from vlai_l1_runtime.camera_preview import CameraPreviewServer
from vlai_l1_runtime.cameras import CameraFrameMetadata
from vlai_l1_runtime.collection.schema import CameraSample
from vlai_l1_runtime.configuration import load_system_config

ROOT = Path(__file__).resolve().parents[1]


class _Source:
    def __init__(self) -> None:
        self.sequence = 0
        self.error: RuntimeError | None = None

    def latest(self) -> dict[str, CameraSample]:
        if self.error is not None:
            raise self.error
        self.sequence += 1
        now_ns = time.monotonic_ns()
        return {
            role: CameraSample(
                CameraFrameMetadata(role, f"{role}-id", "epoch", self.sequence, now_ns),
                object(),
            )
            for role in ("wrist_left", "wrist_right", "agent")
        }


def test_camera_preview_serves_collection_owned_frames_and_health() -> None:
    system = load_system_config(ROOT / "configs/system/vlai_l1.toml")
    system = replace(
        system,
        camera_preview=replace(
            system.camera_preview,
            bind="127.0.0.1",
            port=0,
            fps=30,
        ),
    )
    source = _Source()
    with CameraPreviewServer(system, source, encoder=lambda _image, _quality: b"jpeg") as preview:
        deadline = time.monotonic() + 1
        while not preview.health()["ok"] and time.monotonic() < deadline:
            time.sleep(0.01)
        assert preview.health()["ok"] is True

        connection = HTTPConnection("127.0.0.1", preview.bound_port, timeout=1)
        connection.request("GET", "/snapshot/agent.jpg")
        response = connection.getresponse()
        assert response.status == 200
        assert response.read() == b"jpeg"
        connection.close()


def test_camera_preview_reports_source_failure_without_stopping_the_server() -> None:
    system = load_system_config(ROOT / "configs/system/vlai_l1.toml")
    system = replace(
        system,
        camera_preview=replace(
            system.camera_preview,
            bind="127.0.0.1",
            port=0,
            fps=30,
        ),
    )
    source = _Source()
    source.error = RuntimeError("camera disconnected")
    with CameraPreviewServer(system, source, encoder=lambda _image, _quality: b"jpeg") as preview:
        deadline = time.monotonic() + 1
        while not preview.health()["streams"]["agent"]["error"] and time.monotonic() < deadline:
            time.sleep(0.01)
        health = preview.health()

    assert health["ok"] is False
    assert "camera disconnected" in health["streams"]["agent"]["error"]
