"""Persistent ownership and lifecycle for commissioned L1 cameras."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from contextlib import ExitStack, suppress
from dataclasses import dataclass
from http.client import HTTPConnection
from pathlib import Path
from typing import Any

from . import console
from .camera_bridge import V4L2CameraSet
from .camera_ipc import RawCameraBridgeServer
from .camera_preview import CameraPreviewServer
from .configuration import SystemConfig

_MARKER_NAME = "camera-service.json"
_LOG_NAME = "camera-service.log"
_HEALTH_MAX_BYTES = 64 * 1024


@dataclass(frozen=True)
class CameraServiceStatus:
    running: bool
    healthy: bool
    pid: int | None
    log_path: Path
    detail: str


def run_camera_service(config: SystemConfig) -> int:
    """Own all physical camera handles and expose raw and preview consumers."""

    if not isinstance(config, SystemConfig):
        raise TypeError("run_camera_service requires SystemConfig")
    console.step("Opening commissioned cameras")
    with ExitStack() as stack:
        cameras = stack.enter_context(V4L2CameraSet(config))
        bridge = stack.enter_context(RawCameraBridgeServer(config, cameras))
        preview = stack.enter_context(CameraPreviewServer(config, cameras))
        console.success(
            "Persistent camera service ready"
            f" · preview=http://{config.camera_preview.bind}:{preview.bound_port}"
            f" · raw={bridge.socket_path}"
        )
        while True:
            error = bridge.exception()
            if error is not None:
                raise RuntimeError("raw camera bridge server failed") from error
            cameras.latest()
            time.sleep(0.1)


class CameraServiceController:
    """Supervise one marked detached camera owner without touching CAN."""

    def __init__(self, config: SystemConfig) -> None:
        if not isinstance(config, SystemConfig):
            raise TypeError("CameraServiceController requires SystemConfig")
        self.config = config
        self.state_dir = config.camera_preview.bridge_socket_path.parent
        self.marker_path = self.state_dir / _MARKER_NAME
        self.log_path = self.state_dir / _LOG_NAME

    def start(self) -> CameraServiceStatus:
        self._require_state_directory()
        marker = self._read_marker()
        if marker is not None:
            if self._marker_process(marker):
                if Path(marker["config"]) != self.config.path:
                    raise RuntimeError("camera service is running with a different System config")
                status = self.status()
                if status.healthy:
                    return status
                self._stop_marked(marker)
            else:
                self._remove_stale_marker()
        elif _preview_endpoint_present(self.config):
            raise RuntimeError("camera preview endpoint is owned by an unmanaged process")
        self._remove_stale_socket()

        log = self.log_path.open("ab", buffering=0)
        command = (
            sys.executable,
            "-m",
            "vlai_l1_runtime.cli",
            "camera-service-run",
            "--config",
            str(self.config.path),
        )
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                cwd=self.config.path.parents[2],
                start_new_session=True,
            )
        finally:
            log.close()
        try:
            start_ticks = _process_start_ticks(process.pid)
        except (FileNotFoundError, ProcessLookupError) as exc:
            process.poll()
            raise RuntimeError(
                f"camera service exited before creating its process marker: {self._log_tail()}"
            ) from exc
        marker = {
            "version": 1,
            "pid": process.pid,
            "start_ticks": start_ticks,
            "config": str(self.config.path),
        }
        self._write_marker(marker)
        deadline = time.monotonic() + self.config.camera_preview.startup_timeout_s
        while time.monotonic() < deadline:
            if process.poll() is not None:
                detail = self._log_tail()
                self._remove_stale_marker()
                self._remove_stale_socket()
                raise RuntimeError(
                    f"camera service exited during startup with status "
                    f"{process.returncode}: {detail}"
                )
            if _preview_healthy(self.config):
                return self.status()
            time.sleep(0.1)
        self._stop_marked(marker)
        raise TimeoutError(
            "camera service did not become healthy within "
            f"{self.config.camera_preview.startup_timeout_s:.1f}s: {self._log_tail()}"
        )

    def stop(self) -> CameraServiceStatus:
        marker = self._read_marker()
        if marker is None:
            if _preview_endpoint_present(self.config):
                raise RuntimeError("camera preview endpoint is owned by an unmanaged process")
            self._remove_stale_socket()
            return self.status()
        if self._marker_process(marker):
            self._stop_marked(marker)
        else:
            self._remove_stale_marker()
            self._remove_stale_socket()
        return self.status()

    def status(self) -> CameraServiceStatus:
        marker = self._read_marker()
        running = marker is not None and self._marker_process(marker)
        pid = int(marker["pid"]) if running and marker is not None else None
        healthy = running and _preview_healthy(self.config)
        if healthy:
            detail = "persistent camera service is healthy"
        elif running:
            detail = "camera service process is running but preview is unhealthy"
        elif _preview_endpoint_present(self.config):
            detail = "camera preview endpoint has no marked owner"
        else:
            detail = "persistent camera service is not running"
        return CameraServiceStatus(running, healthy, pid, self.log_path, detail)

    def log_tail(self) -> str:
        return self._log_tail(lines=160)

    def _require_state_directory(self) -> None:
        if not self.state_dir.is_dir():
            raise RuntimeError(
                f"camera runtime directory does not exist: {self.state_dir}; "
                "run the repository camera lifecycle command"
            )
        if not os.access(self.state_dir, os.W_OK | os.X_OK):
            raise RuntimeError(f"camera runtime directory is not writable: {self.state_dir}")

    def _read_marker(self) -> dict[str, Any] | None:
        try:
            raw = json.loads(self.marker_path.read_text())
        except FileNotFoundError:
            return None
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"cannot read camera service marker: {self.marker_path}") from exc
        if (
            not isinstance(raw, dict)
            or set(raw) != {"version", "pid", "start_ticks", "config"}
            or raw.get("version") != 1
            or isinstance(raw.get("pid"), bool)
            or not isinstance(raw.get("pid"), int)
            or raw["pid"] <= 1
            or isinstance(raw.get("start_ticks"), bool)
            or not isinstance(raw.get("start_ticks"), int)
            or raw["start_ticks"] < 0
            or not isinstance(raw.get("config"), str)
        ):
            raise RuntimeError(f"camera service marker is invalid: {self.marker_path}")
        return raw

    def _marker_process(self, marker: dict[str, Any]) -> bool:
        pid = int(marker["pid"])
        try:
            return _process_start_ticks(pid) == marker["start_ticks"] and _process_command(pid) == (
                sys.executable,
                "-m",
                "vlai_l1_runtime.cli",
                "camera-service-run",
                "--config",
                marker["config"],
            )
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            return False

    def _stop_marked(self, marker: dict[str, Any]) -> None:
        if not self._marker_process(marker):
            self._remove_stale_marker()
            self._remove_stale_socket()
            return
        pid = int(marker["pid"])
        os.killpg(pid, signal.SIGINT)
        deadline = time.monotonic() + self.config.camera_preview.shutdown_timeout_s
        while time.monotonic() < deadline:
            if not self._marker_process(marker):
                self._remove_stale_marker()
                self._remove_stale_socket()
                return
            time.sleep(0.1)
        os.killpg(pid, signal.SIGTERM)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if not self._marker_process(marker):
                self._remove_stale_marker()
                self._remove_stale_socket()
                return
            time.sleep(0.1)
        raise RuntimeError("marked camera service did not stop")

    def _write_marker(self, marker: dict[str, Any]) -> None:
        temporary = self.marker_path.with_name(f".{self.marker_path.name}.{os.getpid()}.tmp")
        try:
            temporary.write_text(json.dumps(marker, sort_keys=True) + "\n")
            os.replace(temporary, self.marker_path)
        finally:
            with suppress(FileNotFoundError):
                temporary.unlink()

    def _remove_stale_marker(self) -> None:
        with suppress(FileNotFoundError):
            self.marker_path.unlink()

    def _remove_stale_socket(self) -> None:
        path = self.config.camera_preview.bridge_socket_path
        if path.is_symlink():
            raise RuntimeError(f"camera bridge socket path is a symbolic link: {path}")
        if path.exists() and not path.is_socket():
            raise RuntimeError(f"camera bridge socket path is not a socket: {path}")
        try:
            if path.is_socket():
                path.unlink()
        except FileNotFoundError:
            pass

    def _log_tail(self, *, lines: int = 20) -> str:
        try:
            content = self.log_path.read_text(errors="replace").splitlines()
        except FileNotFoundError:
            return "no camera service log"
        return " | ".join(content[-lines:]) or "camera service log is empty"


def _preview_healthy(config: SystemConfig) -> bool:
    try:
        status, payload = _preview_health(config)
    except (OSError, TimeoutError, ValueError, json.JSONDecodeError):
        return False
    return status == 200 and payload.get("ok") is True


def _preview_endpoint_present(config: SystemConfig) -> bool:
    try:
        status, payload = _preview_health(config)
    except (OSError, TimeoutError, ValueError, json.JSONDecodeError):
        return False
    return status in {200, 503} and isinstance(payload.get("streams"), dict)


def _preview_health(config: SystemConfig) -> tuple[int, dict[str, Any]]:
    connection = HTTPConnection("127.0.0.1", config.camera_preview.port, timeout=0.4)
    try:
        connection.request("GET", "/healthz", headers={"Cache-Control": "no-store"})
        response = connection.getresponse()
        body = response.read(_HEALTH_MAX_BYTES + 1)
    finally:
        connection.close()
    if len(body) > _HEALTH_MAX_BYTES:
        raise ValueError("camera preview health response is too large")
    payload = json.loads(body)
    if not isinstance(payload, dict):
        raise ValueError("camera preview health response must be an object")
    return response.status, payload


def _process_start_ticks(pid: int) -> int:
    content = Path(f"/proc/{pid}/stat").read_text()
    closing = content.rfind(")")
    if closing < 0:
        raise RuntimeError(f"cannot parse process stat for pid {pid}")
    fields = content[closing + 2 :].split()
    return int(fields[19])


def _process_command(pid: int) -> tuple[str, ...]:
    raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    return tuple(part.decode() for part in raw.rstrip(b"\0").split(b"\0"))
