"""Read-only MJPEG presentation for the collection-owned camera bridge."""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Protocol
from urllib.parse import urlsplit

from .configuration import SystemConfig


class CameraSnapshotSource(Protocol):
    def latest(self) -> dict[str, Any]: ...


@dataclass
class _StreamState:
    last_source_sequence: int = -1
    source_monotonic_ns: int | None = None
    jpeg: bytes | None = None
    encoded_sequence: int = -1
    encode_times: deque[float] = field(default_factory=lambda: deque(maxlen=30))
    error: str | None = None


class CameraPreviewServer:
    """Serve low-rate previews without opening or advancing camera readers."""

    def __init__(
        self,
        config: SystemConfig,
        source: CameraSnapshotSource,
        *,
        encoder: Callable[[Any, int], bytes] | None = None,
    ) -> None:
        if not isinstance(config, SystemConfig):
            raise TypeError("CameraPreviewServer requires SystemConfig")
        self._config = config.camera_preview
        self._source = source
        self._encoder = encoder or _encode_rgb_jpeg
        self._streams = {
            stream.role: _StreamState() for stream in config.cameras.streams if stream.enabled
        }
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._server: ThreadingHTTPServer | None = None
        self._server_thread: threading.Thread | None = None
        self._encoder_thread: threading.Thread | None = None

    def __enter__(self) -> CameraPreviewServer:
        self.start()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def start(self) -> None:
        if self._server is not None:
            raise RuntimeError("camera preview is already running")
        self._stop.clear()
        server = ThreadingHTTPServer(
            (self._config.bind, self._config.port),
            self._handler_type(),
        )
        server.daemon_threads = True
        self._server = server
        self._encoder_thread = threading.Thread(
            target=self._encode_loop,
            name="vlai-camera-preview-encoder",
            daemon=True,
        )
        self._server_thread = threading.Thread(
            target=server.serve_forever,
            name="vlai-camera-preview-http",
            daemon=True,
        )
        try:
            self._server_thread.start()
            self._encoder_thread.start()
        except BaseException:
            self._stop.set()
            if self._server_thread.is_alive():
                server.shutdown()
                self._server_thread.join(timeout=2)
            server.server_close()
            self._server = None
            self._server_thread = None
            self._encoder_thread = None
            raise

    @property
    def bound_port(self) -> int:
        if self._server is None:
            raise RuntimeError("camera preview is not running")
        return int(self._server.server_address[1])

    def close(self) -> None:
        server = self._server
        if server is None:
            return
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._server_thread is not None and self._server_thread.is_alive():
            server.shutdown()
        server.server_close()
        for thread in (self._server_thread, self._encoder_thread):
            if thread is not None:
                thread.join(timeout=2)
        alive = [
            thread.name
            for thread in (self._server_thread, self._encoder_thread)
            if thread is not None and thread.is_alive()
        ]
        self._server = None
        self._server_thread = None
        self._encoder_thread = None
        if alive:
            raise RuntimeError(f"camera preview threads did not stop: {alive}")

    def health(self) -> dict[str, Any]:
        now_ns = time.monotonic_ns()
        now = time.monotonic()
        with self._condition:
            streams = {
                role: _stream_health(
                    state,
                    now_ns=now_ns,
                    now=now,
                    max_age_s=self._config.max_age_s,
                )
                for role, state in self._streams.items()
            }
        return {
            "ok": all(
                stream["ready"] and stream["fresh"] and stream["error"] is None
                for stream in streams.values()
            ),
            "streams": streams,
        }

    def _encode_loop(self) -> None:
        interval_s = 1.0 / self._config.fps
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                samples = self._source.latest()
                for role, sample in samples.items():
                    state = self._streams.get(role)
                    if state is None:
                        continue
                    sequence = sample.metadata.source_sequence
                    if sequence == state.last_source_sequence:
                        continue
                    jpeg = self._encoder(sample.image, self._config.jpeg_quality)
                    now = time.monotonic()
                    with self._condition:
                        state.last_source_sequence = sequence
                        state.source_monotonic_ns = sample.metadata.monotonic_ns
                        state.jpeg = jpeg
                        state.encoded_sequence += 1
                        state.encode_times.append(now)
                        state.error = None
                        self._condition.notify_all()
            except Exception as exc:
                message = f"{type(exc).__name__}: {exc}"
                with self._condition:
                    for state in self._streams.values():
                        state.error = message
                    self._condition.notify_all()
            self._stop.wait(max(0.001, interval_s - (time.monotonic() - started)))

    def _handler_type(self) -> type[BaseHTTPRequestHandler]:
        preview = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "VLAICameraPreview/1"

            def do_GET(self) -> None:
                path = urlsplit(self.path).path
                if path == "/healthz":
                    self._send_health()
                    return
                if path.startswith("/") and path.endswith(".mjpg"):
                    preview._send_mjpeg(self, path[1:-5])
                    return
                if path.startswith("/snapshot/") and path.endswith(".jpg"):
                    preview._send_snapshot(self, path[10:-4])
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def _send_health(self) -> None:
                payload = preview.health()
                body = json.dumps(payload).encode()
                self.send_response(
                    HTTPStatus.OK if payload["ok"] else HTTPStatus.SERVICE_UNAVAILABLE
                )
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format: str, *_args: Any) -> None:
                return

        return Handler

    def _send_snapshot(self, handler: BaseHTTPRequestHandler, role: str) -> None:
        with self._condition:
            state = self._streams.get(role)
            jpeg = None if state is None else state.jpeg
        if jpeg is None:
            handler.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera frame not ready")
            return
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", "image/jpeg")
        handler.send_header("Content-Length", str(len(jpeg)))
        handler.send_header("Cache-Control", "no-store")
        handler.end_headers()
        handler.wfile.write(jpeg)

    def _send_mjpeg(self, handler: BaseHTTPRequestHandler, role: str) -> None:
        if role not in self._streams:
            handler.send_error(HTTPStatus.NOT_FOUND)
            return
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        handler.end_headers()
        last_sequence = -1
        try:
            while not self._stop.is_set():
                with self._condition:
                    state = self._streams[role]
                    self._condition.wait_for(
                        lambda state=state, last_sequence=last_sequence: (
                            self._stop.is_set()
                            or (state.jpeg is not None and state.encoded_sequence != last_sequence)
                        ),
                        timeout=2,
                    )
                    if (
                        self._stop.is_set()
                        or state.jpeg is None
                        or state.encoded_sequence == last_sequence
                    ):
                        continue
                    jpeg = state.jpeg
                    last_sequence = state.encoded_sequence
                handler.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                handler.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                handler.wfile.write(jpeg)
                handler.wfile.write(b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return


def _stream_health(
    state: _StreamState,
    *,
    now_ns: int,
    now: float,
    max_age_s: float,
) -> dict[str, Any]:
    age_s = (
        None
        if state.source_monotonic_ns is None
        else max(0.0, (now_ns - state.source_monotonic_ns) / 1_000_000_000)
    )
    times = tuple(state.encode_times)
    preview_fps = 0.0
    if len(times) >= 2 and times[-1] > times[0]:
        preview_fps = (len(times) - 1) / (times[-1] - times[0])
    return {
        "ready": state.jpeg is not None,
        "fresh": age_s is not None and age_s <= max_age_s,
        "preview_fps": round(preview_fps, 2),
        "age_s": None if age_s is None else round(age_s, 3),
        "error": state.error,
    }


def _encode_rgb_jpeg(image: Any, quality: int) -> bytes:
    try:
        import cv2
    except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
        raise RuntimeError("camera preview requires 'vlai-l1-runtime[camera]'") from exc
    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(
        ".jpg",
        bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), quality],
    )
    if not ok:
        raise RuntimeError("OpenCV JPEG encoder returned failure")
    return encoded.tobytes()
