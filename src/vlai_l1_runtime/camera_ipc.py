"""Exact raw-frame transport between the persistent camera owner and collectors."""

from __future__ import annotations

import json
import math
import socket
import socketserver
import struct
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Protocol

from embodied_ops import add_contract_digest

from .cameras import CameraFrameMetadata
from .collection.schema import CameraSample, validate_camera_image
from .configuration import SystemConfig

_PROTOCOL_VERSION = 1
_LENGTH = struct.Struct("!I")
_MAX_HEADER_BYTES = 64 * 1024
_MAX_WAIT_S = 1.0


class CameraSnapshotSource(Protocol):
    def latest(self) -> dict[str, CameraSample]: ...


class FrameCodec(Protocol):
    def encode(self, image: Any, *, shape: tuple[int, int, int]) -> tuple[str, bytes]: ...

    def decode(
        self,
        payload: bytes,
        *,
        shape: tuple[int, int, int],
        dtype: str,
    ) -> Any: ...


def camera_contract_digest(config: SystemConfig) -> str:
    """Return the identity of the complete configured camera contract."""

    if not isinstance(config, SystemConfig):
        raise TypeError("camera contract digest requires SystemConfig")
    contract = add_contract_digest(
        {
            "protocol": "vlai-l1-raw-camera-v1",
            "cameras": asdict(config.cameras),
        }
    )
    return str(contract["contract_sha256"])


class RawCameraBridgeServer:
    """Serve synchronized latest RGB frames without opening physical devices."""

    def __init__(
        self,
        config: SystemConfig,
        source: CameraSnapshotSource,
        *,
        socket_path: Path | None = None,
        codec: FrameCodec | None = None,
    ) -> None:
        if not isinstance(config, SystemConfig):
            raise TypeError("RawCameraBridgeServer requires SystemConfig")
        self._config = config
        self._source = source
        self.socket_path = (
            config.camera_preview.bridge_socket_path
            if socket_path is None
            else socket_path.expanduser().resolve()
        )
        self._streams = tuple(stream for stream in config.cameras.streams if stream.enabled)
        self._stream_by_role = {stream.role: stream for stream in self._streams}
        self._roles = tuple(self._stream_by_role)
        self._contract_digest = camera_contract_digest(config)
        self._max_skew_ns = round(config.cameras.max_pair_skew_s * 1_000_000_000)
        self._codec = codec or _NumpyFrameCodec()
        self._server: _ThreadingUnixServer | None = None
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None

    def __enter__(self) -> RawCameraBridgeServer:
        self.start()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def start(self) -> None:
        if self._server is not None:
            raise RuntimeError("raw camera bridge is already running")
        if self.socket_path.exists() or self.socket_path.is_symlink():
            raise RuntimeError(f"raw camera bridge socket already exists: {self.socket_path}")
        if not self.socket_path.parent.is_dir():
            raise RuntimeError(
                f"raw camera bridge directory does not exist: {self.socket_path.parent}"
            )
        server = _ThreadingUnixServer(str(self.socket_path), _RawCameraRequestHandler)
        server.bridge = self
        thread = threading.Thread(
            target=self._serve,
            args=(server,),
            name="vlai-raw-camera-bridge",
            daemon=False,
        )
        self._server = server
        self._thread = thread
        thread.start()

    def close(self) -> None:
        server, self._server = self._server, None
        thread, self._thread = self._thread, None
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=2)
            if thread.is_alive():
                raise RuntimeError("raw camera bridge did not stop")
        try:
            if self.socket_path.is_socket():
                self.socket_path.unlink()
        except FileNotFoundError:
            pass

    def exception(self) -> BaseException | None:
        return self._error

    def response(self, request: Any) -> tuple[dict[str, Any], bytes]:
        try:
            return self._response(request)
        except BaseException as exc:
            return (
                {
                    "ok": False,
                    "version": _PROTOCOL_VERSION,
                    "error": f"{type(exc).__name__}: {exc}",
                    "payload_bytes": 0,
                },
                b"",
            )

    def _serve(self, server: _ThreadingUnixServer) -> None:
        try:
            server.serve_forever(poll_interval=0.1)
        except BaseException as exc:
            self._error = exc

    def _response(self, request: Any) -> tuple[dict[str, Any], bytes]:
        if not isinstance(request, dict):
            raise ValueError("raw camera bridge request must be an object")
        if set(request) != {"version", "op", "contract_digest", "after", "wait_s"}:
            raise ValueError("raw camera bridge request fields are invalid")
        if request["version"] != _PROTOCOL_VERSION:
            raise ValueError("raw camera bridge protocol version mismatch")
        if request["op"] != "next":
            raise ValueError("unsupported raw camera bridge operation")
        if request["contract_digest"] != self._contract_digest:
            raise ValueError("camera contract mismatch; restart the persistent camera service")
        after = request["after"]
        if not isinstance(after, dict) or tuple(after) != self._roles:
            raise ValueError("raw camera bridge sequence map does not match camera roles")
        sequence_after = {
            role: _nonnegative_integer(after[role], label=f"after.{role}", allow_minus_one=True)
            for role in self._roles
        }
        wait_s = _finite_positive(request["wait_s"], label="wait_s")
        if wait_s > _MAX_WAIT_S:
            raise ValueError(f"wait_s must not exceed {_MAX_WAIT_S:.1f}")

        deadline = time.monotonic() + wait_s
        while True:
            samples = self._source.latest()
            if tuple(samples) != self._roles:
                raise RuntimeError("persistent camera owner did not return every enabled role")
            timestamps = tuple(sample.metadata.monotonic_ns for sample in samples.values())
            all_changed = all(
                samples[role].metadata.source_sequence > sequence_after[role]
                for role in self._roles
            )
            synchronized = max(timestamps) - min(timestamps) <= self._max_skew_ns
            if all_changed and synchronized:
                return self._encode_samples(samples)
            if time.monotonic() >= deadline:
                return (
                    {
                        "ok": True,
                        "version": _PROTOCOL_VERSION,
                        "changed": False,
                        "payload_bytes": 0,
                    },
                    b"",
                )
            time.sleep(0.002)

    def _encode_samples(self, samples: dict[str, CameraSample]) -> tuple[dict[str, Any], bytes]:
        arrays: list[dict[str, Any]] = []
        chunks: list[bytes] = []
        offset = 0
        for role in self._roles:
            sample = samples[role]
            stream = self._stream_by_role[role]
            validate_camera_image(sample.image, stream)
            if sample.metadata.role != role or sample.metadata.device_id != str(stream.device_id):
                raise RuntimeError(f"{role} camera metadata does not match tracked identity")
            expected_shape = (stream.height, stream.width, 3)
            dtype, chunk = self._codec.encode(sample.image, shape=expected_shape)
            chunks.append(chunk)
            arrays.append(
                {
                    "role": role,
                    "device_id": sample.metadata.device_id,
                    "stream_epoch": sample.metadata.stream_epoch,
                    "source_sequence": sample.metadata.source_sequence,
                    "monotonic_ns": sample.metadata.monotonic_ns,
                    "shape": list(expected_shape),
                    "dtype": dtype,
                    "offset": offset,
                    "nbytes": len(chunk),
                }
            )
            offset += len(chunk)
        return (
            {
                "ok": True,
                "version": _PROTOCOL_VERSION,
                "changed": True,
                "contract_digest": self._contract_digest,
                "arrays": arrays,
                "payload_bytes": offset,
            },
            b"".join(chunks),
        )


class RawCameraBridgeClient:
    """Collection camera source backed by the persistent raw-frame bridge."""

    def __init__(
        self,
        config: SystemConfig,
        *,
        socket_path: Path | None = None,
        codec: FrameCodec | None = None,
    ) -> None:
        if not isinstance(config, SystemConfig):
            raise TypeError("RawCameraBridgeClient requires SystemConfig")
        self._config = config
        self.socket_path = (
            config.camera_preview.bridge_socket_path
            if socket_path is None
            else socket_path.expanduser().resolve()
        )
        self._streams = tuple(stream for stream in config.cameras.streams if stream.enabled)
        self._stream_by_role = {stream.role: stream for stream in self._streams}
        self._roles = tuple(self._stream_by_role)
        self._contract_digest = camera_contract_digest(config)
        self._codec = codec or _NumpyFrameCodec()
        self._after = dict.fromkeys(self._roles, -1)
        self._socket: socket.socket | None = None

    def __enter__(self) -> RawCameraBridgeClient:
        if self._socket is not None:
            raise RuntimeError("raw camera bridge client is already connected")
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.settimeout(self._config.camera_preview.startup_timeout_s)
        try:
            client.connect(str(self.socket_path))
        except BaseException:
            client.close()
            raise
        self._socket = client
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        client, self._socket = self._socket, None
        if client is not None:
            client.close()

    def capture(self, *, timeout_s: float) -> dict[str, CameraSample]:
        client = self._socket
        if client is None:
            raise RuntimeError("raw camera bridge client is not connected")
        timeout = _finite_positive(timeout_s, label="camera capture timeout")
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("persistent camera bridge produced no synchronized frame set")
            wait_s = min(remaining, _MAX_WAIT_S)
            client.settimeout(remaining + 0.1)
            _send_request(
                client,
                {
                    "version": _PROTOCOL_VERSION,
                    "op": "next",
                    "contract_digest": self._contract_digest,
                    "after": self._after,
                    "wait_s": wait_s,
                },
            )
            header = _receive_header(client)
            changed = _validate_response_header(header)
            payload_bytes = _nonnegative_integer(
                header.get("payload_bytes"),
                label="payload_bytes",
            )
            expected_payload_bytes = sum(
                stream.width * stream.height * 3 for stream in self._streams
            )
            if payload_bytes > expected_payload_bytes:
                raise ValueError("raw camera bridge payload exceeds the configured image set")
            payload = _receive_exact(client, payload_bytes)
            if header["ok"] is not True:
                raise RuntimeError(str(header["error"]))
            if not changed:
                if payload:
                    raise ValueError("unchanged raw camera bridge response has a payload")
                continue
            samples = self._decode_samples(header, payload)
            self._after = {
                role: sample.metadata.source_sequence for role, sample in samples.items()
            }
            return samples

    def _decode_samples(self, header: dict[str, Any], payload: bytes) -> dict[str, CameraSample]:
        if header.get("contract_digest") != self._contract_digest:
            raise ValueError("raw camera bridge response camera contract mismatch")
        arrays = header.get("arrays")
        if not isinstance(arrays, list) or len(arrays) != len(self._roles):
            raise ValueError("raw camera bridge response does not contain every camera")
        samples: dict[str, CameraSample] = {}
        expected_offset = 0
        for role, raw in zip(self._roles, arrays, strict=True):
            if not isinstance(raw, dict) or set(raw) != {
                "role",
                "device_id",
                "stream_epoch",
                "source_sequence",
                "monotonic_ns",
                "shape",
                "dtype",
                "offset",
                "nbytes",
            }:
                raise ValueError("raw camera bridge array descriptor is invalid")
            stream = self._stream_by_role[role]
            expected_shape = (stream.height, stream.width, 3)
            expected_nbytes = stream.height * stream.width * 3
            if (
                raw.get("role") != role
                or raw.get("device_id") != str(stream.device_id)
                or raw.get("shape") != list(expected_shape)
                or raw.get("dtype") not in {"|u1", "<u1", ">u1"}
                or raw.get("offset") != expected_offset
                or raw.get("nbytes") != expected_nbytes
            ):
                raise ValueError(f"{role} raw camera bridge descriptor is invalid")
            stream_epoch = raw.get("stream_epoch")
            if not isinstance(stream_epoch, str) or not stream_epoch:
                raise ValueError(f"{role} raw camera bridge stream epoch is invalid")
            sequence = _nonnegative_integer(
                raw.get("source_sequence"),
                label=f"{role}.source_sequence",
            )
            monotonic_ns = _nonnegative_integer(
                raw.get("monotonic_ns"),
                label=f"{role}.monotonic_ns",
            )
            end = expected_offset + expected_nbytes
            if end > len(payload):
                raise ValueError("raw camera bridge payload is truncated")
            image = self._codec.decode(
                payload[expected_offset:end],
                shape=expected_shape,
                dtype=str(raw["dtype"]),
            )
            sample = CameraSample(
                CameraFrameMetadata(
                    role,
                    str(stream.device_id),
                    stream_epoch,
                    sequence,
                    monotonic_ns,
                ),
                image,
            )
            validate_camera_image(sample.image, stream)
            samples[role] = sample
            expected_offset = end
        if expected_offset != len(payload):
            raise ValueError("raw camera bridge payload has trailing bytes")
        return samples


class _ThreadingUnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True
    block_on_close = True
    bridge: RawCameraBridgeServer


class _RawCameraRequestHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        while True:
            request = _receive_request(self.request)
            if request is None:
                return
            header, payload = self.server.bridge.response(request)
            _send_response(self.request, header, payload)


class _NumpyFrameCodec:
    def encode(self, image: Any, *, shape: tuple[int, int, int]) -> tuple[str, bytes]:
        try:
            import numpy as np
        except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("raw camera bridge requires 'vlai-l1-runtime[camera]'") from exc
        array = np.ascontiguousarray(image)
        if tuple(array.shape) != shape or array.dtype != np.uint8:
            raise ValueError("raw camera bridge image does not match its configured contract")
        return array.dtype.str, array.tobytes()

    def decode(
        self,
        payload: bytes,
        *,
        shape: tuple[int, int, int],
        dtype: str,
    ) -> Any:
        try:
            import numpy as np
        except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError(
                "raw camera bridge client requires 'vlai-l1-runtime[camera]'"
            ) from exc
        if dtype not in {"|u1", "<u1", ">u1"}:
            raise ValueError("raw camera bridge image dtype must be uint8")
        return np.frombuffer(payload, dtype=np.uint8).reshape(shape).copy()


def _send_request(client: socket.socket, value: dict[str, Any]) -> None:
    encoded = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
    if len(encoded) > _MAX_HEADER_BYTES:
        raise ValueError("raw camera bridge request is too large")
    client.sendall(_LENGTH.pack(len(encoded)))
    client.sendall(encoded)


def _receive_request(client: socket.socket) -> dict[str, Any] | None:
    length_bytes = _receive_exact_or_eof(client, _LENGTH.size)
    if length_bytes is None:
        return None
    length = _LENGTH.unpack(length_bytes)[0]
    if length > _MAX_HEADER_BYTES:
        raise ValueError("raw camera bridge request is too large")
    value = json.loads(_receive_exact(client, length))
    if not isinstance(value, dict):
        raise ValueError("raw camera bridge request must be an object")
    return value


def _send_response(
    client: socket.socket,
    header: dict[str, Any],
    payload: bytes,
) -> None:
    encoded = json.dumps(header, separators=(",", ":"), allow_nan=False).encode()
    if len(encoded) > _MAX_HEADER_BYTES:
        raise ValueError("raw camera bridge response header is too large")
    client.sendall(_LENGTH.pack(len(encoded)))
    client.sendall(encoded)
    if payload:
        client.sendall(payload)


def _receive_header(client: socket.socket) -> dict[str, Any]:
    length = _LENGTH.unpack(_receive_exact(client, _LENGTH.size))[0]
    if length > _MAX_HEADER_BYTES:
        raise ValueError("raw camera bridge response header is too large")
    value = json.loads(_receive_exact(client, length))
    if not isinstance(value, dict):
        raise ValueError("raw camera bridge response header must be an object")
    return value


def _validate_response_header(header: dict[str, Any]) -> bool:
    if header.get("version") != _PROTOCOL_VERSION:
        raise ValueError("raw camera bridge response version mismatch")
    ok = header.get("ok")
    if not isinstance(ok, bool):
        raise ValueError("raw camera bridge response status is invalid")
    if ok is False:
        if set(header) != {"ok", "version", "error", "payload_bytes"}:
            raise ValueError("raw camera bridge error response fields are invalid")
        if not isinstance(header.get("error"), str) or not header["error"]:
            raise ValueError("raw camera bridge error response is missing detail")
        if header.get("payload_bytes") != 0:
            raise ValueError("raw camera bridge error response has a payload")
        return False
    changed = header.get("changed")
    if not isinstance(changed, bool):
        raise ValueError("raw camera bridge changed status is invalid")
    expected = (
        {"ok", "version", "changed", "contract_digest", "arrays", "payload_bytes"}
        if changed
        else {"ok", "version", "changed", "payload_bytes"}
    )
    if set(header) != expected:
        raise ValueError("raw camera bridge response fields are invalid")
    return changed


def _receive_exact(client: socket.socket, length: int) -> bytes:
    chunks: list[bytes] = []
    remaining = length
    while remaining:
        chunk = client.recv(remaining)
        if not chunk:
            raise ConnectionError("raw camera bridge closed the connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _receive_exact_or_eof(client: socket.socket, length: int) -> bytes | None:
    first = client.recv(length)
    if not first:
        return None
    if len(first) == length:
        return first
    return first + _receive_exact(client, length - len(first))


def _finite_positive(value: Any, *, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError(f"{label} must be finite and positive")
    return float(value)


def _nonnegative_integer(
    value: Any,
    *,
    label: str,
    allow_minus_one: bool = False,
) -> int:
    minimum = -1 if allow_minus_one else 0
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{label} must be an integer greater than or equal to {minimum}")
    return value
