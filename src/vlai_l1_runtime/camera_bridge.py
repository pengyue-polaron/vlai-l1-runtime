"""Config-driven live camera ownership for VLAI L1 collection."""

from __future__ import annotations

import math
import threading
import time
import uuid
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from .cameras import CameraFrameMetadata, CameraSetValidator
from .collection.schema import CameraSample, validate_camera_image
from .configuration import CameraConfig, SystemConfig


@dataclass(frozen=True)
class CameraCapture:
    source_sequence: int
    monotonic_ns: int
    image: Any

    def __post_init__(self) -> None:
        for label, value in (
            ("source_sequence", self.source_sequence),
            ("monotonic_ns", self.monotonic_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"camera capture {label} must be a non-negative integer")
        if self.image is None:
            raise ValueError("camera capture image is missing")


class CameraReader(Protocol):
    def capture(self, *, timeout_s: float) -> CameraCapture: ...

    def close(self) -> None: ...


class CameraBackend(Protocol):
    def open(self, stream: CameraConfig) -> CameraReader: ...


@dataclass(frozen=True)
class CameraHealthStream:
    device_id: str
    first_sequence: int
    last_sequence: int
    shape: tuple[int, int, int]
    configured_fps: int


@dataclass(frozen=True)
class CameraHealthReport:
    sample_count: int
    elapsed_s: float
    effective_fps: float
    max_pair_skew_ms: float
    streams: dict[str, CameraHealthStream]


class V4L2CameraSet:
    """Open each configured V4L2 stream once and return role-keyed RGB frames."""

    def __init__(self, config: SystemConfig, *, backend: CameraBackend | None = None) -> None:
        if not isinstance(config, SystemConfig):
            raise TypeError("V4L2CameraSet requires SystemConfig")
        if not config.cameras.collection_ready:
            raise ValueError("required camera identities are not commissioned")
        self._streams = tuple(stream for stream in config.cameras.streams if stream.enabled)
        unsupported = [stream.role for stream in self._streams if stream.driver != "v4l2"]
        if unsupported:
            raise ValueError(f"enabled camera roles require unsupported drivers: {unsupported}")
        self._backend = backend or _OpenCvBackend()
        self._readers: dict[str, CameraReader] = {}
        self._epochs: dict[str, str] = {}

    def __enter__(self) -> V4L2CameraSet:
        if self._readers:
            raise RuntimeError("camera set is already open")
        try:
            for stream in self._streams:
                self._readers[stream.role] = self._backend.open(stream)
                self._epochs[stream.role] = uuid.uuid4().hex
        except BaseException:
            with suppress(RuntimeError):
                self._close()
            raise
        return self

    def capture(self, *, timeout_s: float) -> dict[str, CameraSample]:
        if len(self._readers) != len(self._streams):
            raise RuntimeError("camera set is not open")
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not math.isfinite(timeout_s)
            or timeout_s <= 0
        ):
            raise ValueError("camera capture timeout must be finite and positive")
        deadline = time.monotonic() + timeout_s
        samples: dict[str, CameraSample] = {}
        for stream in self._streams:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("camera set capture timed out")
            capture = self._readers[stream.role].capture(timeout_s=remaining)
            samples[stream.role] = CameraSample(
                CameraFrameMetadata(
                    role=stream.role,
                    device_id=str(stream.device_id),
                    stream_epoch=self._epochs[stream.role],
                    source_sequence=capture.source_sequence,
                    monotonic_ns=capture.monotonic_ns,
                ),
                capture.image,
            )
        return samples

    def __exit__(self, exc_type, exc, traceback) -> None:
        self._close()

    def _close(self) -> None:
        errors: list[BaseException] = []
        for role in reversed(tuple(self._readers)):
            try:
                self._readers.pop(role).close()
            except BaseException as error:
                errors.append(error)
        self._epochs.clear()
        if errors:
            raise RuntimeError(f"failed to stop {len(errors)} camera stream(s)") from errors[0]


def check_v4l2_cameras(
    config: SystemConfig,
    *,
    sample_count: int,
    timeout_s: float,
    backend: CameraBackend | None = None,
) -> CameraHealthReport:
    """Open the configured cameras once and validate a finite live sample window."""

    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or sample_count <= 0:
        raise ValueError("camera sample_count must be a positive integer")
    if (
        isinstance(timeout_s, bool)
        or not isinstance(timeout_s, (int, float))
        or not math.isfinite(timeout_s)
        or timeout_s <= 0
    ):
        raise ValueError("camera timeout must be finite and positive")

    stream_by_role = {stream.role: stream for stream in config.cameras.streams if stream.enabled}
    validator = CameraSetValidator(config.cameras)
    first_sequences: dict[str, int] = {}
    last_sequences: dict[str, int] = {}
    shapes: dict[str, tuple[int, int, int]] = {}
    max_pair_skew_ns = 0

    with V4L2CameraSet(config, backend=backend) as cameras:
        started_ns = time.monotonic_ns()
        for _ in range(sample_count):
            samples = cameras.capture(timeout_s=timeout_s)
            metadata = {role: sample.metadata for role, sample in samples.items()}
            validator.validate(metadata, now_ns=time.monotonic_ns())
            timestamps = [frame.monotonic_ns for frame in metadata.values()]
            max_pair_skew_ns = max(max_pair_skew_ns, max(timestamps) - min(timestamps))
            for role, sample in samples.items():
                stream = stream_by_role[role]
                validate_camera_image(sample.image, stream)
                first_sequences.setdefault(role, sample.metadata.source_sequence)
                last_sequences[role] = sample.metadata.source_sequence
                shapes[role] = tuple(sample.image.shape)
        elapsed_s = (time.monotonic_ns() - started_ns) / 1_000_000_000

    return CameraHealthReport(
        sample_count=sample_count,
        elapsed_s=elapsed_s,
        effective_fps=sample_count / elapsed_s,
        max_pair_skew_ms=max_pair_skew_ns / 1_000_000,
        streams={
            role: CameraHealthStream(
                device_id=str(stream.device_id),
                first_sequence=first_sequences[role],
                last_sequence=last_sequences[role],
                shape=shapes[role],
                configured_fps=stream.fps,
            )
            for role, stream in stream_by_role.items()
        },
    )


class _OpenCvReader:
    def __init__(self, capture: Any, cv2: Any, *, role: str) -> None:
        self._capture = capture
        self._cv2 = cv2
        self._role = role
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._latest: CameraCapture | None = None
        self._last_delivered_sequence = 0
        self._error: RuntimeError | None = None
        self._thread = threading.Thread(
            target=self._read_frames,
            name=f"vlai-{role}-camera",
            daemon=True,
        )
        self._thread.start()

    def capture(self, *, timeout_s: float) -> CameraCapture:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if self._error is not None:
                    raise self._error
                if self._stop.is_set():
                    raise RuntimeError(f"{self._role} camera reader is closed")
                if (
                    self._latest is not None
                    and self._latest.source_sequence > self._last_delivered_sequence
                ):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"{self._role} camera capture timed out")
                self._condition.wait(remaining)
            assert self._latest is not None
            sample = self._latest
            self._last_delivered_sequence = sample.source_sequence
            return sample

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self._capture.release()
        with self._condition:
            self._condition.notify_all()
        self._thread.join(timeout=2)
        if self._thread.is_alive():
            raise RuntimeError(f"{self._role} camera reader did not stop")

    def _read_frames(self) -> None:
        sequence = 0
        while not self._stop.is_set():
            try:
                ok, image = self._capture.read()
            except Exception as exc:  # pragma: no cover - native backend failure
                error = RuntimeError(f"{self._role} camera read failed")
                error.__cause__ = exc
                self._fail(error)
                return
            captured_ns = time.monotonic_ns()
            if not ok or image is None:
                if not self._stop.is_set():
                    self._fail(RuntimeError(f"{self._role} camera read failed"))
                return
            try:
                image = self._cv2.cvtColor(image, self._cv2.COLOR_BGR2RGB)
            except Exception as exc:  # pragma: no cover - native backend failure
                error = RuntimeError(f"{self._role} camera color conversion failed")
                error.__cause__ = exc
                self._fail(error)
                return
            sequence += 1
            sample = CameraCapture(sequence, captured_ns, image)
            with self._condition:
                self._latest = sample
                self._condition.notify_all()

    def _fail(self, error: RuntimeError) -> None:
        with self._condition:
            self._error = error
            self._condition.notify_all()


class _OpenCvBackend:
    def open(self, stream: CameraConfig) -> CameraReader:
        try:
            import cv2
        except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("V4L2 collection requires 'vlai-l1-runtime[camera]'") from exc

        device = _resolve_v4l2_device(stream)
        capture = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open {stream.role} V4L2 device {device}")
        try:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, stream.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, stream.height)
            capture.set(cv2.CAP_PROP_FPS, stream.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            actual = (
                round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                round(capture.get(cv2.CAP_PROP_FPS)),
            )
            expected = (stream.width, stream.height, stream.fps)
            if actual != expected:
                raise RuntimeError(f"{stream.role} V4L2 mode is {actual}, expected {expected}")
            return _OpenCvReader(capture, cv2, role=stream.role)
        except BaseException:
            capture.release()
            raise


def _resolve_v4l2_device(stream: CameraConfig) -> Path:
    if stream.device_id is None or stream.video_index is None:
        raise ValueError(f"{stream.role} V4L2 identity is incomplete")
    by_id = Path("/dev/v4l/by-id")
    if not by_id.is_dir():
        raise RuntimeError("V4L2 by-id directory is unavailable")
    suffix = f"_{stream.device_id}-video-index{stream.video_index}"
    matches = tuple(path for path in by_id.iterdir() if path.name.endswith(suffix))
    if len(matches) != 1:
        raise RuntimeError(
            f"{stream.role} expected one V4L2 device ending {suffix!r}, found {len(matches)}"
        )
    resolved = matches[0].resolve(strict=True)
    if resolved.parent != Path("/dev") or not resolved.name.startswith("video"):
        raise RuntimeError(f"{stream.role} V4L2 identity resolves outside /dev/video*: {resolved}")
    return resolved
