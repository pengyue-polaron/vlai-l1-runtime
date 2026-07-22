"""Config-driven live camera ownership for VLAI L1 collection."""

from __future__ import annotations

import math
import time
import uuid
from contextlib import suppress
from dataclasses import dataclass
from typing import Any, Protocol

from .cameras import CameraFrameMetadata
from .collection.schema import CameraSample
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


class RealSenseCameraSet:
    """Open each enabled RealSense once and return role-keyed RGB frames."""

    def __init__(self, config: SystemConfig, *, backend: CameraBackend | None = None) -> None:
        if not isinstance(config, SystemConfig):
            raise TypeError("RealSenseCameraSet requires SystemConfig")
        if not config.cameras.collection_ready:
            raise ValueError("required camera identities are not commissioned")
        self._streams = tuple(stream for stream in config.cameras.streams if stream.enabled)
        unsupported = [stream.role for stream in self._streams if stream.driver != "realsense"]
        if unsupported:
            raise ValueError(f"enabled camera roles require unsupported drivers: {unsupported}")
        self._backend = backend or _PyRealSenseBackend()
        self._readers: dict[str, CameraReader] = {}
        self._epochs: dict[str, str] = {}

    def __enter__(self) -> RealSenseCameraSet:
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


class _PyRealSenseReader:
    def __init__(self, pipeline: Any) -> None:
        self._pipeline = pipeline
        self._closed = False

    def capture(self, *, timeout_s: float) -> CameraCapture:
        if self._closed:
            raise RuntimeError("RealSense reader is closed")
        timeout_ms = max(1, math.ceil(timeout_s * 1000))
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms)
            color = frames.get_color_frame()
        except RuntimeError as exc:
            raise TimeoutError("RealSense frame capture timed out") from exc
        if not color:
            raise RuntimeError("RealSense frameset has no color frame")
        try:
            import numpy as np
        except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("RealSense collection requires NumPy") from exc
        image = np.asanyarray(color.get_data()).copy()
        return CameraCapture(int(color.get_frame_number()), time.monotonic_ns(), image)

    def close(self) -> None:
        if not self._closed:
            self._pipeline.stop()
            self._closed = True


class _PyRealSenseBackend:
    def open(self, stream: CameraConfig) -> CameraReader:
        try:
            import pyrealsense2 as rs
        except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("camera collection requires pyrealsense2") from exc
        pipeline = rs.pipeline()
        settings = rs.config()
        settings.enable_device(str(stream.device_id))
        settings.enable_stream(
            rs.stream.color,
            stream.width,
            stream.height,
            rs.format.rgb8,
            stream.fps,
        )
        try:
            pipeline.start(settings)
        except RuntimeError as exc:
            raise RuntimeError(
                f"cannot start {stream.role} RealSense {stream.device_id}: {exc}"
            ) from exc
        return _PyRealSenseReader(pipeline)
