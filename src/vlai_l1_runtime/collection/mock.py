"""Deterministic sample source for hardware-free collection integration tests."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from typing import Any

from ..cameras import CameraFrameMetadata
from ..contracts import NamedJointVector, SampleMetadata
from .configuration import CollectionConfig
from .schema import CameraSample, CollectionSample

ImageFactory = Callable[[str, int, int, int], Any]


class SyntheticSampleSource:
    """Generate valid stationary samples without opening cameras or robot transports."""

    def __init__(
        self,
        config: CollectionConfig,
        *,
        image_factory: ImageFactory,
        stream_epoch: str = "synthetic-v1",
    ) -> None:
        if not config.system.cameras.collection_ready:
            raise ValueError("synthetic source requires commissioned camera identities")
        if not callable(image_factory):
            raise TypeError("image_factory must be callable")
        if not stream_epoch or stream_epoch != stream_epoch.strip():
            raise ValueError("stream_epoch must be normalized non-empty text")
        self._config = config
        self._image_factory = image_factory
        self._stream_epoch = stream_epoch

    def samples(
        self,
        count: int,
        *,
        start_ns: int = 1_000_000_000,
    ) -> Iterator[tuple[CollectionSample, int]]:
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError("synthetic sample count must be a positive integer")
        if isinstance(start_ns, bool) or not isinstance(start_ns, int) or start_ns < 0:
            raise ValueError("synthetic start_ns must be a non-negative integer")
        period_ns = round(1_000_000_000 / self._config.fps)
        pose = dict.fromkeys(self._config.feature_names, 0.0)
        for sequence in range(count):
            timestamp_ns = start_ns + sequence * period_ns
            metadata = SampleMetadata(sequence, timestamp_ns)
            cameras = {
                stream.role: CameraSample(
                    CameraFrameMetadata(
                        stream.role,
                        str(stream.device_id),
                        self._stream_epoch,
                        sequence,
                        timestamp_ns,
                    ),
                    self._image_factory(
                        stream.role,
                        stream.height,
                        stream.width,
                        sequence,
                    ),
                )
                for stream in self._config.recording_camera_streams
            }
            yield (
                CollectionSample(
                    NamedJointVector(pose, metadata, self._config.feature_names),
                    NamedJointVector(pose, metadata, self._config.feature_names),
                    cameras,
                ),
                timestamp_ns,
            )
