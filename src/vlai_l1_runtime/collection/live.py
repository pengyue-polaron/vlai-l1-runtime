"""Live VLAI L1 sample source composed from commissioned Runtime adapters."""

from __future__ import annotations

import time
from collections.abc import Iterator
from contextlib import ExitStack
from typing import Protocol

from embodied_ops import EpisodeDecision

from ..camera_bridge import RealSenseCameraSet
from ..contracts import NamedJointVector
from ..teleoperation import XAirStateReceiver
from .configuration import CollectionConfig
from .dataset import (
    DirectLeRobotEpisode,
    identity_from_config,
    provenance_from_config,
)
from .orchestration import EpisodeResult, record_episode
from .schema import CameraSample, CollectionSample, SampleAssembler


class StateSource(Protocol):
    def __enter__(self) -> StateSource: ...

    def receive(self, *, timeout_s: float) -> tuple[NamedJointVector, NamedJointVector] | None: ...

    def __exit__(self, exc_type, exc, traceback) -> None: ...


class CameraSource(Protocol):
    def __enter__(self) -> CameraSource: ...

    def capture(self, *, timeout_s: float) -> dict[str, CameraSample]: ...

    def __exit__(self, exc_type, exc, traceback) -> None: ...


class LiveCollectionSource:
    """Own cameras and the x_air observation socket for one collection run."""

    def __init__(
        self,
        config: CollectionConfig,
        *,
        state_source: StateSource | None = None,
        camera_source: CameraSource | None = None,
    ) -> None:
        if not isinstance(config, CollectionConfig):
            raise TypeError("LiveCollectionSource requires CollectionConfig")
        if not config.collection_ready:
            raise ValueError(
                f"live collection is unavailable: {', '.join(config.collection_blockers)}"
            )
        self._config = config
        self._state_source = state_source or XAirStateReceiver(config.system)
        self._camera_source = camera_source or RealSenseCameraSet(config.system)
        self._stack: ExitStack | None = None

    def __enter__(self) -> LiveCollectionSource:
        if self._stack is not None:
            raise RuntimeError("live collection source is already open")
        stack = ExitStack()
        try:
            stack.enter_context(self._state_source)
            stack.enter_context(self._camera_source)
        except BaseException:
            stack.close()
            raise
        self._stack = stack
        return self

    def samples(self, frame_count: int) -> Iterator[tuple[CollectionSample, int]]:
        if self._stack is None:
            raise RuntimeError("live collection source is not open")
        if isinstance(frame_count, bool) or not isinstance(frame_count, int) or frame_count <= 0:
            raise ValueError("frame_count must be a positive integer")
        timeout_s = self._config.max_sample_age_s
        period_ns = round(1_000_000_000 / self._config.fps)
        next_tick = time.monotonic_ns()
        for _ in range(frame_count):
            cameras = self._camera_source.capture(timeout_s=timeout_s)
            robot = self._state_source.receive(timeout_s=timeout_s)
            if robot is None:
                raise TimeoutError("timed out waiting for paired x_air state")
            observation, action = robot
            now_ns = time.monotonic_ns()
            yield CollectionSample(observation, action, cameras), now_ns
            next_tick += period_ns
            remaining_ns = next_tick - time.monotonic_ns()
            if remaining_ns > 0:
                time.sleep(remaining_ns / 1_000_000_000)

    def __exit__(self, exc_type, exc, traceback) -> None:
        stack, self._stack = self._stack, None
        if stack is not None:
            stack.__exit__(exc_type, exc, traceback)


def collect_live_episode(
    config: CollectionConfig,
    *,
    experiment: str,
    task: str,
    frame_count: int,
    decision: EpisodeDecision,
) -> EpisodeResult:
    """Record one finite live episode after every readiness gate is commissioned."""

    source = LiveCollectionSource(config)
    sink = DirectLeRobotEpisode(
        identity=identity_from_config(config, experiment),
        task=task,
        provenance=provenance_from_config(config),
    )
    with source:
        return record_episode(
            samples=source.samples(frame_count),
            assembler=SampleAssembler(config),
            sink=sink,
            task=task,
            decision=decision,
        )
