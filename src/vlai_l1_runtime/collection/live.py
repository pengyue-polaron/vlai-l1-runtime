"""Live VLAI L1 sample source composed from commissioned Runtime adapters."""

from __future__ import annotations

import time
from collections.abc import Iterator
from contextlib import ExitStack
from typing import Protocol

from ..camera_ipc import RawCameraBridgeClient
from ..contracts import NamedJointVector
from ..teleoperation import XAirStateReceiver
from .configuration import CollectionConfig
from .schema import CameraSample, CollectionSample


class StateSource(Protocol):
    def __enter__(self) -> StateSource: ...

    def receive_closest(
        self,
        *,
        target_monotonic_ns: int,
        timeout_s: float,
    ) -> tuple[NamedJointVector, NamedJointVector] | None: ...

    def __exit__(self, exc_type, exc, traceback) -> None: ...


class CameraSource(Protocol):
    def __enter__(self) -> CameraSource: ...

    def capture(self, *, timeout_s: float) -> dict[str, CameraSample]: ...

    def __exit__(self, exc_type, exc, traceback) -> None: ...


class LiveCollectionSource:
    """Consume persistent cameras and the x_air observation socket for one run."""

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
        self._camera_source = camera_source or RawCameraBridgeClient(config.system)
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

    @property
    def camera_source(self) -> CameraSource:
        if self._stack is None:
            raise RuntimeError("live collection source is not open")
        return self._camera_source

    def samples(self) -> Iterator[tuple[CollectionSample, int]]:
        if self._stack is None:
            raise RuntimeError("live collection source is not open")
        timeout_s = self._config.max_sample_age_s
        period_ns = round(1_000_000_000 / self._config.fps)
        next_tick = time.monotonic_ns()
        while True:
            available = self._camera_source.capture(timeout_s=timeout_s)
            try:
                cameras = {role: available[role] for role in self._config.record_camera_roles}
            except KeyError as exc:
                raise RuntimeError(f"recording camera is missing: {exc.args[0]}") from exc
            camera_timestamps = [sample.metadata.monotonic_ns for sample in cameras.values()]
            target_monotonic_ns = (min(camera_timestamps) + max(camera_timestamps)) // 2
            robot = self._state_source.receive_closest(
                target_monotonic_ns=target_monotonic_ns,
                timeout_s=timeout_s,
            )
            if robot is None:
                raise TimeoutError("timed out waiting for selected x_air state")
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
