"""Managed collection session that owns teleoperation, cameras, and cleanup."""

from __future__ import annotations

import os
import re
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass
from typing import Protocol, TextIO

from embodied_ops import EpisodeDecision
from embodied_ops.operator_panel import announce_input

from .. import console
from ..camera_bridge import V4L2CameraSet
from ..cameras import CameraSetValidator
from ..teleoperation import XAirStateReceiver, describe_xair_side, verify_xair_dependency
from .configuration import CollectionConfig
from .dataset import (
    DirectLeRobotEpisode,
    LeRobotBackendFactory,
    identity_from_config,
    provenance_from_config,
)
from .orchestration import EpisodeResult, complete_episode, write_episode_frames
from .schema import CollectionSample, SampleAssembler

_LEVEL = re.compile(r"^(INFO|STEP|PASS|WARN|FAIL)\s+(.*)$")


class RuntimeSession(Protocol):
    def __enter__(self) -> RuntimeSession: ...

    def wait_until_ready(self, receiver: XAirStateReceiver) -> None: ...

    def __exit__(self, exc_type, exc, traceback) -> None: ...


@dataclass
class _RuntimeProcess:
    side: str
    process: subprocess.Popen[str]
    output: deque[str]
    pump: threading.Thread


class ManagedXAirRuntimes:
    """Start both configured sidecars and always disable their four CAN links."""

    def __init__(self, config: CollectionConfig) -> None:
        self._config = config
        self._runtimes: list[_RuntimeProcess] = []
        self._sudo = () if os.geteuid() == 0 else ("sudo", "-n")

    @staticmethod
    def authorize() -> None:
        if os.geteuid() != 0:
            try:
                subprocess.run(
                    ("sudo", "-n", "true"),
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except (OSError, subprocess.CalledProcessError) as exc:
                raise RuntimeError(
                    "managed robot lifecycle requires working passwordless sudo"
                ) from exc

    def __enter__(self) -> ManagedXAirRuntimes:
        if self._runtimes:
            raise RuntimeError("managed x_air runtimes are already active")
        system = self._config.system
        try:
            verify_xair_dependency(system)
            for side in ("left", "right"):
                console.step(f"Starting {side} teleoperation runtime")
                command = (
                    *self._sudo,
                    sys.executable,
                    "-m",
                    "vlai_l1_runtime.cli",
                    "run-xair",
                    "--config",
                    str(system.path),
                    "--side",
                    side,
                )
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
                if process.stdout is None:
                    process.kill()
                    process.wait()
                    raise RuntimeError(f"{side} teleoperation runtime has no output stream")
                output: deque[str] = deque(maxlen=40)
                pump = threading.Thread(
                    target=_pump_runtime_output,
                    args=(side, process.stdout, output),
                    name=f"vlai-{side}-runtime-output",
                    daemon=True,
                )
                runtime = _RuntimeProcess(side, process, output, pump)
                self._runtimes.append(runtime)
                pump.start()
        except BaseException:
            self._stop_all()
            raise
        return self

    def wait_until_ready(self, receiver: XAirStateReceiver) -> None:
        timeout_s = self._config.system.teleoperation.startup_timeout_s
        deadline = time.monotonic() + timeout_s
        status = console.LiveStatusLine()
        try:
            while True:
                self._raise_if_runtime_exited()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"bimanual teleoperation did not become ready within {timeout_s:.1f}s"
                    )
                status.update(
                    f"Waiting for paired robot state · {timeout_s - remaining:4.1f}s",
                )
                sample = receiver.receive(timeout_s=min(0.25, remaining))
                if sample is not None:
                    status.close()
                    console.success("Both teleoperation runtimes are publishing fresh state")
                    return
        finally:
            status.close()

    def __exit__(self, exc_type, exc, traceback) -> None:
        self._stop_all()

    def _raise_if_runtime_exited(self) -> None:
        for runtime in self._runtimes:
            status = runtime.process.poll()
            if status is not None:
                detail = runtime.output[-1] if runtime.output else "no runtime output"
                raise RuntimeError(
                    f"{runtime.side} teleoperation exited during startup "
                    f"with status {status}: {detail}"
                )

    def _stop_all(self) -> None:
        if not self._runtimes:
            return
        console.step("Stopping teleoperation runtimes")
        system = self._config.system
        timeout_s = system.teleoperation.shutdown_timeout_s
        for runtime in self._runtimes:
            _signal_runtime(self._sudo, runtime.process.pid, signal_name="INT")
        deadline = time.monotonic() + timeout_s
        for runtime in self._runtimes:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                runtime.process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                console.warning(f"{runtime.side} runtime exceeded graceful shutdown timeout")
                _signal_runtime(self._sudo, runtime.process.pid, signal_name="KILL")
                try:
                    runtime.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    runtime.process.kill()
                    runtime.process.wait()
        for runtime in self._runtimes:
            if runtime.pump.ident is not None:
                runtime.pump.join(timeout=1)
        for side in ("left", "right"):
            launch = describe_xair_side(system, side)
            subprocess.run(
                (
                    *self._sudo,
                    "/opt/xarm_teleop/disable_unilateral_pair.sh",
                    str(launch["arm_side"]),
                    str(launch["leader_can"]),
                    str(launch["follower_can"]),
                ),
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        self._runtimes.clear()
        console.success("Teleoperation stopped; managed CAN links are disabled and down")


def collect_managed_episode(
    config: CollectionConfig,
    *,
    experiment: str,
    task: str,
    frame_count: int,
    decision: EpisodeDecision,
    runtime_factory: type[RuntimeSession] = ManagedXAirRuntimes,
) -> EpisodeResult:
    """Own a complete preflight, bimanual runtime, capture, and stop session."""

    console.info(
        f"Collection session · experiment={experiment} · frames={frame_count} "
        f"· decision={EpisodeDecision(decision).value}"
    )
    console.step("Authorizing realtime lifecycle")
    ManagedXAirRuntimes.authorize()
    receiver = XAirStateReceiver(config.system)
    cameras = V4L2CameraSet(config.system)
    from .live import LiveCollectionSource

    source = LiveCollectionSource(
        config,
        state_source=receiver,
        camera_source=cameras,
    )
    sink = DirectLeRobotEpisode(
        identity=identity_from_config(config, experiment),
        task=task,
        provenance=provenance_from_config(config),
        backend_factory=LeRobotBackendFactory(config.image_writer_threads),
    )

    console.step("Preparing atomic dataset transaction")
    with sink:
        console.step("Preflighting state socket and three cameras")
        with source:
            _preflight_cameras(cameras, config)
            console.success("Three cameras are fresh, synchronized, and ready")
            with runtime_factory(config) as runtimes:
                runtimes.wait_until_ready(receiver)
                if not _wait_for_episode_start():
                    console.info("Collection cancelled before recording")
                    return EpisodeResult(EpisodeDecision.QUIT, 0, None)
                console.step("Rechecking cameras after operator confirmation")
                _preflight_cameras(cameras, config)
                console.success("Three cameras are fresh, synchronized, and ready to record")
                console.step(f"Recording episode · target {config.fps} FPS")
                progress = _with_progress(
                    source.samples(frame_count),
                    frame_count,
                    minimum_fps=config.minimum_capture_fps,
                )
                try:
                    captured_frames = write_episode_frames(
                        samples=progress,
                        assembler=SampleAssembler(config),
                        sink=sink,
                        task=task,
                    )
                finally:
                    progress.close()
        console.step("Finalizing dataset after hardware shutdown")
        result = complete_episode(
            sink=sink,
            frame_count=captured_frames,
            decision=decision,
        )
    if result.decision is EpisodeDecision.SAVE:
        console.success(f"Saved {result.frame_count} frames to {result.dataset_root}")
    else:
        console.success(f"Captured and discarded {result.frame_count} frames")
    return result


def _wait_for_episode_start(
    read_input: Callable[[str], str] = input,
) -> bool:
    """Wait until the operator confirms the teleoperated episode start pose."""

    if not callable(read_input):
        raise TypeError("read_input must be callable")
    console.step("Use teleoperation to place the robot at the episode start pose")
    while True:
        announce_input(("enter",))
        try:
            command = read_input("  Enter=start recording, q=quit > ").strip().lower()
        except EOFError as exc:
            raise RuntimeError("operator confirmation input closed before recording") from exc
        if not command:
            console.success("Operator confirmed the episode start pose")
            return True
        if command in {"q", "quit", "exit"}:
            return False
        console.warning("Press Enter to start recording, or enter q to quit")


def _preflight_cameras(cameras: V4L2CameraSet, config: CollectionConfig) -> None:
    validator = CameraSetValidator(config.system.cameras)
    for _ in range(3):
        samples = cameras.capture(timeout_s=config.max_sample_age_s)
        validator.validate(
            {role: sample.metadata for role, sample in samples.items()},
            now_ns=time.monotonic_ns(),
        )


def _with_progress(
    samples: Iterable[tuple[CollectionSample, int]],
    total: int,
    *,
    minimum_fps: float,
) -> Iterator[tuple[CollectionSample, int]]:
    status = console.LiveStatusLine()
    started = time.monotonic()
    captured = 0
    try:
        for index, sample in enumerate(samples, start=1):
            status.update(f"Recording frame {index:>4}/{total}")
            captured = index
            yield sample
    finally:
        status.close()
    elapsed_s = time.monotonic() - started
    effective_fps = captured / elapsed_s
    console.info(f"Captured {captured} frames in {elapsed_s:.2f}s · {effective_fps:.2f} FPS")
    if effective_fps < minimum_fps:
        raise RuntimeError(
            f"capture rate {effective_fps:.2f} FPS is below the tracked "
            f"minimum {minimum_fps:.2f} FPS"
        )


def _pump_runtime_output(side: str, stream: TextIO, output: deque[str]) -> None:
    for raw in stream:
        line = raw.strip()
        if not line:
            continue
        output.append(line)
        match = _LEVEL.fullmatch(line)
        if match is None:
            console.info(f"{side.upper():>5} · {line}")
        else:
            console.emit(match.group(1), f"{side.upper():>5} · {match.group(2)}")


def _signal_runtime(sudo: tuple[str, ...], process_group: int, *, signal_name: str) -> None:
    subprocess.run(
        (
            *sudo,
            "kill",
            f"-{signal_name}",
            "--",
            f"-{process_group}",
        ),
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
