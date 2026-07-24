"""Managed collection session that owns teleoperation, cameras, and cleanup."""

from __future__ import annotations

import os
import re
import select
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Callable, Iterable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Protocol, TextIO

from embodied_ops import EpisodeDecision, normalize_episode_decision
from embodied_ops.operator_panel import announce_input, announce_progress

from .. import console
from ..camera_service import CameraServiceController
from ..cameras import CameraSetValidator
from ..teleoperation import (
    XAirStateReceiver,
    describe_xair_side,
    remove_orphaned_xair_control_socket,
    request_xair_adjust_position,
    verify_xair_dependency,
)
from .configuration import CollectionConfig
from .dataset import (
    DirectLeRobotEpisode,
    LeRobotBackendFactory,
    identity_from_config,
    inspect_direct_dataset,
    provenance_from_config,
)
from .orchestration import EpisodeResult, complete_episode
from .schema import CameraSample, CollectionSample, SampleAssembler, normalize_task

_LEVEL = re.compile(r"^(INFO|STEP|PASS|WARN|FAIL)\s+(.*)$")


class RuntimeSession(Protocol):
    def __enter__(self) -> RuntimeSession: ...

    def wait_until_ready(self, receiver: XAirStateReceiver) -> None: ...

    def require_running(self) -> None: ...

    def adjust_position(self, receiver: XAirStateReceiver) -> None: ...

    def __exit__(self, exc_type, exc, traceback) -> None: ...


class CameraCaptureSource(Protocol):
    def capture(self, *, timeout_s: float) -> dict[str, CameraSample]: ...


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

    def adjust_position(self, receiver: XAirStateReceiver) -> None:
        """Run the SDK alignment routine on both sides and require fresh paired state."""

        if len(self._runtimes) != 2:
            raise RuntimeError("both x_air runtimes must be active before AdjustPosition")
        self._raise_if_runtime_exited()
        console.step("Resetting both teleoperation sides with x_air AdjustPosition")
        system = self._config.system
        with ThreadPoolExecutor(max_workers=2, thread_name_prefix="vlai-adjust-position") as pool:
            requests = {
                side: pool.submit(request_xair_adjust_position, system, side)
                for side in ("left", "right")
            }
            for side in ("left", "right"):
                try:
                    requests[side].result()
                except BaseException as exc:
                    raise RuntimeError(f"{side} AdjustPosition failed: {exc}") from exc
        self._raise_if_runtime_exited()
        receiver.reset_pairing()
        timeout_s = system.teleoperation.startup_timeout_s
        deadline = time.monotonic() + timeout_s
        while True:
            self._raise_if_runtime_exited()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    "fresh paired robot state did not resume after AdjustPosition "
                    f"within {timeout_s:.1f}s"
                )
            if receiver.receive(timeout_s=min(0.25, remaining)) is not None:
                console.success("AdjustPosition complete; fresh paired state resumed")
                return

    def require_running(self) -> None:
        self._raise_if_runtime_exited()

    def __exit__(self, exc_type, exc, traceback) -> None:
        self._stop_all()

    def _raise_if_runtime_exited(self) -> None:
        for runtime in self._runtimes:
            status = runtime.process.poll()
            if status is not None:
                detail = runtime.output[-1] if runtime.output else "no runtime output"
                raise RuntimeError(
                    f"{runtime.side} teleoperation exited unexpectedly "
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
        cleanup_failures = []
        for side in ("left", "right"):
            try:
                remove_orphaned_xair_control_socket(system, side)
            except (OSError, RuntimeError) as exc:
                cleanup_failures.append(f"{side}: {exc}")
        self._runtimes.clear()
        if cleanup_failures:
            raise RuntimeError(
                "teleoperation control endpoint cleanup failed: " + "; ".join(cleanup_failures)
            )
        console.success("Teleoperation stopped; managed CAN links are disabled and down")


def collect_managed_session(
    config: CollectionConfig,
    *,
    experiment: str,
    task: str,
    runtime_factory: type[RuntimeSession] = ManagedXAirRuntimes,
) -> tuple[EpisodeResult, ...]:
    """Collect operator-bounded episodes in one persistent teleoperation session."""

    task = normalize_task(task)
    identity = identity_from_config(config, experiment)
    provenance = provenance_from_config(config)
    console.info(f"Collection session · experiment={experiment} · task={task}")
    console.step("Validating the atomic dataset destination")
    inspect_direct_dataset(
        identity,
        expected_task=task,
        expected_provenance=provenance,
    )
    backend_factory = LeRobotBackendFactory(config.image_writer_threads)
    backend_factory.verify_dependency()
    console.step("Starting or verifying persistent camera service")
    camera_status = CameraServiceController(config.system).start()
    console.success(
        f"Three-camera service ready · preview port "
        f"{config.system.camera_preview.port} · pid {camera_status.pid}"
    )
    console.step("Authorizing realtime lifecycle")
    ManagedXAirRuntimes.authorize()
    receiver = XAirStateReceiver(config.system)
    from .live import LiveCollectionSource

    source = LiveCollectionSource(
        config,
        state_source=receiver,
    )
    results: list[EpisodeResult] = []

    console.step("Preflighting state socket and three cameras")
    with source:
        _preflight_cameras(source.camera_source, config)
        console.success("Three cameras are fresh, synchronized, and ready")
        with runtime_factory(config) as runtimes:
            runtimes.wait_until_ready(receiver)
            while True:
                if not _wait_for_episode_start(
                    lambda: runtimes.adjust_position(receiver),
                    runtimes.require_running,
                ):
                    break
                console.step("Rechecking cameras after operator confirmation")
                _preflight_cameras(source.camera_source, config)
                console.success("Three cameras are fresh, synchronized, and ready to record")
                sink = DirectLeRobotEpisode(
                    identity=identity,
                    task=task,
                    provenance=provenance,
                    backend_factory=backend_factory,
                )
                console.step("Preparing atomic episode transaction")
                with sink:
                    captured_frames, decision = _record_interactive_episode(
                        samples=source.samples(),
                        assembler=SampleAssembler(config),
                        sink=sink,
                        task=task,
                        target_fps=config.fps,
                        minimum_fps=config.minimum_capture_fps,
                    )
                    result = _complete_interactive_episode(
                        sink,
                        captured_frames=captured_frames,
                        decision=decision,
                    )
                _report_episode_result(result)
                if result.decision is EpisodeDecision.QUIT:
                    break
                results.append(result)
                runtimes.adjust_position(receiver)

    console.info(
        f"Collection stopped · saved="
        f"{sum(item.decision is EpisodeDecision.SAVE for item in results)} "
        f"· discarded="
        f"{sum(item.decision is EpisodeDecision.DISCARD for item in results)}"
    )
    return tuple(results)


def _complete_interactive_episode(
    sink: DirectLeRobotEpisode,
    *,
    captured_frames: int,
    decision: EpisodeDecision,
) -> EpisodeResult:
    if captured_frames == 0:
        sink.discard()
        if decision is EpisodeDecision.SAVE:
            console.warning("Empty episode discarded instead of publishing")
            decision = EpisodeDecision.DISCARD
        return EpisodeResult(decision, 0, None)
    action = "Saving episode and encoding videos"
    if decision is not EpisodeDecision.SAVE:
        action = f"Discarding {captured_frames} captured frames"
    console.step(action)
    return complete_episode(
        sink=sink,
        frame_count=captured_frames,
        decision=decision,
    )


def _report_episode_result(result: EpisodeResult) -> None:
    if result.decision is EpisodeDecision.SAVE:
        console.success(f"Saved {result.frame_count} frames to {result.dataset_root}")
        return
    verb = "quit after" if result.decision is EpisodeDecision.QUIT else "discarded"
    console.success(f"Captured and {verb} {result.frame_count} frames")


def _wait_for_episode_start(
    reset_position: Callable[[], None],
    require_runtime: Callable[[], None],
    read_command: Callable[[], str | None] | None = None,
) -> bool:
    """Wait until the operator confirms the teleoperated episode start pose."""

    if (
        not callable(reset_position)
        or not callable(require_runtime)
        or (read_command is not None and not callable(read_command))
    ):
        raise TypeError("episode-start callbacks must be callable")
    if read_command is None:
        read_command = _poll_stdin_line
    console.step("Use teleoperation to place the robot at the episode start pose")
    console.info("Enter=start recording, r=reset with AdjustPosition, q=quit")
    while True:
        announce_input(("start", "reset", "quit"))
        require_runtime()
        command = read_command()
        if command is None:
            time.sleep(0.05)
            continue
        command = command.strip().lower()
        if not command:
            console.success("Operator confirmed the episode start pose")
            return True
        if command in {"r", "reset"}:
            reset_position()
            console.step("Use teleoperation to place the robot at the episode start pose")
            console.info("Enter=start recording, r=reset with AdjustPosition, q=quit")
            continue
        if command in {"q", "quit", "exit"}:
            return False
        console.warning("Press Enter to start, enter r to reset, or enter q to quit")


def reset_managed_teleoperation(config: CollectionConfig) -> None:
    """Run the SDK startup AdjustPosition lifecycle and shut both sides down."""

    if config.system.teleoperation.blockers:
        blockers = ", ".join(config.system.teleoperation.blockers)
        raise RuntimeError(f"teleoperation reset is unavailable: {blockers}")
    ManagedXAirRuntimes.authorize()
    receiver = XAirStateReceiver(config.system)
    with receiver, ManagedXAirRuntimes(config) as runtimes:
        runtimes.wait_until_ready(receiver)
    console.success("Standalone teleoperation reset complete; startup alignment succeeded")


def _record_interactive_episode(
    *,
    samples: Iterable[tuple[CollectionSample, int]],
    assembler: SampleAssembler,
    sink: DirectLeRobotEpisode,
    task: str,
    target_fps: int,
    minimum_fps: float,
) -> tuple[int, EpisodeDecision]:
    """Record until the operator explicitly saves, discards, or quits."""

    console.step(f"Recording episode · target {target_fps} FPS")
    console.warning("Enter=save, d+Enter=discard, q+Enter=quit")
    announce_input(("save", "discard", "quit"))
    status = console.LiveStatusLine()
    started = time.monotonic()
    captured = 0
    decision: EpisodeDecision | None = None
    announce_progress(
        "collection",
        "Recording episode",
        0,
        None,
        phase="capture",
        detail=f"minimum {minimum_fps:.2f} FPS",
        force=True,
    )
    try:
        for sample, now_ns in samples:
            command = _poll_stdin_line()
            if command is not None:
                try:
                    decision = normalize_episode_decision(command)
                except ValueError:
                    console.warning("Press Enter to save, enter d to discard, or enter q to quit")
                    announce_input(("save", "discard", "quit"))
                else:
                    break
            sink.add_frame(assembler.validate(sample, now_ns=now_ns).lerobot_frame(task=task))
            captured += 1
            status.update(f"Recording frame {captured}")
            announce_progress(
                "collection",
                "Recording episode",
                captured,
                None,
                phase="capture",
                detail=f"frame {captured}",
            )
    finally:
        status.close()
    elapsed_s = time.monotonic() - started
    effective_fps = captured / elapsed_s if elapsed_s > 0 else 0.0
    if decision is None:
        raise RuntimeError("live sample stream ended before an operator decision")
    announce_progress(
        "collection",
        "Recording episode",
        captured,
        None,
        phase="complete" if decision is EpisodeDecision.SAVE else decision.value,
        detail=f"{effective_fps:.2f} FPS",
        force=True,
    )
    console.info(
        f"Captured {captured} frames in {elapsed_s:.2f}s "
        f"· {effective_fps:.2f} FPS · {decision.value}"
    )
    if captured > 0 and effective_fps < minimum_fps:
        raise RuntimeError(
            f"capture rate {effective_fps:.2f} FPS is below the tracked "
            f"minimum {minimum_fps:.2f} FPS"
        )
    return captured, decision


def _poll_stdin_line() -> str | None:
    try:
        readable, _, _ = select.select([sys.stdin], [], [], 0)
    except (OSError, ValueError):
        return None
    if not readable:
        return None
    line = sys.stdin.readline()
    if line == "":
        return "q"
    return line.strip().lower()


def _preflight_cameras(cameras: CameraCaptureSource, config: CollectionConfig) -> None:
    validator = CameraSetValidator(config.system.cameras)
    for _ in range(3):
        samples = cameras.capture(timeout_s=config.max_sample_age_s)
        validator.validate(
            {role: sample.metadata for role, sample in samples.items()},
            now_ns=time.monotonic_ns(),
        )


def _pump_runtime_output(side: str, stream: TextIO, output: deque[str]) -> None:
    for raw in stream:
        line = raw.strip()
        if not line:
            continue
        output.append(line)
        match = _LEVEL.fullmatch(line)
        if match is not None:
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
