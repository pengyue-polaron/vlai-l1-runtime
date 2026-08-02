from __future__ import annotations

import io
import threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest
from embodied_ops import EpisodeDecision

from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.managed import (
    ManagedXAirRuntimes,
    _pump_runtime_output,
    _record_interactive_episode,
    _release_bimanual_startup,
    _wait_for_episode_start,
    collect_managed_session,
)
from vlai_l1_runtime.collection.orchestration import EpisodeResult

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_collection_config(ROOT / "configs/collection/default.toml")
RIGHT_CONFIG = load_collection_config(ROOT / "configs/collection/right_only.toml")


@pytest.fixture(autouse=True)
def _stub_persistent_camera_service(monkeypatch) -> None:
    class Controller:
        def __init__(self, _config):
            pass

        def start(self):
            return SimpleNamespace(pid=4321)

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.CameraServiceController",
        Controller,
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.remove_orphaned_xair_control_socket",
        lambda _config, _side: None,
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.LeRobotBackendFactory.verify_dependency",
        lambda _factory: None,
    )


class _Process:
    _next_pid = 1200

    def __init__(self, side: str = "left") -> None:
        self.stdin = io.StringIO()
        self.stdout = io.StringIO(
            f"PASS x_air {side} motion-free preflight ready\nPASS runtime ready\n"
        )
        self.returncode: int | None = None
        self.pid = self._next_pid
        type(self)._next_pid += 1

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        assert timeout is None or timeout >= 0
        self.returncode = 0
        return 0

    def kill(self) -> None:
        self.returncode = -9


class _StartupInput:
    def __init__(self) -> None:
        self.closed = False
        self.writes: list[str] = []

    def write(self, value: str) -> None:
        self.writes.append(value)

    def flush(self) -> None:
        return None

    def close(self) -> None:
        self.closed = True


def test_bimanual_startup_releases_both_motion_capable_constructors_together() -> None:
    runtimes = []
    for side in ("left", "right"):
        ready = threading.Event()
        ready.set()
        runtimes.append(
            SimpleNamespace(
                side=side,
                process=SimpleNamespace(
                    poll=lambda: None,
                    stdin=_StartupInput(),
                ),
                output=deque(),
                preflight_ready=ready,
            )
        )

    _release_bimanual_startup(runtimes, timeout_s=0.1)

    assert [runtime.process.stdin.writes for runtime in runtimes] == [
        ["START\n"],
        ["START\n"],
    ]
    assert all(runtime.process.stdin.closed for runtime in runtimes)


def test_bimanual_startup_never_releases_peer_when_one_preflight_exits() -> None:
    left_ready = threading.Event()
    left_ready.set()
    left_input = _StartupInput()
    runtimes = [
        SimpleNamespace(
            side="left",
            process=SimpleNamespace(poll=lambda: None, stdin=left_input),
            output=deque(["PASS x_air left motion-free preflight ready"]),
            preflight_ready=left_ready,
        ),
        SimpleNamespace(
            side="right",
            process=SimpleNamespace(poll=lambda: 1, stdin=_StartupInput()),
            output=deque(["FAIL can2 unhealthy"]),
            preflight_ready=threading.Event(),
        ),
    ]

    with pytest.raises(RuntimeError, match=r"right.*motion-free preflight"):
        _release_bimanual_startup(runtimes, timeout_s=0.1)

    assert left_input.writes == []


def test_episode_start_waits_for_an_explicit_operator_confirmation(monkeypatch) -> None:
    announcements: list[tuple[str, ...]] = []
    resets: list[str] = []
    responses = iter(("r", "not yet", ""))
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.announce_input",
        lambda actions: announcements.append(tuple(actions)),
    )

    assert _wait_for_episode_start(
        lambda: resets.append("reset"),
        lambda: None,
        lambda: next(responses),
    )
    assert resets == ["reset"]
    assert announcements == [
        ("start", "reset", "quit"),
        ("start", "reset", "quit"),
        ("start", "reset", "quit"),
    ]


def test_episode_start_can_quit_before_recording(monkeypatch) -> None:
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.announce_input",
        lambda _actions: None,
    )

    assert _wait_for_episode_start(lambda: None, lambda: None, lambda: "q") is False


def test_episode_start_fails_if_one_runtime_exits_while_waiting() -> None:
    def require_runtime() -> None:
        raise RuntimeError("right teleoperation exited")

    with pytest.raises(RuntimeError, match="right teleoperation exited"):
        _wait_for_episode_start(
            lambda: None,
            require_runtime,
            lambda: None,
        )


def test_managed_runtimes_stop_both_sides_and_run_disable_fallback(monkeypatch) -> None:
    processes: list[_Process] = []
    signals: list[tuple[int, str]] = []
    commands: list[tuple[str, ...]] = []
    cleaned_sides: list[str] = []

    def popen(command, **kwargs):
        side = command[command.index("--side") + 1]
        process = _Process(side)
        processes.append(process)
        return process

    monkeypatch.setattr("vlai_l1_runtime.collection.managed.os.geteuid", lambda: 0)
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.verify_xair_dependency",
        lambda config: None,
    )
    monkeypatch.setattr("vlai_l1_runtime.collection.managed.subprocess.Popen", popen)
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._signal_runtime",
        lambda sudo, process_group, signal_name: signals.append((process_group, signal_name)),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.subprocess.run",
        lambda command, **kwargs: commands.append(tuple(command)),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.remove_orphaned_xair_control_socket",
        lambda _config, side: cleaned_sides.append(side),
    )

    with pytest.raises(RuntimeError, match="capture failed"), ManagedXAirRuntimes(CONFIG):
        raise RuntimeError("capture failed")

    assert len(processes) == 2
    assert signals == [(processes[0].pid, "INT"), (processes[1].pid, "INT")]
    assert [command[1] for command in commands] == ["left_arm", "right_arm"]
    assert cleaned_sides == ["left", "right"]


def test_right_only_runtime_uses_isolation_and_never_disables_left(monkeypatch) -> None:
    processes: list[_Process] = []
    launches: list[tuple[str, ...]] = []
    disable_commands: list[tuple[str, ...]] = []
    cleaned_sides: list[str] = []

    def popen(command, **kwargs):
        del kwargs
        launches.append(tuple(command))
        process = _Process("right")
        processes.append(process)
        return process

    monkeypatch.setattr("vlai_l1_runtime.collection.managed.os.geteuid", lambda: 0)
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.verify_xair_dependency",
        lambda config: None,
    )
    monkeypatch.setattr("vlai_l1_runtime.collection.managed.subprocess.Popen", popen)
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._signal_runtime",
        lambda _sudo, _process_group, signal_name: setattr(processes[0], "returncode", 0),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.subprocess.run",
        lambda command, **kwargs: disable_commands.append(tuple(command)),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.remove_orphaned_xair_control_socket",
        lambda _config, side: cleaned_sides.append(side),
    )

    with pytest.raises(RuntimeError, match="capture failed"), ManagedXAirRuntimes(RIGHT_CONFIG):
        raise RuntimeError("capture failed")

    assert len(launches) == 1
    assert launches[0][launches[0].index("--side") + 1] == "right"
    assert "--isolated-side" in launches[0]
    assert "--managed-startup-gate" not in launches[0]
    assert [command[1] for command in disable_commands] == ["right_arm"]
    assert cleaned_sides == ["right"]


def test_partial_runtime_start_failure_stops_started_side(monkeypatch) -> None:
    process = _Process()
    attempts = 0
    signals: list[tuple[int, str]] = []
    commands: list[tuple[str, ...]] = []

    def popen(*args, **kwargs):
        nonlocal attempts
        attempts += 1
        if attempts == 2:
            raise OSError("cannot spawn right runtime")
        return process

    monkeypatch.setattr("vlai_l1_runtime.collection.managed.os.geteuid", lambda: 0)
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.verify_xair_dependency",
        lambda config: None,
    )
    monkeypatch.setattr("vlai_l1_runtime.collection.managed.subprocess.Popen", popen)
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._signal_runtime",
        lambda sudo, process_group, signal_name: signals.append((process_group, signal_name)),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.subprocess.run",
        lambda command, **kwargs: commands.append(tuple(command)),
    )

    with pytest.raises(OSError, match="cannot spawn right runtime"):
        ManagedXAirRuntimes(CONFIG).__enter__()

    assert signals == [(process.pid, "INT")]
    assert [command[1] for command in commands] == ["left_arm", "right_arm"]


def test_runtime_output_hides_vendor_chatter_but_preserves_it_for_failures(
    monkeypatch,
) -> None:
    emitted: list[tuple[str, str]] = []
    output: deque[str] = deque()
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.console.emit",
        lambda level, message: emitted.append((level, message)),
    )

    _pump_runtime_output("right", io.StringIO("vendor detail\nPASS runtime ready\n"), output)

    assert list(output) == ["vendor detail", "PASS runtime ready"]
    assert emitted == [("PASS", "RIGHT · runtime ready")]


def test_managed_runtimes_reset_both_sides_and_resume_fresh_pairing(monkeypatch) -> None:
    requested: list[str] = []
    receiver_events: list[str] = []
    manager = ManagedXAirRuntimes(CONFIG)
    manager._runtimes = [
        SimpleNamespace(
            side=side,
            process=SimpleNamespace(poll=lambda: None),
            output=[],
        )
        for side in ("left", "right")
    ]

    class Receiver:
        def reset_pairing(self):
            receiver_events.append("reset_pairing")

        def receive(self, *, timeout_s):
            assert 0 < timeout_s <= 0.25
            receiver_events.append("fresh_pair")
            return object()

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.request_xair_adjust_position",
        lambda _config, side: requested.append(side),
    )

    manager.adjust_position(Receiver())

    assert set(requested) == {"left", "right"}
    assert receiver_events == ["reset_pairing", "fresh_pair"]


def test_camera_startup_failure_never_starts_robot_runtimes(monkeypatch) -> None:
    entered_runtime = False

    class Controller:
        def __init__(self, _config):
            pass

        def start(self):
            raise RuntimeError("camera disappeared")

    class Runtime:
        def __init__(self, config):
            nonlocal entered_runtime
            entered_runtime = True

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.CameraServiceController",
        Controller,
    )

    with pytest.raises(RuntimeError, match="camera disappeared"):
        collect_managed_session(
            CONFIG,
            experiment="startup_failure",
            task="hold position",
            runtime_factory=Runtime,
        )

    assert entered_runtime is False


def test_managed_collection_keeps_runtime_and_resets_between_episodes(monkeypatch) -> None:
    events: list[str] = []
    assemblers: list[object] = []
    starts = iter((True, True, False))
    recordings = iter(
        (
            (10, EpisodeDecision.SAVE),
            (8, EpisodeDecision.DISCARD),
        )
    )

    class Source:
        def __init__(self, config, **kwargs):
            self.camera_source = object()

        def __enter__(self):
            events.append("source_enter")
            return self

        def samples(self):
            events.append("samples")
            return ()

        def __exit__(self, exc_type, exc, traceback):
            events.append("source_exit")

    class Runtime:
        def __init__(self, config):
            pass

        def __enter__(self):
            events.append("runtime_enter")
            return self

        def wait_until_ready(self, receiver):
            events.append("runtime_ready")

        def require_running(self):
            events.append("runtime_check")

        def adjust_position(self, receiver):
            events.append("runtime_adjust")

        def __exit__(self, exc_type, exc, traceback):
            events.append("runtime_exit")

    class Sink:
        def __enter__(self):
            events.append("sink_enter")
            return self

        def __exit__(self, exc_type, exc, traceback):
            events.append("sink_exit")

    monkeypatch.setattr(
        ManagedXAirRuntimes,
        "authorize",
        lambda: events.append("authorize"),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.XAirStateReceiver",
        lambda _config, **_kwargs: object(),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.live.LiveCollectionSource",
        Source,
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.DirectLeRobotEpisode",
        lambda **_kwargs: Sink(),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.inspect_direct_dataset",
        lambda *_args, **_kwargs: events.append("dataset_preflight"),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._preflight_cameras",
        lambda _cameras, _config: events.append("camera_preflight"),
    )

    def wait_for_start(_reset, require_runtime):
        require_runtime()
        start = next(starts)
        events.append("operator_start" if start else "operator_quit")
        return start

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._wait_for_episode_start",
        wait_for_start,
    )

    def record(**kwargs):
        events.append("record")
        assemblers.append(kwargs["assembler"])
        return next(recordings)

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._record_interactive_episode",
        record,
    )

    def complete(*, frame_count, decision, **_kwargs):
        events.append("complete")
        root = Path("/dataset") if decision is EpisodeDecision.SAVE else None
        return EpisodeResult(decision, frame_count, root)

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.complete_episode",
        complete,
    )

    results = collect_managed_session(
        CONFIG,
        experiment="operator_gate",
        task="hold position",
        runtime_factory=Runtime,
    )

    assert [result.decision for result in results] == [
        EpisodeDecision.SAVE,
        EpisodeDecision.DISCARD,
    ]
    assert len(assemblers) == 2
    assert assemblers[0] is not assemblers[1]
    assert events == [
        "dataset_preflight",
        "authorize",
        "source_enter",
        "camera_preflight",
        "runtime_enter",
        "runtime_ready",
        "runtime_check",
        "operator_start",
        "camera_preflight",
        "sink_enter",
        "samples",
        "record",
        "runtime_adjust",
        "complete",
        "sink_exit",
        "runtime_check",
        "operator_start",
        "camera_preflight",
        "sink_enter",
        "samples",
        "record",
        "runtime_adjust",
        "complete",
        "sink_exit",
        "runtime_check",
        "operator_quit",
        "runtime_exit",
        "source_exit",
    ]


def test_recording_decision_is_taken_during_capture(monkeypatch) -> None:
    commands = iter((None, "d"))
    times = iter((0.0, 0.1))
    announcements: list[tuple[str, ...]] = []

    class Validated:
        def lerobot_frame(self, *, task):
            return {"task": task}

    class Assembler:
        def validate(self, sample, *, now_ns):
            assert sample in {"first", "second"}
            assert now_ns in {1, 2}
            return Validated()

    class Sink:
        def __init__(self):
            self.frames = []

        def add_frame(self, frame):
            self.frames.append(frame)

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._poll_stdin_line",
        lambda: next(commands),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.announce_input",
        lambda actions: announcements.append(tuple(actions)),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.time.monotonic",
        lambda: next(times),
    )

    sink = Sink()
    captured, decision = _record_interactive_episode(
        samples=(("first", 1), ("second", 2)),
        assembler=Assembler(),
        sink=sink,
        task="place fruit",
        target_fps=30,
        minimum_fps=1.0,
    )

    assert (captured, decision) == (1, EpisodeDecision.DISCARD)
    assert sink.frames == [{"task": "place fruit"}]
    assert announcements == [("save", "discard", "quit")]


def test_interactive_capture_rejects_non_realtime_collection(monkeypatch) -> None:
    times = iter((0.0, 1.0))
    commands = iter((None, None, None, ""))

    class Validated:
        def lerobot_frame(self, *, task):
            return {"task": task}

    class Assembler:
        def validate(self, _sample, *, now_ns):
            assert now_ns in {0, 1, 2}
            return Validated()

    class Sink:
        def add_frame(self, _frame):
            return None

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._poll_stdin_line",
        lambda: next(commands),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.time.monotonic",
        lambda: next(times),
    )

    with pytest.raises(RuntimeError, match=r"3\.00 FPS.*minimum 27\.00 FPS"):
        _record_interactive_episode(
            samples=((object(), 0), (object(), 1), (object(), 2), (object(), 3)),
            assembler=Assembler(),
            sink=Sink(),
            task="place fruit",
            target_fps=30,
            minimum_fps=27.0,
        )


def test_capture_progress_is_published_to_the_panel(monkeypatch) -> None:
    events: list[tuple[object, ...]] = []
    times = iter((0.0, 0.1))
    commands = iter((None, None, None, ""))

    class Validated:
        def lerobot_frame(self, *, task):
            return {"task": task}

    class Assembler:
        def validate(self, _sample, *, now_ns):
            assert now_ns in {0, 1, 2}
            return Validated()

    class Sink:
        def add_frame(self, _frame):
            return None

    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.time.monotonic",
        lambda: next(times),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.announce_progress",
        lambda *args, **kwargs: events.append((*args, kwargs)),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._poll_stdin_line",
        lambda: next(commands),
    )

    assert _record_interactive_episode(
        samples=((object(), 0), (object(), 1), (object(), 2), (object(), 3)),
        assembler=Assembler(),
        sink=Sink(),
        task="place fruit",
        target_fps=30,
        minimum_fps=27.0,
    ) == (3, EpisodeDecision.SAVE)
    assert events[0] == (
        "collection",
        "Recording episode",
        0,
        None,
        {"phase": "capture", "detail": "minimum 27.00 FPS", "force": True},
    )
    assert events[-1] == (
        "collection",
        "Recording episode",
        3,
        None,
        {"phase": "complete", "detail": "30.00 FPS", "force": True},
    )
