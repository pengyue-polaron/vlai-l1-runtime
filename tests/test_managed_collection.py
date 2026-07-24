from __future__ import annotations

import io
from pathlib import Path
from types import SimpleNamespace

import pytest
from embodied_ops import EpisodeDecision

from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.managed import (
    ManagedXAirRuntimes,
    _record_interactive_episode,
    _wait_for_episode_start,
    collect_managed_episode,
    collect_managed_session,
)
from vlai_l1_runtime.collection.orchestration import EpisodeResult

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_collection_config(ROOT / "configs/collection/default.toml")


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


class _Process:
    _next_pid = 1200

    def __init__(self) -> None:
        self.stdout = io.StringIO("PASS runtime ready\n")
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
        lambda _prompt: next(responses),
    )
    assert resets == ["reset"]
    assert announcements == [
        ("enter", "reset", "quit"),
        ("enter", "reset", "quit"),
        ("enter", "reset", "quit"),
    ]


def test_episode_start_can_quit_before_recording(monkeypatch) -> None:
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.announce_input",
        lambda _actions: None,
    )

    assert _wait_for_episode_start(lambda: None, lambda _prompt: "q") is False


def test_managed_runtimes_stop_both_sides_and_run_disable_fallback(monkeypatch) -> None:
    processes: list[_Process] = []
    signals: list[tuple[int, str]] = []
    commands: list[tuple[str, ...]] = []
    cleaned_sides: list[str] = []

    def popen(*args, **kwargs):
        process = _Process()
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
        collect_managed_episode(
            CONFIG,
            experiment="startup_failure",
            task="hold position",
            runtime_factory=Runtime,
        )

    assert entered_runtime is False


def test_managed_collection_confirms_start_then_rechecks_cameras(monkeypatch) -> None:
    events: list[str] = []

    class Source:
        def __init__(self, config, **kwargs):
            self.camera_source = object()

        def __enter__(self):
            events.append("source_enter")
            return self

        def samples(self, frame_count):
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
        lambda _config: object(),
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
        "vlai_l1_runtime.collection.managed._preflight_cameras",
        lambda _cameras, _config: events.append("camera_preflight"),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._wait_for_episode_start",
        lambda reset: reset() or events.append("operator_confirmed") or True,
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._record_interactive_episode",
        lambda **_kwargs: events.append("record") or (3, EpisodeDecision.DISCARD),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.complete_episode",
        lambda **_kwargs: (
            events.append("complete") or EpisodeResult(EpisodeDecision.DISCARD, 3, None)
        ),
    )

    result = collect_managed_episode(
        CONFIG,
        experiment="operator_gate",
        task="hold position",
        runtime_factory=Runtime,
    )

    assert result == EpisodeResult(EpisodeDecision.DISCARD, 3, None)
    assert events == [
        "authorize",
        "sink_enter",
        "source_enter",
        "camera_preflight",
        "runtime_enter",
        "runtime_ready",
        "runtime_adjust",
        "operator_confirmed",
        "camera_preflight",
        "samples",
        "record",
        "runtime_exit",
        "source_exit",
        "complete",
        "sink_exit",
    ]


def test_collection_session_repeats_until_quit(monkeypatch) -> None:
    outcomes = iter(
        (
            EpisodeResult(EpisodeDecision.SAVE, 10, Path("/dataset")),
            EpisodeResult(EpisodeDecision.DISCARD, 8, None),
            EpisodeResult(EpisodeDecision.QUIT, 0, None),
        )
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.collect_managed_episode",
        lambda *_args, **_kwargs: next(outcomes),
    )

    results = collect_managed_session(
        CONFIG,
        experiment="multi_episode",
        task="place fruit",
    )

    assert [result.decision for result in results] == [
        EpisodeDecision.SAVE,
        EpisodeDecision.DISCARD,
    ]


def test_recording_decision_is_taken_during_capture(monkeypatch) -> None:
    commands = iter((None, "d"))
    times = iter((0.0, 0.1))

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
        "vlai_l1_runtime.collection.managed.time.monotonic",
        lambda: next(times),
    )

    sink = Sink()
    captured, decision = _record_interactive_episode(
        samples=(("first", 1), ("second", 2)),
        assembler=Assembler(),
        sink=sink,
        task="place fruit",
        total=300,
        target_fps=30,
        minimum_fps=1.0,
    )

    assert (captured, decision) == (1, EpisodeDecision.DISCARD)
    assert sink.frames == [{"task": "place fruit"}]


def test_interactive_capture_rejects_non_realtime_collection(monkeypatch) -> None:
    times = iter((0.0, 1.0))

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
        lambda: None,
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.time.monotonic",
        lambda: next(times),
    )

    with pytest.raises(RuntimeError, match=r"3\.00 FPS.*minimum 27\.00 FPS"):
        _record_interactive_episode(
            samples=((object(), 0), (object(), 1), (object(), 2)),
            assembler=Assembler(),
            sink=Sink(),
            task="place fruit",
            total=3,
            target_fps=30,
            minimum_fps=27.0,
        )


def test_capture_progress_is_published_to_the_panel(monkeypatch) -> None:
    events: list[tuple[object, ...]] = []
    times = iter((0.0, 0.1))

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
        lambda: None,
    )

    assert _record_interactive_episode(
        samples=((object(), 0), (object(), 1), (object(), 2)),
        assembler=Assembler(),
        sink=Sink(),
        task="place fruit",
        total=3,
        target_fps=30,
        minimum_fps=27.0,
    ) == (3, EpisodeDecision.SAVE)
    assert events[0] == (
        "collection",
        "Recording episode",
        0,
        3,
        {"phase": "capture", "detail": "minimum 27.00 FPS", "force": True},
    )
    assert events[-1] == (
        "collection",
        "Recording episode",
        3,
        3,
        {"phase": "complete", "detail": "30.00 FPS", "force": True},
    )
