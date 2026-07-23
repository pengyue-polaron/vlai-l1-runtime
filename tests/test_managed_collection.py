from __future__ import annotations

import io
from contextlib import nullcontext
from pathlib import Path

import pytest
from embodied_ops import EpisodeDecision

from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.managed import (
    ManagedXAirRuntimes,
    _wait_for_episode_start,
    _with_progress,
    collect_managed_episode,
)
from vlai_l1_runtime.collection.orchestration import EpisodeResult

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_collection_config(ROOT / "configs/collection/default.toml")


@pytest.fixture(autouse=True)
def _stub_camera_preview(monkeypatch) -> None:
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.CameraPreviewServer",
        lambda _config, _cameras: nullcontext(),
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
    responses = iter(("not yet", ""))
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.announce_input",
        lambda actions: announcements.append(tuple(actions)),
    )

    assert _wait_for_episode_start(lambda _prompt: next(responses)) is True
    assert announcements == [("enter",), ("enter",)]


def test_episode_start_can_quit_before_recording(monkeypatch) -> None:
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.announce_input",
        lambda _actions: None,
    )

    assert _wait_for_episode_start(lambda _prompt: "q") is False


def test_managed_runtimes_stop_both_sides_and_run_disable_fallback(monkeypatch) -> None:
    processes: list[_Process] = []
    signals: list[tuple[int, str]] = []
    commands: list[tuple[str, ...]] = []

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

    with pytest.raises(RuntimeError, match="capture failed"), ManagedXAirRuntimes(CONFIG):
        raise RuntimeError("capture failed")

    assert len(processes) == 2
    assert signals == [(processes[0].pid, "INT"), (processes[1].pid, "INT")]
    assert [command[1] for command in commands] == ["left_arm", "right_arm"]


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


def test_camera_startup_failure_never_starts_robot_runtimes(monkeypatch) -> None:
    entered_runtime = False

    class State:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            return None

    class Camera:
        def __enter__(self):
            raise RuntimeError("camera disappeared")

        def __exit__(self, exc_type, exc, traceback):
            return None

    class Runtime:
        def __init__(self, config):
            nonlocal entered_runtime
            entered_runtime = True

    class Sink:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            return None

    monkeypatch.setattr(ManagedXAirRuntimes, "authorize", lambda: None)
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.XAirStateReceiver",
        lambda config: State(),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.V4L2CameraSet",
        lambda config: Camera(),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.DirectLeRobotEpisode",
        lambda **kwargs: Sink(),
    )

    with pytest.raises(RuntimeError, match="camera disappeared"):
        collect_managed_episode(
            CONFIG,
            experiment="startup_failure",
            task="hold position",
            frame_count=3,
            decision="discard",
            runtime_factory=Runtime,
        )

    assert entered_runtime is False


def test_managed_collection_confirms_start_then_rechecks_cameras(monkeypatch) -> None:
    events: list[str] = []

    class Source:
        def __init__(self, config, **kwargs):
            pass

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

        def __exit__(self, exc_type, exc, traceback):
            events.append("runtime_exit")

    class Preview:
        def __init__(self, config, cameras):
            pass

        def __enter__(self):
            events.append("preview_enter")
            return self

        def __exit__(self, exc_type, exc, traceback):
            events.append("preview_exit")

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
        "vlai_l1_runtime.collection.managed.V4L2CameraSet",
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
        "vlai_l1_runtime.collection.managed.CameraPreviewServer",
        Preview,
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._preflight_cameras",
        lambda _cameras, _config: events.append("camera_preflight"),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed._wait_for_episode_start",
        lambda: events.append("operator_confirmed") or True,
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.write_episode_frames",
        lambda **_kwargs: events.append("write") or 3,
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
        frame_count=3,
        decision=EpisodeDecision.DISCARD,
        runtime_factory=Runtime,
    )

    assert result == EpisodeResult(EpisodeDecision.DISCARD, 3, None)
    assert events == [
        "authorize",
        "sink_enter",
        "source_enter",
        "preview_enter",
        "camera_preflight",
        "runtime_enter",
        "runtime_ready",
        "operator_confirmed",
        "camera_preflight",
        "samples",
        "write",
        "runtime_exit",
        "preview_exit",
        "source_exit",
        "complete",
        "sink_exit",
    ]


def test_capture_rate_gate_rejects_non_realtime_collection(monkeypatch) -> None:
    times = iter((0.0, 1.0))
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.time.monotonic",
        lambda: next(times),
    )

    with pytest.raises(RuntimeError, match=r"3\.00 FPS.*minimum 27\.00 FPS"):
        list(
            _with_progress(
                [(object(), 0), (object(), 1), (object(), 2)],
                3,
                minimum_fps=27.0,
            )
        )


def test_capture_progress_is_published_to_the_panel(monkeypatch) -> None:
    events: list[tuple[object, ...]] = []
    times = iter((0.0, 0.1))
    samples = [(object(), 0), (object(), 1), (object(), 2)]
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.time.monotonic",
        lambda: next(times),
    )
    monkeypatch.setattr(
        "vlai_l1_runtime.collection.managed.announce_progress",
        lambda *args, **kwargs: events.append((*args, kwargs)),
    )

    assert (
        list(
            _with_progress(
                samples,
                3,
                minimum_fps=27.0,
            )
        )
        == samples
    )
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
