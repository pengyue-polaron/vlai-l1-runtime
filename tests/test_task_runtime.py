"""Task orchestration against deterministic native events, with no devices/threads."""

import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import pytest

from vlai_l1_runtime import task_runtime as runtime
from vlai_l1_runtime.configuration import load_system_config

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")
FEATURES = runtime.FEATURES


class ImmediateExecutor:
    def __init__(self, **kwargs):
        pass

    def submit(self, function):
        future = Future()
        try:
            future.set_result(function())
        except Exception as error:
            future.set_exception(error)
        return future

    def shutdown(self, **kwargs):
        pass


class NativeEvents:
    """One owner, one active segment, one pending segment, explicit completion."""

    def __init__(self, controller):
        self.controller = controller
        self.now = 1_000_000_000
        self.q = [0.0] * 8
        self.reference = list(self.q)
        self.accepted = self.completed = 0
        self.active = self.pending = None
        self.commands = []
        self.owner_pid = 12345

    def state(self):
        if self.active is not None and self.now >= self.active["end"]:
            self.q = self.reference = list(self.active["q"])
            self.completed = self.active["seq"]
            scheduled_end = self.active["end"]
            self.active = self.pending
            self.pending = None
            if self.active is not None:
                self.active["end"] = scheduled_end + self.active["duration"]
        return {
            "state": "task_state",
            "owner_pid": self.owner_pid,
            "monotonic_ns": self.now,
            "earliest_motor_response_ns": self.now,
            "q_deg": list(self.q),
            "reference_deg": list(self.reference),
            "accepted_sequence": self.accepted,
            "completed_sequence": self.completed,
            "pending_sequence": 0 if self.pending is None else self.pending["seq"],
            "phase": "holding" if self.active is None else "moving",
        }

    def step(self):
        self.now += 20_000_000
        self.controller.on_state(self.state())
        line = self.controller.tick(self.now)
        if line is None:
            return None
        parts = line.split()
        assert len(parts) == 14
        assert parts[0] == "VLAI_TASK_V1"
        sequence, source, observation = map(int, parts[1:4])
        op, duration = parts[4], float(parts[5])
        q = list(map(float, parts[6:]))
        assert source == self.now
        assert observation <= self.now
        self.accepted = sequence
        self.commands.append((op, q, sequence))
        if op in ("start", "target", "return"):
            if op == "return":
                q = [self.controller.task.pose["follower_deg"][key] for key in FEATURES]
            segment = {
                "seq": sequence,
                "q": q,
                "duration": round(duration * 1e9),
                "end": self.now + round(duration * 1e9),
            }
            if self.active is None:
                self.active = segment
            else:
                assert self.pending is None, "controller overfilled native pending slot"
                self.pending = segment
        elif op == "hold":
            self.pending = None
        else:
            assert op == "heartbeat"
        return op

    def until(self, phase, maximum=1000):
        for _ in range(maximum):
            self.step()
            if self.controller.phase == phase:
                return
        pytest.fail(f"did not reach {phase}: {self.controller.status()}")


@pytest.fixture
def setup_controller(monkeypatch, tmp_path):
    monkeypatch.setattr(runtime, "ThreadPoolExecutor", ImmediateExecutor)
    task = SimpleNamespace(
        id="push-distance",
        root=tmp_path,
        start_pose=tmp_path / "data/start_poses/push-distance.json",
        pose={"follower_deg": dict.fromkeys(FEATURES, 0.0)},
        expected_checkpoint={"kind": "pi0", "path": "/checkpoint", "deployment_sha256": "a" * 64},
        settings={
            "observation_max_age_s": 8.0,
            "producer_timeout_s": 0.25,
        },
    )
    policy = {
        "maximum_relative_excursion_deg": [2.0] * 8,
        "maximum_reference_speed_deg_s": 10.0,
        "maximum_reference_acceleration_deg_s2": 10.0,
        "maximum_tracking_error_deg": [1.0] * 8,
    }
    calls = []
    simulation = None

    def predict():
        start = len(calls) * 32
        calls.append(simulation.now)
        chunk = []
        for i in range(32):
            q = [0.0] * 8
            q[6] = (start + i + 1) * 0.001
            chunk.append(q)
        return chunk, simulation.now

    controller = runtime.TaskController(CONFIG, task, policy, tmp_path, predictor=predict)
    simulation = NativeEvents(controller)
    simulation.until("ready")
    yield controller, simulation, calls
    controller.close()


def test_multiple_policy_chunks_keep_one_owner_then_return_and_hold(setup_controller):
    controller, simulation, calls = setup_controller
    controller.request("policy", actions=64)
    simulation.until("returned_hold")
    assert controller.actions_sent == 64
    assert controller.chunks_received >= 2
    assert len(calls) >= 2
    assert [op for op, _, _ in simulation.commands].count("start") == 1
    assert [op for op, _, _ in simulation.commands].count("target") == 64
    assert [op for op, _, _ in simulation.commands].count("return") == 1
    assert simulation.q == [0.0] * 8
    offset = len(simulation.commands)
    for _ in range(100):
        simulation.step()
    assert controller.phase == "returned_hold"
    assert not controller.shutdown_requested
    assert controller.status()["owner_pid"] == 12345
    assert {op for op, _, _ in simulation.commands[offset:]} == {"heartbeat"}


@pytest.mark.parametrize("fault", ["stale", "future", "shape", "bounds", "nan"])
def test_invalid_prediction_cannot_issue_target_and_retains_owner_hold(setup_controller, fault):
    controller, simulation, _ = setup_controller

    def bad_prediction():
        chunk = [[0.0] * 8 for _ in range(32)]
        stamp = simulation.now
        if fault == "stale":
            stamp -= 9_000_000_000
        elif fault == "future":
            stamp += 1_000_000_000
        elif fault == "shape":
            chunk[-1].pop()
        elif fault == "bounds":
            chunk[0][4] = 96.0
        elif fault == "nan":
            chunk[0][6] = float("nan")
        return chunk, stamp

    controller.predictor = bad_prediction
    controller.request("policy", actions=32)
    simulation.until("policy_rejected_hold")
    for _ in range(20):
        simulation.step()
    assert not any(op == "target" for op, _, _ in simulation.commands)
    assert any(op == "hold" for op, _, _ in simulation.commands)
    assert not controller.shutdown_requested
    assert controller.status()["owner_pid"] == 12345


def test_finish_cancels_inflight_result_before_any_model_action(setup_controller):
    controller, simulation, _ = setup_controller
    pending = Future()
    controller._executor = SimpleNamespace(submit=lambda _: pending, shutdown=lambda **_: None)
    controller.request("policy", actions=64)
    simulation.step()
    assert controller._future is pending
    controller.request("finish")
    pending.set_result(([[0.0] * 8 for _ in range(32)], simulation.now))
    simulation.until("returned_hold")
    assert not any(op == "target" for op, _, _ in simulation.commands)
    assert controller.actions_sent == 0
    assert not controller.shutdown_requested


def test_finish_during_motion_clears_pending_then_returns_with_same_owner(setup_controller):
    controller, simulation, _ = setup_controller
    controller.request("policy", actions=64)
    for _ in range(100):
        simulation.step()
        if simulation.active is not None and simulation.pending is not None:
            break
    else:
        pytest.fail("policy never filled the native motion queue")
    commands_before_finish = len(simulation.commands)
    controller.request("finish")
    simulation.until("returned_hold")
    commands_after_finish = [op for op, _, _ in simulation.commands[commands_before_finish:]]
    assert commands_after_finish[0] == "hold"
    assert "target" not in commands_after_finish
    assert commands_after_finish.count("return") == 1
    assert controller.actions_sent < 64
    assert controller.status()["owner_pid"] == 12345
    assert simulation.q == [0.0] * 8


@pytest.mark.parametrize(
    "payload", ["{", "[]", '{"operation":"policy","actions":true}', '{"operation":"unknown"}']
)
def test_bad_operator_request_is_rejected_without_releasing_hold(setup_controller, payload):
    controller, simulation, _ = setup_controller
    request_path = controller.output / "request.json"
    request_path.write_text(payload)
    controller.service_request()
    result = json.loads((controller.output / "request-result.json").read_text())
    assert result["accepted"] is False
    assert controller.phase == "ready"
    assert not controller.shutdown_requested
    assert not request_path.exists()
    simulation.step()


def test_directory_request_cannot_throw_out_of_supervisor_and_disable_owner(setup_controller):
    controller, simulation, _ = setup_controller
    (controller.output / "request.json").mkdir()
    controller.service_request()
    result = json.loads((controller.output / "request-result.json").read_text())
    assert result["accepted"] is False
    assert controller.phase == "ready"
    assert not controller.shutdown_requested
    simulation.step()


def test_owner_replacement_and_stale_feedback_fail_closed(setup_controller):
    controller, simulation, _ = setup_controller
    event = simulation.state()
    event["owner_pid"] += 1
    event["monotonic_ns"] += 1
    with pytest.raises(ValueError, match="owner changed"):
        controller.on_state(event)
    with pytest.raises(TimeoutError, match="stale"):
        controller.tick(simulation.now + 100_000_001)


def test_policy_request_has_no_artificial_action_count_cap(setup_controller):
    controller, _, _ = setup_controller
    controller.request("policy", actions=9001)
    assert controller.actions_remaining == 9001


def test_default_policy_continues_across_chunks_until_explicit_finish(setup_controller):
    controller, simulation, calls = setup_controller
    controller.request("policy")
    for _ in range(200):
        simulation.step()
        if controller.actions_sent >= 70:
            break
    assert controller.actions_sent >= 70
    assert controller.chunks_received >= 3
    assert len(calls) >= 3
    assert controller.phase == "policy"
    assert controller.actions_remaining is None
    assert not any(op == "return" for op, _, _ in simulation.commands)
    controller.request("finish")
    simulation.until("returned_hold")
    assert simulation.q == [0.0] * 8
    assert controller.status()["owner_pid"] == 12345
    assert not controller.shutdown_requested


def test_completed_start_becomes_ready_on_first_arrival_feedback(setup_controller):
    controller, simulation, _ = setup_controller
    controller.phase = "going_to_start"
    event = simulation.state()
    event["monotonic_ns"] += 1
    event["earliest_motor_response_ns"] = event["monotonic_ns"]
    event["completed_sequence"] = controller.start_sequence
    event["phase"] = "holding"
    controller.on_state(event)
    assert controller.phase == "ready"


def test_recorded_policy_passes_without_diagnostic_excursion_or_quintic_point_gates(
    setup_controller,
):
    controller, simulation, _ = setup_controller
    record = json.loads((ROOT / "tests/fixtures/recorded_policy_20261109.json").read_text())
    assert record["feature_names"] == list(FEATURES)
    assert abs(record["actions"][-1][0] - record["start_deg"][0]) > 3
    assert 1.875 * abs(record["actions"][0][6] - record["start_deg"][6]) / (1 / 30) > 1
    for row in record["actions"]:
        runtime.check_target(CONFIG, row)
    controller.task.pose["follower_deg"] = dict(zip(FEATURES, record["start_deg"], strict=True))
    controller.destination = list(record["start_deg"])
    simulation.q = simulation.reference = list(record["start_deg"])
    controller.task.settings["observation_max_age_s"] = 0.05
    controller.predictor = lambda: (record["actions"], simulation.now)
    controller.request("policy", actions=32)
    for _ in range(100):
        simulation.step()
        if controller.actions_sent == 32:
            break
    emitted = [q for op, q, _ in simulation.commands if op == "target"]
    assert emitted == record["actions"]
    assert controller.error is None


def test_checkpoint_mismatch_is_rejected_before_camera_or_prediction(monkeypatch):
    from vlai_l1_runtime import camera_ipc

    spec = SimpleNamespace(response_timeout_s=1)
    monkeypatch.setattr(runtime.Pi0Spec, "load", lambda _: spec)
    monkeypatch.setattr(runtime, "replace", lambda value, **_: value)
    monkeypatch.setattr(runtime, "Pi0ObservationBuilder", lambda *_, **__: None)
    closed = []

    class Client:
        def __init__(self, *_):
            self.server_metadata = {"checkpoint": {"kind": "pi0", "path": "/wrong"}}

        def __enter__(self):
            return self

        def close(self):
            closed.append(True)

    monkeypatch.setattr(runtime, "Pi0PredictionClient", Client)
    monkeypatch.setattr(
        camera_ipc,
        "RawCameraBridgeClient",
        lambda *_: pytest.fail("camera opened before identity check"),
    )
    task = SimpleNamespace(
        inference_spec=None,
        prompt="push",
        distance_cm=10,
        expected_checkpoint={"kind": "pi0", "path": "/correct", "deployment_sha256": "a" * 64},
        settings={"observation_max_age_s": 1},
    )
    predictor = runtime.LivePredictor(CONFIG, task, lambda _: None)
    for _ in range(2):
        with pytest.raises(ValueError, match="checkpoint"):
            predictor()
    assert len(closed) == 2


@pytest.mark.parametrize("failure_stage", ["camera", "prediction"])
def test_explicit_retry_opens_fresh_client_after_prediction_or_camera_failure(
    monkeypatch, failure_stage
):
    from vlai_l1_runtime import camera_ipc

    expected = {"kind": "pi0", "path": "/checkpoint", "deployment_sha256": "a" * 64}
    spec = SimpleNamespace(response_timeout_s=1)
    monkeypatch.setattr(runtime.Pi0Spec, "load", lambda _: spec)
    monkeypatch.setattr(runtime, "replace", lambda value, **_: value)
    observation = SimpleNamespace(oldest_source_ns=123)
    monkeypatch.setattr(
        runtime,
        "Pi0ObservationBuilder",
        lambda *_, **__: SimpleNamespace(build=lambda *_, **__: observation),
    )
    created, closed, camera_closed = [], [], []

    class Client:
        def __init__(self, *_):
            created.append(self)
            self.server_metadata = {"checkpoint": expected}
            self.number = len(created)

        def __enter__(self):
            return self

        def predict(self, _observation):
            if self.number == 1 and failure_stage == "prediction":
                raise TimeoutError("injected prediction timeout")
            return SimpleNamespace(tolist=lambda: [[0.0] * 8 for _ in range(32)]), b"reply"

        def close(self):
            closed.append(self.number)

    class Cameras:
        def __init__(self, *_):
            self.number = len(created)

        def __enter__(self):
            return self

        def capture(self, **kwargs):
            if self.number == 1 and failure_stage == "camera":
                raise TimeoutError("injected camera timeout")
            return {
                name: SimpleNamespace(metadata=SimpleNamespace(monotonic_ns=123))
                for name in ("agent", "wrist_right")
            }

        def __exit__(self, *args):
            camera_closed.append(self.number)

    monkeypatch.setattr(runtime, "Pi0PredictionClient", Client)
    monkeypatch.setattr(camera_ipc, "RawCameraBridgeClient", Cameras)
    task = SimpleNamespace(
        inference_spec=None,
        prompt="push",
        distance_cm=10,
        expected_checkpoint=expected,
        settings={"observation_max_age_s": 1},
    )
    predictor = runtime.LivePredictor(CONFIG, task, lambda _: None)
    with pytest.raises(TimeoutError, match="injected"):
        predictor()
    assert len(created) == 1  # No automatic reconnect on the failed request.
    assert closed == camera_closed == [1]
    assert predictor.client is None
    assert predictor.cameras is None
    actions, stamp = predictor()  # Next explicit request.
    assert len(created) == 2
    assert actions == [[0.0] * 8 for _ in range(32)]
    assert stamp == 123
    predictor.close()
    assert closed == camera_closed == [1, 2]


@pytest.mark.parametrize(
    ("phase", "complete"), [("going_to_start", "ready"), ("returning", "returned_hold")]
)
def test_relaxed_tracking_guard_does_not_relax_arrival(setup_controller, phase, complete):
    controller, simulation, _ = setup_controller
    controller.task.settings["arrival_tolerance_deg"] = 2.0
    controller.policy["maximum_tracking_error_deg"] = [3.0] * 8
    controller.phase = phase
    controller.return_sequence = controller.start_sequence
    event = simulation.state()
    event["monotonic_ns"] += 1
    event["earliest_motor_response_ns"] = event["monotonic_ns"]
    event["completed_sequence"] = controller.start_sequence
    event["phase"] = "holding"
    event["q_deg"][0] = 2.5
    controller.on_state(event)
    assert controller.phase == phase
    with pytest.raises(ValueError, match="arrival"):
        controller.request("policy")
    event = {**event, "q_deg": event["q_deg"].copy()}
    event["monotonic_ns"] += 1
    event["earliest_motor_response_ns"] = event["monotonic_ns"]
    event["q_deg"][0] = 1.9
    controller.on_state(event)
    assert controller.phase == complete
