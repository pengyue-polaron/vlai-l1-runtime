"""Real TaskController + native simulator process; never an SDK/CAN executable."""

import json
import os
import selectors
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.task_runtime import FEATURES, TaskController, render_task_spec

ROOT = Path(__file__).resolve().parents[1]


def simulator():
    for path in (
        ROOT / "build/xair-sidecar/vlai_l1_task_session_sim",
        ROOT / "build/task-session-safety/vlai_l1_task_session_sim",
    ):
        if path.is_file():
            return path
    pytest.skip("hardware-free vlai_l1_task_session_sim has not been built")


@pytest.mark.parametrize("recorded", [False, True], ids=["synthetic64", "recorded32"])
def test_task_controller_native_process_policy_clock_then_persistent_return_hold(
    tmp_path, recorded
):
    binary = simulator()
    config = load_system_config(ROOT / "configs/system/vlai_l1.toml")
    manifest = json.loads((ROOT / "configs/tasks/push-distance.json").read_text())
    policy = json.loads((ROOT / manifest["automatic_policy"]).read_text())
    fixture = json.loads((ROOT / "tests/fixtures/recorded_policy_20261109.json").read_text())
    initial = fixture["start_deg"] if recorded else [0.0] * 8
    action_count = 32 if recorded else 64
    task = SimpleNamespace(
        id="offline-push-distance",
        root=tmp_path,
        start_pose=tmp_path / "pose.json",
        pose={"follower_deg": dict(zip(FEATURES, initial, strict=True))},
        expected_checkpoint={"kind": "fake-predictor-offline-only"},
        settings={
            "observation_max_age_s": 8.0,
            "producer_timeout_s": 0.5,
            "arrival_tolerance_deg": manifest["settings"]["arrival_tolerance_deg"],
        },
    )
    calls = []

    def predict():
        offset = 32 * len(calls)
        stamp = time.monotonic_ns()
        calls.append(stamp)
        if recorded:
            return fixture["actions"], stamp
        chunk = []
        for index in range(32):
            row = initial.copy()
            row[6] = (offset + index + 1) * 0.00001
            chunk.append(row)
        return chunk, stamp

    spec = tmp_path / "task.spec"
    spec.write_text(render_task_spec(config, policy, task))
    pose = tmp_path / "initial8.txt"
    pose.write_text(" ".join(map(str, initial)))
    controller = TaskController(config, task, policy, tmp_path, predictor=predict)
    command = [str(binary), str(spec), str(pose)]
    if recorded:
        # Explicit kinematic fake feedback, not motor physics or hardware safety
        # evidence. The simulator uses the configured task feedback guards.
        command.extend(("--plant", "bounded-slew"))
    child = subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    selector = selectors.DefaultSelector()
    selector.register(child.stdout, selectors.EVENT_READ)
    buffer = b""
    states = []
    commands = []
    terminal = None
    requested = False
    returned_at = None
    deadline = time.monotonic() + (45 if recorded else 15)
    try:
        while time.monotonic() < deadline:
            assert child.poll() is None, child.stderr.read().decode()
            now = time.monotonic_ns()
            if controller.phase == "ready" and not requested:
                controller.request("policy", actions=action_count)
                requested = True
            line = controller.tick(now)
            if line is not None:
                commands.append(line.split())
                child.stdin.write(line.encode())
                child.stdin.flush()
            # Same <=20ms select / os.read / newline parsing as automatic's
            # supervisor. Inference uses the real separate worker thread.
            for key, _mask in selector.select(timeout=0.02):
                chunk = os.read(key.fd, 65536)
                assert chunk, "native simulator output closed during task"
                buffer += chunk
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    event = json.loads(raw)
                    if event["state"] == "task_state":
                        states.append(event)
                        controller.on_state(event)
            assert controller.phase != "policy_rejected_hold", controller.status()
            if controller.phase == "returned_hold":
                returned_at = returned_at or time.monotonic()
                assert controller.state["reference_deg"] == initial
                if time.monotonic() - returned_at >= 2:
                    break
        else:
            pytest.fail(f"native controller integration deadline: {controller.status()}")
        assert controller.actions_sent == action_count
        assert controller.chunks_received == len(calls) == action_count // 32
        operations = [parts[4] for parts in commands]
        assert operations.count("start") == operations.count("return") == 1
        assert operations.count("target") == action_count
        assert all(float(p[5]) == 1 / 30 for p in commands if p[4] == "target")
        assert {s["owner_pid"] for s in states} == {child.pid}
        targets = {
            int(parts[1]): list(map(float, parts[6:])) for parts in commands if parts[4] == "target"
        }
        applied = {}
        for state in states:
            seq = state["applied_sequence"]
            if seq in targets and seq not in applied:
                applied[seq] = state
        assert list(applied) == list(targets), "native skipped/reordered a predicted target"
        for seq, event in applied.items():
            assert event["reference_deg"] == pytest.approx(targets[seq], abs=1e-12)
        if recorded:
            assert list(targets.values()) == fixture["actions"]
        application_times = [event["monotonic_ns"] for event in applied.values()]
        applied_span_s = (application_times[-1] - application_times[0]) / 1e9
        assert abs(applied_span_s - (action_count - 1) / 30) < 0.05
        assert max(event["policy_gap_count"] for event in states) == 0
        assert not controller.shutdown_requested
        assert child.poll() is None
        child.stdin.close()  # Explicit test teardown; EOF is a native fault.
        remainder = child.stdout.read()
        stderr = child.stderr.read().decode()
        assert child.wait(timeout=3) == 1
        for raw in (buffer + remainder).splitlines():
            event = json.loads(raw)
            if event["state"] == "simulation_terminal":
                terminal = event
        assert terminal is not None
        assert terminal["owner_pid"] == child.pid
        assert terminal["enable_calls"] == terminal["disable_calls"] == 1
        if recorded:
            assert terminal["simulated_plant"] == "kinematic_rate_limited_not_physics"
        assert "task producer EOF" in stderr
        evidence = {
            "simulation_only": True,
            "real_task_controller": True,
            "normal_completion_owner_alive": True,
            "terminal_fault_expected_from_test_stdin_eof": True,
            "native_process": terminal,
            "actions_sent": controller.actions_sent,
            "chunks_received": controller.chunks_received,
            "target_duration_s": 1 / 30,
            "applied_target_count": len(applied),
            "applied_target_span_s": applied_span_s,
            "recorded_policy_replay": recorded,
            "policy_gap_count": max(event["policy_gap_count"] for event in states),
            "return_duration_s": next(
                float(parts[5]) for parts in commands if parts[4] == "return"
            ),
            "applied_targets": [
                {
                    "sequence": seq,
                    "monotonic_ns": event["monotonic_ns"],
                    "reference_deg": event["reference_deg"],
                }
                for seq, event in applied.items()
            ],
            "returned_hold_alive_s": time.monotonic() - returned_at,
            "transit_speed_limit_deg_s": policy["maximum_reference_speed_deg_s"],
            "transit_acceleration_limit_deg_s2": policy["maximum_reference_acceleration_deg_s2"],
            "final_reference_deg": controller.state["reference_deg"],
            "feedback_events": len(states),
        }
        (tmp_path / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    finally:
        controller.close()
        selector.close()
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=3)
