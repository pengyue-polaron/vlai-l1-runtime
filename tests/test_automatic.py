from __future__ import annotations

import copy
import io
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.teleoperation.automatic import (
    render_native_spec,
    run_automatic,
    validate_policy,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")
POLICY_PATH = ROOT / "configs/diagnostics/right_automatic_motion.json"
POLICY = json.loads(POLICY_PATH.read_text())


def test_pure_render_uses_tracked_identity_gains_and_ranges():
    rows = render_native_spec(CONFIG, POLICY).splitlines()
    assert rows[1] == "can2"
    assert len(rows) == 12
    assert rows[-1] == "0"
    motor = rows[3].split()
    assert motor[:5] == ["joint_1", "1", "17", "1", "4"]
    assert list(map(float, motor[5:10])) == [12.5, 45, 54, 240, 3]
    assert list(map(float, rows[5].split()[5:8])) == [12.5, 10, 28]
    assert CONFIG.safety.command_ready is False


def test_handling_parameters_are_explicit_and_preserve_motion_limits():
    policy = json.loads((ROOT / "configs/diagnostics/right_joint4_paired.json").read_text())
    rows = render_native_spec(CONFIG, policy)
    assert rows.splitlines()[-1].split() == [
        "1",
        "100000000",
        "2000000000",
        "20000000000",
        "5.0",
        "0.05",
        "15",
        "15",
        "15",
        "15",
        "45",
        "45",
        "45",
        "15",
    ]
    assert policy["maximum_observed_speed_deg_s"] == [15] * 8


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("hard_speed_deg_s", [45] * 7),
        ("hard_speed_deg_s", [float("inf")] * 8),
        ("hard_speed_deg_s", [1] * 8),
        ("hard_speed_deg_s", [10000] * 8),
        ("speed_persistence_seconds", 1),
        ("settle_seconds", 0.05),
        ("settle_timeout_seconds", 1),
        ("settle_speed_deg_s", 15),
        ("settle_span_deg", 0.01),
        ("settle_span_deg", 1),
    ],
)
def test_handling_rejects_invalid_limits(field, value):
    policy = json.loads((ROOT / "configs/diagnostics/right_joint4_paired.json").read_text())
    policy["manual_handling"][field] = value
    with pytest.raises(ValueError, match="manual"):
        validate_policy(CONFIG, policy)


def test_handling_rejected_outside_paired_before_hardware(tmp_path):
    with pytest.raises(ValueError, match="requires paired"):
        run_automatic(
            CONFIG,
            ROOT / "configs/diagnostics/right_joint4_paired.json",
            tmp_path / "out",
            mode="sweep",
        )
    assert not (tmp_path / "out").exists()


@pytest.mark.parametrize(("motor", "index"), [("joint_4", 3), ("joint_6", 5), ("joint_7", 6)])
def test_selected_motor_is_explicit_in_versioned_native_input(motor, index):
    policy = copy.deepcopy(POLICY)
    policy["active_motor"] = motor
    rows = render_native_spec(CONFIG, policy).splitlines()
    assert rows[0] == "VLAI_RIGHT_FOLLOWER_DIAGNOSTIC_V3"
    assert int(rows[2].split()[-1]) == index
    policy["maximum_relative_excursion_deg"][index] = 0.1
    with pytest.raises(ValueError, match="envelope"):
        validate_policy(CONFIG, policy)


@pytest.mark.parametrize("motor", ["joint_1", "joint_2", "joint_3", "joint_5", "gripper", None])
def test_uncommissioned_motor_selection_is_rejected(motor):
    with pytest.raises(ValueError, match="only"):
        validate_policy(CONFIG, {**POLICY, "active_motor": motor})


@pytest.mark.parametrize(("joint", "amplitude"), [(4, 0.25), (6, 0.5)])
@pytest.mark.parametrize(("stage", "cycles"), [("single", 1), ("repeat", 2)])
def test_other_joint_commissioning_preserves_existing_slow_guards(joint, amplitude, stage, cycles):
    base = json.loads(
        (ROOT / "configs/diagnostics/right_automatic_slow_halfdegree.json").read_text()
    )
    selected = json.loads(
        (ROOT / f"configs/diagnostics/right_joint{joint}_slow_{stage}.json").read_text()
    )
    validate_policy(CONFIG, selected)
    assert selected["active_motor"] == f"joint_{joint}"
    assert selected["amplitude_deg"] == amplitude
    assert selected["cycles"] == cycles
    for key in set(base) - {"active_motor", "amplitude_deg", "cycles", "description"}:
        assert selected[key] == base[key]


def test_slow_comparison_preserves_commissioned_guards():
    slow = json.loads(
        (ROOT / "configs/diagnostics/right_automatic_slow_halfdegree.json").read_text()
    )
    validate_policy(CONFIG, slow)
    assert {key for key in slow if slow[key] != POLICY[key]} == {
        "transition_seconds",
        "description",
    }
    assert slow["transition_seconds"] == 2 * POLICY["transition_seconds"]


def test_slow_repeat_changes_only_cycle_count():
    root = ROOT / "configs/diagnostics"
    single = json.loads((root / "right_automatic_slow_halfdegree.json").read_text())
    repeated = json.loads((root / "right_automatic_slow_repeat.json").read_text())
    validate_policy(CONFIG, repeated)
    assert {key for key in repeated if repeated[key] != single[key]} == {"cycles", "description"}
    assert repeated["cycles"] == 2


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("side", "left"),
        ("role", "leader"),
        ("active_motor", "joint_2"),
        ("amplitude_deg", float("nan")),
        ("amplitude_deg", 3),
        ("amplitude_deg", True),
        ("maximum_motor_torque_nm", [100] * 8),
        ("maximum_tracking_error_deg", [1] * 7),
        ("transition_seconds", 0.01),
        ("raw_log_capacity", 8),
        ("cycles", True),
        ("disabled_raw_status", 1),
        ("maximum_temperature_c", True),
    ],
)
def test_unsafe_policy_rejected(key, value):
    policy = copy.deepcopy(POLICY)
    policy[key] = value
    with pytest.raises(
        ValueError,
        match=r"only|finite|envelope|range|values|exceeds|capacity|integer|differ|temperature",
    ):
        validate_policy(CONFIG, policy)


@pytest.fixture
def fake_hardware(monkeypatch):
    import os
    import selectors
    import subprocess

    from vlai_l1_runtime import automatic_review, unloaded_baseline
    from vlai_l1_runtime.teleoperation import can_probe, feedback_capture, lifecycle, xair

    events = []
    responses = [(0, b'{"state":"disabled"}\n')]
    baseline = {
        "ifname": "can2",
        "parentdev": "1-3.1.2:1.0",
        "operstate": "DOWN",
        "linkinfo": {
            "info_data": {"state": "STOPPED", "berr_counter": {"tx": 0, "rx": 0}},
            "info_xstats": {"bus_off": 0},
        },
    }

    class Lock:
        def fileno(self):
            return 123

        def close(self):
            events.append("lock_closed")

    class Selector:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def register(self, *_args):
            pass

    class Child:
        def __init__(self, command, **_kwargs):
            self.mode = command[-4]
            events.append(f"start:{self.mode}")
            self.code, payload = responses.pop(0)
            self.stdout = io.BytesIO(payload)

        def poll(self):
            return self.code

        def wait(self, **_kwargs):
            events.append(f"reaped:{self.mode}")
            return self.code

    original_read = unloaded_baseline.read_artifact
    monkeypatch.setattr(
        unloaded_baseline,
        "read_artifact",
        lambda p: b"fake native" if p.name == "vlai_l1_automatic_follower" else original_read(p),
    )
    monkeypatch.setattr(os, "geteuid", lambda: 0)
    monkeypatch.setattr(lifecycle, "_acquire_lifecycle_lock", lambda *_a, **_k: Lock())
    for name in ("_require_exclusive_control", "_require_interfaces_down", "_require_can_identity"):
        monkeypatch.setattr(lifecycle, name, lambda *_a: None)
    monkeypatch.setattr(lifecycle, "_configure_can", lambda *_a: events.append("configured"))
    monkeypatch.setattr(feedback_capture, "_require_services_inactive", lambda *_a: None)
    monkeypatch.setattr(feedback_capture, "_link", lambda *_a: copy.deepcopy(baseline))
    monkeypatch.setattr(feedback_capture, "_require_healthy", lambda *_a: None)
    monkeypatch.setattr(feedback_capture, "_down", lambda *_a: events.append("down"))
    monkeypatch.setattr(
        xair,
        "verify_xair_dependency",
        lambda *_a: SimpleNamespace(revision="fake", can_library=Path("fake.so")),
    )
    monkeypatch.setattr(
        can_probe, "probe_motor_feedback", lambda *_a, **_k: SimpleNamespace(rounds=40)
    )
    monkeypatch.setattr(subprocess, "Popen", Child)
    monkeypatch.setattr(selectors, "DefaultSelector", Selector)
    monkeypatch.setattr(automatic_review, "review_automatic", lambda *_a: {"status": "passed"})
    return SimpleNamespace(events=events, responses=responses, baseline=baseline)


def test_normal_lifecycle_verifies_disable_before_down(tmp_path, fake_hardware):
    report = run_automatic(CONFIG, POLICY_PATH, tmp_path / "normal", mode="hold")
    assert report["status"] == "passed"
    assert report["disabled_verified"] is True
    assert fake_hardware.events == [
        "configured",
        "start:hold",
        "reaped:hold",
        "down",
        "lock_closed",
    ]


def test_unconfirmed_shutdown_uses_only_sequential_disable_owner(tmp_path, fake_hardware):
    fake_hardware.responses[:] = [(1, b"FAIL native fault\n"), (0, b'{"state":"disabled"}\n')]
    report = run_automatic(CONFIG, POLICY_PATH, tmp_path / "failure", mode="sweep")
    assert report["status"] == "failed"
    assert report["disabled_verified"] is True
    assert "native:native fault" in report["error"]
    assert fake_hardware.events == [
        "configured",
        "start:sweep",
        "reaped:sweep",
        "start:disable",
        "reaped:disable",
        "down",
        "lock_closed",
    ]


def test_native_fault_after_disabled_marker_is_preserved_without_retry(tmp_path, fake_hardware):
    fake_hardware.responses[:] = [
        (1, b'{"state":"disabled"}\nFAIL position/speed/effort/temperature guard:joint_7\n')
    ]
    report = run_automatic(CONFIG, POLICY_PATH, tmp_path / "guard_fault", mode="sweep")
    assert report["status"] == "failed"
    assert report["disabled_verified"] is True
    assert "native:position/speed/effort/temperature guard:joint_7" in report["error"]
    assert fake_hardware.events == [
        "configured",
        "start:sweep",
        "reaped:sweep",
        "down",
        "lock_closed",
    ]


def test_hold_finish_is_forwarded_once_after_running_and_verified(
    tmp_path, fake_hardware, monkeypatch
):
    import os
    import selectors
    import signal
    import subprocess

    output = tmp_path / "interactive_hold"

    class InteractiveChild:
        def __init__(self, _command, **_kwargs):
            self.code = None
            reader, self.writer = os.pipe()
            self.stdout = os.fdopen(reader, "rb", buffering=0)
            (output / "finish.request").write_bytes(b"finish\n")
            os.write(self.writer, b'{"state":"running"}\n')

        def poll(self):
            return self.code

        def send_signal(self, sig):
            assert sig == signal.SIGUSR1
            fake_hardware.events.append("finish_signal")
            os.write(self.writer, b'{"state":"disabled"}\n')
            os.write(
                self.writer,
                json.dumps(
                    {
                        "state": "completed",
                        "disabled_verified": True,
                        "completion_kind": "requested_hold",
                        "elapsed_seconds": 5.0,
                    }
                ).encode()
                + b"\n",
            )
            os.close(self.writer)
            self.code = 0

        def wait(self, **_kwargs):
            fake_hardware.events.append("reaped:hold")
            return self.code

    monkeypatch.setattr(subprocess, "Popen", InteractiveChild)
    monkeypatch.setattr(selectors, "DefaultSelector", selectors.EpollSelector)
    policy = ROOT / "configs/diagnostics/right_automatic_operator_hold.json"
    report = run_automatic(CONFIG, policy, output, mode="hold")
    assert report["status"] == "passed"
    assert report["disabled_verified"] is True
    assert report["completed_hold_seconds"] == 5.0
    assert report["operator_finish_requested"] is True
    assert fake_hardware.events.count("finish_signal") == 1
    assert fake_hardware.events.index("reaped:hold") < fake_hardware.events.index("down")


def test_paired_acknowledgements_advance_only_the_current_wait(
    tmp_path, fake_hardware, monkeypatch
):
    import os
    import selectors
    import signal
    import subprocess

    output = tmp_path / "paired"

    class Child:
        def __init__(self, _command, **_kwargs):
            self.code = None
            self.count = 0
            r, self.w = os.pipe()
            self.stdout = os.fdopen(r, "rb", buffering=0)
            os.write(self.w, b'{"state":"phase","phase":"awaiting_attach"}\n')

        def poll(self):
            status = output / "status.json"
            if self.code is None and status.exists():
                phase = json.loads(status.read_text()).get("phase")
                if phase == "awaiting_attach":
                    (output / "attach.ready").write_bytes(b"attached\n")
                if phase == "awaiting_remove":
                    (output / "remove.ready").write_bytes(b"removed\n")
            return self.code

        def send_signal(self, sig):
            assert sig == signal.SIGUSR2
            self.count += 1
            fake_hardware.events.append("advance")
            if self.count == 1:
                os.write(self.w, b'{"state":"phase","phase":"awaiting_remove"}\n')
            else:
                os.write(
                    self.w,
                    b'{"state":"disabled"}\n{"state":"completed","completion_kind":"paired_sweep","disabled_verified":true,"elapsed_seconds":330}\n',
                )
                os.close(self.w)
                self.code = 0

        def wait(self, **_kwargs):
            fake_hardware.events.append("reaped:paired")
            return self.code

    monkeypatch.setattr(subprocess, "Popen", Child)
    monkeypatch.setattr(selectors, "DefaultSelector", selectors.EpollSelector)
    report = run_automatic(
        CONFIG, ROOT / "configs/diagnostics/right_joint4_paired.json", output, mode="paired"
    )
    assert report["status"] == "passed"
    assert report["paired_acknowledgements"] == ["attach.ready", "remove.ready"]
    assert fake_hardware.events.count("advance") == 2
    assert fake_hardware.events.index("reaped:paired") < fake_hardware.events.index("down")


@pytest.mark.parametrize("premature", [True, False])
def test_invalid_paired_ack_stops_without_advancing_or_retrying(
    tmp_path, fake_hardware, monkeypatch, premature
):
    import os
    import selectors
    import subprocess

    output = tmp_path / "invalid_paired"

    class Child:
        def __init__(self, command, **_kwargs):
            self.mode = command[-4]
            fake_hardware.events.append(f"start:{self.mode}")
            r, self.writer = os.pipe()
            self.stdout = os.fdopen(r, "rb", buffering=0)
            self.code = None
            if self.mode == "disable":
                os.write(self.writer, b'{"state":"disabled"}\n')
                os.close(self.writer)
                self.code = 0
            else:
                phase = "baseline" if premature else "awaiting_attach"
                os.write(
                    self.writer, json.dumps({"state": "phase", "phase": phase}).encode() + b"\n"
                )

        def poll(self):
            if self.code is None and (output / "status.json").exists():
                (output / "attach.ready").write_bytes(b"attached\n" if premature else b"invalid\n")
            return self.code

        def send_signal(self, _sig):
            pytest.fail("invalid acknowledgement must not advance motion")

        def terminate(self):
            fake_hardware.events.append(f"terminate:{self.mode}")
            os.close(self.writer)
            self.code = 1

        def wait(self, **_kwargs):
            fake_hardware.events.append(f"reaped:{self.mode}")
            return self.code

    monkeypatch.setattr(subprocess, "Popen", Child)
    monkeypatch.setattr(selectors, "DefaultSelector", selectors.EpollSelector)
    report = run_automatic(
        CONFIG, ROOT / "configs/diagnostics/right_joint4_paired.json", output, mode="paired"
    )
    assert report["status"] == "failed"
    assert report["disabled_verified"] is True
    assert "invalid or premature paired acknowledgement" in report["error"]
    assert fake_hardware.events.count("start:paired") == 1
    assert fake_hardware.events.index("reaped:paired") < fake_hardware.events.index("start:disable")
    assert fake_hardware.events.index("reaped:disable") < fake_hardware.events.index("down")


@pytest.mark.parametrize(
    ("initial_state", "delay", "repeat_disabled", "native_error", "expected_error"),
    [
        ("disabled", 2.0, False, False, None),
        ("disabled", 9.0, False, False, "native_finalization_timeout"),
        ("disabled", 9.0, True, False, "native_finalization_timeout"),
        ("running", 2.0, False, False, "native_heartbeat_timeout"),
        ("disabled", 2.0, False, True, "native:sample output failed"),
    ],
)
def test_finalization_has_fixed_deadline_after_verified_disable(
    tmp_path,
    fake_hardware,
    monkeypatch,
    initial_state,
    delay,
    repeat_disabled,
    native_error,
    expected_error,
):
    import os
    import selectors
    import subprocess
    import time

    clock = [0.0]
    children = []

    class BufferedChild:
        def __init__(self, _command, **_kwargs):
            self.code = None
            reader, self.writer = os.pipe()
            self.stdout = os.fdopen(reader, "rb", buffering=0)
            os.write(self.writer, json.dumps({"state": initial_state}).encode() + b"\n")
            children.append(self)

        def finish(self, code):
            self.code = code
            os.write(self.writer, b'{"state":"disabled"}\n')
            os.write(
                self.writer,
                b"FAIL sample output failed\n"
                if code
                else b'{"state":"completed","disabled_verified":true}\n',
            )
            os.close(self.writer)

        def poll(self):
            if self.code is None and clock[0] >= delay:
                self.finish(int(native_error))
            return self.code

        def terminate(self):
            fake_hardware.events.append("terminate")
            self.finish(0)

        def wait(self, **_kwargs):
            fake_hardware.events.append("reaped:hold")
            return self.code

    class ClockSelector(selectors.EpollSelector):
        def select(self, timeout=None):
            clock[0] += 0.25
            if repeat_disabled and children[0].code is None:
                os.write(children[0].writer, b'{"state":"disabled"}\n')
            return super().select(timeout=0)

    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(subprocess, "Popen", BufferedChild)
    monkeypatch.setattr(selectors, "DefaultSelector", ClockSelector)
    report = run_automatic(CONFIG, POLICY_PATH, tmp_path / "finalization", mode="hold")
    assert report["disabled_verified"] is True
    assert len(children) == 1
    assert fake_hardware.events.index("reaped:hold") < fake_hardware.events.index("down")
    if expected_error:
        assert report["status"] == "failed"
        assert expected_error in report["error"]
    else:
        assert report["status"] == "passed"
        assert "terminate" not in fake_hardware.events


def test_startup_error_never_runs_probe_or_native(tmp_path, fake_hardware):
    fake_hardware.baseline["linkinfo"]["info_data"]["berr_counter"]["tx"] = 128
    report = run_automatic(CONFIG, POLICY_PATH, tmp_path / "startup_fault", mode="hold")
    assert report["status"] == "failed"
    assert "startup CAN fault" in report["error"]
    assert fake_hardware.events == ["lock_closed"]


def test_failed_preflight_never_enables_or_retries(tmp_path, fake_hardware, monkeypatch):
    from vlai_l1_runtime.teleoperation import can_probe

    def failed(*_args, **_kwargs):
        raise RuntimeError("missing motor")

    monkeypatch.setattr(can_probe, "probe_motor_feedback", failed)
    report = run_automatic(CONFIG, POLICY_PATH, tmp_path / "probe_fault", mode="hold")
    assert report["status"] == "failed"
    assert "missing motor" in report["error"]
    assert fake_hardware.events == ["configured", "down", "lock_closed"]


def test_failed_cleanup_never_claims_disabled(tmp_path, fake_hardware):
    fake_hardware.responses[:] = [(1, b"FAIL\n"), (1, b"FAIL disable\n")]
    report = run_automatic(CONFIG, POLICY_PATH, tmp_path / "cleanup_fault", mode="hold")
    assert report["status"] == "failed"
    assert report["disabled_verified"] is False
    assert "intervention" in report["error"]


def test_existing_output_refused_before_hardware(tmp_path, fake_hardware):
    with pytest.raises(FileExistsError):
        run_automatic(CONFIG, POLICY_PATH, tmp_path, mode="hold")
    assert fake_hardware.events == []


def test_sequence_runs_sweep_only_after_hold_review(tmp_path, monkeypatch):
    from vlai_l1_runtime.teleoperation import automatic

    calls = []

    def run(_config, _policy, _output, *, mode):
        calls.append(mode)
        return {"status": "passed", "disabled_verified": True, "analysis_status": "passed"}

    monkeypatch.setattr(automatic, "run_automatic", run)
    result = automatic.run_automatic_sequence(CONFIG, POLICY_PATH, tmp_path / "sequence")
    assert result["status"] == "passed"
    assert calls == ["hold", "sweep"]


@pytest.mark.parametrize(
    ("field", "value"),
    [("status", "failed"), ("disabled_verified", False), ("analysis_status", "failed")],
)
def test_sequence_stops_without_retry_on_failed_gate(tmp_path, monkeypatch, field, value):
    from vlai_l1_runtime.teleoperation import automatic

    calls = []

    def run(_config, _policy, _output, *, mode):
        calls.append(mode)
        return {
            "status": "passed",
            "disabled_verified": True,
            "analysis_status": "passed",
            field: value,
        }

    monkeypatch.setattr(automatic, "run_automatic", run)
    result = automatic.run_automatic_sequence(CONFIG, POLICY_PATH, tmp_path / "sequence")
    assert result["status"] == "failed"
    assert calls == ["hold"]


def test_review_failure_keeps_hardware_disabled(tmp_path, fake_hardware, monkeypatch):
    from vlai_l1_runtime import automatic_review

    def fail(*_args):
        raise ValueError("bad data")

    monkeypatch.setattr(automatic_review, "review_automatic", fail)
    result = run_automatic(CONFIG, POLICY_PATH, tmp_path / "bad_review", mode="hold")
    assert result["status"] == "failed"
    assert result["disabled_verified"] is True
    assert "bad data" in result["error"]
    assert fake_hardware.events.count("start:hold") == 1
