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
    assert len(rows) == 11
    motor = rows[3].split()
    assert motor[:5] == ["joint_1", "1", "17", "1", "4"]
    assert list(map(float, motor[5:10])) == [12.5, 45, 54, 240, 3]
    assert list(map(float, rows[5].split()[5:8])) == [12.5, 10, 28]
    assert CONFIG.safety.command_ready is False


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
    assert fake_hardware.events == [
        "configured",
        "start:sweep",
        "reaped:sweep",
        "start:disable",
        "reaped:disable",
        "down",
        "lock_closed",
    ]


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
