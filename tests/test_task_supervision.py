"""Original automatic task launch composition, with every resource call mocked."""

import io
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from test_automatic import fake_hardware as fake_hardware

from vlai_l1_runtime import task_runtime
from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.teleoperation import can_probe, lifecycle
from vlai_l1_runtime.teleoperation.automatic import run_automatic

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")
POLICY_PATH = ROOT / "configs/diagnostics/right_automatic_motion.json"


@pytest.mark.parametrize("probe_failure", [False, True])
def test_task_launch_uses_only_original_follower_owner_and_never_retries_enable(
    fake_hardware, monkeypatch, tmp_path, probe_failure
):
    monkeypatch.setattr(os, "chown", lambda *_, **__: None)
    monkeypatch.setattr(os, "set_blocking", lambda *_, **__: None)
    monkeypatch.setattr(signal, "signal", lambda *_, **__: None)
    monkeypatch.setattr(time, "sleep", lambda *_: None)
    monkeypatch.setattr(lifecycle, "_invoking_identity", lambda: (1000, 1000))
    probes, launches, closed = [], [], []

    def probe(config, interface, library, **kwargs):
        probes.append(interface)
        if probe_failure:
            raise RuntimeError("injected preflight failure")
        return SimpleNamespace(rounds=40)

    monkeypatch.setattr(can_probe, "probe_motor_feedback", probe)

    class Input(io.BytesIO):
        def fileno(self):
            return 456

    class Child:
        def __init__(self, command, **kwargs):
            launches.append((command, kwargs))
            self.stdout = io.BytesIO(b'{"state":"disabled"}\n')
            self.stdin = Input()

        def poll(self):
            return 0

        def wait(self, **kwargs):
            return 0

    monkeypatch.setattr(subprocess, "Popen", Child)

    class Controller:
        def __init__(self, *args):
            pass

        def status(self):
            return {"phase": "returned_hold"}

        def close(self):
            closed.append(True)

    monkeypatch.setattr(task_runtime, "TaskController", Controller)
    pose = tmp_path / "pose.json"
    pose.write_text(json.dumps({"follower_deg": [0] * 8}))
    task = SimpleNamespace(
        id="push-distance",
        start_pose=pose,
        expected_checkpoint={"kind": "pi0"},
        settings={"observation_max_age_s": 1, "producer_timeout_s": 0.25},
    )
    report = run_automatic(CONFIG, POLICY_PATH, tmp_path / "run", mode="task", task=task)
    assert probes == ["can2"]
    assert closed == [True]
    if probe_failure:
        assert not launches
        assert report["status"] == "failed"
        assert "preflight failure" in report["error"]
    else:
        assert len(launches) == 1
        command, kwargs = launches[0]
        assert command[3].endswith("build/xair-sidecar/vlai_l1_automatic_follower")
        assert command[5] == "task"
        assert kwargs["stdin"] == subprocess.PIPE
        assert report["disabled_verified"] is True
        assert report["status"] == "passed"
        spec = (tmp_path / "run/native_spec.txt").read_text()
        assert spec.startswith("VLAI_RIGHT_FOLLOWER_TASK_V2\ncan2\n")
        assert "analysis_status" not in report
