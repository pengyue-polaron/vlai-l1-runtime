"""Hardware-free failure injection for the existing disable/down cleanup."""

import subprocess
from pathlib import Path

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.teleoperation import lifecycle

CONFIG = load_system_config(Path(__file__).resolve().parents[1] / "configs/system/vlai_l1.toml")


@pytest.mark.parametrize("fault", [None, "send", "missing", "up", "down", "still_up"])
def test_cleanup_checks_failures_and_always_attempts_both_links(monkeypatch, fault):
    commands = []
    checked = []

    def run(command, **kwargs):
        assert kwargs["check"] is True
        commands.append(command)
        if command[0] == "cansend" and command[1] == "can0":
            if fault == "send" and command[2].startswith("001"):
                raise subprocess.CalledProcessError(1, command)
            if fault == "missing":
                raise FileNotFoundError("cansend")
        if command[0] == "ip" and command[3] == "can0" and command[-1] == fault:
            raise subprocess.CalledProcessError(1, command)

    def link_up(interface):
        checked.append(interface)
        return fault == "still_up" and interface == "can0"

    monkeypatch.setattr(lifecycle.subprocess, "run", run)
    monkeypatch.setattr(lifecycle.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(lifecycle, "_read_link_is_up", link_up)
    if fault is None:
        lifecycle._disable_and_close(CONFIG, ("can0", "can2"))
    else:
        with pytest.raises(RuntimeError, match="disable/down failed"):
            lifecycle._disable_and_close(CONFIG, ("can0", "can2"))
    assert [c[3] for c in commands if c[0] == "ip" and c[-1] == "down"] == ["can0", "can2"]
    for interface in ("can0", "can2"):
        sent = [c for c in commands if c[0] == "cansend" and c[1] == interface]
        assert len(sent) == (0 if fault == "up" and interface == "can0" else 8)
        assert all(c[2].endswith("FFFFFFFFFFFFFFFD") for c in sent)
    assert checked == (["can2"] if fault == "down" else ["can0", "can2"])
