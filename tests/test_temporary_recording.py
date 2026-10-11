from __future__ import annotations

import csv
import json
import math
import socket
import struct
import threading
import time
from contextlib import suppress
from dataclasses import replace
from pathlib import Path

import pytest

import vlai_l1_runtime.temporary_recording as recording
from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.teleoperation.xair import XAirStatePacket

ROOT = Path(__file__).resolve().parents[1]


def fake_camera(role, device, size, fps, output, gate, stop, events):
    """Hardware-free process double, deliberately not an actual camera recording."""
    events.put((role, "ready", None))
    count = 0
    while not stop.wait(0.01):
        if gate.is_set():
            count += 1
            events.put((role, "heartbeat", None))
    events.put((role, "done", {"frames": count, "effective_fps": 30.0}))


def test_camera_identity_parser_does_not_confuse_asic_serial():
    output = """Device info:
    Serial Number : 260322278153
    Physical Port : 1-3.1.3.1-25
    Asic Serial Number : 255323074499
Device info:
    Serial Number : 260422274280
    Physical Port : 2-1.4-7
"""
    assert recording.parse_camera_ports(output) == {
        "260322278153": "1-3.1.3.1",
        "260422274280": "2-1.4",
    }


def test_motor_columns_keep_all_eight_leader_and_follower_positions_in_degrees():
    packet = XAirStatePacket("right", 42, 100, (math.pi / 2,) * 8, (-math.pi,) * 8)
    row = recording.motor_row(packet, 120)
    assert len(row) == len(recording.motor_header()) == 19
    assert recording.motor_header()[10] == "leader_gripper_deg"
    assert recording.motor_header()[18] == "follower_gripper_deg"
    assert row[:3] == [42, 100, 120]
    assert row[3:11] == pytest.approx([90.0] * 8)
    assert row[11:] == pytest.approx([-180.0] * 8)


def test_motor_state_rejects_stale_future_and_repeated_feedback():
    packet = XAirStatePacket("left", 1, 1_000_000_000, (0.0,) * 8, (0.0,) * 8)
    recording.validate_packet(packet, None, 1_050_000_000, 0.1)
    for now in (900_000_000, 1_200_000_000):
        with pytest.raises(RuntimeError, match="stale or from the future"):
            recording.validate_packet(packet, None, now, 0.1)
    with pytest.raises(RuntimeError, match="did not increase"):
        recording.validate_packet(packet, packet, 1_050_000_000, 0.1)


@pytest.fixture
def isolated_recorder(tmp_path, monkeypatch):
    system = load_system_config(ROOT / "configs/system/vlai_l1.toml")
    path = tmp_path / "state.sock"
    system = replace(system, teleoperation=replace(system.teleoperation, state_socket_path=path))
    spec = tmp_path / "record.json"
    spec.write_text(
        json.dumps(
            {
                "system_config": "unused.toml",
                "startup_timeout_s": 2,
                "minimum_video_fps": 27,
                "cameras": {},
            }
        )
    )
    monkeypatch.setattr(recording, "load_system_config", lambda _: system)
    monkeypatch.setattr(recording, "verify_devices", lambda _: {"left": "fake1", "right": "fake2"})
    monkeypatch.setattr(recording, "camera_worker", fake_camera)
    return spec, path


@pytest.mark.parametrize("lose_right", [False, True])
def test_recording_tracks_both_motor_streams_and_fails_on_one_side_loss(
    tmp_path, isolated_recorder, lose_right
):
    spec, path = isolated_recorder
    stopped = threading.Event()

    def send():
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
            sender.settimeout(0.1)
            sequence = 0
            loss_deadline = None
            while not stopped.wait(0.01):
                if not path.exists():
                    continue
                # Output only grows after the recorder opens the start gate.
                motor_file = tmp_path / "run/left_motors.csv"
                if lose_right and motor_file.exists() and motor_file.stat().st_size > 4096:
                    loss_deadline = 0
                for side in (0, 1):
                    if side == 1 and loss_deadline is not None:
                        continue
                    payload = struct.pack(
                        "<4sHBBQQ16d",
                        b"VL1S",
                        1,
                        side,
                        0,
                        sequence,
                        time.monotonic_ns(),
                        *([0.1] * 16),
                    )
                    with suppress(FileNotFoundError, ConnectionRefusedError, TimeoutError):
                        sender.sendto(payload, str(path))
                sequence += 5  # SDK callback sequence is not the 100 Hz publication sequence.

    thread = threading.Thread(target=send)
    thread.start()
    try:
        report = recording.run_recording(spec, tmp_path / "run", 1.0)
    finally:
        stopped.set()
        thread.join(timeout=2)
    assert report["status"] == ("failed" if lose_right else "complete")
    if lose_right:
        assert any("stale" in error for error in report["errors"])
    assert not path.exists()
    for side in ("left", "right"):
        with (tmp_path / f"run/{side}_motors.csv").open() as handle:
            rows = list(csv.reader(handle))
        assert len(rows) > 1
        assert len(rows[1]) == 19
        assert report["motor_samples"][side] == len(rows) - 1
    assert json.loads((tmp_path / "run/metadata.json").read_text())["status"] == report["status"]


def test_existing_experiment_is_preserved_before_any_device_access(
    tmp_path, isolated_recorder, monkeypatch
):
    spec, _ = isolated_recorder
    output = tmp_path / "existing"
    output.mkdir()
    marker = output / "keep.txt"
    marker.write_text("user data")
    monkeypatch.setattr(
        recording, "verify_devices", lambda _: pytest.fail("must not touch hardware")
    )
    with pytest.raises(FileExistsError):
        recording.run_recording(spec, output, 1)
    assert marker.read_text() == "user data"


def test_existing_state_receiver_is_not_replaced(tmp_path, isolated_recorder):
    spec, path = isolated_recorder
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as owner:
        owner.bind(str(path))
        inode = path.stat().st_ino
        report = recording.run_recording(spec, tmp_path / "blocked", 1)
        assert report["status"] == "failed"
        assert any("already in use" in error for error in report["errors"])
        assert path.stat().st_ino == inode
