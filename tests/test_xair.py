from __future__ import annotations

import math
import socket
import struct
from dataclasses import replace
from pathlib import Path

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.teleoperation import (
    XAirBimanualAssembler,
    XAirStatePacket,
    XAirStateReceiver,
    describe_xair_side,
    render_xair_control_config,
    verify_xair_dependency,
)
from vlai_l1_runtime.teleoperation.lifecycle import _find_competing_control, _qdisc_drops

ROOT = Path(__file__).resolve().parents[1]
SYSTEM_CONFIG = ROOT / "configs/system/vlai_l1.toml"
PACKET = struct.Struct("<4sHBBQQ16d")


def _packet(*, side: int, sequence: int, timestamp: int, value: float) -> bytes:
    return PACKET.pack(
        b"VL1S",
        1,
        side,
        0,
        sequence,
        timestamp,
        *([value] * 8),
        *([value + 0.1] * 8),
    )


def test_pinned_xair_dependency_and_generated_launch_inputs(tmp_path: Path) -> None:
    config = load_system_config(SYSTEM_CONFIG)

    report = verify_xair_dependency(config)
    assert report.revision == config.teleoperation.source_revision
    assert len(report.teleop_library_sha256) == 64
    assert describe_xair_side(config, "right") == {
        "provider": "x_air_sdk",
        "mode": "unilateral",
        "arm_type": "v10",
        "sdk_version": "1.0.0",
        "source_revision": "bf300508e179f652b23f0efaf3b6c9048f1f12e9",
        "state_protocol_version": 1,
        "side": "right",
        "arm_side": "right_arm",
        "leader_can": "can0",
        "follower_can": "can2",
        "can_fd": True,
        "can_nominal_bitrate": 1000000,
        "can_data_bitrate": 5000000,
        "can_restart_ms": 100,
        "can_tx_queue_length": 1000,
        "state_socket_path": "/run/vlai-l1/teleop-state.sock",
        "publish_hz": 100,
        "state_timeout_ms": 100,
        "rt_priority": 20,
        "can_health_poll_ms": 100,
        "commissioned": True,
    }
    leader, follower = render_xair_control_config(config, tmp_path)
    assert "LeaderArmParam:" in leader.read_text()
    assert "Fc: [0.306" in leader.read_text()
    assert "FollowerArmParam:" in follower.read_text()
    assert "Fc: [0.306" in follower.read_text()


def test_xair_ownership_allows_only_disjoint_can_pairs() -> None:
    active_left = (
        (
            "/runtime/vlai_l1_xair_sidecar",
            "--leader-can",
            "can1",
            "--follower-can",
            "can3",
        ),
    )

    assert _find_competing_control(("can0", "can2"), active_left) is None
    assert _find_competing_control(("can1", "can3"), active_left) is not None


def test_qdisc_health_query_isolated_from_runtime_shutdown_signal(monkeypatch) -> None:
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return type("Result", (), {"stdout": '[{"drops":7}]'})()

    monkeypatch.setattr("vlai_l1_runtime.teleoperation.lifecycle.subprocess.run", run)

    assert _qdisc_drops("can1") == 7
    assert calls[0][1]["start_new_session"] is True


def test_xair_packets_form_exact_degree_valued_bimanual_state() -> None:
    assembler = XAirBimanualAssembler(max_side_skew_s=0.05)
    left = XAirStatePacket.decode(_packet(side=0, sequence=4, timestamp=1_000, value=0.1))
    right = XAirStatePacket.decode(_packet(side=1, sequence=9, timestamp=2_000, value=-0.2))

    assert assembler.accept(left) is None
    observation, action = assembler.accept(right)
    assert action.values["left_joint_1.pos"] == pytest.approx(math.degrees(0.1))
    assert action.values["right_gripper.pos"] == pytest.approx(math.degrees(-0.2))
    assert observation.values["left_joint_1.pos"] == pytest.approx(math.degrees(0.2))
    assert observation.values["right_gripper.pos"] == pytest.approx(math.degrees(-0.1))
    assert observation.metadata == action.metadata

    with pytest.raises(ValueError, match="sequence did not increase"):
        assembler.accept(left)


def test_xair_receiver_owns_and_cleans_its_unix_socket(tmp_path: Path) -> None:
    config = load_system_config(SYSTEM_CONFIG)
    state_socket = tmp_path / "state.sock"
    config = replace(
        config,
        teleoperation=replace(config.teleoperation, state_socket_path=state_socket),
    )
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        with XAirStateReceiver(config) as receiver:
            sender.sendto(
                _packet(side=0, sequence=1, timestamp=1_000, value=0.0), str(state_socket)
            )
            sender.sendto(
                _packet(side=1, sequence=1, timestamp=2_000, value=0.0), str(state_socket)
            )
            assert receiver.receive(timeout_s=0.1) is not None
            sender.sendto(
                _packet(side=0, sequence=2, timestamp=3_000, value=0.0), str(state_socket)
            )
            sender.sendto(
                _packet(side=1, sequence=2, timestamp=4_000, value=0.0), str(state_socket)
            )
            sender.sendto(
                _packet(side=0, sequence=3, timestamp=5_000, value=0.2), str(state_socket)
            )
            sender.sendto(
                _packet(side=1, sequence=3, timestamp=6_000, value=0.2), str(state_socket)
            )
            closest = receiver.receive_closest(
                target_monotonic_ns=4_200,
                timeout_s=0.1,
            )
            assert closest is not None
            assert closest[0].metadata.monotonic_ns == 4_000
            assert closest[1].values["left_joint_1.pos"] == pytest.approx(0.0)
            assert receiver.receive(timeout_s=0.0) is None
        assert not state_socket.exists()
    finally:
        sender.close()
