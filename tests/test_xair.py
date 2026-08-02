from __future__ import annotations

import io
import math
import socket
import struct
import subprocess
import threading
from dataclasses import replace
from pathlib import Path

import pytest

import vlai_l1_runtime.teleoperation.lifecycle as lifecycle_module
from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.teleoperation import (
    XAirBimanualAssembler,
    XAirSingleSideAssembler,
    XAirStatePacket,
    XAirStateReceiver,
    describe_xair_side,
    render_xair_control_config,
    request_xair_adjust_position,
    verify_xair_dependency,
    xair_control_socket_path,
)
from vlai_l1_runtime.teleoperation.can_probe import (
    MotorFeedbackProbeError,
    probe_motor_feedback,
)
from vlai_l1_runtime.teleoperation.lifecycle import (
    _acquire_lifecycle_lock,
    _await_managed_startup_release,
    _decode_can_health,
    _decode_link_is_up,
    _find_competing_control,
    _inactive_can_interfaces,
    _qdisc_drops,
    _relay_sidecar_output,
    _remove_orphaned_control_socket,
    _require_can_identity,
    _require_interfaces_down,
    _sidecar_command,
    _stop_sidecar,
)

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
        "motors": [
            {
                "name": name,
                "send_id": send_id,
                "receive_id": receive_id,
                "motor_type": motor_type,
            }
            for name, send_id, receive_id, motor_type in (
                ("joint_1", 1, 17, "DM8009"),
                ("joint_2", 2, 18, "DM8009"),
                ("joint_3", 3, 19, "DM4340"),
                ("joint_4", 4, 20, "DM4340"),
                ("joint_5", 5, 21, "DM4310"),
                ("joint_6", 6, 22, "DM4310"),
                ("joint_7", 7, 23, "DM4310"),
                ("gripper", 8, 24, "DM4310"),
            )
        ],
        "motor_probe_duration_s": 2.0,
        "motor_probe_rate_hz": 20,
        "state_socket_path": "/run/vlai-l1/teleop-state.sock",
        "control_socket_path": "/run/vlai-l1/teleop-state-right-control.sock",
        "publish_hz": 100,
        "state_timeout_ms": 100,
        "rt_priority": 20,
        "can_health_poll_ms": 100,
        "joint_min_deg": [-80.0, -10.0, -90.0, 0.0, -90.0, -45.0, -90.0],
        "joint_max_deg": [120.0, 180.0, 90.0, 140.0, 90.0, 45.0, 90.0],
        "max_following_error_deg": [9.0, 8.0, 3.0, 10.0, 4.0, 2.0, 6.0],
        "following_error_timeout_ms": 100,
        "following_error_action": "warn",
        "commissioned": True,
    }
    leader, follower = render_xair_control_config(config, tmp_path)
    assert "LeaderArmParam:" in leader.read_text()
    assert "Fc: [0.306" in leader.read_text()
    assert "FollowerArmParam:" in follower.read_text()
    assert "Fc: [0.306" in follower.read_text()


def test_sidecar_command_carries_config_owned_joint_safety() -> None:
    config = load_system_config(SYSTEM_CONFIG)
    launch = describe_xair_side(config, "right")
    command = _sidecar_command(
        config,
        "right",
        sidecar=Path("/runtime/vlai_l1_xair_sidecar"),
        assets=Path("/runtime/assets"),
        launch=launch,
        owner_uid=1000,
        owner_gid=1000,
    )

    def value(option: str) -> str:
        return command[command.index(option) + 1]

    assert value("--joint-min-deg") == "-80,-10,-90,0,-90,-45,-90"
    assert value("--joint-max-deg") == "120,180,90,140,90,45,90"
    assert value("--max-following-error-deg") == "9,8,3,10,4,2,6"
    assert value("--following-error-timeout-ms") == "100"
    assert value("--following-error-action") == "warn"


class _FakeProbeBackend:
    def __init__(self, responses, calls, *args, **kwargs) -> None:
        calls.append((args, kwargs))
        self._responses = iter(responses)
        self.periods = calls

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        return None

    def sample_round(self, period_s: float):
        self.periods.append(period_s)
        return next(self._responses)


def test_motor_feedback_probe_requires_every_motor_for_the_full_stability_window() -> None:
    config = load_system_config(SYSTEM_CONFIG)
    expected = frozenset(range(0x11, 0x19))
    calls = []

    result = probe_motor_feedback(
        config,
        "can0",
        Path("/sdk/libxarm_can_sdk.so"),
        _backend_factory=lambda *args, **kwargs: _FakeProbeBackend(
            [expected] * 40,
            calls,
            *args,
            **kwargs,
        ),
    )

    assert result.interface == "can0"
    assert result.rounds == 40
    assert result.response_ids == tuple(range(0x11, 0x19))
    assert calls[0][0] == (Path("/sdk/libxarm_can_sdk.so"), "can0")
    assert calls[0][1]["can_fd"] is True
    assert calls[1:] == [0.05] * 40


def test_motor_feedback_probe_rejects_one_missing_motor_before_motion() -> None:
    config = load_system_config(SYSTEM_CONFIG)
    calls = []

    with pytest.raises(MotorFeedbackProbeError, match=r"0x018"):
        probe_motor_feedback(
            config,
            "can0",
            Path("/sdk/libxarm_can_sdk.so"),
            _backend_factory=lambda *args, **kwargs: _FakeProbeBackend(
                [frozenset(range(0x11, 0x18))],
                calls,
                *args,
                **kwargs,
            ),
        )


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


def test_isolated_side_lock_excludes_every_other_runtime(tmp_path: Path) -> None:
    config = load_system_config(SYSTEM_CONFIG)
    config = replace(
        config,
        teleoperation=replace(
            config.teleoperation,
            state_socket_path=tmp_path / "state.sock",
        ),
    )
    shared_left = _acquire_lifecycle_lock(config, exclusive=False)
    shared_right = _acquire_lifecycle_lock(config, exclusive=False)
    try:
        with pytest.raises(RuntimeError, match=r"isolated-side.*ownership"):
            _acquire_lifecycle_lock(config, exclusive=True)
    finally:
        shared_left.close()
        shared_right.close()

    isolated = _acquire_lifecycle_lock(config, exclusive=True)
    try:
        with pytest.raises(RuntimeError, match=r"shared.*ownership"):
            _acquire_lifecycle_lock(config, exclusive=False)
    finally:
        isolated.close()

    lock_path = tmp_path / "teleop-lifecycle.lock"
    lock_path.unlink()
    lock_path.symlink_to(tmp_path / "redirected")
    with pytest.raises(RuntimeError, match="cannot open x_air lifecycle lock"):
        _acquire_lifecycle_lock(config, exclusive=True)


def test_right_only_interlock_requires_both_left_can_links_down(monkeypatch) -> None:
    config = load_system_config(SYSTEM_CONFIG)
    assert _inactive_can_interfaces(config, "right") == ("can1", "can3")
    assert not _decode_link_is_up("can1", '[{"flags":["NOARP","ECHO"]}]')
    assert _decode_link_is_up("can1", '[{"flags":["NOARP","UP","ECHO"]}]')

    states = {"can1": False, "can3": False}
    monkeypatch.setattr(lifecycle_module, "_read_link_is_up", states.__getitem__)
    _require_interfaces_down(("can1", "can3"))

    states["can1"] = True
    with pytest.raises(RuntimeError, match=r"inactive CAN interfaces DOWN: can1"):
        _require_interfaces_down(("can1", "can3"))


def test_qdisc_health_query_isolated_from_runtime_shutdown_signal(monkeypatch) -> None:
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return type("Result", (), {"stdout": '[{"drops":7}]'})()

    monkeypatch.setattr("vlai_l1_runtime.teleoperation.lifecycle.subprocess.run", run)

    assert _qdisc_drops("can1") == 7
    assert calls[0][1]["start_new_session"] is True


def test_can_health_and_physical_identity_are_exact(tmp_path: Path) -> None:
    config = load_system_config(SYSTEM_CONFIG)
    payload = """
    [{
      "parentdev": "1-3.2:1.0",
      "linkinfo": {
        "info_data": {
          "state": "ERROR-ACTIVE",
          "berr_counter": {"tx": 0, "rx": 0}
        }
      }
    }]
    """
    health = _decode_can_health("can1", payload)
    assert health.healthy
    assert health.detail() == ("can1 parent=1-3.2:1.0 state=ERROR-ACTIVE txerr=0 rxerr=0")
    assert not _decode_can_health(
        "can1",
        payload.replace('"ERROR-ACTIVE"', '"ERROR-PASSIVE"').replace('"tx": 0', '"tx": 128'),
    ).healthy

    device = tmp_path / "devices/1-3.2:1.0"
    driver = tmp_path / "bus/usb/drivers/peak_usb"
    interface = tmp_path / "class/net/can1"
    device.mkdir(parents=True)
    driver.mkdir(parents=True)
    interface.mkdir(parents=True)
    (device / "driver").symlink_to(driver)
    (interface / "device").symlink_to(device)
    assert _require_can_identity(config, "can1", sysfs_root=tmp_path) == "1-3.2:1.0"

    (interface / "device").unlink()
    wrong_device = tmp_path / "devices/1-3.9:1.0"
    wrong_device.mkdir()
    (wrong_device / "driver").symlink_to(driver)
    (interface / "device").symlink_to(wrong_device)
    with pytest.raises(RuntimeError, match="USB identity mismatch"):
        _require_can_identity(config, "can1", sysfs_root=tmp_path)


def test_sidecar_ready_output_is_relayed(capsys) -> None:
    ready = threading.Event()
    _relay_sidecar_output(
        io.StringIO("vendor detail\nPASS x_air left_arm control running (SDK 1.0.0)\n"),
        ready,
    )
    assert ready.is_set()
    assert capsys.readouterr().out.endswith("control running (SDK 1.0.0)\n")


def test_managed_startup_gate_checks_health_before_accepting_release() -> None:
    sender, receiver = socket.socketpair()
    health_checks: list[str] = []
    sender.sendall(b"START\n")
    try:
        with receiver.makefile("r", encoding="utf-8") as input_stream:
            assert _await_managed_startup_release(
                input_stream=input_stream,
                side="left",
                timeout_s=0.2,
                poll_s=0.01,
                require_healthy=lambda: health_checks.append("healthy"),
                stop_requested=lambda: False,
            )
    finally:
        sender.close()
        receiver.close()

    assert health_checks == ["healthy", "healthy"]


def test_sidecar_shutdown_timeout_is_contained() -> None:
    class Child:
        returncode = None

        def __init__(self) -> None:
            self.waits = 0
            self.signal = None
            self.killed = False

        def poll(self):
            return None

        def send_signal(self, value):
            self.signal = value

        def wait(self, timeout=None):
            self.waits += 1
            if self.waits == 1:
                raise subprocess.TimeoutExpired(("sidecar",), timeout)
            self.returncode = -9
            return -9

        def kill(self):
            self.killed = True

    child = Child()
    assert _stop_sidecar(child, None, timeout_s=0.1) == -9
    assert child.signal is not None
    assert child.killed


def test_control_socket_cleanup_preserves_active_owner_and_removes_orphan(
    tmp_path: Path,
) -> None:
    path = tmp_path / "control.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    server.listen(1)
    try:
        with pytest.raises(RuntimeError, match="still active"):
            _remove_orphaned_control_socket(path)
        assert path.exists()
    finally:
        server.close()

    _remove_orphaned_control_socket(path)
    assert not path.exists()


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


def test_xair_packets_form_exact_degree_valued_right_only_state() -> None:
    assembler = XAirSingleSideAssembler("right")
    left = XAirStatePacket.decode(_packet(side=0, sequence=4, timestamp=1_000, value=0.1))
    right = XAirStatePacket.decode(_packet(side=1, sequence=9, timestamp=2_000, value=-0.2))

    assert assembler.accept(left) is None
    observation, action = assembler.accept(right)
    assert tuple(action.values) == tuple(
        f"right_{motor}.pos"
        for motor in (
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
            "gripper",
        )
    )
    assert action.values["right_joint_1.pos"] == pytest.approx(math.degrees(-0.2))
    assert observation.values["right_gripper.pos"] == pytest.approx(math.degrees(-0.1))
    assert len(action.values) == 8

    with pytest.raises(ValueError, match="sequence did not increase"):
        assembler.accept(right)


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
            sender.sendto(
                _packet(side=0, sequence=4, timestamp=7_000, value=0.3), str(state_socket)
            )
            receiver.reset_pairing()
            assert receiver.receive(timeout_s=0.0) is None
            sender.sendto(
                _packet(side=0, sequence=5, timestamp=8_000, value=0.4), str(state_socket)
            )
            sender.sendto(
                _packet(side=1, sequence=5, timestamp=9_000, value=0.4), str(state_socket)
            )
            assert receiver.receive(timeout_s=0.1) is not None
        assert not state_socket.exists()
    finally:
        sender.close()


def test_xair_receiver_right_only_does_not_wait_for_left_packets(tmp_path: Path) -> None:
    config = load_system_config(SYSTEM_CONFIG)
    state_socket = tmp_path / "right-state.sock"
    config = replace(
        config,
        teleoperation=replace(config.teleoperation, state_socket_path=state_socket),
    )
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        with XAirStateReceiver(config, sides=("right",)) as receiver:
            sender.sendto(
                _packet(side=1, sequence=1, timestamp=2_000, value=0.3),
                str(state_socket),
            )
            sample = receiver.receive(timeout_s=0.1)
            assert sample is not None
            observation, action = sample
            assert tuple(observation.values) == observation.feature_names
            assert tuple(action.values) == action.feature_names
            assert len(observation.values) == 8
        assert not state_socket.exists()
    finally:
        sender.close()


def test_xair_adjust_position_uses_the_owning_sidecar_control_socket(tmp_path: Path) -> None:
    config = load_system_config(SYSTEM_CONFIG)
    config = replace(
        config,
        teleoperation=replace(
            config.teleoperation,
            state_socket_path=tmp_path / "state.sock",
        ),
    )
    control_path = xair_control_socket_path(config, "left")
    request: list[bytes] = []
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(control_path))
    server.listen(1)

    def serve() -> None:
        client, _ = server.accept()
        with client:
            request.append(client.recv(256))
            client.sendall(b"OK\n")

    worker = threading.Thread(target=serve)
    worker.start()
    try:
        request_xair_adjust_position(config, "left")
    finally:
        worker.join(timeout=1)
        server.close()

    assert not worker.is_alive()
    assert request == [b"ADJUST_POSITION\n"]


def test_xair_adjust_position_surfaces_sidecar_failure(tmp_path: Path) -> None:
    config = load_system_config(SYSTEM_CONFIG)
    config = replace(
        config,
        teleoperation=replace(
            config.teleoperation,
            state_socket_path=tmp_path / "state.sock",
        ),
    )
    control_path = xair_control_socket_path(config, "right")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(control_path))
    server.listen(1)

    def serve() -> None:
        client, _ = server.accept()
        with client:
            client.recv(256)
            client.sendall(b"ERROR controller rejected alignment\n")

    worker = threading.Thread(target=serve)
    worker.start()
    try:
        with pytest.raises(RuntimeError, match=r"right.*controller rejected alignment"):
            request_xair_adjust_position(config, "right")
    finally:
        worker.join(timeout=1)
        server.close()


def test_configure_can_clears_diagnostic_modes(monkeypatch) -> None:
    config = load_system_config(SYSTEM_CONFIG)
    calls = []

    monkeypatch.setattr(
        lifecycle_module,
        "_require_can_identity",
        lambda checked_config, interface: "1-3.2:1.0",
    )

    def fake_run(command, **kwargs):
        calls.append((tuple(command), kwargs))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(lifecycle_module.subprocess, "run", fake_run)

    lifecycle_module._configure_can(config, "can1")

    configure = calls[1][0]
    assert configure[:5] == ("ip", "link", "set", "can1", "type")
    assert configure[configure.index("listen-only") + 1] == "off"
    assert configure[configure.index("one-shot") + 1] == "off"
