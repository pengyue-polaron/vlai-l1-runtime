from __future__ import annotations

import ctypes
import struct
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from vlai_l1_runtime.configuration import MOTOR_NAMES, load_system_config
from vlai_l1_runtime.feedback import MotorFeedback, RawMotorFrame
from vlai_l1_runtime.teleoperation.can_probe import (
    MotorFeedbackProbeError,
    _CanSdkSession,
    _JointState,
    _SocketCanProbeBackend,
    probe_motor_feedback,
)

CONFIG = Path(__file__).resolve().parents[1] / "configs/system/vlai_l1.toml"


def feedback(timestamp=100):
    return MotorFeedback(MOTOR_NAMES, timestamp, (0.1,) * 8, (0.2,) * 8, (0.3,) * 8)


@pytest.mark.parametrize(
    "change",
    [
        {"names": tuple(reversed(MOTOR_NAMES))},
        {"position_rad": (0.0,) * 7},
        {"velocity_rad_s": (float("nan"),) * 8},
        {"torque_nm": (float("inf"),) * 8},
        {"torque_nm": (True,) * 8},
        {"monotonic_ns": 0},
        {"monotonic_ns": True},
    ],
)
def test_feedback_rejects_invalid_named_state(change):
    with pytest.raises(ValueError, match="feedback"):
        replace(feedback(), **change)


class Backend:
    def __init__(self, snapshots, responses=None):
        self.snapshots = iter(snapshots)
        self.responses = responses or frozenset(range(17, 25))
        self.closed = False
        self.reads = 0

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def sample_round(self, period):
        return self.responses

    def read_feedback(self):
        self.reads += 1
        return next(self.snapshots)


def run_probe(backend, **kwargs):
    return probe_motor_feedback(
        load_system_config(CONFIG),
        "can2",
        Path("/unused.so"),
        _backend_factory=lambda *a, **k: backend,
        **kwargs,
    )


def test_probe_publishes_only_complete_feedback_and_checks_health_each_round():
    backend = Backend([feedback(i + 1) for i in range(40)])
    samples, checks = [], []
    run_probe(
        backend,
        on_feedback=lambda *args: samples.append(args),
        check_health=lambda: checks.append(True),
    )
    assert len(samples) == 40
    assert samples[-1][0] == 40
    assert len(checks) == 80
    assert backend.closed


def test_probe_missing_motor_never_reads_or_publishes_cache():
    backend = Backend([], frozenset(range(17, 24)))
    with pytest.raises(MotorFeedbackProbeError, match="0x018"):
        run_probe(backend, on_feedback=lambda *args: pytest.fail("published incomplete data"))
    assert backend.reads == 0
    assert backend.closed


@pytest.mark.parametrize("timestamps", [(20, 20), (20, 10)])
def test_probe_rejects_nonincreasing_timestamps_and_closes(timestamps):
    backend = Backend([feedback(t) for t in timestamps])
    with pytest.raises(MotorFeedbackProbeError, match="timestamp did not increase"):
        run_probe(backend, on_feedback=lambda *args: None)
    assert backend.closed


def test_probe_health_failure_closes_without_reading():
    backend = Backend([])

    def fail():
        raise RuntimeError("CAN fault")

    with pytest.raises(RuntimeError, match="CAN fault"):
        run_probe(backend, on_feedback=lambda *args: None, check_health=fail)
    assert backend.closed
    assert backend.reads == 0


def test_probe_write_failure_closes_without_retry():
    backend = Backend([feedback()])

    def fail(*args):
        raise OSError("disk full")

    with pytest.raises(OSError, match="disk full"):
        run_probe(backend, on_feedback=fail)
    assert backend.closed
    assert backend.reads == 1


def test_sdk_cache_reader_uses_existing_handle_and_receive_only():
    calls = []

    class Function:
        def __init__(self, name):
            self.name = name

        def __call__(self, handle, *args):
            assert handle.value == 123
            calls.append(self.name)
            if self.name == "arm":
                states, count = args
                for i in range(count):
                    states[i] = _JointState(i + 0.1, i + 0.2, i + 0.3)
            elif self.name == "gripper":
                ctypes.cast(args[0], ctypes.POINTER(_JointState))[0] = _JointState(7.1, 7.2, 7.3)
            return 0

    session = object.__new__(_CanSdkSession)
    session._motor_names = MOTOR_NAMES
    session._handle = ctypes.c_void_p(123)
    session._library = SimpleNamespace(
        xarm_sdk_recv_all=Function("receive"),
        xarm_sdk_get_arm_joint_states=Function("arm"),
        xarm_sdk_get_gripper_state=Function("gripper"),
    )
    actual = session.read_feedback()
    assert calls == ["receive", "arm", "gripper"]
    assert actual.names == MOTOR_NAMES
    assert actual.torque_nm == pytest.approx([i + 0.3 for i in range(8)])
    assert actual.position_rad == pytest.approx([i + 0.1 for i in range(8)])


def test_diagnostic_imports_and_cli_parsing_do_not_initialize_hardware():
    program = """
import ctypes
import socket
import subprocess
import sys

def denied(*args, **kwargs):
    raise AssertionError("hardware or process initialization during static import")

ctypes.CDLL = denied
socket.socket = denied
subprocess.run = denied
subprocess.Popen = denied
import vlai_l1_runtime.feedback
import vlai_l1_runtime.feedback_wire
import vlai_l1_runtime.motor_parameters
import vlai_l1_runtime.teleoperation.parameter_probe
import vlai_l1_runtime.kinematics
import vlai_l1_runtime.wrench
import vlai_l1_runtime.teleoperation.feedback_capture
from vlai_l1_runtime.cli import build_parser
args = build_parser().parse_args([
    "capture-follower-feedback", "--config", "unused.toml",
    "--side", "right", "--output", "unused-output",
])
assert args.command == "capture-follower-feedback"
assert not {"rclpy", "rospy", "pyrealsense2"}.intersection(sys.modules)
"""
    subprocess.run((sys.executable, "-c", program), check=True, capture_output=True, text=True)


def raw_backend():
    config = load_system_config(CONFIG)
    backend = object.__new__(_SocketCanProbeBackend)
    backend._motors = config.motors
    backend._expected = frozenset(motor.receive_id for motor in config.motors)
    backend._names_by_id = {motor.receive_id: motor.name for motor in config.motors}
    backend._raw_frames, backend._raw_counts = {}, {}

    class Socket:
        def __init__(self):
            self.queue = []

        def recv(self, size):
            if not self.queue:
                raise BlockingIOError
            return self.queue.pop(0)

    backend._socket = Socket()
    return backend


@pytest.mark.parametrize("can_fd", [False, True])
def test_raw_frames_preserve_payload_ids_and_transport(monkeypatch, can_fd):
    backend = raw_backend()
    monkeypatch.setattr("vlai_l1_runtime.teleoperation.can_probe.time.monotonic_ns", lambda: 10)
    data = bytes.fromhex("1180019008012021")
    layout = "=IB3x64s" if can_fd else "=IB3x8s"
    backend._socket.queue = [struct.pack(layout, i, 8, data) for i in range(17, 25)]
    assert backend._drain() == set(range(17, 25))
    backend._session = SimpleNamespace(read_feedback=lambda: feedback(20))
    sample = backend.read_feedback()
    assert tuple(frame.name for frame in sample.raw_frames) == MOTOR_NAMES
    assert all(
        frame.payload_hex == data.hex() and frame.can_fd == can_fd for frame in sample.raw_frames
    )
    assert all(frame.host_receive_monotonic_ns == 10 for frame in sample.raw_frames)


def test_old_raw_frames_are_cleared_before_query():
    import time

    backend = raw_backend()
    backend._socket.queue = [struct.pack("=IB3x8s", 17, 8, b"old-data")]

    def refresh():
        backend._socket.queue = [struct.pack("=IB3x8s", i, 8, b"new-data") for i in range(17, 25)]

    backend._session = SimpleNamespace(
        refresh=refresh, read_feedback=lambda: feedback(time.monotonic_ns())
    )
    assert backend.sample_round(0) == backend._expected
    assert all(
        frame.payload_hex == b"new-data".hex() for frame in backend.read_feedback().raw_frames
    )


def test_malformed_data_length_and_ambiguous_duplicate_responses_are_rejected():
    backend = raw_backend()
    backend._socket.queue = [struct.pack("=IB3x8s", 17, 7, b"bad-data")]
    with pytest.raises(MotorFeedbackProbeError, match="eight data bytes"):
        backend._drain()
    backend._socket.queue = [
        struct.pack("=IB3x8s", i, 8, b"new-data") for i in (*range(17, 25), 17)
    ]
    backend._drain()
    with pytest.raises(MotorFeedbackProbeError, match="duplicate"):
        backend.read_feedback()


def test_raw_timestamp_must_increase_for_each_motor_across_rounds():
    raw = tuple(
        RawMotorFrame(name, i + 17, 10, "0000000000000000", False)
        for i, name in enumerate(MOTOR_NAMES)
    )
    backend = Backend(
        [replace(feedback(20), raw_frames=raw), replace(feedback(30), raw_frames=raw)]
    )
    with pytest.raises(MotorFeedbackProbeError, match="raw feedback host timestamp"):
        run_probe(backend, on_feedback=lambda *args: None)
    assert backend.closed


def test_invalid_raw_order_and_future_receive_times_are_rejected():
    raw = tuple(
        RawMotorFrame(name, i + 17, 10, "0000000000000000", False)
        for i, name in enumerate(MOTOR_NAMES)
    )
    with pytest.raises(ValueError, match="ordered"):
        replace(feedback(20), raw_frames=tuple(reversed(raw)))
    with pytest.raises(ValueError, match="after SDK"):
        replace(feedback(5), raw_frames=raw)


@pytest.mark.parametrize(
    "change",
    [
        {"name": "unknown"},
        {"response_id": True},
        {"response_id": 0x800},
        {"host_receive_monotonic_ns": 0},
        {"payload_hex": "gg" * 8},
        {"payload_hex": "00" * 7},
        {"can_fd": 1},
    ],
)
def test_raw_frame_contract_rejects_malformed_fields(change):
    frame = RawMotorFrame("joint_1", 17, 10, "00" * 8, False)
    with pytest.raises(ValueError, match="raw feedback"):
        replace(frame, **change)
