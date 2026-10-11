from __future__ import annotations

import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from vlai_l1_runtime.configuration import MOTOR_NAMES
from vlai_l1_runtime.feedback_wire import (
    PACKET_SIZE,
    FeedbackSession,
    FeedbackSessionError,
    FeedbackTelemetryPacket,
)

ROOT = Path(__file__).resolve().parents[1]


def packet(*, sequence=1, received=1000, assembled=1010, status=0, endpoint=False):
    # Independent specification fixture, not a Python producer implementation.
    header = struct.pack(
        "<4sHBB16s32sQQ", b"VL1T", 1, 1, 0, b"\x11" * 16, b"\x22" * 32, sequence, assembled
    )
    motors = []
    for index in range(8):
        raw = bytes([(status << 4) | (index + 1), 0x80, 0, 0x80, 0x08, 0, 30, 31])
        if endpoint:
            raw = bytes([index + 1, 0, 0, *raw[3:]])
        motors.append(
            struct.pack(
                "<HBBQ8sfff",
                17 + index,
                3 if endpoint else 1,
                0,
                received + index,
                raw,
                -2 if endpoint else 2 / 65535,
                4 / 4095,
                8 / 4095,
            )
        )
    return header + b"".join(motors)


def session(**changes):
    policy = {
        "side": "right",
        "session_id": b"\x11" * 16,
        "config_sha256": b"\x22" * 32,
        "response_ids": tuple(range(17, 25)),
        "slave_ids": tuple(range(1, 9)),
        "allowed_statuses": (frozenset({0}),) * 8,  # Test policy, no enable inference.
        "started_ns": 900,
        "max_age_ns": 100,
        "max_motor_skew_ns": 10,
        "startup_timeout_ns": 200,
        "silence_timeout_ns": 50,
    }
    return FeedbackSession(**(policy | changes))


def alter(data, offset, fmt, value):
    changed = bytearray(data)
    struct.pack_into(fmt, changed, offset, value)
    return bytes(changed)


def test_protocol_preserves_exact_motor_order_raw_status_timestamps_and_values():
    parsed = FeedbackTelemetryPacket.decode(packet(status=8))
    assert PACKET_SIZE == 328
    assert parsed.side == "right"
    assert parsed.feedback.names == MOTOR_NAMES
    assert parsed.source_sequence == 1
    assert parsed.feedback.monotonic_ns == 1010
    assert tuple(raw.host_receive_monotonic_ns for raw in parsed.feedback.raw_frames) == tuple(
        range(1000, 1008)
    )
    assert all(bytes.fromhex(raw.payload_hex)[0] >> 4 == 8 for raw in parsed.feedback.raw_frames)
    assert parsed.feedback.position_rad == pytest.approx((2 / 65535,) * 8)
    assert parsed.feedback.velocity_rad_s == pytest.approx((4 / 4095,) * 8)
    assert parsed.feedback.torque_nm == pytest.approx((8 / 4095,) * 8)
    assert parsed.at_protocol_endpoint == (False,) * 8


@pytest.mark.parametrize(
    ("offset", "fmt", "value"),
    [
        (0, "4s", b"VL1S"),
        (4, "H", 2),
        (6, "B", 2),
        (7, "B", 1),
        (8, "16s", bytes(16)),
        (24, "32s", bytes(32)),
        (56, "Q", 0),
        (64, "Q", 0),
        (72, "H", 0x800),
        (74, "B", 4),
        (75, "B", 1),
        (76, "Q", 0),
        (76, "Q", 1011),
        (104, "H", 17),
        (74, "B", 3),
        (92, "f", float("nan")),
        (96, "f", float("inf")),
        (100, "f", -float("inf")),
    ],
)
def test_parser_rejects_malformed_packets(offset, fmt, value):
    with pytest.raises(
        ValueError,
        match=r"telemetry|feedback|session_id|config_sha256|source_sequence|assembly time|endpoint",
    ):
        FeedbackTelemetryPacket.decode(alter(packet(), offset, "<" + fmt, value))


@pytest.mark.parametrize("payload", [b"", packet()[:-1], packet() + b"x", bytearray(packet())])
def test_parser_requires_exact_bytes_and_length(payload):
    with pytest.raises(ValueError, match="328 bytes"):
        FeedbackTelemetryPacket.decode(payload)


def test_session_accepts_fresh_complete_data_and_allows_sequence_gaps():
    receiver = session()
    receiver.check_deadline(now_ns=950)
    first = receiver.accept(packet(), now_ns=1011)
    second = receiver.accept(packet(sequence=3, received=1020, assembled=1030), now_ns=1031)
    assert first.source_sequence == 1
    assert second.source_sequence == 3
    receiver.check_deadline(now_ns=1031)  # Same local tick is valid.
    assert not receiver.faulted


@pytest.mark.parametrize(
    ("data", "message"),
    [
        (alter(packet(), 6, "<B", 0), "identity"),
        (alter(packet(), 8, "16s", b"x" * 16), "identity"),
        (alter(packet(), 24, "32s", b"x" * 32), "identity"),
        (alter(packet(), 72, "<H", 100), "motor identity"),
        (alter(packet(), 84, "<B", 2), "motor identity"),
        (packet(status=8), "motor status"),
        (packet(status=2), "motor status"),
        (packet(endpoint=True), "protocol endpoint"),
        (packet(received=800, assembled=850), "assembly time"),
        (packet(received=800, assembled=1010), "pre-session"),
        (packet(received=910, assembled=1010), "expired"),
        (packet(assembled=1100), "assembly time"),
        (alter(packet(), 76, "<Q", 990), "skew"),
        (packet()[:-1], "328 bytes"),
    ],
)
def test_session_faults_remain_latched_after_bad_packets(data, message):
    receiver = session()
    with pytest.raises(FeedbackSessionError, match=message):
        receiver.accept(data, now_ns=1011)
    assert receiver.faulted
    with pytest.raises(FeedbackSessionError, match="session fault"):
        receiver.accept(packet(sequence=2, received=1020, assembled=1030), now_ns=1031)
    with pytest.raises(FeedbackSessionError, match="session fault"):
        receiver.check_deadline(now_ns=1032)


@pytest.mark.parametrize(
    "data",
    [
        packet(sequence=1, received=1020, assembled=1030),
        packet(sequence=2, received=1000, assembled=1010),
        packet(sequence=2, received=1000, assembled=1030),
        alter(packet(sequence=2, received=1020, assembled=1030), 76, "<Q", 999),
    ],
)
def test_session_rejects_repeated_or_regressed_source_time_or_sequence(data):
    receiver = session()
    receiver.accept(packet(), now_ns=1011)
    with pytest.raises(FeedbackSessionError, match="did not increase"):
        receiver.accept(data, now_ns=1031)
    assert receiver.faulted


def test_startup_and_silence_deadlines_fail_without_receiving_another_packet():
    receiver = session()
    receiver.check_deadline(now_ns=1100)
    with pytest.raises(FeedbackSessionError, match="startup"):
        receiver.check_deadline(now_ns=1101)
    receiver = session()
    receiver.accept(packet(), now_ns=1011)
    receiver.check_deadline(now_ns=1061)
    with pytest.raises(FeedbackSessionError, match="silence"):
        receiver.check_deadline(now_ns=1062)


def test_motor_age_expires_even_with_a_long_silence_allowance():
    receiver = session(silence_timeout_ns=200)
    receiver.accept(packet(), now_ns=1011)
    receiver.check_deadline(now_ns=1100)
    with pytest.raises(FeedbackSessionError, match="expired"):
        receiver.check_deadline(now_ns=1101)


def test_late_packet_cannot_recover_a_deadline_or_clock_regression():
    receiver = session()
    with pytest.raises(FeedbackSessionError, match="startup"):
        receiver.accept(packet(received=1110, assembled=1120), now_ns=1121)
    receiver = session()
    receiver.check_deadline(now_ns=950)
    with pytest.raises(FeedbackSessionError, match=r"clock|time regressed"):
        receiver.check_deadline(now_ns=949)


def test_owner_can_latch_queue_or_can_fault_without_a_packet():
    receiver = session()
    receiver.accept(packet(), now_ns=1011)
    receiver.invalidate("receive queue overflow")
    receiver.invalidate("another fault")
    with pytest.raises(FeedbackSessionError, match="receive queue overflow"):
        receiver.accept(packet(sequence=2, received=1020, assembled=1030), now_ns=1031)


@pytest.mark.parametrize("now", [True, 0, -1, 1 << 64, float("nan")])
def test_invalid_consumer_clock_latches_session(now):
    receiver = session()
    with pytest.raises(FeedbackSessionError, match="now_ns"):
        receiver.check_deadline(now_ns=now)
    assert receiver.faulted


def test_per_motor_status_policy_does_not_imply_a_universal_enable_code():
    policy = (frozenset({1}),) + (frozenset({0}),) * 7
    data = alter(packet(), 84, "B", 0x11)
    accepted = session(allowed_statuses=policy).accept(data, now_ns=1011)
    assert bytes.fromhex(accepted.feedback.raw_frames[0].payload_hex)[0] == 0x11
    with pytest.raises(FeedbackSessionError, match="joint_1"):
        session(allowed_statuses=policy).accept(packet(), now_ns=1011)


@pytest.mark.parametrize(
    "change",
    [
        {"side": "other"},
        {"session_id": bytes(16)},
        {"config_sha256": b"bad"},
        {"started_ns": True},
        {"max_age_ns": 0},
        {"max_motor_skew_ns": 101},
        {"startup_timeout_ns": -1},
        {"silence_timeout_ns": 1 << 64},
        {"response_ids": (17,) * 8},
        {"response_ids": tuple(range(2048, 2056))},
        {"slave_ids": tuple(range(8))},
        {"slave_ids": (1,) * 8},
        {"allowed_statuses": (frozenset(),) * 8},
        {"allowed_statuses": (frozenset({True}),) * 8},
        {"allowed_statuses": (frozenset({16}),) * 8},
        {"allowed_statuses": (frozenset({0}),) * 7},
    ],
)
def test_session_requires_explicit_valid_policy(change):
    with pytest.raises(ValueError, match=next(iter(change))):
        session(**change)


def test_cpp_packet_is_byte_exact_and_accepted_by_python(tmp_path):
    if not shutil.which("cmake") or not shutil.which("c++"):
        pytest.skip("CMake and a C++17 compiler are required for cross-language validation")
    build = tmp_path / "build"
    subprocess.run(
        [
            "cmake",
            "-S",
            str(ROOT / "native/xair_sidecar"),
            "-B",
            str(build),
            "-DVLAI_L1_SAFETY_ONLY=ON",
            "-DBUILD_TESTING=ON",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            "cmake",
            "--build",
            str(build),
            "--target",
            "vlai_l1_motor_feedback_test",
            "--parallel",
            "2",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build / "vlai_l1_motor_feedback_test"
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)
    target = tmp_path / "packet.bin"
    subprocess.run(
        [str(executable), "--emit-packet", str(target)], check=True, capture_output=True, text=True
    )
    data = target.read_bytes()
    assert data == packet()
    decoded = session().accept(data, now_ns=1011)
    assert decoded.source_sequence == 1
    assert decoded.feedback.names == MOTOR_NAMES

    # Exercise the actual replay-to-wire exporter with a synthetic complete
    # group, independently specified from the C++ fixture.
    replay = tmp_path / "replay.tsv"
    lines = []
    for index, raw in enumerate(decoded.feedback.raw_frames):
        lines.append(
            " ".join(
                map(
                    str,
                    (
                        raw.name,
                        raw.response_id,
                        index + 1,
                        2,
                        4,
                        8,
                        raw.host_receive_monotonic_ns,
                        raw.payload_hex,
                        decoded.feedback.position_rad[index],
                        decoded.feedback.velocity_rad_s[index],
                        decoded.feedback.torque_nm[index],
                    ),
                )
            )
        )
    replay.write_text("\n".join(lines) + "\n")
    replay_wire = tmp_path / "replay.bin"
    subprocess.run(
        [str(executable), "--replay-wire", str(replay), str(replay_wire)],
        check=True,
        capture_output=True,
        text=True,
    )
    assert replay_wire.read_bytes() == packet(assembled=1007)
