from __future__ import annotations

import struct
from dataclasses import replace

import pytest

from vlai_l1_runtime.feedback import RawMotorFrame
from vlai_l1_runtime.motor_parameters import ParameterRead, ReadRegister


def raw(query, value, *, timestamp=20):
    fmt = "<f" if query.register in (21, 22, 23) else "<I"
    payload = struct.pack("<HBB", query.slave_id, 0x33, query.register) + struct.pack(fmt, value)
    return RawMotorFrame(query.name, query.response_id, timestamp, payload.hex(), True)


@pytest.mark.parametrize("register", list(ReadRegister))
def test_only_allowlisted_reads_and_exact_little_endian_reply(register):
    query = ParameterRead("joint_3", 3, 19, register)
    can_id, payload = query.request
    assert can_id == 0x7FF
    assert payload == bytes((3, 0, 0x33, register, 0, 0, 0, 0))
    value = {7: 19, 8: 3, 21: 12.5, 22: 8.0, 23: 28.0}.get(register, 0x01020304)
    assert query.decode(raw(query, value), sent_ns=10, deadline_ns=30) == value


@pytest.mark.parametrize("register", [0, 10, 21, 50, 55, 0xFC, True])
def test_plain_integers_and_nonallowlisted_operations_are_rejected(register):
    with pytest.raises(ValueError, match="allowlisted"):
        ParameterRead("joint_3", 3, 19, register)


@pytest.mark.parametrize(
    "change",
    [{"slave_id": 0}, {"slave_id": 0x7FF}, {"response_id": True}, {"name": "other"}],
)
def test_invalid_identity_rejected_before_transport(change):
    with pytest.raises(ValueError, match="parameter read"):
        replace(ParameterRead("joint_3", 3, 19, ReadRegister.POSITION_MAX), **change)


@pytest.mark.parametrize("value", [0, -1, float("inf"), float("nan")])
def test_invalid_ranges_are_not_reported_as_configuration(value):
    query = ParameterRead("joint_3", 3, 19, ReadRegister.TORQUE_MAX)
    with pytest.raises(ValueError, match="finite and positive"):
        query.decode(raw(query, value), sent_ns=10, deadline_ns=30)


@pytest.mark.parametrize(("offset", "value"), [(0, 4), (2, 0x55), (2, 0xCC), (3, 22)])
def test_reply_must_echo_target_read_opcode_and_register(offset, value):
    query = ParameterRead("joint_3", 3, 19, ReadRegister.POSITION_MAX)
    response = raw(query, 12.5)
    data = bytearray.fromhex(response.payload_hex)
    data[offset] = value
    with pytest.raises(ValueError, match="slave/opcode/register"):
        query.decode(replace(response, payload_hex=data.hex()), sent_ns=10, deadline_ns=30)


@pytest.mark.parametrize("timestamp", [9, 10, 31])
def test_old_or_late_reply_cannot_satisfy_a_new_query(timestamp):
    query = ParameterRead("joint_3", 3, 19, ReadRegister.POSITION_MAX)
    with pytest.raises(ValueError, match="identity/time"):
        query.decode(raw(query, 12.5, timestamp=timestamp), sent_ns=10, deadline_ns=30)


@pytest.mark.parametrize("register", [ReadRegister.MASTER_ID, ReadRegister.SLAVE_ID])
def test_reported_id_disagreement_is_not_silently_remapped(register):
    query = ParameterRead("joint_3", 3, 19, register)
    with pytest.raises(ValueError, match="differs from configured"):
        query.decode(raw(query, 999), sent_ns=10, deadline_ns=30)


@pytest.mark.parametrize(
    ("register", "payload", "expected"),
    [
        (ReadRegister.MASTER_ID, "0300330713000000", 19),
        (ReadRegister.SLAVE_ID, "0300330803000000", 3),
        (ReadRegister.HARDWARE_VERSION, "0300330d00000000", 0),
        (ReadRegister.FIRMWARE_VERSION, "0300330e35313137", 925970741),
        (ReadRegister.POSITION_MAX, "0300331500004841", 12.5),
        (ReadRegister.VELOCITY_MAX, "0300331600002041", 10.0),
        (ReadRegister.TORQUE_MAX, "030033170000e041", 28.0),
        (ReadRegister.SUB_VERSION, "0300332435000000", 53),
    ],
)
def test_recorded_right_j3_parameter_replies(register, payload, expected):
    # Eight received CAN-FD payloads from the authorized 2026-10-09 06:16 UTC
    # capture; provenance and limits of this evidence are in FORCE_ESTIMATION.md.
    query = ParameterRead("joint_3", 3, 19, register)
    response = RawMotorFrame("joint_3", 19, 20, payload, True)
    assert query.decode(response, sent_ns=10, deadline_ns=30) == expected
