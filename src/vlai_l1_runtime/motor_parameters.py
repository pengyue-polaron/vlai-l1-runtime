"""Pure, allowlisted DM parameter-read transactions. No transport or writes.

Protocol evidence: dmBots/motor-sdk revision
0b2ede457bdbf0882e29ab9958ab8fda047b7f4a, Python例程/u2can/DM_CAN.py.
Installed firmware compatibility still requires a separately approved test.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum

from .configuration import MOTOR_NAMES
from .feedback import RawMotorFrame


class ReadRegister(IntEnum):
    MASTER_ID = 7
    SLAVE_ID = 8
    TIMEOUT = 9  # Read-only raw communication-timeout register; units not assumed.
    CTRL_MODE = 10
    HARDWARE_VERSION = 13
    FIRMWARE_VERSION = 14
    POSITION_MAX = 21
    VELOCITY_MAX = 22
    TORQUE_MAX = 23
    SUB_VERSION = 36


# enactic/openarm_can f3ca25bbda1e239f4cc81f4563fbc75538fef874,
# include/openarm/damiao_motor/dm_motor_constants.hpp:
# RID::CTRL_MODE=10; ControlMode values below. These are protocol expectations,
# not evidence that any installed motor currently uses MIT.
class ControlMode(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    POS_FORCE = 4


# Preserve the existing identity/version/range survey; mode reads are explicit.
DEFAULT_READ_REGISTERS = tuple(
    r for r in ReadRegister if r not in (ReadRegister.CTRL_MODE, ReadRegister.TIMEOUT)
)


def require_mit_mode(value: object) -> None:
    if type(value) is not int or value != int(ControlMode.MIT):
        raise ValueError(f"CTRL_MODE must report MIT (1), received {value!r}")


@dataclass(frozen=True)
class ParameterRead:
    name: str
    slave_id: int
    response_id: int
    register: ReadRegister

    def __post_init__(self) -> None:
        if self.name not in MOTOR_NAMES or not isinstance(self.register, ReadRegister):
            raise ValueError("parameter read requires a named motor and allowlisted register")
        for label, value in (("slave_id", self.slave_id), ("response_id", self.response_id)):
            if isinstance(value, bool) or not isinstance(value, int) or not 0 < value < 0x7FF:
                raise ValueError(f"parameter read requires a nonzero standard {label}")

    @property
    def request(self) -> tuple[int, bytes]:
        """The sole available operation is 0x33 read; no arbitrary opcode input."""
        return 0x7FF, struct.pack("<HBB4x", self.slave_id, 0x33, self.register)

    def decode(self, raw: RawMotorFrame, *, sent_ns: int, deadline_ns: int) -> float | int:
        if (
            isinstance(sent_ns, bool)
            or not isinstance(sent_ns, int)
            or isinstance(deadline_ns, bool)
            or not isinstance(deadline_ns, int)
            or not 0 < sent_ns < deadline_ns
        ):
            raise ValueError("parameter transaction requires increasing positive times")
        if (
            not isinstance(raw, RawMotorFrame)
            or raw.name != self.name
            or raw.response_id != self.response_id
            or not sent_ns < raw.host_receive_monotonic_ns <= deadline_ns
        ):
            raise ValueError("parameter reply identity/time does not match the transaction")
        data = bytes.fromhex(raw.payload_hex)
        slave, opcode, register = struct.unpack_from("<HBB", data)
        if (slave, opcode, register) != (self.slave_id, 0x33, self.register):
            raise ValueError("parameter reply slave/opcode/register mismatch")
        if self.register in (
            ReadRegister.POSITION_MAX,
            ReadRegister.VELOCITY_MAX,
            ReadRegister.TORQUE_MAX,
        ):
            value = struct.unpack_from("<f", data, 4)[0]
            if not math.isfinite(value) or value <= 0:
                raise ValueError("parameter range must be finite and positive")
            return value
        value = struct.unpack_from("<I", data, 4)[0]
        if self.register == ReadRegister.CTRL_MODE and value not in ControlMode._value2member_map_:
            raise ValueError(f"unknown CTRL_MODE value: {value}")
        if self.register == ReadRegister.MASTER_ID and value != self.response_id:
            raise ValueError("reported master ID differs from configured response ID")
        if self.register == ReadRegister.SLAVE_ID and value != self.slave_id:
            raise ValueError("reported slave ID differs from configured motor ID")
        return value  # Version numbers remain raw integers; no guessed formatting.
