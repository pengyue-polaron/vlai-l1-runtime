"""Named raw motor telemetry. Importing this module never opens hardware."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .configuration import MOTOR_NAMES


@dataclass(frozen=True)
class RawMotorFrame:
    """One received feedback payload; timestamp is host dequeue time."""

    name: str
    response_id: int
    host_receive_monotonic_ns: int
    payload_hex: str
    can_fd: bool

    def __post_init__(self) -> None:
        if self.name not in MOTOR_NAMES:
            raise ValueError("raw feedback motor name is invalid")
        if (
            isinstance(self.response_id, bool)
            or not isinstance(self.response_id, int)
            or not 0 <= self.response_id <= 0x7FF
        ):
            raise ValueError("raw feedback requires a standard CAN response ID")
        if (
            isinstance(self.host_receive_monotonic_ns, bool)
            or not isinstance(self.host_receive_monotonic_ns, int)
            or self.host_receive_monotonic_ns <= 0
        ):
            raise ValueError("raw feedback timestamp must be a positive integer")
        if (
            not isinstance(self.payload_hex, str)
            or len(self.payload_hex) != 16
            or any(character not in "0123456789abcdef" for character in self.payload_hex)
        ):
            raise ValueError("raw feedback must contain eight bytes as lowercase hex")
        if not isinstance(self.can_fd, bool):
            raise ValueError("raw feedback can_fd must be boolean")


@dataclass(frozen=True)
class MotorFeedback:
    """Raw feedback snapshot in motor coordinates, not a calibrated joint wrench.

    ``monotonic_ns`` is producer completion: SDK cache read completion for the
    diagnostic, or batch assembly time for native telemetry. Neither is a
    hardware sample timestamp; each transport must document its source.
    A complete probe round establishes responses, not simultaneous sampling.
    """

    names: tuple[str, ...]
    monotonic_ns: int
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    torque_nm: tuple[float, ...]
    raw_frames: tuple[RawMotorFrame, ...] | None = None

    def __post_init__(self) -> None:
        if self.names != MOTOR_NAMES:
            raise ValueError("feedback requires the exact configured eight-motor order")
        if (
            isinstance(self.monotonic_ns, bool)
            or not isinstance(self.monotonic_ns, int)
            or self.monotonic_ns <= 0
        ):
            raise ValueError("feedback monotonic_ns must be a positive integer")
        for label in ("position_rad", "velocity_rad_s", "torque_nm"):
            values = getattr(self, label)
            if not isinstance(values, tuple) or len(values) != len(self.names):
                raise ValueError(f"feedback {label} requires exactly eight values")
            if any(
                isinstance(value, bool)
                or not isinstance(value, (float, int))
                or not math.isfinite(value)
                for value in values
            ):
                raise ValueError(f"feedback {label} must contain finite numbers")
        if self.raw_frames is not None:
            if (
                not isinstance(self.raw_frames, tuple)
                or any(not isinstance(frame, RawMotorFrame) for frame in self.raw_frames)
                or tuple(frame.name for frame in self.raw_frames) != self.names
                or len({frame.response_id for frame in self.raw_frames}) != len(self.names)
            ):
                raise ValueError("raw feedback requires eight ordered motors with distinct IDs")
            if any(
                frame.host_receive_monotonic_ns > self.monotonic_ns for frame in self.raw_frames
            ):
                raise ValueError("raw feedback receive time is after SDK read completion")
