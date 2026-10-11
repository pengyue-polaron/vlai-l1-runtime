"""Pure VL1T/1 raw Follower telemetry parsing and session checks; no IO.

This protocol is separate from live VL1S position state and is not activated.
Passing these checks does not establish physical calibration or FT validity.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .configuration import MOTOR_NAMES, SIDES
from .feedback import MotorFeedback, RawMotorFrame

_HEADER = struct.Struct("<4sHBB16s32sQQ")
_MOTOR = struct.Struct("<HBBQ8sfff")
PACKET_SIZE = _HEADER.size + len(MOTOR_NAMES) * _MOTOR.size
_UINT64_MAX = (1 << 64) - 1


def _positive_integer(value: int, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value <= _UINT64_MAX:
        raise ValueError(f"{label} must be a positive uint64")


def _identity(value: bytes, size: int, label: str) -> None:
    if not isinstance(value, bytes) or len(value) != size or not any(value):
        raise ValueError(f"{label} must contain {size} bytes and be nonzero")


def _endpoint(payload: bytes) -> bool:
    position = (payload[1] << 8) | payload[2]
    velocity = (payload[3] << 4) | (payload[4] >> 4)
    torque = ((payload[4] & 15) << 8) | payload[5]
    return position in (0, 65535) or velocity in (0, 4095) or torque in (0, 4095)


@dataclass(frozen=True)
class FeedbackTelemetryPacket:
    side: str
    session_id: bytes
    config_sha256: bytes
    source_sequence: int
    feedback: MotorFeedback
    at_protocol_endpoint: tuple[bool, ...]

    def __post_init__(self) -> None:
        if self.side not in SIDES:
            raise ValueError("invalid Follower telemetry side")
        _identity(self.session_id, 16, "session_id")
        _identity(self.config_sha256, 32, "config_sha256")
        _positive_integer(self.source_sequence, "source_sequence")
        if not isinstance(self.feedback, MotorFeedback) or self.feedback.raw_frames is None:
            raise ValueError("telemetry requires paired raw feedback")
        _positive_integer(self.feedback.monotonic_ns, "assembly time")
        if (
            not isinstance(self.at_protocol_endpoint, tuple)
            or len(self.at_protocol_endpoint) != len(MOTOR_NAMES)
            or any(not isinstance(value, bool) for value in self.at_protocol_endpoint)
        ):
            raise ValueError("endpoint flags must be an ordered eight-boolean tuple")
        for raw, flag in zip(self.feedback.raw_frames, self.at_protocol_endpoint, strict=True):
            if flag != _endpoint(bytes.fromhex(raw.payload_hex)):
                raise ValueError(f"inconsistent protocol endpoint flag for {raw.name}")

    @classmethod
    def decode(cls, payload: bytes) -> FeedbackTelemetryPacket:
        if not isinstance(payload, bytes) or len(payload) != PACKET_SIZE:
            raise ValueError(f"Follower telemetry requires exactly {PACKET_SIZE} bytes")
        magic, version, side, reserved, session, digest, sequence, assembled = _HEADER.unpack_from(
            payload
        )
        if magic != b"VL1T" or version != 1 or reserved != 0 or side not in (0, 1):
            raise ValueError("invalid Follower telemetry header")
        raw_frames, positions, velocities, torques, endpoints = [], [], [], [], []
        for index, name in enumerate(MOTOR_NAMES):
            response_id, flags, reserved, received, raw, q, dq, tau = _MOTOR.unpack_from(
                payload, _HEADER.size + index * _MOTOR.size
            )
            if flags & ~3 or reserved != 0:
                raise ValueError(f"invalid Follower telemetry flags for {name}")
            raw_frames.append(
                RawMotorFrame(name, response_id, received, raw.hex(), bool(flags & 1))
            )
            positions.append(q)
            velocities.append(dq)
            torques.append(tau)
            endpoints.append(bool(flags & 2))
        return cls(
            SIDES[side],
            session,
            digest,
            sequence,
            MotorFeedback(
                MOTOR_NAMES,
                assembled,
                tuple(positions),
                tuple(velocities),
                tuple(torques),
                tuple(raw_frames),
            ),
            tuple(endpoints),
        )


class FeedbackSessionError(ValueError):
    """The telemetry session is unusable; a new session is required."""


class FeedbackSession:
    """Validate an explicitly identified local-host telemetry session.

    All policy inputs are required. Future live construction must use reviewed
    Runtime configuration. No firmware status meanings or physical defaults are
    inferred here. The owner must call check_deadline even when packets stop.
    Host dequeue timestamps alone cannot establish kernel queue/motor freshness.
    """

    def __init__(
        self,
        *,
        side: str,
        session_id: bytes,
        config_sha256: bytes,
        response_ids: tuple[int, ...],
        slave_ids: tuple[int, ...],
        allowed_statuses: tuple[frozenset[int], ...],
        started_ns: int,
        max_age_ns: int,
        max_motor_skew_ns: int,
        startup_timeout_ns: int,
        silence_timeout_ns: int,
    ) -> None:
        if side not in SIDES:
            raise ValueError("invalid Follower telemetry side")
        _identity(session_id, 16, "session_id")
        _identity(config_sha256, 32, "config_sha256")
        for label, value in (
            ("started_ns", started_ns),
            ("max_age_ns", max_age_ns),
            ("max_motor_skew_ns", max_motor_skew_ns),
            ("startup_timeout_ns", startup_timeout_ns),
            ("silence_timeout_ns", silence_timeout_ns),
        ):
            _positive_integer(value, label)
        if max_motor_skew_ns > max_age_ns:
            raise ValueError("max_motor_skew_ns must not exceed max_age_ns")
        for label, values, lower, upper in (
            ("response_ids", response_ids, 0, 0x7FF),
            ("slave_ids", slave_ids, 1, 15),
        ):
            if (
                not isinstance(values, tuple)
                or len(values) != len(MOTOR_NAMES)
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, int)
                    or not lower <= value <= upper
                    for value in values
                )
                or len(set(values)) != len(values)
            ):
                raise ValueError(f"{label} requires eight distinct valid IDs in motor order")
        if (
            not isinstance(allowed_statuses, tuple)
            or len(allowed_statuses) != len(MOTOR_NAMES)
            or any(
                not isinstance(values, frozenset)
                or not values
                or any(
                    isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 15
                    for value in values
                )
                for values in allowed_statuses
            )
        ):
            raise ValueError("allowed_statuses requires eight explicit nonempty nibble sets")
        self._side, self._session_id, self._config_sha256 = side, session_id, config_sha256
        self._response_ids, self._slave_ids = response_ids, slave_ids
        self._allowed_statuses = allowed_statuses
        self._started_ns = started_ns
        self._max_age_ns, self._max_skew_ns = max_age_ns, max_motor_skew_ns
        self._startup_timeout_ns, self._silence_timeout_ns = startup_timeout_ns, silence_timeout_ns
        self._last_now_ns = started_ns
        self._last_received_ns: int | None = None
        self._last: FeedbackTelemetryPacket | None = None
        self._fault: str | None = None

    @property
    def faulted(self) -> bool:
        return self._fault is not None

    def invalidate(self, reason: str) -> None:
        """Latch a transport/owner fault, e.g. queue overflow or failed CAN health."""
        if self._fault is None:
            self._fault = reason or "unspecified transport fault"
        self._last = None

    def _healthy(self) -> None:
        if self.faulted:
            raise FeedbackSessionError(f"Follower telemetry session fault: {self._fault}")

    def check_deadline(self, *, now_ns: int) -> None:
        self._healthy()
        try:
            _positive_integer(now_ns, "now_ns")
            if now_ns < self._last_now_ns:
                raise ValueError("consumer monotonic time regressed")
            self._last_now_ns = now_ns
            if self._last_received_ns is None:
                if now_ns - self._started_ns > self._startup_timeout_ns:
                    raise ValueError("startup feedback deadline exceeded")
            elif now_ns - self._last_received_ns > self._silence_timeout_ns:
                raise ValueError("feedback silence deadline exceeded")
            if self._last is not None:
                assert self._last.feedback.raw_frames is not None
                if any(
                    now_ns - raw.host_receive_monotonic_ns > self._max_age_ns
                    for raw in self._last.feedback.raw_frames
                ):
                    raise ValueError("last motor feedback expired")
        except ValueError as error:
            self.invalidate(str(error))
            raise FeedbackSessionError(str(error)) from error

    def accept(self, payload: bytes, *, now_ns: int) -> FeedbackTelemetryPacket:
        self.check_deadline(now_ns=now_ns)
        try:
            packet = FeedbackTelemetryPacket.decode(payload)
            if (
                packet.side != self._side
                or packet.session_id != self._session_id
                or packet.config_sha256 != self._config_sha256
            ):
                raise ValueError("unexpected telemetry side/session/config identity")
            feedback = packet.feedback
            if not self._started_ns <= feedback.monotonic_ns <= now_ns:
                raise ValueError("assembly time outside current session/consumer clock")
            if self._last is not None and (
                packet.source_sequence <= self._last.source_sequence
                or feedback.monotonic_ns <= self._last.feedback.monotonic_ns
            ):
                raise ValueError("source sequence/assembly time did not increase")
            assert feedback.raw_frames is not None
            timestamps = []
            for index, raw in enumerate(feedback.raw_frames):
                data = bytes.fromhex(raw.payload_hex)
                if (
                    raw.response_id != self._response_ids[index]
                    or data[0] & 15 != self._slave_ids[index]
                ):
                    raise ValueError(f"unexpected motor identity for {raw.name}")
                if data[0] >> 4 not in self._allowed_statuses[index]:
                    raise ValueError(f"unaccepted raw motor status for {raw.name}: {data[0] >> 4}")
                if packet.at_protocol_endpoint[index]:
                    raise ValueError(f"protocol endpoint value for {raw.name}")
                received = raw.host_receive_monotonic_ns
                if received < self._started_ns or now_ns - received > self._max_age_ns:
                    raise ValueError(f"expired/pre-session motor feedback for {raw.name}")
                if self._last is not None:
                    assert self._last.feedback.raw_frames is not None
                    previous = self._last.feedback.raw_frames[index].host_receive_monotonic_ns
                    if received <= previous:
                        raise ValueError(f"motor receive time did not increase for {raw.name}")
                timestamps.append(received)
            if max(timestamps) - min(timestamps) > self._max_skew_ns:
                raise ValueError("motor feedback skew exceeds policy")
        except ValueError as error:
            self.invalidate(str(error))
            raise FeedbackSessionError(str(error)) from error
        self._last = packet
        self._last_received_ns = now_ns
        return packet
