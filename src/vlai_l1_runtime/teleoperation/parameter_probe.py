"""Bounded parameter queries for an already-guarded, idle Follower endpoint.

Only feedback_capture calls this live transport after its full state probe has
closed the SDK handle. Import/description/tests never create a socket.
"""

from __future__ import annotations

import select
import socket
import struct
import time
from collections.abc import Callable
from dataclasses import asdict
from typing import Any

from ..configuration import MotorConfig, SystemConfig
from ..feedback import RawMotorFrame
from ..motor_parameters import (
    DEFAULT_READ_REGISTERS,
    ParameterRead,
    ReadRegister,
    require_mit_mode,
)

_SO_RXQ_OVFL = 40  # Linux socket ABI, not a robot-specific setting.
_CAN_RAW_ERR_FILTER = 2


class _ParameterSocket:
    def __init__(self, interface: str, motor: MotorConfig, *, can_fd: bool) -> None:
        self._motor = motor
        self._can_fd = can_fd
        self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self._socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            self._socket.setsockopt(socket.SOL_SOCKET, _SO_RXQ_OVFL, 1)
            self._socket.setsockopt(
                socket.SOL_CAN_RAW, _CAN_RAW_ERR_FILTER, struct.pack("=I", 0x1FFFFFFF)
            )
            self._socket.setsockopt(
                socket.SOL_CAN_RAW,
                socket.CAN_RAW_FILTER,
                struct.pack("=II", motor.receive_id, 0x7FF | 0x80000000 | 0x40000000),
            )
            self._socket.setblocking(False)
            self._socket.bind((interface,))
        except BaseException:
            self._socket.close()
            raise

    def __enter__(self) -> _ParameterSocket:
        return self

    def __exit__(self, *args: object) -> None:
        self._socket.close()

    def query(self, request: ParameterRead, *, deadline_ns: int) -> dict[str, Any]:
        if (request.name, request.slave_id, request.response_id) != (
            self._motor.name,
            self._motor.send_id,
            self._motor.receive_id,
        ):
            raise ValueError("parameter query target differs from the socket owner")
        if select.select((self._socket,), (), (), 0)[0]:
            raise RuntimeError("unsolicited/late feedback before parameter query; no query sent")
        sent_ns = time.monotonic_ns()
        if sent_ns >= deadline_ns:
            raise TimeoutError("parameter query window expired before send")
        can_id, payload = request.request
        frame = struct.pack("=IB3x64s" if self._can_fd else "=IB3x8s", can_id, 8, payload)
        if self._socket.send(frame) != len(frame):
            raise RuntimeError("short parameter query send; no retry")
        result = None
        while (remaining_ns := deadline_ns - time.monotonic_ns()) > 0:
            if not select.select((self._socket,), (), (), remaining_ns / 1e9)[0]:
                continue
            data, ancillary, flags, _address = self._socket.recvmsg(72, socket.CMSG_SPACE(4))
            received_ns = time.monotonic_ns()
            if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC) or len(data) not in (16, 72):
                raise RuntimeError("truncated/malformed parameter CAN frame")
            for level, kind, value in ancillary:
                if level != socket.SOL_SOCKET or kind != _SO_RXQ_OVFL or len(value) != 4:
                    raise RuntimeError("unexpected parameter socket metadata")
                if struct.unpack("=I", value)[0] != 0:
                    raise RuntimeError("parameter receive queue overflow")
            response_id = struct.unpack_from("=I", data)[0]
            if response_id != request.response_id or data[4] != 8:
                raise RuntimeError("parameter response ID/flags/length mismatch or CAN error")
            raw = RawMotorFrame(
                request.name, response_id, received_ns, data[8:16].hex(), len(data) == 72
            )
            value = request.decode(raw, sent_ns=sent_ns, deadline_ns=deadline_ns)
            if result is not None:
                raise RuntimeError("duplicate parameter reply; transaction is ambiguous")
            result = {
                "motor": request.name,
                "register": request.register.name,
                "register_id": int(request.register),
                "sent_monotonic_ns": sent_ns,
                "request_id": can_id,
                "request_payload_hex": payload.hex(),
                "value": value,
                "raw_response": asdict(raw),
            }
        if result is None:
            raise TimeoutError(f"no parameter reply for {request.name}/{request.register.name}")
        return result


def read_motor_parameters(
    config: SystemConfig,
    interface: str,
    motor: MotorConfig,
    *,
    check_health: Callable[[], None],
    on_reading: Callable[[dict[str, Any]], None],
    role: str = "follower",
    registers: tuple[ReadRegister, ...] = DEFAULT_READ_REGISTERS,
    endpoint_deadline_ns: int | None = None,
    _backend_factory: Any = _ParameterSocket,
) -> None:
    """Live read-only protocol transactions, no retry or state-changing opcode.

    Reuses the tracked diagnostic window and query period as upper bounds. The
    guarded caller owns interface identity, lock, down/cleanup and authorization.
    This helper is not a standalone hardware entry point.
    """
    if (
        role not in ("follower", "leader")
        or motor not in config.motors
        or not any(
            endpoint.interface == interface and endpoint.role == role
            for endpoint in config.can.endpoints
        )
    ):
        raise ValueError("parameter query requires the explicit configured role/motor")
    if (
        not isinstance(registers, tuple)
        or not registers
        or any(not isinstance(register, ReadRegister) for register in registers)
        or len(set(registers)) != len(registers)
    ):
        raise ValueError("parameter reads require distinct allowlisted registers")
    requests = tuple(
        ParameterRead(motor.name, motor.send_id, motor.receive_id, register)
        for register in registers
    )
    deadline_ns = time.monotonic_ns() + int(config.teleoperation.motor_probe_duration_s * 1e9)
    if endpoint_deadline_ns is not None:
        if type(endpoint_deadline_ns) is not int or endpoint_deadline_ns <= 0:
            raise ValueError("endpoint deadline must be a positive monotonic timestamp")
        deadline_ns = min(deadline_ns, endpoint_deadline_ns)
    period_ns = int(1e9 / config.teleoperation.motor_probe_rate_hz)
    with _backend_factory(interface, motor, can_fd=config.can.fd) as backend:
        for request in requests:
            check_health()
            query_deadline_ns = min(deadline_ns, time.monotonic_ns() + period_ns)
            result = backend.query(
                request,
                deadline_ns=query_deadline_ns,
            )
            check_health()
            on_reading(result)


def read_motor_control_mode(
    config: SystemConfig,
    interface: str,
    motor: MotorConfig,
    *,
    check_health: Callable[[], None],
    on_reading: Callable[[dict[str, Any]], None],
    role: str = "follower",
    endpoint_deadline_ns: int | None = None,
    _backend_factory: Any = _ParameterSocket,
) -> None:
    """Explicit guarded mode-only read; preserve evidence before checking MIT.

    No SDK/OpenArm initializer, mode write, enable, disable or motion command.
    The caller owns endpoint identity, exclusivity, authorization and cleanup,
    exactly as for read_motor_parameters. This is not a live entry point.
    """

    def accept(reading: dict[str, Any]) -> None:
        on_reading(reading)
        if reading.get("register") != "CTRL_MODE":
            raise ValueError("missing CTRL_MODE reply")
        require_mit_mode(reading.get("value"))

    read_motor_parameters(
        config,
        interface,
        motor,
        check_health=check_health,
        on_reading=accept,
        role=role,
        registers=(ReadRegister.CTRL_MODE,),
        endpoint_deadline_ns=endpoint_deadline_ns,
        _backend_factory=_backend_factory,
    )
