"""Motion-free x_air motor feedback probe used before the opaque teleop SDK."""

from __future__ import annotations

import ctypes
import math
import select
import socket
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import Any

from ..configuration import MOTOR_TYPE_CODES, MotorConfig, SystemConfig

_CAN_EFF_FLAG = 0x80000000
_CAN_RTR_FLAG = 0x40000000
_CAN_SFF_MASK = 0x000007FF
_CAN_FILTER = struct.Struct("=II")
_CAN_ID = struct.Struct("=I")
_CAN_MTU = 16
_CAN_FD_MTU = 72
_SDK_OK = 0
_RECEIVE_TIMEOUT_US = 2_000


@dataclass(frozen=True)
class MotorFeedbackProbeResult:
    interface: str
    rounds: int
    response_ids: tuple[int, ...]


class MotorFeedbackProbeError(RuntimeError):
    """A CAN endpoint did not prove complete motion-free motor feedback."""


class _CanSdkSession:
    def __init__(
        self,
        library_path: Path,
        interface: str,
        *,
        can_fd: bool,
        motors: tuple[MotorConfig, ...],
    ) -> None:
        self._library = ctypes.CDLL(str(library_path))
        self._configure_abi()
        self._handle = ctypes.c_void_p()
        self._check(
            self._library.xarm_sdk_create(
                interface.encode("utf-8"),
                int(can_fd),
                ctypes.byref(self._handle),
            ),
            "create",
        )
        try:
            arm = motors[:-1]
            gripper = motors[-1]
            motor_types = (ctypes.c_int * len(arm))(
                *(MOTOR_TYPE_CODES[motor.motor_type] for motor in arm)
            )
            send_ids = (ctypes.c_uint32 * len(arm))(*(motor.send_id for motor in arm))
            receive_ids = (ctypes.c_uint32 * len(arm))(*(motor.receive_id for motor in arm))
            self._check(
                self._library.xarm_sdk_init_arm_motors(
                    self._handle,
                    motor_types,
                    send_ids,
                    receive_ids,
                    len(arm),
                ),
                "initialize arm motors",
            )
            self._check(
                self._library.xarm_sdk_init_gripper_motor(
                    self._handle,
                    MOTOR_TYPE_CODES[gripper.motor_type],
                    gripper.send_id,
                    gripper.receive_id,
                ),
                "initialize gripper motor",
            )
        except BaseException:
            self.close()
            raise

    def _configure_abi(self) -> None:
        library = self._library
        library.xarm_sdk_create.argtypes = (
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_void_p),
        )
        library.xarm_sdk_create.restype = ctypes.c_int
        library.xarm_sdk_destroy.argtypes = (ctypes.c_void_p,)
        library.xarm_sdk_destroy.restype = ctypes.c_int
        library.xarm_sdk_init_arm_motors.argtypes = (
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_int,
        )
        library.xarm_sdk_init_arm_motors.restype = ctypes.c_int
        library.xarm_sdk_init_gripper_motor.argtypes = (
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_uint32,
            ctypes.c_uint32,
        )
        library.xarm_sdk_init_gripper_motor.restype = ctypes.c_int
        library.xarm_sdk_refresh_all.argtypes = (ctypes.c_void_p,)
        library.xarm_sdk_refresh_all.restype = ctypes.c_int
        library.xarm_sdk_recv_all.argtypes = (ctypes.c_void_p, ctypes.c_int)
        library.xarm_sdk_recv_all.restype = ctypes.c_int
        library.xarm_sdk_get_last_error.argtypes = (ctypes.c_char_p, ctypes.c_int)
        library.xarm_sdk_get_last_error.restype = ctypes.c_int

    def _check(self, result: int, operation: str) -> None:
        if result == _SDK_OK:
            return
        detail = ctypes.create_string_buffer(512)
        self._library.xarm_sdk_get_last_error(detail, len(detail))
        message = detail.value.decode("utf-8", errors="replace").strip() or "unknown SDK error"
        raise MotorFeedbackProbeError(f"x_air CAN probe {operation} failed: {message}")

    def refresh(self) -> None:
        self._check(self._library.xarm_sdk_refresh_all(self._handle), "refresh")
        self._check(
            self._library.xarm_sdk_recv_all(self._handle, _RECEIVE_TIMEOUT_US),
            "receive",
        )

    def close(self) -> None:
        if not self._handle:
            return
        handle, self._handle = self._handle, ctypes.c_void_p()
        result = self._library.xarm_sdk_destroy(handle)
        if result != _SDK_OK:
            self._check(result, "destroy")

    def __enter__(self) -> _CanSdkSession:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        try:
            self.close()
        except MotorFeedbackProbeError:
            if exc_type is None:
                raise


class _SocketCanProbeBackend:
    def __init__(
        self,
        library_path: Path,
        interface: str,
        *,
        can_fd: bool,
        motors: tuple[MotorConfig, ...],
    ) -> None:
        self._expected = frozenset(motor.receive_id for motor in motors)
        self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self._socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            filters = b"".join(
                _CAN_FILTER.pack(
                    receive_id,
                    _CAN_SFF_MASK | _CAN_EFF_FLAG | _CAN_RTR_FLAG,
                )
                for receive_id in sorted(self._expected)
            )
            self._socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self._socket.setblocking(False)
            self._socket.bind((interface,))
            self._session = _CanSdkSession(
                library_path,
                interface,
                can_fd=can_fd,
                motors=motors,
            )
        except BaseException:
            self._socket.close()
            raise

    def sample_round(self, period_s: float) -> frozenset[int]:
        self._drain()
        self._session.refresh()
        deadline = time.monotonic() + period_s
        observed: set[int] = set()
        while time.monotonic() < deadline:
            observed.update(self._drain())
            remaining = self._expected - observed
            if not remaining:
                time.sleep(max(0.0, deadline - time.monotonic()))
                break
            timeout = min(0.002, max(0.0, deadline - time.monotonic()))
            if timeout:
                select.select((self._socket,), (), (), timeout)
        observed.update(self._drain())
        return frozenset(observed)

    def _drain(self) -> set[int]:
        observed: set[int] = set()
        while True:
            try:
                payload = self._socket.recv(_CAN_FD_MTU)
            except BlockingIOError:
                break
            if len(payload) not in (_CAN_MTU, _CAN_FD_MTU):
                raise MotorFeedbackProbeError(
                    f"x_air CAN probe received a malformed {len(payload)}-byte frame"
                )
            can_id = _CAN_ID.unpack_from(payload)[0]
            if can_id & (_CAN_EFF_FLAG | _CAN_RTR_FLAG):
                continue
            response_id = can_id & _CAN_SFF_MASK
            if response_id in self._expected:
                observed.add(response_id)
        return observed

    def close(self) -> None:
        try:
            self._session.close()
        finally:
            self._socket.close()

    def __enter__(self) -> _SocketCanProbeBackend:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        try:
            self.close()
        except MotorFeedbackProbeError:
            if exc_type is None:
                raise


def probe_motor_feedback(
    config: SystemConfig,
    interface: str,
    can_library: Path,
    *,
    _backend_factory: Callable[..., Any] | None = None,
) -> MotorFeedbackProbeResult:
    """Require repeated feedback from every configured motor without enabling it."""

    duration_s = config.teleoperation.motor_probe_duration_s
    rate_hz = config.teleoperation.motor_probe_rate_hz
    rounds_float = duration_s * rate_hz
    if not math.isfinite(rounds_float) or not rounds_float.is_integer():
        raise ValueError("tracked x_air motor probe rounds are invalid")
    rounds = int(rounds_float)
    period_s = 1.0 / rate_hz
    expected = frozenset(motor.receive_id for motor in config.motors)
    factory = _SocketCanProbeBackend if _backend_factory is None else _backend_factory
    with factory(
        can_library,
        interface,
        can_fd=config.can.fd,
        motors=config.motors,
    ) as backend:
        for round_index in range(1, rounds + 1):
            observed = backend.sample_round(period_s)
            missing = expected - observed
            unexpected = observed - expected
            if unexpected:
                rendered = ", ".join(f"0x{value:03X}" for value in sorted(unexpected))
                raise MotorFeedbackProbeError(
                    f"{interface} motor probe returned unexpected response ids: {rendered}"
                )
            if missing:
                rendered = ", ".join(f"0x{value:03X}" for value in sorted(missing))
                raise MotorFeedbackProbeError(
                    f"{interface} motor probe round {round_index}/{rounds} "
                    f"missed response ids: {rendered}"
                )
    return MotorFeedbackProbeResult(interface, rounds, tuple(sorted(expected)))
