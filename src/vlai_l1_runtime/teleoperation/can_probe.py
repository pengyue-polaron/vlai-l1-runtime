"""Motion-free x_air motor feedback probe used before the opaque teleop SDK."""

from __future__ import annotations

import ctypes
import math
import select
import socket
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from types import TracebackType
from typing import Any

from ..configuration import MOTOR_TYPE_CODES, MotorConfig, SystemConfig
from ..feedback import MotorFeedback, RawMotorFrame

_CAN_EFF_FLAG = 0x80000000
_CAN_RTR_FLAG = 0x40000000
_CAN_ERR_FLAG = 0x20000000
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


class _JointState(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("pos", "vel", "torque")]


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
        self._motor_names = tuple(motor.name for motor in motors)
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

    def read_feedback(self) -> MotorFeedback:
        """Read this owner's existing handle; never refresh or enable motors here."""

        library = self._library
        library.xarm_sdk_get_arm_joint_states.argtypes = (
            ctypes.c_void_p,
            ctypes.POINTER(_JointState),
            ctypes.c_int,
        )
        library.xarm_sdk_get_arm_joint_states.restype = ctypes.c_int
        library.xarm_sdk_get_gripper_state.argtypes = (
            ctypes.c_void_p,
            ctypes.POINTER(_JointState),
        )
        library.xarm_sdk_get_gripper_state.restype = ctypes.c_int
        # Responses can arrive after refresh()'s first receive; drain without TX.
        self._check(library.xarm_sdk_recv_all(self._handle, _RECEIVE_TIMEOUT_US), "receive")
        arm = (_JointState * (len(self._motor_names) - 1))()
        gripper = _JointState()
        self._check(
            library.xarm_sdk_get_arm_joint_states(self._handle, arm, len(arm)),
            "read arm feedback",
        )
        self._check(
            library.xarm_sdk_get_gripper_state(self._handle, ctypes.byref(gripper)),
            "read gripper feedback",
        )
        states = (*arm, gripper)
        return MotorFeedback(
            self._motor_names,
            time.monotonic_ns(),
            tuple(state.pos for state in states),
            tuple(state.vel for state in states),
            tuple(state.torque for state in states),
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
        self._motors = motors
        self._raw_frames: dict[int, RawMotorFrame] = {}
        self._raw_counts: dict[int, int] = {}
        self._names_by_id = {motor.receive_id: motor.name for motor in motors}
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
        self._raw_frames.clear()
        self._raw_counts.clear()
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
            if can_id & (_CAN_EFF_FLAG | _CAN_RTR_FLAG | _CAN_ERR_FLAG):
                continue
            response_id = can_id & _CAN_SFF_MASK
            if response_id in self._expected:
                if payload[4] != 8:
                    raise MotorFeedbackProbeError(
                        f"motor 0x{response_id:03X} feedback must have eight data bytes"
                    )
                self._raw_frames[response_id] = RawMotorFrame(
                    self._names_by_id[response_id],
                    response_id,
                    time.monotonic_ns(),
                    payload[8:16].hex(),
                    len(payload) == _CAN_FD_MTU,
                )
                self._raw_counts[response_id] = self._raw_counts.get(response_id, 0) + 1
                observed.add(response_id)
        return observed

    def close(self) -> None:
        try:
            self._session.close()
        finally:
            self._socket.close()

    def read_feedback(self) -> MotorFeedback:
        if set(self._raw_frames) != self._expected:
            raise MotorFeedbackProbeError("raw feedback round is incomplete")
        if any(count != 1 for count in self._raw_counts.values()):
            raise MotorFeedbackProbeError("raw/SDK pairing is ambiguous: duplicate motor responses")
        return replace(
            self._session.read_feedback(),
            raw_frames=tuple(self._raw_frames[motor.receive_id] for motor in self._motors),
        )

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
    on_feedback: Callable[[int, MotorFeedback], None] | None = None,
    check_health: Callable[[], None] | None = None,
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
        previous_timestamp = 0
        previous_raw_timestamps: dict[str, int] = {}
        for round_index in range(1, rounds + 1):
            if check_health is not None:
                check_health()
            observed = backend.sample_round(period_s)
            if check_health is not None:
                check_health()
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
            if on_feedback is not None:
                feedback = backend.read_feedback()
                if not isinstance(feedback, MotorFeedback):
                    raise MotorFeedbackProbeError("probe backend returned invalid feedback")
                if feedback.monotonic_ns <= previous_timestamp:
                    raise MotorFeedbackProbeError("feedback host timestamp did not increase")
                previous_timestamp = feedback.monotonic_ns
                for frame in feedback.raw_frames or ():
                    if frame.host_receive_monotonic_ns <= previous_raw_timestamps.get(
                        frame.name, 0
                    ):
                        raise MotorFeedbackProbeError(
                            "raw feedback host timestamp did not increase"
                        )
                    previous_raw_timestamps[frame.name] = frame.host_receive_monotonic_ns
                on_feedback(round_index, feedback)
    return MotorFeedbackProbeResult(interface, rounds, tuple(sorted(expected)))
