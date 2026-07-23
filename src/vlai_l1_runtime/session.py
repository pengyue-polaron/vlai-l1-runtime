"""Pure reference state machine for exclusive command ownership."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .configuration import SystemConfig, _is_loader_validated
from .contracts import (
    CommandEnvelope,
    ContractError,
    NamedJointVector,
    _snapshot_command_envelope,
    _snapshot_named_joint_vector,
    validate_contiguous_command,
    validate_first_command_hold,
    validate_named_values,
)


class SessionError(RuntimeError):
    """The requested session transition is invalid or unsafe."""


class SessionMode(str, Enum):
    DISCONNECTED = "disconnected"
    READ_ONLY = "read_only"
    COMMAND = "command"


@dataclass(frozen=True)
class _SessionPolicy:
    command_ready: bool
    liveness_timeout_ns: int
    command_inactivity_timeout_ns: int
    first_command_hold_tolerance_deg: float

    def __post_init__(self) -> None:
        if not isinstance(self.command_ready, bool):
            raise ValueError("command_ready must be a boolean")
        if (
            isinstance(self.liveness_timeout_ns, bool)
            or not isinstance(self.liveness_timeout_ns, int)
            or isinstance(self.command_inactivity_timeout_ns, bool)
            or not isinstance(self.command_inactivity_timeout_ns, int)
            or self.liveness_timeout_ns <= 0
            or self.command_inactivity_timeout_ns <= 0
        ):
            raise ValueError("session timeouts must be positive")
        if (
            isinstance(self.first_command_hold_tolerance_deg, bool)
            or not isinstance(self.first_command_hold_tolerance_deg, (int, float))
            or not math.isfinite(self.first_command_hold_tolerance_deg)
            or self.first_command_hold_tolerance_deg < 0
        ):
            raise ValueError("first-command hold tolerance must be finite and non-negative")


class CommandSession:
    """Model lease semantics without owning or touching a physical resource."""

    def __init__(self, config: SystemConfig) -> None:
        if not isinstance(config, SystemConfig):
            raise TypeError("CommandSession requires a validated SystemConfig")
        try:
            loader_validated = _is_loader_validated(config)
        except (AttributeError, KeyError, TypeError, ValueError):
            loader_validated = False
        if not loader_validated:
            raise ValueError("CommandSession rejects modified or fabricated SystemConfig values")
        expected_ready = not config.command_blockers
        if config.safety.command_ready != expected_ready:
            raise ValueError("SystemConfig command readiness is internally inconsistent")
        policy = _SessionPolicy(
            command_ready=config.safety.command_ready,
            liveness_timeout_ns=math.ceil(config.safety.session_liveness_timeout_s * 1e9),
            command_inactivity_timeout_ns=math.ceil(
                config.safety.command_inactivity_timeout_s * 1e9
            ),
            first_command_hold_tolerance_deg=(config.safety.first_command_hold_tolerance_deg),
        )
        self._initialize(policy)

    @classmethod
    def _for_hardware_free_test(
        cls,
        policy: _SessionPolicy,
    ) -> CommandSession:
        """Build the pure state machine without weakening the production constructor."""

        session = cls.__new__(cls)
        session._initialize(policy)
        return session

    def _initialize(self, policy: _SessionPolicy) -> None:
        if not isinstance(policy, _SessionPolicy):
            raise TypeError("policy must be a _SessionPolicy")
        self.policy = policy
        self.mode = SessionMode.DISCONNECTED
        self.lease_id: str | None = None
        self.last_heartbeat_ns: int | None = None
        self.last_command_accepted_ns: int | None = None
        self.previous_sequence: int | None = None
        self.previous_command_monotonic_ns: int | None = None
        self.release_reason: str | None = None
        self._previous_feedback_sequence: int | None = None
        self._previous_feedback_monotonic_ns: int | None = None
        self._lease_acquired_ns: int | None = None

    def connect_read_only(self, *, now_ns: int) -> None:
        self._require_time(now_ns)
        if self.mode is not SessionMode.DISCONNECTED:
            raise SessionError("session is already connected")
        self.mode = SessionMode.READ_ONLY
        self.release_reason = None

    def disconnect(self) -> None:
        self._clear_command_state()
        self.mode = SessionMode.DISCONNECTED

    def acquire_command(self, *, lease_id: str, measured: NamedJointVector, now_ns: int) -> None:
        self._require_time(now_ns)
        if self.mode is not SessionMode.READ_ONLY:
            raise SessionError("command lease requires a read-only session")
        if not self.policy.command_ready:
            raise SessionError("tracked command readiness gates are not satisfied")
        if not isinstance(lease_id, str) or not lease_id or lease_id != lease_id.strip():
            raise SessionError("lease_id must be normalized non-empty text")
        try:
            measured_snapshot = _snapshot_named_joint_vector(measured)
            validate_named_values(measured_snapshot.values)
        except ContractError as exc:
            raise SessionError(str(exc)) from exc
        if measured_snapshot.metadata.monotonic_ns > now_ns:
            raise SessionError("measured pose timestamp cannot be in the future")
        if (
            now_ns - measured_snapshot.metadata.monotonic_ns
            > self.policy.command_inactivity_timeout_ns
        ):
            raise SessionError("measured pose is too old for command acquisition")
        self.mode = SessionMode.COMMAND
        self.lease_id = lease_id
        self.last_heartbeat_ns = now_ns
        self.last_command_accepted_ns = now_ns
        self.previous_sequence = None
        self.previous_command_monotonic_ns = None
        self.release_reason = None
        self._previous_feedback_sequence = measured_snapshot.metadata.source_sequence
        self._previous_feedback_monotonic_ns = measured_snapshot.metadata.monotonic_ns
        self._lease_acquired_ns = now_ns

    def heartbeat(self, *, lease_id: str, now_ns: int) -> None:
        self._require_active_lease(lease_id)
        self.check_timeouts(now_ns=now_ns)
        if self.mode is not SessionMode.COMMAND:
            raise SessionError(f"command lease was released: {self.release_reason}")
        if self.last_heartbeat_ns is not None and now_ns < self.last_heartbeat_ns:
            self._fail_closed("non_monotonic_heartbeat")
            raise SessionError("heartbeat time moved backwards")
        self.last_heartbeat_ns = now_ns

    def accept(
        self,
        command: CommandEnvelope,
        *,
        now_ns: int,
        feedback: NamedJointVector | None = None,
    ) -> None:
        try:
            command_snapshot = _snapshot_command_envelope(command)
        except ContractError as exc:
            if self.mode is SessionMode.COMMAND:
                self._fail_closed(f"invalid_command:{exc}")
            raise SessionError(str(exc)) from exc
        self._require_active_lease(command_snapshot.lease_id)
        self.check_timeouts(now_ns=now_ns)
        if self.mode is not SessionMode.COMMAND:
            raise SessionError(f"command lease was released: {self.release_reason}")
        try:
            validate_named_values(command_snapshot.action)
            validate_contiguous_command(
                command_snapshot,
                previous_sequence=self.previous_sequence,
                previous_monotonic_ns=self.previous_command_monotonic_ns,
                now_ns=now_ns,
            )
            if now_ns - command_snapshot.monotonic_ns > self.policy.command_inactivity_timeout_ns:
                raise ContractError("command timestamp is stale")
            feedback_snapshot = self._validate_feedback(feedback, now_ns=now_ns)
            if self.previous_sequence is None:
                if (
                    self._lease_acquired_ns is None
                    or feedback_snapshot.metadata.monotonic_ns <= self._lease_acquired_ns
                ):
                    raise ContractError("first-command feedback must postdate lease acquisition")
                if command_snapshot.monotonic_ns < feedback_snapshot.metadata.monotonic_ns:
                    raise ContractError("first command predates its measured hold reference")
                validate_first_command_hold(
                    command_snapshot.action,
                    feedback_snapshot.values,
                    tolerance_deg=self.policy.first_command_hold_tolerance_deg,
                )
        except ContractError as exc:
            self._fail_closed(f"invalid_command:{exc}")
            raise SessionError(str(exc)) from exc
        self.previous_sequence = command_snapshot.sequence
        self.previous_command_monotonic_ns = command_snapshot.monotonic_ns
        self._previous_feedback_sequence = feedback_snapshot.metadata.source_sequence
        self._previous_feedback_monotonic_ns = feedback_snapshot.metadata.monotonic_ns
        self.last_command_accepted_ns = now_ns

    def release(self, *, lease_id: str, reason: str = "requested") -> None:
        self._require_active_lease(lease_id)
        if not isinstance(reason, str) or not reason or reason != reason.strip():
            self._fail_closed("invalid_release_reason")
            raise SessionError("release reason must be normalized non-empty text")
        self._fail_closed(reason)

    def check_timeouts(self, *, now_ns: int) -> bool:
        try:
            self._require_time(now_ns)
        except SessionError:
            if self.mode is SessionMode.COMMAND:
                self._fail_closed("invalid_session_time")
            raise
        if self.mode is not SessionMode.COMMAND:
            return False
        if self.last_heartbeat_ns is None or self.last_command_accepted_ns is None:
            self._fail_closed("incomplete_command_state")
            return True
        if now_ns < self.last_heartbeat_ns or now_ns < self.last_command_accepted_ns:
            self._fail_closed("non_monotonic_session_time")
            return True
        if now_ns - self.last_heartbeat_ns > self.policy.liveness_timeout_ns:
            self._fail_closed("liveness_timeout")
            return True
        if now_ns - self.last_command_accepted_ns > self.policy.command_inactivity_timeout_ns:
            self._fail_closed("command_inactivity_timeout")
            return True
        return False

    def _validate_feedback(
        self,
        feedback: NamedJointVector,
        *,
        now_ns: int,
    ) -> NamedJointVector:
        snapshot = _snapshot_named_joint_vector(feedback)
        if self._previous_feedback_sequence is None or self._previous_feedback_monotonic_ns is None:
            raise ContractError("previous command feedback is unavailable")
        validate_named_values(snapshot.values)
        metadata = snapshot.metadata
        if metadata.source_sequence <= self._previous_feedback_sequence:
            raise ContractError("command feedback sequence did not increase")
        if metadata.monotonic_ns <= self._previous_feedback_monotonic_ns:
            raise ContractError("command feedback timestamp did not increase")
        if metadata.monotonic_ns > now_ns:
            raise ContractError("command feedback timestamp is in the future")
        if now_ns - metadata.monotonic_ns > self.policy.command_inactivity_timeout_ns:
            raise ContractError("command feedback is stale")
        return snapshot

    def _require_active_lease(self, lease_id: str) -> None:
        if self.mode is not SessionMode.COMMAND:
            raise SessionError("command lease is not active")
        if lease_id != self.lease_id:
            self._fail_closed("lease_mismatch")
            raise SessionError("command lease does not match")

    @staticmethod
    def _require_time(now_ns: int) -> None:
        if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns < 0:
            raise SessionError("now_ns must be a non-negative integer")

    def _fail_closed(self, reason: str) -> None:
        self.release_reason = reason
        self._clear_command_state()
        self.mode = SessionMode.READ_ONLY

    def _clear_command_state(self) -> None:
        self.lease_id = None
        self.last_heartbeat_ns = None
        self.last_command_accepted_ns = None
        self.previous_sequence = None
        self.previous_command_monotonic_ns = None
        self._previous_feedback_sequence = None
        self._previous_feedback_monotonic_ns = None
        self._lease_acquired_ns = None
