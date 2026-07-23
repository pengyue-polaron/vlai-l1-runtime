"""Pure public robot contracts shared by future Runtime and LeRobot clients."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

from .configuration import MOTOR_NAMES, SIDES, SystemConfig

FEATURE_NAMES = tuple(f"{side}_{motor}.pos" for side in SIDES for motor in MOTOR_NAMES)


class ContractError(ValueError):
    """A public observation or command violates the static robot contract."""


@dataclass(frozen=True)
class FeatureSpec:
    name: str
    dtype: str
    unit: str


@dataclass(frozen=True)
class RobotDescription:
    robot_id: str
    topology_id: str
    observation_features: tuple[FeatureSpec, ...]
    action_features: tuple[FeatureSpec, ...]
    teleoperation_ready: bool
    teleoperation_blockers: tuple[str, ...]
    command_ready: bool
    command_blockers: tuple[str, ...]
    camera_roles: tuple[str, ...]
    collection_ready: bool


@dataclass(frozen=True)
class SampleMetadata:
    source_sequence: int
    monotonic_ns: int

    def __post_init__(self) -> None:
        if (
            isinstance(self.source_sequence, bool)
            or not isinstance(self.source_sequence, int)
            or self.source_sequence < 0
        ):
            raise ContractError("source_sequence must be a non-negative integer")
        if (
            isinstance(self.monotonic_ns, bool)
            or not isinstance(self.monotonic_ns, int)
            or self.monotonic_ns < 0
        ):
            raise ContractError("monotonic_ns must be a non-negative integer")


def _snapshot_sample_metadata(metadata: object) -> SampleMetadata:
    if not isinstance(metadata, SampleMetadata):
        raise ContractError("joint vector metadata must be SampleMetadata")
    try:
        source_sequence = metadata.source_sequence
        monotonic_ns = metadata.monotonic_ns
    except AttributeError as exc:
        raise ContractError("joint vector metadata is incomplete") from exc
    return SampleMetadata(source_sequence, monotonic_ns)


@dataclass(frozen=True)
class NamedJointVector:
    values: Mapping[str, float]
    metadata: SampleMetadata

    def __post_init__(self) -> None:
        metadata = _snapshot_sample_metadata(self.metadata)
        normalized = validate_named_values(self.values)
        object.__setattr__(self, "metadata", metadata)
        object.__setattr__(self, "values", MappingProxyType(normalized))


@dataclass(frozen=True)
class CommandEnvelope:
    lease_id: str
    sequence: int
    monotonic_ns: int
    action: Mapping[str, float]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.lease_id, str)
            or not self.lease_id
            or self.lease_id != self.lease_id.strip()
        ):
            raise ContractError("lease_id must be normalized non-empty text")
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or self.sequence < 0
        ):
            raise ContractError("sequence must be a non-negative integer")
        if (
            isinstance(self.monotonic_ns, bool)
            or not isinstance(self.monotonic_ns, int)
            or self.monotonic_ns < 0
        ):
            raise ContractError("monotonic_ns must be a non-negative integer")
        object.__setattr__(self, "action", MappingProxyType(validate_named_values(self.action)))


def _snapshot_named_joint_vector(vector: object) -> NamedJointVector:
    if not isinstance(vector, NamedJointVector):
        raise ContractError("command feedback must be a NamedJointVector")
    try:
        values = vector.values
        metadata = vector.metadata
    except AttributeError as exc:
        raise ContractError("command feedback is incomplete") from exc
    return NamedJointVector(values, metadata)


def _snapshot_command_envelope(command: object) -> CommandEnvelope:
    if not isinstance(command, CommandEnvelope):
        raise ContractError("command must be a CommandEnvelope")
    try:
        lease_id = command.lease_id
        sequence = command.sequence
        monotonic_ns = command.monotonic_ns
        action = command.action
    except AttributeError as exc:
        raise ContractError("command envelope is incomplete") from exc
    return CommandEnvelope(lease_id, sequence, monotonic_ns, action)


def robot_description(config: SystemConfig) -> RobotDescription:
    features = tuple(FeatureSpec(name, "float64", config.position_unit) for name in FEATURE_NAMES)
    return RobotDescription(
        robot_id=config.robot_id,
        topology_id=config.topology_id,
        observation_features=features,
        action_features=features,
        teleoperation_ready=not config.teleoperation.blockers,
        teleoperation_blockers=config.teleoperation.blockers,
        command_ready=config.safety.command_ready,
        command_blockers=config.command_blockers,
        camera_roles=tuple(stream.role for stream in config.cameras.streams),
        collection_ready=not config.teleoperation.blockers and config.cameras.collection_ready,
    )


def validate_named_values(values: Mapping[str, float]) -> dict[str, float]:
    if not isinstance(values, Mapping):
        raise ContractError("joint values must be a mapping")
    if not all(isinstance(name, str) for name in values):
        raise ContractError("joint feature names must be text")
    actual = set(values)
    expected = set(FEATURE_NAMES)
    if actual != expected:
        missing = sorted(expected - actual)
        unknown = sorted(actual - expected)
        raise ContractError(f"joint feature mismatch; missing={missing}, unknown={unknown}")
    result: dict[str, float] = {}
    for name in FEATURE_NAMES:
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ContractError(f"{name} must be numeric")
        number = float(value)
        if not math.isfinite(number):
            raise ContractError(f"{name} must be finite")
        result[name] = number
    return result


def validate_contiguous_command(
    command: CommandEnvelope,
    *,
    previous_sequence: int | None,
    previous_monotonic_ns: int | None,
    now_ns: int,
) -> None:
    command = _snapshot_command_envelope(command)
    if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns < 0:
        raise ContractError("now_ns must be a non-negative integer")
    if previous_sequence is not None and (
        isinstance(previous_sequence, bool)
        or not isinstance(previous_sequence, int)
        or previous_sequence < 0
    ):
        raise ContractError("previous_sequence must be a non-negative integer or None")
    if previous_monotonic_ns is not None and (
        isinstance(previous_monotonic_ns, bool)
        or not isinstance(previous_monotonic_ns, int)
        or previous_monotonic_ns < 0
    ):
        raise ContractError("previous_monotonic_ns must be a non-negative integer or None")
    expected_sequence = 0 if previous_sequence is None else previous_sequence + 1
    if command.sequence != expected_sequence:
        raise ContractError(
            f"command sequence must be contiguous: expected {expected_sequence}, got {command.sequence}"
        )
    if previous_monotonic_ns is not None and command.monotonic_ns <= previous_monotonic_ns:
        raise ContractError("command monotonic timestamp must increase")
    if command.monotonic_ns > now_ns:
        raise ContractError("command monotonic timestamp cannot be in the future")


def validate_first_command_hold(
    action: Mapping[str, float], measured: Mapping[str, float], *, tolerance_deg: float
) -> None:
    if (
        isinstance(tolerance_deg, bool)
        or not isinstance(tolerance_deg, (int, float))
        or not math.isfinite(tolerance_deg)
        or tolerance_deg < 0
    ):
        raise ContractError("hold tolerance must be finite and non-negative")
    candidate = validate_named_values(action)
    reference = validate_named_values(measured)
    for name in FEATURE_NAMES:
        if abs(candidate[name] - reference[name]) > tolerance_deg:
            raise ContractError(f"first command must hold measured pose; mismatch at {name}")
