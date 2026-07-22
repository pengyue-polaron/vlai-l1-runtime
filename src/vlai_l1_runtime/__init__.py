"""Hardware contracts and runtime composition for the VLAI L1."""

from .cameras import CameraContractError, CameraFrameMetadata, CameraSetValidator
from .configuration import ConfigError, SystemConfig, load_system_config
from .contracts import (
    FEATURE_NAMES,
    CommandEnvelope,
    ContractError,
    NamedJointVector,
    RobotDescription,
    SampleMetadata,
    limits_by_feature,
    robot_description,
)
from .session import CommandSession, SessionError, SessionMode

__all__ = [
    "FEATURE_NAMES",
    "CameraContractError",
    "CameraFrameMetadata",
    "CameraSetValidator",
    "CommandEnvelope",
    "CommandSession",
    "ConfigError",
    "ContractError",
    "NamedJointVector",
    "RobotDescription",
    "SampleMetadata",
    "SessionError",
    "SessionMode",
    "SystemConfig",
    "limits_by_feature",
    "load_system_config",
    "robot_description",
]
