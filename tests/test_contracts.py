from __future__ import annotations

from pathlib import Path

import pytest

from vlai_l1_runtime import (
    FEATURE_NAMES,
    CommandEnvelope,
    ContractError,
    NamedJointVector,
    SampleMetadata,
    limits_by_feature,
    load_system_config,
    robot_description,
)
from vlai_l1_runtime.contracts import (
    validate_contiguous_command,
    validate_first_command_hold,
    validate_named_values,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")


def _pose(value: float = 0.0) -> dict[str, float]:
    return dict.fromkeys(FEATURE_NAMES, value)


def test_description_is_lerobot_compatible_and_fail_closed() -> None:
    description = robot_description(CONFIG)

    assert len(description.observation_features) == 16
    assert tuple(feature.name for feature in description.action_features) == FEATURE_NAMES
    assert description.action_features[0].name == "left_joint_1.pos"
    assert description.action_features[-1].name == "right_gripper.pos"
    assert {feature.unit for feature in description.action_features} == {"degree"}
    assert description.command_ready is False
    assert description.collection_ready is False


def test_named_joint_vector_requires_exact_finite_features() -> None:
    vector = NamedJointVector(_pose(), SampleMetadata(4, 50))
    assert tuple(vector.values) == FEATURE_NAMES

    missing = _pose()
    missing.pop(FEATURE_NAMES[0])
    with pytest.raises(ContractError, match="feature mismatch"):
        validate_named_values(missing)
    invalid = _pose()
    invalid[FEATURE_NAMES[0]] = float("nan")
    with pytest.raises(ContractError, match="finite"):
        validate_named_values(invalid)

    non_text_name = _pose()
    non_text_name[1] = 0.0
    with pytest.raises(ContractError, match="names must be text"):
        validate_named_values(non_text_name)
    with pytest.raises(ContractError, match="metadata must be SampleMetadata"):
        NamedJointVector(_pose(), None)


def test_joint_limits_are_named_and_never_clamped() -> None:
    invalid = _pose()
    invalid["right_joint_2.pos"] = 91.0
    with pytest.raises(ContractError, match="outside"):
        validate_named_values(invalid, limits=limits_by_feature(CONFIG))


def test_command_sequence_timestamp_and_first_hold_are_explicit() -> None:
    command = CommandEnvelope("lease", 0, 100, _pose())
    validate_contiguous_command(
        command, previous_sequence=None, previous_monotonic_ns=None, now_ns=100
    )
    validate_first_command_hold(command.action, _pose(0.05), tolerance_deg=0.1)

    with pytest.raises(ContractError, match="contiguous"):
        validate_contiguous_command(
            CommandEnvelope("lease", 2, 101, _pose()),
            previous_sequence=0,
            previous_monotonic_ns=100,
            now_ns=101,
        )
    with pytest.raises(ContractError, match="hold measured"):
        validate_first_command_hold(_pose(1.0), _pose(), tolerance_deg=0.1)
    with pytest.raises(ContractError, match="non-negative integer"):
        validate_contiguous_command(
            command,
            previous_sequence=None,
            previous_monotonic_ns=None,
            now_ns=True,
        )
    with pytest.raises(ContractError, match="finite and non-negative"):
        validate_first_command_hold(_pose(), _pose(), tolerance_deg="0.1")
