from __future__ import annotations

from pathlib import Path

import pytest

from vlai_l1_runtime import (
    CameraContractError,
    CameraFrameMetadata,
    CameraSetValidator,
    load_system_config,
)
from vlai_l1_runtime.configuration import CameraConfig, CamerasConfig, ConfigError

ROOT = Path(__file__).resolve().parents[1]


def _commissioned() -> CamerasConfig:
    return CamerasConfig(
        max_age_s=0.5,
        max_pair_skew_s=0.05,
        streams=(
            CameraConfig("agent", True, 640, 480, 30, "agent-by-id"),
            CameraConfig("wrist", True, 640, 480, 30, "wrist-by-id"),
        ),
    )


def _frames(
    *,
    epoch: str = "boot-a",
    agent_sequence: int = 10,
    wrist_sequence: int = 20,
    agent_ns: int = 1_000_000_000,
    wrist_ns: int = 1_010_000_000,
):
    return {
        "agent": CameraFrameMetadata("agent", "agent-by-id", epoch, agent_sequence, agent_ns),
        "wrist": CameraFrameMetadata("wrist", "wrist-by-id", epoch, wrist_sequence, wrist_ns),
    }


def test_tracked_uncommissioned_cameras_block_collection() -> None:
    system = load_system_config(ROOT / "configs/system/vlai_l1.toml")
    with pytest.raises(CameraContractError, match="commissioned"):
        CameraSetValidator(system.cameras)


def test_camera_set_validates_freshness_skew_and_identity() -> None:
    validator = CameraSetValidator(_commissioned())
    current = validator.validate(_frames(), now_ns=1_020_000_000)
    assert current["agent"].source_sequence == 10
    assert current["wrist"].source_sequence == 20

    with pytest.raises(CameraContractError, match="stale"):
        CameraSetValidator(_commissioned()).validate(_frames(), now_ns=1_600_000_000)
    with pytest.raises(CameraContractError, match="skew"):
        CameraSetValidator(_commissioned()).validate(
            _frames(wrist_ns=1_060_000_000),
            now_ns=1_070_000_000,
        )
    wrong_device = _frames()
    wrong_device["agent"] = CameraFrameMetadata(
        "agent", "other-device", "boot-a", 10, 1_000_000_000
    )
    with pytest.raises(CameraContractError, match="does not match stream"):
        CameraSetValidator(_commissioned()).validate(
            wrong_device,
            now_ns=1_020_000_000,
        )


def test_camera_continuity_is_stateful_and_reset_is_explicit() -> None:
    validator = CameraSetValidator(_commissioned())
    validator.validate(_frames(), now_ns=1_020_000_000)

    with pytest.raises(CameraContractError, match="sequence did not increase"):
        validator.validate(_frames(), now_ns=1_020_000_000)
    with pytest.raises(CameraContractError, match="timestamp did not increase"):
        validator.validate(
            _frames(
                agent_sequence=11,
                wrist_sequence=21,
                agent_ns=999_000_000,
                wrist_ns=1_009_000_000,
            ),
            now_ns=1_020_000_000,
        )
    with pytest.raises(CameraContractError, match="without explicit reset"):
        validator.validate(
            _frames(
                epoch="boot-b",
                agent_sequence=0,
                wrist_sequence=0,
                agent_ns=1_020_000_000,
                wrist_ns=1_020_000_000,
            ),
            now_ns=1_020_000_000,
        )

    with pytest.raises(CameraContractError, match="must differ from the active epoch"):
        validator.reset(restarted_epochs={"agent": "boot-a"})

    validator.reset(restarted_epochs={"agent": "boot-b", "wrist": "boot-b"})
    with pytest.raises(CameraContractError, match="declared restart epoch"):
        validator.validate(
            _frames(
                epoch="boot-a",
                agent_sequence=0,
                wrist_sequence=0,
                agent_ns=1_020_000_000,
                wrist_ns=1_020_000_000,
            ),
            now_ns=1_020_000_000,
        )
    reset = validator.validate(
        _frames(
            epoch="boot-b",
            agent_sequence=0,
            wrist_sequence=0,
            agent_ns=1_020_000_000,
            wrist_ns=1_020_000_000,
        ),
        now_ns=1_020_000_000,
    )
    assert reset["agent"].stream_epoch == "boot-b"


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("missing", None),
        ("source_sequence", True),
        ("source_sequence", float("nan")),
        ("monotonic_ns", float("inf")),
    ],
)
def test_camera_validator_revalidates_external_metadata(field: str, value: object) -> None:
    validator = CameraSetValidator(_commissioned())
    validator.validate(_frames(), now_ns=1_020_000_000)
    candidate = CameraFrameMetadata("agent", "agent-by-id", "boot-a", 11, 1_020_000_000)
    if field == "missing":
        object.__delattr__(candidate, "monotonic_ns")
    else:
        object.__setattr__(candidate, field, value)
    frames = _frames(
        agent_sequence=11,
        wrist_sequence=21,
        agent_ns=1_020_000_000,
        wrist_ns=1_020_000_000,
    )
    frames["agent"] = candidate

    with pytest.raises(CameraContractError):
        validator.validate(frames, now_ns=1_020_000_000)

    accepted = validator.validate(
        _frames(
            agent_sequence=11,
            wrist_sequence=21,
            agent_ns=1_020_000_000,
            wrist_ns=1_020_000_000,
        ),
        now_ns=1_020_000_000,
    )
    assert accepted["agent"].source_sequence == 11


def test_camera_set_requires_every_role_exactly_once() -> None:
    with pytest.raises(CameraContractError, match="every enabled role"):
        CameraSetValidator(_commissioned()).validate(
            {"agent": CameraFrameMetadata("agent", "agent-by-id", "boot-a", 1, 1)},
            now_ns=1,
        )


def test_commissioned_camera_devices_must_be_distinct() -> None:
    with pytest.raises(ConfigError, match="unique device identities"):
        CamerasConfig(
            max_age_s=0.5,
            max_pair_skew_s=0.05,
            streams=(
                CameraConfig("agent", True, 640, 480, 30, "same-device"),
                CameraConfig("wrist", True, 640, 480, 30, "same-device"),
            ),
        )
