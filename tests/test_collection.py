from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from vlai_l1_runtime.cameras import CameraFrameMetadata
from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.live import LiveCollectionSource
from vlai_l1_runtime.collection.mock import SyntheticSampleSource
from vlai_l1_runtime.collection.schema import (
    ACTION_KEY,
    STATE_KEY,
    WRIST_LEFT_IMAGE_KEY,
    WRIST_RIGHT_IMAGE_KEY,
    CameraSample,
    CollectionContractError,
    CollectionSample,
    SampleAssembler,
    canonical_dataset_contract,
)
from vlai_l1_runtime.configuration import CameraConfig, CamerasConfig, ConfigError
from vlai_l1_runtime.contracts import FEATURE_NAMES, NamedJointVector, SampleMetadata

ROOT = Path(__file__).resolve().parents[1]
COLLECTION_CONFIG = ROOT / "configs/collection/default.toml"


class Image:
    shape = (480, 640, 3)
    dtype = "uint8"


class _ContextSource:
    def __init__(self, value):
        self.value = value
        self.open = False

    def __enter__(self):
        self.open = True
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.open = False


class _StateSource(_ContextSource):
    def receive(self, *, timeout_s: float):
        assert self.open
        assert timeout_s > 0
        return self.value


class _CameraSource(_ContextSource):
    def capture(self, *, timeout_s: float):
        assert self.open
        assert timeout_s > 0
        return self.value


def _commissioned_config():
    config = load_collection_config(COLLECTION_CONFIG)
    cameras = CamerasConfig(
        max_age_s=0.5,
        max_pair_skew_s=0.05,
        streams=(
            CameraConfig("wrist_left", True, True, 640, 480, 30, "realsense", "wrist_left-camera"),
            CameraConfig(
                "wrist_right", True, True, 640, 480, 30, "realsense", "wrist_right-camera"
            ),
            CameraConfig("agent", False, False, 640, 480, 30, "unassigned", None),
        ),
    )
    return replace(
        config,
        system=replace(
            config.system,
            cameras=cameras,
            teleoperation=replace(config.system.teleoperation, commissioned=True),
        ),
    )


def _pose(value: float = 0.0) -> dict[str, float]:
    return dict.fromkeys(FEATURE_NAMES, value)


def _sample(
    sequence: int,
    timestamp_ns: int,
    *,
    action: dict[str, float] | None = None,
    camera_timestamp_ns: int | None = None,
):
    camera_timestamp_ns = timestamp_ns if camera_timestamp_ns is None else camera_timestamp_ns
    cameras = {
        role: CameraSample(
            CameraFrameMetadata(
                role,
                f"{role}-camera",
                "boot-a",
                sequence,
                camera_timestamp_ns,
            ),
            Image(),
        )
        for role in ("wrist_left", "wrist_right")
    }
    return CollectionSample(
        NamedJointVector(_pose(), SampleMetadata(sequence, timestamp_ns)),
        NamedJointVector(action or _pose(), SampleMetadata(sequence, timestamp_ns)),
        cameras,
    )


def test_collection_config_and_schema_have_one_complete_contract() -> None:
    config = load_collection_config(COLLECTION_CONFIG)
    contract = canonical_dataset_contract(_commissioned_config().system)

    assert config.collection_ready is False
    assert config.collection_blockers[0] == "teleoperation_uncommissioned"
    assert config.collection_blockers[-2:] == (
        "camera_wrist_left_uncommissioned",
        "camera_wrist_right_uncommissioned",
    )
    assert config.repo_id_for("pick_v1") == "pengyue-polaron/vlai-l1-pick_v1"
    assert set(contract.features()) == {
        STATE_KEY,
        ACTION_KEY,
        WRIST_LEFT_IMAGE_KEY,
        WRIST_RIGHT_IMAGE_KEY,
    }
    assert contract.features()[STATE_KEY]["names"] == list(FEATURE_NAMES)


def test_collection_config_rejects_unknown_or_overlapping_roots(tmp_path: Path) -> None:
    content = COLLECTION_CONFIG.read_text().replace(
        'derivative_root = "../../data/derivatives"',
        'derivative_root = "../../data/datasets/derivatives"',
    )
    candidate = tmp_path / "configs/collection/collection.toml"
    system = tmp_path / "configs/system/system.toml"
    candidate.parent.mkdir(parents=True)
    system.parent.mkdir(parents=True)
    system.write_text((ROOT / "configs/system/vlai_l1.toml").read_text())
    content = content.replace(
        'system_config = "../system/vlai_l1.toml"',
        'system_config = "../system/system.toml"',
    )
    candidate.write_text(content)
    with pytest.raises(ConfigError, match="must not contain"):
        load_collection_config(candidate)


def test_sample_assembler_validates_fresh_named_synchronized_samples() -> None:
    assembler = SampleAssembler(_commissioned_config())
    result = assembler.validate(_sample(1, 1_000_000_000), now_ns=1_010_000_000)

    assert result.state == (0.0,) * 16
    assert tuple(result.images) == (WRIST_LEFT_IMAGE_KEY, WRIST_RIGHT_IMAGE_KEY)

    jump = _pose()
    jump["left_joint_1.pos"] = 21.0
    with pytest.raises(CollectionContractError, match="action step"):
        assembler.validate(_sample(2, 1_020_000_000, action=jump), now_ns=1_020_000_000)


def test_invalid_image_does_not_advance_camera_continuity() -> None:
    assembler = SampleAssembler(_commissioned_config())
    sample = _sample(1, 1_000_000_000)
    sample.cameras["wrist_left"].image.shape = (1, 1, 3)
    with pytest.raises(CollectionContractError, match="image shape"):
        assembler.validate(sample, now_ns=1_010_000_000)

    sample.cameras["wrist_left"].image.shape = (480, 640, 3)
    assembler.validate(sample, now_ns=1_010_000_000)


def test_sample_assembler_rejects_robot_camera_skew() -> None:
    assembler = SampleAssembler(_commissioned_config())
    with pytest.raises(CollectionContractError, match="robot/camera sample skew"):
        assembler.validate(
            _sample(1, 1_000_000_000, camera_timestamp_ns=1_060_000_000),
            now_ns=1_060_000_000,
        )


def test_synthetic_source_exercises_the_same_sample_boundary() -> None:
    config = _commissioned_config()
    source = SyntheticSampleSource(
        config,
        image_factory=lambda _role, height, width, _sequence: type(
            "SyntheticImage", (), {"shape": (height, width, 3), "dtype": "uint8"}
        )(),
    )
    assembler = SampleAssembler(config)

    samples = list(source.samples(2))
    assert len(samples) == 2
    for sample, now_ns in samples:
        assert assembler.validate(sample, now_ns=now_ns).state == (0.0,) * 16


def test_live_source_composes_one_owned_robot_and_camera_sample() -> None:
    config = _commissioned_config()
    sample = _sample(1, 1_000_000_000)
    state = _StateSource((sample.observation, sample.action))
    cameras = _CameraSource(sample.cameras)

    with LiveCollectionSource(config, state_source=state, camera_source=cameras) as source:
        captured, now_ns = next(source.samples(1))
        assert captured.observation == sample.observation
        assert captured.action == sample.action
        assert captured.cameras == sample.cameras
        assert now_ns > 0

    assert state.open is False
    assert cameras.open is False


def test_live_source_refuses_uncommissioned_hardware() -> None:
    config = load_collection_config(COLLECTION_CONFIG)
    with pytest.raises(ValueError, match="teleoperation_uncommissioned"):
        LiveCollectionSource(config)
