from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from vlai_l1_runtime.cameras import CameraFrameMetadata
from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.mock import SyntheticSampleSource
from vlai_l1_runtime.collection.schema import (
    ACTION_KEY,
    AGENT_IMAGE_KEY,
    STATE_KEY,
    WRIST_IMAGE_KEY,
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


def _commissioned_config():
    config = load_collection_config(COLLECTION_CONFIG)
    cameras = CamerasConfig(
        max_age_s=0.5,
        max_pair_skew_s=0.05,
        streams=(
            CameraConfig("agent", True, 640, 480, 30, "agent-camera"),
            CameraConfig("wrist", True, 640, 480, 30, "wrist-camera"),
        ),
    )
    return replace(config, system=replace(config.system, cameras=cameras))


def _pose(value: float = 0.0) -> dict[str, float]:
    return dict.fromkeys(FEATURE_NAMES, value)


def _sample(sequence: int, timestamp_ns: int, *, action: dict[str, float] | None = None):
    cameras = {
        role: CameraSample(
            CameraFrameMetadata(
                role,
                f"{role}-camera",
                "boot-a",
                sequence,
                timestamp_ns,
            ),
            Image(),
        )
        for role in ("agent", "wrist")
    }
    return CollectionSample(
        NamedJointVector(_pose(), SampleMetadata(sequence, timestamp_ns)),
        NamedJointVector(action or _pose(), SampleMetadata(sequence, timestamp_ns)),
        cameras,
    )


def test_collection_config_and_schema_have_one_complete_contract() -> None:
    config = load_collection_config(COLLECTION_CONFIG)
    contract = canonical_dataset_contract(config.system)

    assert config.collection_ready is False
    assert config.collection_blockers[-2:] == (
        "camera_agent_uncommissioned",
        "camera_wrist_uncommissioned",
    )
    assert config.repo_id_for("pick_v1") == "pengyue-polaron/vlai-l1-pick_v1"
    assert set(contract.features()) == {
        STATE_KEY,
        ACTION_KEY,
        AGENT_IMAGE_KEY,
        WRIST_IMAGE_KEY,
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
    assert tuple(result.images) == (AGENT_IMAGE_KEY, WRIST_IMAGE_KEY)

    jump = _pose()
    jump["left_joint_1.pos"] = 21.0
    with pytest.raises(CollectionContractError, match="action step"):
        assembler.validate(_sample(2, 1_020_000_000, action=jump), now_ns=1_020_000_000)


def test_invalid_image_does_not_advance_camera_continuity() -> None:
    assembler = SampleAssembler(_commissioned_config())
    sample = _sample(1, 1_000_000_000)
    sample.cameras["agent"].image.shape = (1, 1, 3)
    with pytest.raises(CollectionContractError, match="image shape"):
        assembler.validate(sample, now_ns=1_010_000_000)

    sample.cameras["agent"].image.shape = (480, 640, 3)
    assembler.validate(sample, now_ns=1_010_000_000)


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
