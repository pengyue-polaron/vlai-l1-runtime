from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest
from legacy_bimanual import config as legacy_bimanual_config

from vlai_l1_runtime.cameras import CameraFrameMetadata
from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.ft import FTSample
from vlai_l1_runtime.collection.live import LiveCollectionSource
from vlai_l1_runtime.collection.mock import SyntheticSampleSource
from vlai_l1_runtime.collection.scale import ScaleSample
from vlai_l1_runtime.collection.schema import (
    ACTION_KEY,
    FT_TIME_DELTA_KEY,
    FT_WRENCH_KEY,
    SCALE_NET_KEY,
    SCALE_STABLE_KEY,
    SCALE_TARED_KEY,
    SCALE_TIME_DELTA_KEY,
    SCALE_WEIGHT_KEY,
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
RIGHT_COLLECTION_CONFIG = ROOT / "configs/collection/right_only.toml"


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
    def __init__(self, value):
        super().__init__(value)
        self.targets = []

    def receive_closest(self, *, target_monotonic_ns: int, timeout_s: float):
        assert self.open
        assert timeout_s > 0
        self.targets.append(target_monotonic_ns)
        return self.value


class _CameraSource(_ContextSource):
    def capture(self, *, timeout_s: float):
        assert self.open
        assert timeout_s > 0
        return self.value


class _FTSource(_ContextSource):
    def wait_first(self, *, timeout_s: float, max_age_s: float):
        assert self.open
        assert timeout_s > 0
        assert max_age_s > 0

    def closest(self, *, target_monotonic_ns: int, timeout_s: float):
        assert self.open
        assert timeout_s > 0
        return self.value


class _ScaleSource(_ContextSource):
    def wait_first(self, *, timeout_s: float, max_age_s: float):
        assert self.open
        assert timeout_s > 0
        assert max_age_s > 0

    def latest_before(self, *, target_monotonic_ns: int):
        assert self.open
        assert self.value.monotonic_ns <= target_monotonic_ns
        return self.value


def _commissioned_config():
    config = legacy_bimanual_config()
    cameras = CamerasConfig(
        max_age_s=0.5,
        max_pair_skew_s=0.05,
        startup_timeout_s=2.0,
        streams=(
            CameraConfig(
                "wrist_left",
                True,
                True,
                640,
                480,
                30,
                "v4l2",
                "wrist_left-camera",
                4,
            ),
            CameraConfig(
                "wrist_right",
                True,
                True,
                640,
                480,
                30,
                "v4l2",
                "wrist_right-camera",
                4,
            ),
            CameraConfig("agent", False, False, 640, 480, 30, "unassigned", None),
        ),
    )
    return replace(
        config,
        record_camera_roles=("wrist_left", "wrist_right"),
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
        FTSample((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), timestamp_ns, sequence),
        ScaleSample(12.0, True, True, False, timestamp_ns, sequence),
    )


def test_collection_config_and_schema_have_one_complete_contract() -> None:
    config = legacy_bimanual_config()
    contract = canonical_dataset_contract(_commissioned_config())
    full_contract = canonical_dataset_contract(config)

    assert config.collection_ready is True
    assert config.collection_blockers == ()
    assert config.schema_version == 7
    assert config.teleoperation_sides == ("left", "right")
    assert config.record_camera_roles == ("wrist_left", "wrist_right", "agent")
    assert config.minimum_capture_fps == 27.0
    assert config.image_writer_threads == 12
    assert config.reset_policy.before_collection is True
    assert config.reset_policy.after_save is True
    assert config.reset_policy.after_discard is True
    assert config.leading_stillness.enabled is True
    assert (
        config.leading_stillness.action_thresholds
        == (
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            0.5,
            1.0,
        )
        * 2
    )
    assert config.repo_id_for("pick_v1") == "pengyue-polaron/vlai-l1-pick_v1"
    assert set(contract.features()) == {
        STATE_KEY,
        ACTION_KEY,
        FT_WRENCH_KEY,
        FT_TIME_DELTA_KEY,
        SCALE_WEIGHT_KEY,
        SCALE_TIME_DELTA_KEY,
        SCALE_STABLE_KEY,
        SCALE_TARED_KEY,
        SCALE_NET_KEY,
        WRIST_LEFT_IMAGE_KEY,
        WRIST_RIGHT_IMAGE_KEY,
    }
    assert contract.features()[STATE_KEY]["names"] == list(FEATURE_NAMES)
    assert full_contract.features()["observation.images.agent"]["shape"] == (480, 480, 3)


def test_right_only_collection_contract_is_exact_and_omits_left_wrist() -> None:
    import numpy as np

    config = load_collection_config(RIGHT_COLLECTION_CONFIG)
    contract = canonical_dataset_contract(config)
    source = SyntheticSampleSource(
        config,
        image_factory=lambda _role, height, width, _sequence: np.zeros(
            (height, width, 3), dtype=np.uint8
        ),
    )

    assert config.teleoperation_sides == ("right",)
    assert config.record_camera_roles == ("wrist_right", "agent")
    assert len(config.leading_stillness.action_thresholds) == 8
    assert contract.state_names == tuple(
        name for name in FEATURE_NAMES if name.startswith("right_")
    )
    assert set(contract.features()) == {
        STATE_KEY,
        ACTION_KEY,
        FT_WRENCH_KEY,
        FT_TIME_DELTA_KEY,
        SCALE_WEIGHT_KEY,
        SCALE_TIME_DELTA_KEY,
        SCALE_STABLE_KEY,
        SCALE_TARED_KEY,
        SCALE_NET_KEY,
        WRIST_RIGHT_IMAGE_KEY,
        "observation.images.agent",
    }
    sample, now_ns = next(source.samples(1))
    validated = SampleAssembler(config).validate(sample, now_ns=now_ns)
    assert len(validated.state) == 8
    assert len(validated.action) == 8
    assert tuple(validated.images) == (WRIST_RIGHT_IMAGE_KEY, "observation.images.agent")


def test_agent_observation_uses_the_exact_tracked_center_crop() -> None:
    import numpy as np

    config = legacy_bimanual_config()
    timestamp_ns = 1_000_000_000
    images = {
        stream.role: np.arange(
            stream.height * stream.width * 3,
            dtype=np.uint8,
        ).reshape(stream.height, stream.width, 3)
        for stream in config.system.cameras.streams
        if stream.enabled
    }
    sample = CollectionSample(
        NamedJointVector(_pose(), SampleMetadata(1, timestamp_ns)),
        NamedJointVector(_pose(), SampleMetadata(1, timestamp_ns)),
        {
            stream.role: CameraSample(
                CameraFrameMetadata(
                    stream.role,
                    stream.device_id,
                    "boot-a",
                    1,
                    timestamp_ns,
                ),
                images[stream.role],
            )
            for stream in config.system.cameras.streams
            if stream.enabled
        },
        FTSample((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), timestamp_ns, 1),
        ScaleSample(12.0, True, True, False, timestamp_ns, 1),
    )

    result = SampleAssembler(config).validate(sample, now_ns=timestamp_ns)

    agent = result.images["observation.images.agent"]
    assert agent.shape == (480, 480, 3)
    assert np.array_equal(agent, images["agent"][:, 80:560])
    assert agent.flags.c_contiguous
    assert result.images[WRIST_LEFT_IMAGE_KEY] is images["wrist_left"]
    assert result.images[WRIST_RIGHT_IMAGE_KEY] is images["wrist_right"]


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

    content = COLLECTION_CONFIG.read_text().replace(
        "minimum_capture_fps = 27.0",
        "minimum_capture_fps = 31.0",
    )
    content = content.replace(
        'system_config = "../system/vlai_l1.toml"',
        'system_config = "../system/system.toml"',
    )
    candidate.write_text(content)
    with pytest.raises(ConfigError, match="must not exceed fps"):
        load_collection_config(candidate)

    content = RIGHT_COLLECTION_CONFIG.read_text().replace(
        'system_config = "../system/vlai_l1.toml"',
        'system_config = "../system/system.toml"',
    )
    candidate.write_text(
        content.replace(
            'record_camera_roles = ["wrist_right", "agent"]',
            'record_camera_roles = ["wrist_left", "wrist_right", "agent"]',
        )
    )
    with pytest.raises(ConfigError, match="exactly the wrist camera"):
        load_collection_config(candidate)

    candidate.write_text(
        content.replace(
            'record_camera_roles = ["wrist_right", "agent"]',
            'record_camera_roles = ["agent", "wrist_right"]',
        )
    )
    with pytest.raises(ConfigError, match="canonical order"):
        load_collection_config(candidate)


def test_sample_assembler_validates_fresh_named_synchronized_samples() -> None:
    assembler = SampleAssembler(_commissioned_config())
    result = assembler.validate(_sample(1, 1_000_000_000), now_ns=1_010_000_000)

    assert result.state == (0.0,) * 16
    assert tuple(result.images) == (WRIST_LEFT_IMAGE_KEY, WRIST_RIGHT_IMAGE_KEY)

    jump = _pose()
    jump["left_joint_1.pos"] = 21.0
    second = assembler.validate(
        _sample(2, 1_020_000_000, action=jump),
        now_ns=1_020_000_000,
    )
    assert second.action[0] == 21.0


def test_ft_wrench_and_host_time_delta_are_recorded() -> None:
    config = _commissioned_config()
    sample = _sample(1, 1_000_000_000)
    sample = replace(sample, ft=replace(sample.ft, monotonic_ns=1_005_000_000))
    validated = SampleAssembler(config).validate(sample, now_ns=1_010_000_000)
    assert validated.ft_wrench == (1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    assert validated.ft_time_delta_s == 0.005
    frame = validated.lerobot_frame(task="record a force sample")
    assert frame["task"] == "record a force sample"
    assert tuple(frame[FT_WRENCH_KEY]) == pytest.approx((1.0, 2.0, 3.0, 0.1, 0.2, 0.3))
    assert tuple(frame[FT_TIME_DELTA_KEY]) == pytest.approx((0.005,))
    assert config.max_ft_skew_s == 0.05
    assert canonical_dataset_contract(config).features()[FT_WRENCH_KEY]["shape"] == (6,)
    assert canonical_dataset_contract(config).features()[FT_TIME_DELTA_KEY]["shape"] == (1,)


def test_scale_weight_status_and_repeated_samples_are_recorded() -> None:
    config = _commissioned_config()
    assembler = SampleAssembler(config)
    first = _sample(1, 1_000_000_000)
    first = replace(first, scale=replace(first.scale, monotonic_ns=900_000_000))
    validated = assembler.validate(first, now_ns=1_010_000_000)
    assert validated.scale_weight_g == 12.0
    assert validated.scale_time_delta_s == -0.1
    frame = validated.lerobot_frame(task="pour into tared container")
    assert tuple(frame[SCALE_WEIGHT_KEY]) == (12.0,)
    assert tuple(frame[SCALE_TIME_DELTA_KEY]) == pytest.approx((-0.1,))
    assert tuple(frame[SCALE_STABLE_KEY]) == (1.0,)
    assert tuple(frame[SCALE_TARED_KEY]) == (1.0,)
    assert tuple(frame[SCALE_NET_KEY]) == (0.0,)
    second = _sample(2, 1_030_000_000)
    second = replace(second, scale=first.scale)
    assert assembler.validate(second, now_ns=1_040_000_000).scale_weight_g == 12.0


def test_scale_missing_stale_skewed_and_backward_samples_fail_closed() -> None:
    config = _commissioned_config()
    assembler = SampleAssembler(config)
    first = _sample(1, 1_000_000_000)
    with pytest.raises(CollectionContractError, match="scale sample is missing"):
        assembler.validate(replace(first, scale=None), now_ns=1_010_000_000)
    with pytest.raises(CollectionContractError, match="scale sample is stale"):
        assembler.validate(
            replace(first, scale=replace(first.scale, monotonic_ns=400_000_000)),
            now_ns=1_010_000_000,
        )
    with pytest.raises(CollectionContractError, match="scale/robot sample skew"):
        assembler.validate(
            replace(first, scale=replace(first.scale, monotonic_ns=490_000_000)),
            now_ns=1_000_000_000,
        )
    assembler.validate(first, now_ns=1_010_000_000)
    second = _sample(2, 1_030_000_000)
    second = replace(second, scale=replace(second.scale, sequence=0))
    with pytest.raises(CollectionContractError, match="positive integer"):
        assembler.validate(second, now_ns=1_040_000_000)


def test_ft_missing_stale_skewed_and_repeated_frames_fail_closed() -> None:
    config = _commissioned_config()
    assembler = SampleAssembler(config)
    first = _sample(1, 1_000_000_000)
    with pytest.raises(CollectionContractError, match="FT sample is missing"):
        assembler.validate(replace(first, ft=None), now_ns=1_010_000_000)
    with pytest.raises(CollectionContractError, match="FT sample is stale"):
        assembler.validate(
            replace(first, ft=replace(first.ft, monotonic_ns=700_000_000)),
            now_ns=1_010_000_000,
        )
    with pytest.raises(CollectionContractError, match="FT/robot sample skew"):
        assembler.validate(
            replace(first, ft=replace(first.ft, monotonic_ns=940_000_000)),
            now_ns=1_010_000_000,
        )
    assembler.validate(first, now_ns=1_010_000_000)
    repeated = _sample(2, 1_030_000_000)
    repeated = replace(repeated, ft=replace(repeated.ft, sequence=1))
    with pytest.raises(CollectionContractError, match="FT sample sequence did not increase"):
        assembler.validate(repeated, now_ns=1_040_000_000)


def test_ft_preflight_failure_releases_all_sources() -> None:
    config = _commissioned_config()
    sample = _sample(1, 1_000_000_000)
    state = _StateSource((sample.observation, sample.action))
    cameras = _CameraSource(sample.cameras)

    class NoFT(_FTSource):
        def wait_first(self, *, timeout_s: float, max_age_s: float):
            raise TimeoutError("no valid HKVL frame received")

    ft = NoFT(sample.ft)
    with (
        pytest.raises(TimeoutError, match="no valid HKVL frame"),
        LiveCollectionSource(config, state_source=state, camera_source=cameras, ft_source=ft),
    ):
        pytest.fail("FT preflight must fail before runtime startup")
    assert not state.open
    assert not cameras.open
    assert not ft.open


def test_scale_preflight_failure_releases_all_sources() -> None:
    config = _commissioned_config()
    sample = _sample(1, 1_000_000_000)
    state = _StateSource((sample.observation, sample.action))
    cameras = _CameraSource(sample.cameras)
    ft = _FTSource(sample.ft)

    class NoScale(_ScaleSource):
        def wait_first(self, *, timeout_s: float, max_age_s: float):
            raise TimeoutError("no fresh scale stream received")

    scale = NoScale(sample.scale)
    with (
        pytest.raises(TimeoutError, match="no fresh scale stream"),
        LiveCollectionSource(
            config,
            state_source=state,
            camera_source=cameras,
            ft_source=ft,
            scale_source=scale,
        ),
    ):
        pytest.fail("scale preflight must fail before runtime startup")
    assert not state.open
    assert not cameras.open
    assert not ft.open
    assert not scale.open


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
    with pytest.raises(
        CollectionContractError,
        match=r"robot/wrist_left sample skew 60\.000 ms exceeds 50\.000 ms",
    ):
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
    right = sample.cameras["wrist_right"]
    cameras = dict(sample.cameras)
    cameras["wrist_right"] = CameraSample(
        replace(right.metadata, monotonic_ns=1_008_000_000),
        right.image,
    )
    sample = CollectionSample(sample.observation, sample.action, cameras, sample.ft, sample.scale)
    state = _StateSource((sample.observation, sample.action))
    cameras = _CameraSource(sample.cameras)
    ft = _FTSource(sample.ft)
    scale = _ScaleSource(sample.scale)

    with LiveCollectionSource(
        config, state_source=state, camera_source=cameras, ft_source=ft, scale_source=scale
    ) as source:
        captured, now_ns = next(source.samples())
        assert captured.observation == sample.observation
        assert captured.action == sample.action
        assert captured.cameras == sample.cameras
        assert captured.ft == sample.ft
        assert captured.scale == sample.scale
        assert now_ns > 0
        assert state.targets == [1_004_000_000]

    assert state.open is False
    assert cameras.open is False
    assert ft.open is False
    assert scale.open is False


def test_live_source_refuses_uncommissioned_hardware() -> None:
    config = legacy_bimanual_config()
    config = replace(
        config,
        system=replace(
            config.system,
            teleoperation=replace(config.system.teleoperation, commissioned=False),
        ),
    )
    with pytest.raises(ValueError, match="teleoperation_uncommissioned"):
        LiveCollectionSource(config)
