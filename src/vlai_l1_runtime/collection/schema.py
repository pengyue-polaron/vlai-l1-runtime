"""Canonical VLAI L1 LeRobot v3 sample and feature contracts."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from embodied_ops import require_fresh_sample, require_pair_skew

from ..cameras import CameraFrameMetadata, CameraSetValidator
from ..configuration import CameraConfig, SystemConfig
from ..contracts import FEATURE_NAMES, NamedJointVector, validate_named_values
from .configuration import CollectionConfig
from .dependencies import collection_dependency_error, require_collection_python

STATE_KEY = "observation.state"
ACTION_KEY = "action"
IMAGE_PREFIX = "observation.images."
WRIST_LEFT_IMAGE_KEY = f"{IMAGE_PREFIX}wrist_left"
WRIST_RIGHT_IMAGE_KEY = f"{IMAGE_PREFIX}wrist_right"
DATASET_SCHEMA = "vlai_l1_lerobot_dataset_v3_v2"


class CollectionContractError(ValueError):
    """One collection sample violates the canonical L1 dataset contract."""


@dataclass(frozen=True)
class CameraSpec:
    role: str
    height: int
    width: int

    @property
    def feature_key(self) -> str:
        return f"{IMAGE_PREFIX}{self.role}"

    def feature(self) -> dict[str, Any]:
        return {
            "dtype": "video",
            "shape": (self.height, self.width, 3),
            "names": ["height", "width", "channel"],
        }


@dataclass(frozen=True)
class DatasetContract:
    state_names: tuple[str, ...]
    action_names: tuple[str, ...]
    cameras: tuple[CameraSpec, ...]

    def features(self) -> dict[str, dict[str, Any]]:
        features = {
            STATE_KEY: _vector_feature(self.state_names),
            ACTION_KEY: _vector_feature(self.action_names),
        }
        features.update({camera.feature_key: camera.feature() for camera in self.cameras})
        return features


@dataclass(frozen=True)
class CameraSample:
    metadata: CameraFrameMetadata
    image: Any


@dataclass(frozen=True)
class CollectionSample:
    observation: NamedJointVector
    action: NamedJointVector
    cameras: Mapping[str, CameraSample]

    def __post_init__(self) -> None:
        try:
            observation = NamedJointVector(self.observation.values, self.observation.metadata)
            action = NamedJointVector(self.action.values, self.action.metadata)
        except (AttributeError, TypeError, ValueError) as exc:
            raise CollectionContractError("joint samples are malformed") from exc
        if not isinstance(self.cameras, Mapping) or not all(
            isinstance(role, str) for role in self.cameras
        ):
            raise CollectionContractError("cameras must be a role-keyed mapping")
        snapshot: dict[str, CameraSample] = {}
        for role, sample in self.cameras.items():
            if not isinstance(sample, CameraSample):
                raise CollectionContractError(f"{role} camera sample has the wrong type")
            if sample.image is None:
                raise CollectionContractError(f"{role} camera sample has no image")
            try:
                metadata = CameraFrameMetadata(
                    sample.metadata.role,
                    sample.metadata.device_id,
                    sample.metadata.stream_epoch,
                    sample.metadata.source_sequence,
                    sample.metadata.monotonic_ns,
                )
            except (AttributeError, TypeError, ValueError) as exc:
                raise CollectionContractError(f"{role} camera metadata is malformed") from exc
            snapshot[role] = CameraSample(metadata, sample.image)
        object.__setattr__(self, "observation", observation)
        object.__setattr__(self, "action", action)
        object.__setattr__(self, "cameras", MappingProxyType(snapshot))


@dataclass(frozen=True)
class ValidatedSample:
    state: tuple[float, ...]
    action: tuple[float, ...]
    images: Mapping[str, Any]

    def lerobot_frame(self, *, task: str) -> dict[str, Any]:
        """Materialize NumPy vectors only at the optional LeRobot boundary."""

        normalized_task = normalize_task(task)
        require_collection_python()
        try:
            import numpy as np
        except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
            raise collection_dependency_error() from exc
        return {
            STATE_KEY: np.asarray(self.state, dtype=np.float32),
            ACTION_KEY: np.asarray(self.action, dtype=np.float32),
            **dict(self.images),
            "task": normalized_task,
        }


class SampleAssembler:
    """Stateful freshness, synchronization, and jump validation for one episode."""

    def __init__(self, config: CollectionConfig) -> None:
        if not isinstance(config, CollectionConfig):
            raise TypeError("SampleAssembler requires CollectionConfig")
        self._config = config
        self._camera_validator = CameraSetValidator(config.system.cameras)
        self._last_observation_sequence: int | None = None
        self._last_action_sequence: int | None = None

    def validate(self, sample: CollectionSample, *, now_ns: int) -> ValidatedSample:
        if not isinstance(sample, CollectionSample):
            raise CollectionContractError("sample must be a CollectionSample")
        if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns < 0:
            raise CollectionContractError("now_ns must be a non-negative integer")

        observation = _timed(sample.observation)
        action = _timed(sample.action)
        now_s = now_ns / 1_000_000_000
        require_fresh_sample(
            observation,
            label="follower observation",
            now_s=now_s,
            max_age_s=self._config.max_sample_age_s,
        )
        require_fresh_sample(
            action,
            label="leader action",
            now_s=now_s,
            max_age_s=self._config.max_sample_age_s,
        )
        require_pair_skew(
            observation,
            action,
            left_label="follower observation",
            right_label="leader action",
            max_skew_s=self._config.max_state_action_skew_s,
        )
        _require_increasing_sequence(
            observation.seq,
            previous=self._last_observation_sequence,
            label="follower observation",
        )
        _require_increasing_sequence(
            action.seq,
            previous=self._last_action_sequence,
            label="leader action",
        )

        state_values = validate_named_values(sample.observation.values)
        action_values = validate_named_values(sample.action.values)
        state = tuple(state_values[name] for name in FEATURE_NAMES)
        candidate_action = tuple(action_values[name] for name in FEATURE_NAMES)

        images: dict[str, Any] = {}
        stream_by_role = {stream.role: stream for stream in self._config.system.cameras.streams}
        enabled_roles = tuple(
            stream.role for stream in self._config.system.cameras.streams if stream.enabled
        )
        if set(sample.cameras) != set(enabled_roles):
            raise CollectionContractError("sample must contain every enabled camera role exactly")
        for role in enabled_roles:
            stream = stream_by_role[role]
            validate_camera_image(sample.cameras[role].image, stream)
            images[f"{IMAGE_PREFIX}{role}"] = _crop_camera_image(
                sample.cameras[role].image,
                stream,
            )
        metadata = {role: camera.metadata for role, camera in sample.cameras.items()}
        self._camera_validator.validate(metadata, now_ns=now_ns)
        max_robot_camera_skew_ns = int(self._config.max_robot_camera_skew_s * 1_000_000_000)
        camera, actual_skew_ns = max(
            (
                (
                    camera,
                    abs(camera.monotonic_ns - sample.observation.metadata.monotonic_ns),
                )
                for camera in metadata.values()
            ),
            key=lambda item: item[1],
        )
        if actual_skew_ns > max_robot_camera_skew_ns:
            raise CollectionContractError(
                f"robot/{camera.role} sample skew {actual_skew_ns / 1_000_000:.3f} ms "
                f"exceeds {self._config.max_robot_camera_skew_s * 1000:.3f} ms"
            )

        self._last_observation_sequence = observation.seq
        self._last_action_sequence = action.seq
        return ValidatedSample(state, candidate_action, MappingProxyType(images))


@dataclass(frozen=True)
class _TimedSample:
    seq: int
    monotonic_s: float


def canonical_dataset_contract(system: SystemConfig) -> DatasetContract:
    cameras = tuple(
        CameraSpec(
            stream.role,
            stream.height if stream.crop is None else stream.crop.height,
            stream.width if stream.crop is None else stream.crop.width,
        )
        for stream in system.cameras.streams
        if stream.enabled
    )
    return DatasetContract(FEATURE_NAMES, FEATURE_NAMES, cameras)


def normalize_task(value: str) -> str:
    if not isinstance(value, str):
        raise CollectionContractError("task must be text")
    task = value.strip()
    if not task or "\n" in task or "\r" in task:
        raise CollectionContractError("task must be one non-empty normalized line")
    return task


def _vector_feature(names: tuple[str, ...]) -> dict[str, Any]:
    if not names or len(names) != len(set(names)):
        raise CollectionContractError("vector feature names must be non-empty and unique")
    return {"dtype": "float32", "shape": (len(names),), "names": list(names)}


def _timed(vector: NamedJointVector) -> _TimedSample:
    if not isinstance(vector, NamedJointVector):
        raise CollectionContractError("joint sample must be a NamedJointVector")
    return _TimedSample(
        seq=vector.metadata.source_sequence,
        monotonic_s=vector.metadata.monotonic_ns / 1_000_000_000,
    )


def _require_increasing_sequence(value: int, *, previous: int | None, label: str) -> None:
    if previous is not None and value <= previous:
        raise CollectionContractError(f"{label} sequence did not increase")


def validate_camera_image(image: Any, config: CameraConfig) -> None:
    """Require one configured RGB image without importing an image library."""

    expected_array_shape = (config.height, config.width, 3)
    shape = getattr(image, "shape", None)
    if shape is not None:
        if tuple(shape) != expected_array_shape:
            raise CollectionContractError(
                f"{config.role} image shape must be {expected_array_shape}, got {tuple(shape)}"
            )
        if str(getattr(image, "dtype", "")) != "uint8":
            raise CollectionContractError(f"{config.role} image array dtype must be uint8")
        return
    size = getattr(image, "size", None)
    if (
        isinstance(size, tuple)
        and size == (config.width, config.height)
        and getattr(image, "mode", None) == "RGB"
    ):
        return
    raise CollectionContractError(
        f"{config.role} image must be an HxWx3 uint8 array or matching RGB PIL image"
    )


def _crop_camera_image(image: Any, config: CameraConfig) -> Any:
    roi = config.crop
    if roi is None:
        return image
    shape = getattr(image, "shape", None)
    if shape is not None:
        cropped = image[roi.y : roi.y + roi.height, roi.x : roi.x + roi.width]
        return cropped.copy()
    crop = getattr(image, "crop", None)
    if callable(crop):
        return crop((roi.x, roi.y, roi.x + roi.width, roi.y + roi.height))
    raise CollectionContractError(f"{config.role} image cannot apply its configured crop")
