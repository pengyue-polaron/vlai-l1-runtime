"""Strict, hardware-free configuration for the L1 collection workflow."""

from __future__ import annotations

import hashlib
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - exercised by the Python 3.10 check
    import tomli as tomllib

from embodied_ops import validate_experiment_name

from ..configuration import (
    ConfigError,
    SystemConfig,
    _exact_keys,
    _integer,
    _mapping,
    _positive_number,
    _read_local_regular_file,
    _text,
    load_system_config,
)

_REPO_PREFIX = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*/[A-Za-z0-9][A-Za-z0-9._-]*$")


@dataclass(frozen=True)
class CollectionConfig:
    path: Path
    repo_root: Path
    schema_version: int
    system: SystemConfig
    dataset_root: Path
    derivative_root: Path
    repo_id_prefix: str
    fps: int
    max_episode_frames: int
    minimum_capture_fps: float
    image_writer_threads: int
    max_sample_age_s: float
    max_state_action_skew_s: float
    max_robot_camera_skew_s: float
    config_sha256: str
    system_config_sha256: str

    @property
    def collection_blockers(self) -> tuple[str, ...]:
        camera_blockers = tuple(
            f"camera_{stream.role}_uncommissioned"
            for stream in self.system.cameras.streams
            if stream.required_for_collection and not stream.enabled
        )
        return (*self.system.teleoperation.blockers, *camera_blockers)

    @property
    def collection_ready(self) -> bool:
        return not self.collection_blockers

    def dataset_root_for(self, experiment: str) -> Path:
        return self.dataset_root / validate_experiment_name(experiment)

    def v21_root_for(self, experiment: str) -> Path:
        return self.derivative_root / f"{validate_experiment_name(experiment)}-v2.1"

    def repo_id_for(self, experiment: str, *, derivative: str | None = None) -> str:
        name = validate_experiment_name(experiment)
        suffix = "" if derivative is None else f"-{validate_experiment_name(derivative)}"
        return f"{self.repo_id_prefix}-{name}{suffix}"


def load_collection_config(path: Path) -> CollectionConfig:
    """Load one complete collection contract without touching any device."""

    resolved = Path(os.path.abspath(os.fspath(path)))
    try:
        content = _read_local_regular_file(resolved, label="collection config")
        raw = tomllib.loads(content.decode("utf-8"))
    except ConfigError:
        raise
    except (OSError, UnicodeDecodeError, tomllib.TOMLDecodeError) as exc:
        raise ConfigError(f"cannot load collection config {resolved}: {exc}") from exc

    repo_root = _repository_root(resolved)
    root = _mapping(raw, "collection")
    keys = {
        "schema_version",
        "system_config",
        "dataset_root",
        "derivative_root",
        "repo_id_prefix",
        "fps",
        "max_episode_frames",
        "minimum_capture_fps",
        "image_writer_threads",
        "max_sample_age_s",
        "max_state_action_skew_s",
        "max_robot_camera_skew_s",
    }
    _exact_keys(root, keys, "collection")
    schema_version = _integer(root["schema_version"], "schema_version", minimum=1)
    if schema_version != 2:
        raise ConfigError(f"unsupported collection schema_version: {schema_version}")

    system_path = _relative_path(root["system_config"], "system_config", resolved.parent)
    dataset_root = _relative_path(root["dataset_root"], "dataset_root", resolved.parent)
    derivative_root = _relative_path(root["derivative_root"], "derivative_root", resolved.parent)
    if system_path.parent != repo_root / "configs/system" or system_path.suffix != ".toml":
        raise ConfigError("system_config must reference a repository TOML under configs/system")
    _validate_data_root(dataset_root, repo_root=repo_root, label="dataset_root")
    _validate_data_root(derivative_root, repo_root=repo_root, label="derivative_root")
    if dataset_root == derivative_root:
        raise ConfigError("dataset_root and derivative_root must be distinct")
    if _is_within(dataset_root, derivative_root) or _is_within(derivative_root, dataset_root):
        raise ConfigError("dataset_root and derivative_root must not contain one another")

    prefix = _text(root["repo_id_prefix"], "repo_id_prefix")
    if _REPO_PREFIX.fullmatch(prefix) is None:
        raise ConfigError("repo_id_prefix must be a portable 'owner/name' prefix")
    fps = _integer(root["fps"], "fps", minimum=1, maximum=240)
    max_episode_frames = _integer(
        root["max_episode_frames"],
        "max_episode_frames",
        minimum=1,
    )
    minimum_capture_fps = _positive_number(
        root["minimum_capture_fps"],
        "minimum_capture_fps",
    )
    if minimum_capture_fps > fps:
        raise ConfigError("minimum_capture_fps must not exceed fps")
    image_writer_threads = _integer(
        root["image_writer_threads"],
        "image_writer_threads",
        minimum=1,
        maximum=128,
    )
    sample_age = _positive_number(root["max_sample_age_s"], "max_sample_age_s")
    state_action_skew = _positive_number(root["max_state_action_skew_s"], "max_state_action_skew_s")
    if state_action_skew > sample_age:
        raise ConfigError("max_state_action_skew_s must not exceed max_sample_age_s")
    robot_camera_skew = _positive_number(root["max_robot_camera_skew_s"], "max_robot_camera_skew_s")
    if robot_camera_skew > sample_age:
        raise ConfigError("max_robot_camera_skew_s must not exceed max_sample_age_s")
    system = load_system_config(system_path)
    return CollectionConfig(
        path=resolved,
        repo_root=repo_root,
        schema_version=schema_version,
        system=system,
        dataset_root=dataset_root,
        derivative_root=derivative_root,
        repo_id_prefix=prefix,
        fps=fps,
        max_episode_frames=max_episode_frames,
        minimum_capture_fps=minimum_capture_fps,
        image_writer_threads=image_writer_threads,
        max_sample_age_s=sample_age,
        max_state_action_skew_s=state_action_skew,
        max_robot_camera_skew_s=robot_camera_skew,
        config_sha256=hashlib.sha256(content).hexdigest(),
        system_config_sha256=hashlib.sha256(
            _read_local_regular_file(system_path, label="system config")
        ).hexdigest(),
    )


def _relative_path(value: Any, label: str, base: Path) -> Path:
    text = _text(value, label)
    candidate = Path(text)
    if candidate.is_absolute():
        raise ConfigError(f"{label} must be relative to the collection config")
    return Path(os.path.abspath(os.fspath(base / candidate)))


def _is_within(candidate: Path, parent: Path) -> bool:
    try:
        candidate.relative_to(parent)
    except ValueError:
        return False
    return True


def _repository_root(config_path: Path) -> Path:
    if config_path.parent.name != "collection" or config_path.parent.parent.name != "configs":
        raise ConfigError("collection config must be tracked under configs/collection")
    return config_path.parents[2]


def _validate_data_root(path: Path, *, repo_root: Path, label: str) -> None:
    data_root = repo_root / "data"
    if path == data_root or not _is_within(path, data_root):
        raise ConfigError(f"{label} must be a dedicated directory under repository data/")
    cursor = repo_root
    for component in path.relative_to(repo_root).parts:
        cursor /= component
        if cursor.is_symlink():
            raise ConfigError(f"{label} path contains a symbolic link: {cursor}")
        if cursor.exists() and not cursor.is_dir():
            raise ConfigError(f"{label} path component is not a directory: {cursor}")
