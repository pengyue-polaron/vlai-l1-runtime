"""Atomic, directly recorded LeRobot v3 episode transactions for VLAI L1."""

from __future__ import annotations

import json
import os
import re
import shutil
import stat
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Protocol

from embodied_ops import OutputDirectoryTransaction, atomic_write_text

from ..contracts import FEATURE_NAMES
from .dependencies import collection_dependency_error, require_collection_python
from .schema import DATASET_SCHEMA, DatasetContract, canonical_dataset_contract, normalize_task

PROVENANCE_PATH = Path("meta/vlai_l1.json")
LEROBOT_VERSION = "v3.0"
GENERATED_FEATURES = {
    "timestamp": {"dtype": "float32", "shape": (1,), "names": None},
    "frame_index": {"dtype": "int64", "shape": (1,), "names": None},
    "episode_index": {"dtype": "int64", "shape": (1,), "names": None},
    "index": {"dtype": "int64", "shape": (1,), "names": None},
    "task_index": {"dtype": "int64", "shape": (1,), "names": None},
}
_REPO_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*/[A-Za-z0-9][A-Za-z0-9._-]*$")
_RESERVED_PROVENANCE_FIELDS = {
    "dataset_schema",
    "repo_id",
    "experiment",
    "task",
    "total_episodes",
    "total_frames",
}
_REPRODUCIBILITY_FIELDS = {
    "collection_schema_version",
    "collection_config_sha256",
    "system_config_sha256",
    "robot_id",
    "topology_id",
    "position_unit",
    "fps",
    "image_storage",
    "feature_names",
    "camera_roles",
    "teleoperation_provider",
    "teleoperation_sdk_version",
    "teleoperation_source_revision",
    "teleoperation_state_protocol_version",
}


class DatasetBackend(Protocol):
    def add_frame(self, frame: dict[str, Any]) -> None: ...

    def save_episode(self, parallel_encoding: bool = False) -> None: ...

    def clear_episode_buffer(self) -> None: ...

    def finalize(self) -> None: ...


class DatasetBackendFactory(Protocol):
    def create(self, identity: DirectDatasetIdentity, root: Path) -> DatasetBackend: ...

    def resume(self, identity: DirectDatasetIdentity, root: Path) -> DatasetBackend: ...


@dataclass(frozen=True)
class DirectDatasetIdentity:
    target_root: Path
    repo_id: str
    fps: int
    contract: DatasetContract
    experiment: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_root", self.target_root.expanduser().absolute())
        if _REPO_ID.fullmatch(self.repo_id) is None:
            raise ValueError("repo_id must be a portable namespaced identifier")
        if isinstance(self.fps, bool) or not isinstance(self.fps, int) or self.fps <= 0:
            raise ValueError("dataset fps must be a positive integer")
        from embodied_ops import validate_experiment_name

        validate_experiment_name(self.experiment)


@dataclass(frozen=True)
class DirectDatasetState:
    total_episodes: int
    total_frames: int
    task: str | None


@dataclass(frozen=True)
class LeRobotBackendFactory:
    """Resolve the optional LeRobot boundary and create dataset backends."""

    image_writer_threads: int

    def __post_init__(self) -> None:
        if (
            isinstance(self.image_writer_threads, bool)
            or not isinstance(self.image_writer_threads, int)
            or self.image_writer_threads <= 0
        ):
            raise ValueError("image_writer_threads must be a positive integer")

    def verify_dependency(self) -> None:
        _lerobot_dataset_type()

    def create(self, identity: DirectDatasetIdentity, root: Path) -> DatasetBackend:
        dataset_type = _lerobot_dataset_type()
        return dataset_type.create(
            repo_id=identity.repo_id,
            root=root,
            fps=identity.fps,
            robot_type="vlai_l1",
            features=identity.contract.features(),
            use_videos=True,
            image_writer_threads=self.image_writer_threads,
        )

    def resume(self, identity: DirectDatasetIdentity, root: Path) -> DatasetBackend:
        dataset_type = _lerobot_dataset_type()
        return dataset_type.resume(
            repo_id=identity.repo_id,
            root=root,
            image_writer_threads=self.image_writer_threads,
        )


def _lerobot_dataset_type() -> type:
    os.environ["SVT_LOG_FILE"] = os.devnull
    os.environ["HF_DATASETS_DISABLE_PROGRESS_BARS"] = "1"
    require_collection_python()
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
        raise collection_dependency_error() from exc
    return LeRobotDataset


class DirectLeRobotEpisode:
    """Append one episode through a hidden sibling snapshot and atomic rename."""

    def __init__(
        self,
        *,
        identity: DirectDatasetIdentity,
        task: str,
        provenance: Mapping[str, Any],
        backend_factory: DatasetBackendFactory,
        inspector: Callable[..., DirectDatasetState] | None = None,
    ) -> None:
        self.identity = identity
        self.task = normalize_task(task)
        if not isinstance(provenance, Mapping) or not all(
            isinstance(key, str) for key in provenance
        ):
            raise ValueError("collection provenance must be a text-keyed mapping")
        reserved = _RESERVED_PROVENANCE_FIELDS & set(provenance)
        if reserved:
            raise ValueError(f"collection provenance contains reserved fields: {sorted(reserved)}")
        missing = _REPRODUCIBILITY_FIELDS - set(provenance)
        if missing:
            raise ValueError(f"collection provenance is missing fields: {sorted(missing)}")
        try:
            snapshot = json.loads(json.dumps(dict(provenance), allow_nan=False))
        except (TypeError, ValueError) as exc:
            raise ValueError("collection provenance must contain finite JSON values") from exc
        self.provenance = snapshot
        self._factory = backend_factory
        self._inspector = inspector or inspect_direct_dataset
        self._transaction: OutputDirectoryTransaction | None = None
        self._dataset: DatasetBackend | None = None
        self._finalized = False
        self._committed = False

    def __enter__(self) -> DirectLeRobotEpisode:
        state = self._inspector(
            self.identity,
            expected_task=self.task,
            expected_provenance=self.provenance,
        )
        exists = state.total_episodes > 0
        if exists:
            previous = read_json(self.identity.target_root / PROVENANCE_PATH)
            changed = sorted(
                key for key, value in self.provenance.items() if previous.get(key) != value
            )
            if changed:
                raise ValueError(
                    "collection provenance changed; use a new experiment "
                    f"identity (fields={changed})"
                )

        transaction = OutputDirectoryTransaction(
            self.identity.target_root,
            overwrite=exists,
            precreate_staging=False,
        )
        transaction.__enter__()
        self._transaction = transaction
        assert transaction.path is not None
        try:
            if exists:
                copy_dataset_snapshot(self.identity.target_root, transaction.path)
                self._dataset = self._factory.resume(self.identity, transaction.path)
            else:
                self._dataset = self._factory.create(self.identity, transaction.path)
        except BaseException as error:
            transaction.__exit__(type(error), error, error.__traceback__)
            self._transaction = None
            raise
        return self

    def add_frame(self, frame: dict[str, Any]) -> None:
        if self._dataset is None:
            raise RuntimeError("episode transaction has not started")
        self._dataset.add_frame(frame)

    def commit(self) -> Path:
        if self._dataset is None or self._transaction is None:
            raise RuntimeError("episode transaction has not started")
        if self._committed:
            raise RuntimeError("episode transaction was already committed")
        self._dataset.save_episode(parallel_encoding=False)
        self._finalize()
        assert self._transaction.path is not None
        info = read_json(self._transaction.path / "meta/info.json")
        payload = {
            **self.provenance,
            "dataset_schema": DATASET_SCHEMA,
            "repo_id": self.identity.repo_id,
            "experiment": self.identity.experiment,
            "task": self.task,
            "total_episodes": _non_negative_int(info, "total_episodes"),
            "total_frames": _non_negative_int(info, "total_frames"),
        }
        atomic_write_text(
            self._transaction.path / PROVENANCE_PATH,
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
        )
        self._inspector(
            replace(self.identity, target_root=self._transaction.path),
            expected_task=self.task,
            expected_provenance=self.provenance,
        )
        try:
            return self._transaction.commit()
        finally:
            self._committed = self._transaction.committed

    def discard(self) -> None:
        if self._dataset is not None:
            self._dataset.clear_episode_buffer()

    def _finalize(self) -> None:
        if self._dataset is not None and not self._finalized:
            self._dataset.finalize()
            self._finalized = True

    def __exit__(self, exc_type, exc, traceback) -> None:
        cleanup_error: BaseException | None = None
        try:
            if not self._committed:
                self.discard()
                self._finalize()
        except BaseException as error:
            cleanup_error = error
        finally:
            if self._transaction is not None:
                self._transaction.__exit__(exc_type, exc, traceback)
        if exc_type is None and cleanup_error is not None:
            raise cleanup_error


def inspect_direct_dataset(
    identity: DirectDatasetIdentity,
    *,
    expected_task: str | None = None,
    expected_provenance: Mapping[str, Any] | None = None,
) -> DirectDatasetState:
    """Validate canonical metadata and all referenced LeRobot payloads."""

    validate_no_transaction_leftovers(identity.target_root)
    if not identity.target_root.exists():
        return DirectDatasetState(0, 0, None)
    if not identity.target_root.is_dir() or identity.target_root.is_symlink():
        raise ValueError(f"dataset target is not a real directory: {identity.target_root}")
    _validate_real_dataset_tree(identity.target_root)
    info = read_json(identity.target_root / "meta/info.json")
    provenance = read_json(identity.target_root / PROVENANCE_PATH)
    if info.get("codebase_version") != LEROBOT_VERSION:
        raise ValueError("canonical dataset must use LeRobot v3.0")
    if info.get("robot_type") != "vlai_l1":
        raise ValueError("canonical dataset robot_type must be 'vlai_l1'")
    if info.get("fps") != identity.fps:
        raise ValueError("canonical dataset FPS differs from collection config")
    _validate_features(info.get("features"), identity.contract)
    identity_provenance = {
        "dataset_schema": DATASET_SCHEMA,
        "repo_id": identity.repo_id,
        "experiment": identity.experiment,
    }
    for key, value in identity_provenance.items():
        if provenance.get(key) != value:
            raise ValueError(f"canonical provenance {key} does not match")
    if expected_provenance is not None:
        changed = sorted(
            key for key, value in expected_provenance.items() if provenance.get(key) != value
        )
        if changed:
            raise ValueError(f"canonical provenance differs from tracked config: {changed}")
    task = normalize_task(provenance.get("task"))
    if expected_task is not None and task != expected_task:
        raise ValueError("canonical dataset task differs from the requested task")
    total_episodes = _non_negative_int(info, "total_episodes")
    total_frames = _non_negative_int(info, "total_frames")
    if total_episodes == 0 or total_frames == 0:
        raise ValueError("a published canonical dataset must not be empty")
    if provenance.get("total_episodes") != total_episodes:
        raise ValueError("provenance episode count differs from LeRobot metadata")
    if provenance.get("total_frames") != total_frames:
        raise ValueError("provenance frame count differs from LeRobot metadata")

    from .integrity import validate_v3_payloads

    validate_v3_payloads(
        identity.target_root,
        info=info,
        total_episodes=total_episodes,
        total_frames=total_frames,
        expected_task=task,
    )
    return DirectDatasetState(total_episodes, total_frames, task)


def identity_from_config(config: Any, experiment: str) -> DirectDatasetIdentity:
    """Build the sole canonical dataset identity from a loaded CollectionConfig."""

    from .configuration import CollectionConfig

    if not isinstance(config, CollectionConfig):
        raise TypeError("identity_from_config requires CollectionConfig")
    return DirectDatasetIdentity(
        target_root=config.dataset_root_for(experiment),
        repo_id=config.repo_id_for(experiment),
        fps=config.fps,
        contract=canonical_dataset_contract(config.system),
        experiment=experiment,
    )


def provenance_from_config(config: Any) -> dict[str, Any]:
    """Build reproducibility metadata directly from the two loaded tracked configs."""

    from .configuration import CollectionConfig

    if not isinstance(config, CollectionConfig):
        raise TypeError("provenance_from_config requires CollectionConfig")
    return {
        "collection_schema_version": config.schema_version,
        "collection_config_sha256": config.config_sha256,
        "system_config_sha256": config.system_config_sha256,
        "robot_id": config.system.robot_id,
        "topology_id": config.system.topology_id,
        "position_unit": config.system.position_unit,
        "fps": config.fps,
        "image_storage": "video",
        "feature_names": list(FEATURE_NAMES),
        "camera_roles": [stream.role for stream in config.system.cameras.streams if stream.enabled],
        "teleoperation_provider": config.system.teleoperation.provider,
        "teleoperation_sdk_version": config.system.teleoperation.sdk_version,
        "teleoperation_source_revision": config.system.teleoperation.source_revision,
        "teleoperation_state_protocol_version": (
            config.system.teleoperation.state_protocol_version
        ),
    }


def validate_no_transaction_leftovers(target_root: Path) -> None:
    leftovers = sorted(
        path.name
        for pattern in (f".{target_root.name}.staging-*", f".{target_root.name}.backup-*")
        for path in target_root.parent.glob(pattern)
    )
    if leftovers:
        raise ValueError(
            f"dataset has an unfinished transaction; inspect it before reuse: {leftovers}"
        )


def copy_dataset_snapshot(source: Path, target: Path) -> None:
    """Copy metadata and hard-link immutable data/video payloads for one append."""

    target.mkdir(parents=True)
    for path in source.rglob("*"):
        relative = path.relative_to(source)
        destination = target / relative
        mode = os.lstat(path).st_mode
        if stat.S_ISLNK(mode):
            raise ValueError(f"dataset snapshot refuses symbolic links: {path}")
        if stat.S_ISDIR(mode):
            destination.mkdir(parents=True, exist_ok=True)
            continue
        if not stat.S_ISREG(mode):
            raise ValueError(f"dataset snapshot refuses special files: {path}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        if relative.parts[0] in {"data", "videos", "images"}:
            try:
                os.link(path, destination)
            except OSError as exc:
                raise RuntimeError(
                    f"atomic append requires hard-link support for payload {path}"
                ) from exc
        else:
            shutil.copy2(path, destination)


def _validate_real_dataset_tree(root: Path) -> None:
    for directory, names, files in os.walk(root, followlinks=False):
        directory_path = Path(directory)
        for name in (*names, *files):
            path = directory_path / name
            mode = os.lstat(path).st_mode
            if stat.S_ISLNK(mode):
                raise ValueError(f"canonical dataset refuses symbolic links: {path}")
            if not stat.S_ISDIR(mode) and not stat.S_ISREG(mode):
                raise ValueError(f"canonical dataset refuses special files: {path}")


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _validate_features(value: Any, contract: DatasetContract) -> None:
    if not isinstance(value, dict):
        raise ValueError("canonical info.features must be an object")
    expected = {**contract.features(), **GENERATED_FEATURES}
    if set(value) != set(expected):
        raise ValueError("canonical feature keys differ from the L1 dataset contract")
    for key, feature in expected.items():
        actual = value.get(key)
        if not isinstance(actual, dict):
            raise ValueError(f"canonical feature {key!r} is malformed")
        for field in ("dtype", "shape", "names"):
            wanted = feature.get(field)
            if isinstance(wanted, tuple):
                wanted = list(wanted)
            if actual.get(field) != wanted:
                raise ValueError(f"canonical feature {key!r}.{field} differs")


def _non_negative_int(value: Mapping[str, Any], key: str) -> int:
    result = value.get(key)
    if isinstance(result, bool) or not isinstance(result, int) or result < 0:
        raise ValueError(f"{key} must be a non-negative integer")
    return result
