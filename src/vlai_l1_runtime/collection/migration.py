"""Hardware-free canonical dataset migrations for VLAI L1."""

from __future__ import annotations

import json
from collections.abc import Callable, Iterable, Mapping, Sequence
from contextlib import suppress
from dataclasses import asdict, dataclass, replace
from typing import Any, Protocol

from embodied_ops import LeadingStillnessConfig, LeadingStillnessTrimmer
from embodied_ops.artifacts import OutputDirectoryTransaction, atomic_write_text

from .configuration import CollectionConfig
from .dataset import (
    PROVENANCE_PATH,
    DatasetBackend,
    DatasetBackendFactory,
    DirectDatasetIdentity,
    DirectDatasetState,
    LeRobotBackendFactory,
    _lerobot_dataset_type,
    identity_from_config,
    inspect_direct_dataset,
    provenance_from_config,
    read_json,
)
from .schema import ACTION_KEY, normalize_task


class IndexedRows(Protocol):
    def __len__(self) -> int: ...

    def __getitem__(self, index: int) -> Mapping[str, Any]: ...


class DatasetReader(Protocol):
    meta: Any

    def __getitem__(self, index: int) -> Mapping[str, Any]: ...

    def select_columns(self, column_names: str | list[str]) -> IndexedRows: ...


@dataclass(frozen=True, slots=True)
class EpisodeTrimPlan:
    episode_index: int
    source_from_index: int
    source_to_index: int
    retained_from_index: int
    source_frames: int
    trimmed_frames: int
    output_frames: int


@dataclass(frozen=True, slots=True)
class LeadingStillnessMigrationPlan:
    episodes: tuple[EpisodeTrimPlan, ...]
    source_frames: int
    trimmed_frames: int
    output_frames: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "episodes": len(self.episodes),
            "source_frames": self.source_frames,
            "trimmed_frames": self.trimmed_frames,
            "output_frames": self.output_frames,
            "episode_plan": [asdict(episode) for episode in self.episodes],
        }


def plan_leading_stillness(
    actions: IndexedRows,
    episode_rows: Iterable[Mapping[str, Any]],
    config: LeadingStillnessConfig,
    *,
    expected_episodes: int,
    expected_frames: int,
) -> LeadingStillnessMigrationPlan:
    """Plan each retained suffix with the same gate used during live collection."""

    if not isinstance(config, LeadingStillnessConfig):
        raise TypeError("plan_leading_stillness requires LeadingStillnessConfig")
    if len(actions) != expected_frames:
        raise ValueError("action row count differs from canonical dataset metadata")

    plans: list[EpisodeTrimPlan] = []
    next_source_index = 0
    for expected_episode_index, row in enumerate(episode_rows):
        episode_index = _non_negative_integer(row, "episode_index")
        source_from_index = _non_negative_integer(row, "dataset_from_index")
        source_to_index = _non_negative_integer(row, "dataset_to_index")
        source_frames = _non_negative_integer(row, "length")
        if episode_index != expected_episode_index:
            raise ValueError("source episode indices must be contiguous from zero")
        if source_from_index != next_source_index:
            raise ValueError("source episode frame ranges must be contiguous")
        if source_to_index <= source_from_index:
            raise ValueError(f"source episode {episode_index} is empty")
        if source_to_index - source_from_index != source_frames:
            raise ValueError(f"source episode {episode_index} length differs from its frame range")

        trimmer = LeadingStillnessTrimmer[int](config)
        emitted: list[int] = []
        for relative_index, source_index in enumerate(range(source_from_index, source_to_index)):
            action_row = actions[source_index]
            if not isinstance(action_row, Mapping) or ACTION_KEY not in action_row:
                raise ValueError(f"source action row {source_index} is malformed")
            emitted.extend(trimmer.push(relative_index, _action_tuple(action_row[ACTION_KEY])))

        result = trimmer.result
        if not result.started or not emitted:
            raise ValueError(
                f"source episode {episode_index} never crosses the configured motion gate"
            )
        retained_relative_index = emitted[0]
        if emitted != list(range(retained_relative_index, source_frames)):
            raise RuntimeError(
                f"source episode {episode_index} produced a non-contiguous trim plan"
            )
        if result.trimmed_frames != retained_relative_index:
            raise RuntimeError(f"source episode {episode_index} trim accounting differs")

        output_frames = source_frames - retained_relative_index
        plans.append(
            EpisodeTrimPlan(
                episode_index=episode_index,
                source_from_index=source_from_index,
                source_to_index=source_to_index,
                retained_from_index=source_from_index + retained_relative_index,
                source_frames=source_frames,
                trimmed_frames=retained_relative_index,
                output_frames=output_frames,
            )
        )
        next_source_index = source_to_index

    if len(plans) != expected_episodes:
        raise ValueError("episode metadata count differs from canonical dataset metadata")
    if next_source_index != expected_frames:
        raise ValueError("episode frame ranges do not cover the canonical dataset")
    trimmed_frames = sum(episode.trimmed_frames for episode in plans)
    output_frames = sum(episode.output_frames for episode in plans)
    if output_frames != expected_frames - trimmed_frames:
        raise RuntimeError("whole-dataset trim accounting differs")
    return LeadingStillnessMigrationPlan(
        episodes=tuple(plans),
        source_frames=expected_frames,
        trimmed_frames=trimmed_frames,
        output_frames=output_frames,
    )


def trim_leading_stillness_dataset(
    config: CollectionConfig,
    *,
    source_experiment: str,
    target_experiment: str,
    dry_run: bool = False,
    reader_loader: Callable[[DirectDatasetIdentity], DatasetReader] | None = None,
    backend_factory: DatasetBackendFactory | None = None,
    inspector: Callable[..., DirectDatasetState] = inspect_direct_dataset,
    episode_completed: Callable[[EpisodeTrimPlan], None] | None = None,
) -> dict[str, Any]:
    """Rebuild a canonical dataset after removing each stationary prefix."""

    if not isinstance(config, CollectionConfig):
        raise TypeError("trim_leading_stillness_dataset requires CollectionConfig")
    if not isinstance(dry_run, bool):
        raise TypeError("dry_run must be a boolean")
    source = identity_from_config(config, source_experiment)
    target = identity_from_config(config, target_experiment)
    if source.experiment == target.experiment or source.target_root == target.target_root:
        raise ValueError("source and target experiments must be distinct")

    source_state = inspector(source)
    if source_state.total_episodes == 0 or source_state.task is None:
        raise ValueError(f"canonical source dataset does not exist: {source.target_root}")
    if target.target_root.exists():
        raise FileExistsError(f"target canonical dataset already exists: {target.target_root}")
    target_state = inspector(target, expected_provenance=provenance_from_config(config))
    if target_state.total_episodes != 0:
        raise FileExistsError(f"target canonical dataset already exists: {target.target_root}")

    loader = reader_loader or _load_lerobot_dataset
    reader = loader(source)
    episode_rows = getattr(getattr(reader, "meta", None), "episodes", None)
    if episode_rows is None:
        raise ValueError("source reader has no episode metadata")
    plan = plan_leading_stillness(
        reader.select_columns(ACTION_KEY),
        episode_rows,
        config.leading_stillness,
        expected_episodes=source_state.total_episodes,
        expected_frames=source_state.total_frames,
    )
    result = {
        "status": "DRY_RUN" if dry_run else "PASS",
        "source_experiment": source.experiment,
        "source_repo_id": source.repo_id,
        "source_root": str(source.target_root),
        "target_experiment": target.experiment,
        "target_repo_id": target.repo_id,
        "target_root": str(target.target_root),
        "leading_stillness": _stillness_payload(config.leading_stillness),
        **plan.as_dict(),
    }
    if dry_run:
        return result

    source_provenance = read_json(source.target_root / PROVENANCE_PATH)
    factory = backend_factory or LeRobotBackendFactory(config.image_writer_threads)
    with OutputDirectoryTransaction(
        target.target_root,
        overwrite=False,
        precreate_staging=False,
    ) as transaction:
        assert transaction.path is not None
        writer = factory.create(target, transaction.path)
        _write_trimmed_episodes(
            reader,
            writer,
            target=target,
            task=source_state.task,
            plan=plan,
            episode_completed=episode_completed,
        )
        info = read_json(transaction.path / "meta/info.json")
        if info.get("total_episodes") != source_state.total_episodes:
            raise RuntimeError("migrated episode count differs from its plan")
        if info.get("total_frames") != plan.output_frames:
            raise RuntimeError("migrated frame count differs from its plan")

        target_provenance = provenance_from_config(config)
        target_provenance.update(
            {
                "dataset_schema": source_provenance["dataset_schema"],
                "repo_id": target.repo_id,
                "experiment": target.experiment,
                "task": source_state.task,
                "total_episodes": source_state.total_episodes,
                "total_frames": plan.output_frames,
                "migration": {
                    "kind": "trim_leading_stillness",
                    "source_experiment": source.experiment,
                    "source_repo_id": source.repo_id,
                    "source_dataset_schema": source_provenance.get("dataset_schema"),
                    "source_collection_schema_version": source_provenance.get(
                        "collection_schema_version"
                    ),
                    "source_collection_config_sha256": source_provenance.get(
                        "collection_config_sha256"
                    ),
                    "source_total_episodes": source_state.total_episodes,
                    "source_total_frames": source_state.total_frames,
                    "leading_stillness": _stillness_payload(config.leading_stillness),
                    "trimmed_frames": plan.trimmed_frames,
                    "episode_plan": [asdict(episode) for episode in plan.episodes],
                },
            }
        )
        atomic_write_text(
            transaction.path / PROVENANCE_PATH,
            json.dumps(target_provenance, indent=2, sort_keys=True) + "\n",
        )
        staged_state = inspector(
            replace(target, target_root=transaction.path),
            expected_task=source_state.task,
            expected_provenance=provenance_from_config(config),
        )
        if staged_state.total_episodes != source_state.total_episodes:
            raise RuntimeError("validated migrated episode count differs from its plan")
        if staged_state.total_frames != plan.output_frames:
            raise RuntimeError("validated migrated frame count differs from its plan")
        transaction.commit()
    return result


def _write_trimmed_episodes(
    reader: DatasetReader,
    writer: DatasetBackend,
    *,
    target: DirectDatasetIdentity,
    task: str,
    plan: LeadingStillnessMigrationPlan,
    episode_completed: Callable[[EpisodeTrimPlan], None] | None,
) -> None:
    finalized = False
    feature_specs = target.contract.features()
    feature_keys = tuple(feature_specs)
    try:
        for episode in plan.episodes:
            for source_index in range(episode.retained_from_index, episode.source_to_index):
                item = reader[source_index]
                if not isinstance(item, Mapping):
                    raise ValueError(f"source frame {source_index} is malformed")
                frame_task = normalize_task(item.get("task"))
                if frame_task != task:
                    raise ValueError(f"source frame {source_index} task differs from the dataset")
                missing = [key for key in feature_keys if key not in item]
                if missing:
                    raise ValueError(f"source frame {source_index} is missing features: {missing}")
                frame = {
                    key: _normalize_frame_value(key, item[key], feature_specs[key])
                    for key in feature_keys
                }
                frame["task"] = frame_task
                writer.add_frame(frame)
            writer.save_episode(parallel_encoding=False)
            if episode_completed is not None:
                episode_completed(episode)
        writer.finalize()
        finalized = True
    except BaseException:
        with suppress(BaseException):
            writer.clear_episode_buffer()
        if not finalized:
            with suppress(BaseException):
                writer.finalize()
        raise


def _load_lerobot_dataset(identity: DirectDatasetIdentity) -> DatasetReader:
    dataset_type = _lerobot_dataset_type()
    return dataset_type(
        identity.repo_id,
        root=identity.target_root,
        return_uint8=True,
    )


def _stillness_payload(config: LeadingStillnessConfig) -> dict[str, Any]:
    return {
        "enabled": config.enabled,
        "action_thresholds": list(config.action_thresholds),
        "reference_frames": config.reference_frames,
        "motion_frames": config.motion_frames,
        "preroll_frames": config.preroll_frames,
    }


def _action_tuple(value: Any) -> tuple[float, ...]:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError("source action must be a numeric sequence")
    try:
        return tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError("source action must be a numeric sequence") from exc


def _normalize_frame_value(
    key: str,
    value: Any,
    feature: Mapping[str, Any],
) -> Any:
    if feature.get("dtype") != "video":
        return value
    expected_shape = tuple(feature.get("shape", ()))
    actual_shape = getattr(value, "shape", None)
    if actual_shape is None:
        return value
    actual_shape = tuple(actual_shape)
    if actual_shape == expected_shape:
        return value
    if len(expected_shape) != 3:
        raise ValueError(f"video feature {key!r} has a malformed contract shape")
    channel_first_shape = (expected_shape[2], expected_shape[0], expected_shape[1])
    if actual_shape != channel_first_shape:
        raise ValueError(
            f"source video feature {key!r} shape {actual_shape} differs from "
            f"HWC {expected_shape} and CHW {channel_first_shape}"
        )

    permute = getattr(value, "permute", None)
    if callable(permute):
        converted = permute(1, 2, 0)
        contiguous = getattr(converted, "contiguous", None)
        if callable(contiguous):
            converted = contiguous()
    else:
        transpose = getattr(value, "transpose", None)
        if not callable(transpose):
            raise ValueError(f"source video feature {key!r} cannot convert CHW to HWC")
        converted = transpose(1, 2, 0)
    if tuple(getattr(converted, "shape", ())) != expected_shape:
        raise RuntimeError(f"source video feature {key!r} did not convert to HWC")
    return converted


def _non_negative_integer(row: Mapping[str, Any], key: str) -> int:
    if not isinstance(row, Mapping):
        raise ValueError("source episode metadata row is malformed")
    value = row.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"source episode metadata {key} must be a non-negative integer")
    return value


__all__ = [
    "EpisodeTrimPlan",
    "LeadingStillnessMigrationPlan",
    "plan_leading_stillness",
    "trim_leading_stillness_dataset",
]
