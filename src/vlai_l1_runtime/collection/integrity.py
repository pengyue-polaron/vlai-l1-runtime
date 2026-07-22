"""Hardware-free integrity checks for a committed LeRobot v3 payload graph."""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any

from .dataset import GENERATED_FEATURES, read_json
from .dependencies import collection_dependency_error, require_collection_python
from .schema import ACTION_KEY, STATE_KEY


def validate_v3_payloads(
    root: Path,
    *,
    info: dict[str, Any],
    total_episodes: int,
    total_frames: int,
    expected_task: str,
) -> None:
    """Validate task/episode metadata and every referenced data/video payload."""

    require_collection_python()
    try:
        import pandas as pd
        import pyarrow.parquet as parquet
    except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
        raise collection_dependency_error() from exc

    stats = read_json(root / "meta/stats.json")
    if ACTION_KEY not in stats or STATE_KEY not in stats:
        raise ValueError("LeRobot stats are missing canonical vector features")

    tasks = _read_parquet(pd, root / "meta/tasks.parquet", label="tasks")
    if list(tasks.columns) != ["task_index"]:
        raise ValueError("LeRobot tasks must contain exactly the task_index column")
    task_records = {int(row["task_index"]): str(task) for task, row in tasks.iterrows()}
    if info.get("total_tasks") != 1 or task_records != {0: expected_task}:
        raise ValueError("canonical dataset must contain exactly its provenance task")

    episode_paths = sorted((root / "meta/episodes").glob("**/*.parquet"))
    if not episode_paths:
        raise ValueError("canonical dataset has no episode metadata")
    episodes = pd.concat(
        [_read_parquet(pd, path, label="episode metadata") for path in episode_paths],
        ignore_index=True,
    )
    required = {
        "episode_index",
        "tasks",
        "length",
        "data/chunk_index",
        "data/file_index",
        "dataset_from_index",
        "dataset_to_index",
    }
    missing = required - set(episodes.columns)
    if missing:
        raise ValueError(f"LeRobot episode metadata is missing {sorted(missing)}")
    if len(episodes) != total_episodes:
        raise ValueError("episode metadata count differs from info.json")
    episodes = episodes.sort_values("episode_index")
    if [_integer(value, "episode_index") for value in episodes["episode_index"]] != list(
        range(total_episodes)
    ):
        raise ValueError("episode indices must be contiguous from zero")

    data_template = _path_template(info, "data_path")
    video_keys = tuple(
        key
        for key, feature in info["features"].items()
        if isinstance(feature, dict) and feature.get("dtype") == "video"
    )
    video_template = _path_template(info, "video_path") if video_keys else None
    expected_rows: dict[Path, int] = defaultdict(int)
    next_frame = 0
    for _, row in episodes.iterrows():
        episode = _integer(row["episode_index"], "episode_index")
        length = _positive_integer(row["length"], f"episode {episode} length")
        start = _non_negative_integer(row["dataset_from_index"], "dataset_from_index")
        end = _non_negative_integer(row["dataset_to_index"], "dataset_to_index")
        if start != next_frame or end != start + length:
            raise ValueError(f"episode {episode} has a non-contiguous frame range")
        next_frame = end
        if tuple(str(task) for task in row["tasks"]) != (expected_task,):
            raise ValueError(f"episode {episode} task differs from provenance")
        data_path = _format_path(
            root,
            data_template,
            chunk_index=_non_negative_integer(row["data/chunk_index"], "data chunk"),
            file_index=_non_negative_integer(row["data/file_index"], "data file"),
        )
        expected_rows[data_path] += length
        for key in video_keys:
            assert video_template is not None
            chunk_key = f"videos/{key}/chunk_index"
            file_key = f"videos/{key}/file_index"
            if chunk_key not in episodes.columns or file_key not in episodes.columns:
                raise ValueError(f"episode metadata is missing video reference {key!r}")
            video = _format_path(
                root,
                video_template,
                video_key=key,
                chunk_index=_non_negative_integer(row[chunk_key], f"{key} video chunk"),
                file_index=_non_negative_integer(row[file_key], f"{key} video file"),
            )
            if not video.is_file() or video.is_symlink():
                raise ValueError(f"LeRobot video payload is missing or unsafe: {video}")
    if next_frame != total_frames:
        raise ValueError("episode frame ranges differ from info.json")

    required_columns = set(GENERATED_FEATURES) | {STATE_KEY, ACTION_KEY}
    for path, rows in expected_rows.items():
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"LeRobot data payload is missing or unsafe: {path}")
        try:
            payload = parquet.ParquetFile(path)
        except (OSError, ValueError) as exc:
            raise ValueError(f"cannot read LeRobot data payload {path}: {exc}") from exc
        if payload.metadata.num_rows != rows:
            raise ValueError(f"LeRobot data row count differs for {path}")
        missing_columns = required_columns - set(payload.schema_arrow.names)
        if missing_columns:
            raise ValueError(f"LeRobot data payload is missing {sorted(missing_columns)}")


def _read_parquet(pd: Any, path: Path, *, label: str) -> Any:
    if path.is_symlink():
        raise ValueError(f"{label} path must not be a symbolic link: {path}")
    try:
        return pd.read_parquet(path)
    except (OSError, ValueError) as exc:
        raise ValueError(f"cannot read LeRobot {label} {path}: {exc}") from exc


def _path_template(info: dict[str, Any], key: str) -> str:
    value = info.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"LeRobot info.{key} must be a path template")
    return value


def _format_path(root: Path, template: str, **values: object) -> Path:
    try:
        relative = Path(template.format(**values))
    except (KeyError, ValueError) as exc:
        raise ValueError(f"invalid LeRobot path template {template!r}") from exc
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"LeRobot payload path escapes the dataset: {relative}")
    return root / relative


def _integer(value: Any, label: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be an integer") from exc
    if isinstance(value, bool) or float(value) != result:
        raise ValueError(f"{label} must be an integer")
    return result


def _non_negative_integer(value: Any, label: str) -> int:
    result = _integer(value, label)
    if result < 0:
        raise ValueError(f"{label} must be non-negative")
    return result


def _positive_integer(value: Any, label: str) -> int:
    result = _integer(value, label)
    if result <= 0:
        raise ValueError(f"{label} must be positive")
    return result
