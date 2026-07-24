"""Transactional LeRobot v3.0 to v2.1 export for canonical VLAI L1 datasets."""

from __future__ import annotations

import json
import shutil
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from embodied_ops import atomic_output_directory, atomic_write_text

from .dataset import DirectDatasetIdentity, inspect_direct_dataset, read_json
from .dependencies import collection_dependency_error, require_collection_python

V21_DATA_PATH = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
V21_VIDEO_PATH = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
CHUNK_SIZE = 1000


@dataclass(frozen=True)
class VideoProbe:
    frames: int
    width: int
    height: int


def export_v21_dataset(
    *,
    source: DirectDatasetIdentity,
    target_root: Path,
    repo_id: str,
    expected_provenance: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Export one validated canonical dataset without replacing an existing derivative."""

    state = inspect_direct_dataset(source, expected_provenance=expected_provenance)
    target = target_root.expanduser().absolute()
    with atomic_output_directory(target, overwrite=False) as staging:
        result = _build_v21(
            source=source,
            target=staging,
            published_target=target,
            repo_id=repo_id,
            source_episodes=state.total_episodes,
            source_frames=state.total_frames,
        )
    return result


def _build_v21(
    *,
    source: DirectDatasetIdentity,
    target: Path,
    published_target: Path,
    repo_id: str,
    source_episodes: int,
    source_frames: int,
) -> dict[str, Any]:
    require_collection_python()
    try:
        import numpy as np
        import pandas as pd
    except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
        raise collection_dependency_error() from exc

    info = read_json(source.target_root / "meta/info.json")
    data_paths = sorted(source.target_root.glob("data/**/*.parquet"))
    episode_paths = sorted(source.target_root.glob("meta/episodes/**/*.parquet"))
    if not data_paths or not episode_paths:
        raise ValueError("v3 source has no data or episode parquet payloads")
    frames = pd.concat([pd.read_parquet(path) for path in data_paths], ignore_index=True)
    episodes = pd.concat(
        [pd.read_parquet(path) for path in episode_paths], ignore_index=True
    ).sort_values("episode_index")
    tasks_frame = pd.read_parquet(source.target_root / "meta/tasks.parquet")
    if len(frames) != source_frames or len(episodes) != source_episodes:
        raise ValueError("v3 payload counts changed after source validation")
    video_keys = [
        key
        for key, feature in info["features"].items()
        if isinstance(feature, dict) and feature.get("dtype") == "video"
    ]

    (target / "meta").mkdir(parents=True)
    tasks = _task_records(tasks_frame)
    _write_jsonl(target / "meta/tasks.jsonl", tasks)
    task_by_index = {record["task_index"]: record["task"] for record in tasks}
    metadata_by_episode = {int(row["episode_index"]): row for _, row in episodes.iterrows()}
    episode_records: list[dict[str, Any]] = []
    stats_records: list[dict[str, Any]] = []
    for _, metadata in episodes.iterrows():
        episode_index = int(metadata["episode_index"])
        episode_frames = frames[frames["episode_index"] == episode_index].copy()
        if episode_frames.empty:
            raise ValueError(f"v3 metadata references empty episode {episode_index}")
        episode_frames = episode_frames.sort_values("frame_index")
        if not np.array_equal(
            episode_frames["frame_index"].to_numpy(),
            np.arange(len(episode_frames), dtype=np.int64),
        ):
            raise ValueError(f"episode {episode_index} frame_index is not contiguous")
        data_path = target / V21_DATA_PATH.format(
            episode_chunk=episode_index // CHUNK_SIZE,
            episode_index=episode_index,
        )
        data_path.parent.mkdir(parents=True, exist_ok=True)
        episode_frames.to_parquet(data_path, index=False)
        task_indices = sorted({int(value) for value in episode_frames["task_index"]})
        try:
            episode_tasks = [task_by_index[index] for index in task_indices]
        except KeyError as exc:
            raise ValueError(f"episode {episode_index} references an unknown task") from exc
        episode_records.append(
            {"episode_index": episode_index, "tasks": episode_tasks, "length": len(episode_frames)}
        )
        stats_records.append({"episode_index": episode_index, "stats": _episode_stats(metadata)})
    _write_jsonl(target / "meta/episodes.jsonl", episode_records)
    _write_jsonl(target / "meta/episodes_stats.jsonl", stats_records)

    video_count = 0
    fps = int(info["fps"])
    for key in video_keys:
        for record in episode_records:
            episode = int(record["episode_index"])
            metadata = metadata_by_episode[episode]
            source_video = _source_video_path(
                source.target_root,
                info,
                key=key,
                chunk=int(metadata[f"videos/{key}/chunk_index"]),
                file=int(metadata[f"videos/{key}/file_index"]),
            )
            start_frame = round(float(metadata[f"videos/{key}/from_timestamp"]) * fps)
            output = target / V21_VIDEO_PATH.format(
                episode_chunk=episode // CHUNK_SIZE,
                video_key=key,
                episode_index=episode,
            )
            output.parent.mkdir(parents=True, exist_ok=True)
            _slice_video(
                source=source_video,
                target=output,
                start_frame=start_frame,
                frame_count=int(record["length"]),
                fps=fps,
            )
            probe = _probe_video(output)
            if probe.frames != int(record["length"]):
                raise RuntimeError(
                    f"video frame mismatch for {key} episode {episode}: "
                    f"expected={record['length']}, actual={probe.frames}"
                )
            expected_height, expected_width, _ = info["features"][key]["shape"]
            if (probe.height, probe.width) != (expected_height, expected_width):
                raise RuntimeError(
                    f"video geometry mismatch for {key} episode {episode}: "
                    f"expected={expected_width}x{expected_height}, "
                    f"actual={probe.width}x{probe.height}"
                )
            video_count += 1

    _write_json(target / "meta/info.json", _v21_info(info, video_keys=video_keys))
    shutil.copy2(source.target_root / "meta/stats.json", target / "meta/stats.json")
    shutil.copy2(source.target_root / "meta/vlai_l1.json", target / "meta/source_vlai_l1.json")
    _write_json(
        target / "meta/vlai_l1_derivative.json",
        {
            "format": "lerobot_v2.1",
            "repo_id": repo_id,
            "source_repo_id": source.repo_id,
            "source_experiment": source.experiment,
            "source_format": "lerobot_v3.0",
            "source_total_episodes": source_episodes,
            "source_total_frames": source_frames,
            "video_codec": "h264",
        },
    )
    _validate_v21_output(
        target,
        episodes=episode_records,
        video_keys=video_keys,
        frames=source_frames,
    )
    return {
        "format": "v2.1",
        "repo_id": repo_id,
        "root": str(published_target),
        "episodes": source_episodes,
        "frames": source_frames,
        "videos": video_count,
        "camera_keys": video_keys,
    }


def _v21_info(source: dict[str, Any], *, video_keys: list[str]) -> dict[str, Any]:
    features = json.loads(json.dumps(source["features"]))
    for key in video_keys:
        height, width, channels = features[key]["shape"]
        features[key]["info"] = {
            "video.height": int(height),
            "video.width": int(width),
            "video.codec": "h264",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "video.fps": int(source["fps"]),
            "video.channels": int(channels),
            "has_audio": False,
        }
    episodes = int(source["total_episodes"])
    return {
        "codebase_version": "v2.1",
        "robot_type": source.get("robot_type"),
        "total_episodes": episodes,
        "total_frames": int(source["total_frames"]),
        "total_tasks": int(source["total_tasks"]),
        "total_videos": episodes * len(video_keys),
        "total_chunks": (episodes + CHUNK_SIZE - 1) // CHUNK_SIZE,
        "chunks_size": CHUNK_SIZE,
        "fps": int(source["fps"]),
        "splits": {"train": f"0:{episodes}"},
        "data_path": V21_DATA_PATH,
        "video_path": V21_VIDEO_PATH if video_keys else None,
        "features": features,
    }


def _task_records(frame: Any) -> list[dict[str, Any]]:
    return [
        {"task_index": int(row["task_index"]), "task": str(task)}
        for task, row in frame.sort_values("task_index").iterrows()
    ]


def _episode_stats(row: Any) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for column, value in row.items():
        if not isinstance(column, str) or not column.startswith("stats/"):
            continue
        _, feature, statistic = column.split("/", 2)
        result.setdefault(feature, {})[statistic] = _json_value(value)
    return result


def _source_video_path(
    root: Path, info: dict[str, Any], *, key: str, chunk: int, file: int
) -> Path:
    template = info.get("video_path")
    if not isinstance(template, str):
        raise ValueError("v3 info.video_path must be a path template")
    relative = Path(template.format(video_key=key, chunk_index=chunk, file_index=file))
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("v3 video path escapes the source dataset")
    path = root / relative
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"v3 video payload is missing or unsafe: {path}")
    return path


def _slice_video(
    *, source: Path, target: Path, start_frame: int, frame_count: int, fps: int
) -> None:
    subprocess.run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-y",
            "-ss",
            f"{start_frame / fps:.9f}",
            "-i",
            str(source),
            "-frames:v",
            str(frame_count),
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
            str(target),
        ],
        check=True,
    )


def _probe_video(path: Path) -> VideoProbe:
    result = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=nb_frames,width,height",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)
    streams = payload.get("streams")
    if not isinstance(streams, list) or len(streams) != 1:
        raise RuntimeError(f"ffprobe returned an invalid video stream for {path}")
    stream = streams[0]
    try:
        return VideoProbe(
            frames=int(stream["nb_frames"]),
            width=int(stream["width"]),
            height=int(stream["height"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"ffprobe returned incomplete video metadata for {path}") from exc


def _validate_v21_output(
    root: Path,
    *,
    episodes: list[dict[str, Any]],
    video_keys: list[str],
    frames: int,
) -> None:
    try:
        import pyarrow.parquet as parquet
    except ModuleNotFoundError as exc:  # pragma: no cover - loaded above in normal use
        raise RuntimeError("v2.1 validation requires pyarrow") from exc
    row_count = 0
    for record in episodes:
        episode = int(record["episode_index"])
        data = root / V21_DATA_PATH.format(
            episode_chunk=episode // CHUNK_SIZE,
            episode_index=episode,
        )
        row_count += parquet.ParquetFile(data).metadata.num_rows
        for key in video_keys:
            video = root / V21_VIDEO_PATH.format(
                episode_chunk=episode // CHUNK_SIZE,
                video_key=key,
                episode_index=episode,
            )
            if not video.is_file():
                raise ValueError(f"v2.1 video is missing: {video}")
    if row_count != frames:
        raise ValueError("v2.1 parquet row count differs from the v3 source")


def _write_json(path: Path, value: dict[str, Any]) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def _write_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    atomic_write_text(
        path,
        "".join(json.dumps(record, separators=(",", ":")) + "\n" for record in records),
    )


def _json_value(value: Any) -> Any:
    if hasattr(value, "tolist"):
        return _json_value(value.tolist())
    if hasattr(value, "item"):
        return _json_value(value.item())
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value
