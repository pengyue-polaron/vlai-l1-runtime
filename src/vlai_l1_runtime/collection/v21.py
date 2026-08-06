"""VLAI publication around the shared LeRobot v3-to-v2.1 builder."""

from __future__ import annotations

import json
import shutil
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from embodied_ops import atomic_output_directory, atomic_write_text, directory_sha256
from embodied_ops.datasets.lerobot import (
    V21_CHUNK_SIZE as CHUNK_SIZE,
)
from embodied_ops.datasets.lerobot import (
    V21_DATA_PATH,
    V21_VIDEO_PATH,
    build_lerobot_v21_dataset,
    make_lerobot_v21_info,
)

from .dataset import DirectDatasetIdentity, inspect_direct_dataset


def export_v21_dataset(
    *,
    source: DirectDatasetIdentity,
    target_root: Path,
    repo_id: str,
    expected_provenance: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Export one validated canonical dataset without replacing a derivative."""

    state = inspect_direct_dataset(source, expected_provenance=expected_provenance)
    target = target_root.expanduser().absolute()
    with atomic_output_directory(target, overwrite=False) as staging:
        result = build_lerobot_v21_dataset(
            source.target_root,
            staging,
            expected_episodes=state.total_episodes,
            expected_frames=state.total_frames,
        )
        shutil.copy2(
            source.target_root / "meta/vlai_l1.json",
            staging / "meta/source_vlai_l1.json",
        )
        _write_json(
            staging / "meta/vlai_l1_derivative.json",
            {
                "format": "lerobot_v2.1",
                "repo_id": repo_id,
                "source_repo_id": source.repo_id,
                "source_experiment": source.experiment,
                "source_format": "lerobot_v3.0",
                "source_total_episodes": state.total_episodes,
                "source_total_frames": state.total_frames,
                "video_codec": "h264",
            },
        )
        result.update(
            {
                "repo_id": repo_id,
                "root": str(target),
                "sha256": directory_sha256(staging),
            }
        )
    return result


def _write_json(path: Path, value: dict[str, Any]) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def _v21_info(source: dict[str, Any], *, video_keys: list[str]) -> dict[str, Any]:
    result = make_lerobot_v21_info(source)
    actual = [
        key
        for key, feature in source["features"].items()
        if isinstance(feature, dict) and feature.get("dtype") == "video"
    ]
    if actual != video_keys:
        raise ValueError("video keys differ from the source feature contract")
    return result


__all__ = [
    "CHUNK_SIZE",
    "V21_DATA_PATH",
    "V21_VIDEO_PATH",
    "export_v21_dataset",
]
