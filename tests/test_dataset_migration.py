from __future__ import annotations

import json
import os
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from embodied_ops import LeadingStillnessConfig

from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.dataset import DirectDatasetState
from vlai_l1_runtime.collection.migration import (
    _normalize_frame_value,
    plan_leading_stillness,
    trim_leading_stillness_dataset,
)
from vlai_l1_runtime.collection.schema import ACTION_KEY

if not hasattr(os, "O_PATH"):
    pytest.skip(
        "VLAI configuration validation requires Linux O_PATH semantics",
        allow_module_level=True,
    )

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_collection_config(ROOT / "configs/collection/right_only.toml")
TRIM_CONFIG = LeadingStillnessConfig(
    enabled=True,
    action_thresholds=(0.5,) * 8,
    reference_frames=2,
    motion_frames=2,
    preroll_frames=1,
)


def _actions(*values: float) -> list[dict[str, Any]]:
    return [{ACTION_KEY: [value] * 8} for value in values]


def test_trim_plan_uses_shared_motion_gate_and_keeps_contiguous_suffixes() -> None:
    plan = plan_leading_stillness(
        _actions(0, 0, 0, 2, 2, 3, 10, 10, 10, 10, 12, 12),
        [
            {
                "episode_index": 0,
                "dataset_from_index": 0,
                "dataset_to_index": 6,
                "length": 6,
            },
            {
                "episode_index": 1,
                "dataset_from_index": 6,
                "dataset_to_index": 12,
                "length": 6,
            },
        ],
        TRIM_CONFIG,
        expected_episodes=2,
        expected_frames=12,
    )

    assert [episode.trimmed_frames for episode in plan.episodes] == [2, 3]
    assert [episode.retained_from_index for episode in plan.episodes] == [2, 9]
    assert plan.source_frames == 12
    assert plan.trimmed_frames == 5
    assert plan.output_frames == 7


def test_trim_plan_fails_closed_when_an_episode_never_moves() -> None:
    with pytest.raises(ValueError, match="episode 0 never crosses"):
        plan_leading_stillness(
            _actions(0, 0, 0, 0),
            [
                {
                    "episode_index": 0,
                    "dataset_from_index": 0,
                    "dataset_to_index": 4,
                    "length": 4,
                }
            ],
            TRIM_CONFIG,
            expected_episodes=1,
            expected_frames=4,
        )


def test_migration_normalizes_decoded_chw_video_frames_to_writer_hwc() -> None:
    chw = np.zeros((3, 4, 5), dtype=np.uint8)
    hwc = _normalize_frame_value(
        "observation.images.test",
        chw,
        {"dtype": "video", "shape": (4, 5, 3)},
    )

    assert hwc.shape == (4, 5, 3)
    assert np.shares_memory(chw, hwc)
    with pytest.raises(ValueError, match="differs from HWC"):
        _normalize_frame_value(
            "observation.images.test",
            np.zeros((4, 3, 5), dtype=np.uint8),
            {"dtype": "video", "shape": (4, 5, 3)},
        )


class FakeReader:
    def __init__(self, frames: list[dict[str, Any]]) -> None:
        self.frames = frames
        self.meta = SimpleNamespace(
            episodes=[
                {
                    "episode_index": 0,
                    "dataset_from_index": 0,
                    "dataset_to_index": len(frames),
                    "length": len(frames),
                }
            ]
        )

    def __getitem__(self, index: int) -> dict[str, Any]:
        return self.frames[index]

    def select_columns(self, column_names: str | list[str]):
        assert column_names == ACTION_KEY
        return [{ACTION_KEY: frame[ACTION_KEY]} for frame in self.frames]


class FakeWriter:
    def __init__(self, root: Path, *, fail: bool = False) -> None:
        self.root = root
        self.fail = fail
        self.episode_buffer: list[dict[str, Any]] = []
        self.episode_lengths: list[int] = []

    def add_frame(self, frame: dict[str, Any]) -> None:
        self.episode_buffer.append(frame)

    def save_episode(self, parallel_encoding: bool = False) -> None:
        assert parallel_encoding is False
        if self.fail:
            raise RuntimeError("encoding failed")
        self.episode_lengths.append(len(self.episode_buffer))
        self.episode_buffer.clear()

    def clear_episode_buffer(self) -> None:
        self.episode_buffer.clear()

    def finalize(self) -> None:
        (self.root / "meta").mkdir(parents=True, exist_ok=True)
        (self.root / "meta/info.json").write_text(
            json.dumps(
                {
                    "total_episodes": len(self.episode_lengths),
                    "total_frames": sum(self.episode_lengths),
                }
            ),
            encoding="utf-8",
        )


class FakeFactory:
    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail
        self.writer: FakeWriter | None = None

    def create(self, identity, root: Path) -> FakeWriter:
        del identity
        self.writer = FakeWriter(root, fail=self.fail)
        return self.writer

    def resume(self, identity, root: Path):
        del identity, root
        raise AssertionError("migration must never resume an existing target")


def _migration_fixture(tmp_path: Path):
    config = replace(
        CONFIG,
        dataset_root=tmp_path,
        leading_stillness=TRIM_CONFIG,
    )
    source_root = tmp_path / "source"
    (source_root / "meta").mkdir(parents=True)
    (source_root / "meta/vlai_l1.json").write_text(
        json.dumps(
            {
                "dataset_schema": "vlai_l1_lerobot_dataset_v3_v3",
                "collection_schema_version": 4,
                "collection_config_sha256": "old-config",
            }
        ),
        encoding="utf-8",
    )
    features = tuple(config.feature_names)
    frame_template = {
        key: object()
        for key in (
            "observation.state",
            "observation.images.wrist_right",
            "observation.images.agent",
        )
    }
    frames = [
        {
            **frame_template,
            ACTION_KEY: [value] * len(features),
            "task": "put the blue block into the red plate",
        }
        for value in (0, 0, 0, 2, 2, 3)
    ]

    def inspector(identity, *, expected_task=None, expected_provenance=None):
        del expected_provenance
        if identity.experiment == "source":
            return DirectDatasetState(1, 6, "put the blue block into the red plate")
        if not identity.target_root.exists():
            return DirectDatasetState(0, 0, None)
        provenance = json.loads((identity.target_root / "meta/vlai_l1.json").read_text())
        assert provenance["task"] == expected_task
        return DirectDatasetState(
            provenance["total_episodes"],
            provenance["total_frames"],
            provenance["task"],
        )

    return config, FakeReader(frames), inspector


def test_migration_publishes_new_target_with_source_and_trim_provenance(
    tmp_path: Path,
) -> None:
    config, reader, inspector = _migration_fixture(tmp_path)
    factory = FakeFactory()
    completed = []

    result = trim_leading_stillness_dataset(
        config,
        source_experiment="source",
        target_experiment="target",
        reader_loader=lambda _identity: reader,
        backend_factory=factory,
        inspector=inspector,
        episode_completed=completed.append,
    )

    assert result["status"] == "PASS"
    assert result["source_frames"] == 6
    assert result["trimmed_frames"] == 2
    assert result["output_frames"] == 4
    assert factory.writer is not None
    assert factory.writer.episode_lengths == [4]
    assert [episode.episode_index for episode in completed] == [0]
    provenance = json.loads((tmp_path / "target/meta/vlai_l1.json").read_text())
    assert provenance["migration"]["source_experiment"] == "source"
    assert provenance["migration"]["source_collection_config_sha256"] == "old-config"
    assert provenance["migration"]["trimmed_frames"] == 2
    assert (tmp_path / "source/meta/vlai_l1.json").exists()


def test_failed_migration_never_publishes_or_replaces_a_target(tmp_path: Path) -> None:
    config, reader, inspector = _migration_fixture(tmp_path)

    with pytest.raises(RuntimeError, match="encoding failed"):
        trim_leading_stillness_dataset(
            config,
            source_experiment="source",
            target_experiment="target",
            reader_loader=lambda _identity: reader,
            backend_factory=FakeFactory(fail=True),
            inspector=inspector,
        )

    assert not (tmp_path / "target").exists()
    assert not list(tmp_path.glob(".target.staging-*"))


def test_migration_refuses_same_or_existing_target(tmp_path: Path) -> None:
    config, reader, inspector = _migration_fixture(tmp_path)
    with pytest.raises(ValueError, match="must be distinct"):
        trim_leading_stillness_dataset(
            config,
            source_experiment="source",
            target_experiment="source",
            reader_loader=lambda _identity: reader,
            inspector=inspector,
        )

    (tmp_path / "target").mkdir()
    with pytest.raises(FileExistsError, match="already exists"):
        trim_leading_stillness_dataset(
            config,
            source_experiment="source",
            target_experiment="target",
            reader_loader=lambda _identity: reader,
            inspector=inspector,
        )
