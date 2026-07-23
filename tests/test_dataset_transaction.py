from __future__ import annotations

import json
import sys
import types
from pathlib import Path
from typing import Any

import pytest

from vlai_l1_runtime.collection.configuration import load_collection_config
from vlai_l1_runtime.collection.dataset import (
    GENERATED_FEATURES,
    DirectDatasetState,
    DirectLeRobotEpisode,
    LeRobotBackendFactory,
    identity_from_config,
    provenance_from_config,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_collection_config(ROOT / "configs/collection/default.toml")


class FakeBackend:
    def __init__(self, identity, root: Path, *, fail: bool = False) -> None:
        self.identity = identity
        self.root = root
        self.fail = fail
        self.frames: list[dict[str, Any]] = []

    def add_frame(self, frame: dict[str, Any]) -> None:
        self.frames.append(frame)

    def save_episode(self, parallel_encoding: bool = False) -> None:
        assert parallel_encoding is False
        if self.fail:
            raise RuntimeError("encoding failed")

    def clear_episode_buffer(self) -> None:
        self.frames.clear()

    def finalize(self) -> None:
        (self.root / "meta").mkdir(parents=True, exist_ok=True)
        info = {
            "codebase_version": "v3.0",
            "robot_type": "vlai_l1",
            "fps": self.identity.fps,
            "features": {**self.identity.contract.features(), **GENERATED_FEATURES},
            "total_episodes": 1,
            "total_frames": len(self.frames),
        }
        (self.root / "meta/info.json").write_text(json.dumps(info))


class FakeFactory:
    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail

    def create(self, identity, root: Path) -> FakeBackend:
        return FakeBackend(identity, root, fail=self.fail)

    def resume(self, identity, root: Path) -> FakeBackend:
        raise AssertionError("unexpected resume")


def _inspector(identity, *, expected_task=None, expected_provenance=None) -> DirectDatasetState:
    del expected_provenance
    if not identity.target_root.exists():
        return DirectDatasetState(0, 0, None)
    provenance = json.loads((identity.target_root / "meta/vlai_l1.json").read_text())
    assert provenance["task"] == expected_task
    return DirectDatasetState(1, provenance["total_frames"], expected_task)


def test_episode_publication_is_atomic_and_writes_provenance(tmp_path: Path) -> None:
    identity = identity_from_config(CONFIG, "atomic_test")
    identity = type(identity)(
        target_root=tmp_path / "dataset",
        repo_id=identity.repo_id,
        fps=identity.fps,
        contract=identity.contract,
        experiment=identity.experiment,
    )
    with DirectLeRobotEpisode(
        identity=identity,
        task="pick the block",
        provenance=provenance_from_config(CONFIG),
        backend_factory=FakeFactory(),
        inspector=_inspector,
    ) as episode:
        episode.add_frame({"frame": 1})
        assert not identity.target_root.exists()
        episode.commit()

    provenance = json.loads((identity.target_root / "meta/vlai_l1.json").read_text())
    assert provenance["total_frames"] == 1
    assert provenance["dataset_schema"] == "vlai_l1_lerobot_dataset_v3_v2"


def test_failed_episode_never_exposes_a_partial_dataset(tmp_path: Path) -> None:
    identity = identity_from_config(CONFIG, "failed_test")
    identity = type(identity)(
        target_root=tmp_path / "dataset",
        repo_id=identity.repo_id,
        fps=identity.fps,
        contract=identity.contract,
        experiment=identity.experiment,
    )
    with DirectLeRobotEpisode(
        identity=identity,
        task="pick the block",
        provenance=provenance_from_config(CONFIG),
        backend_factory=FakeFactory(fail=True),
        inspector=_inspector,
    ) as episode:
        episode.add_frame({"frame": 1})
        with pytest.raises(RuntimeError, match="encoding failed"):
            episode.commit()
    assert not identity.target_root.exists()
    assert not list(tmp_path.glob(".dataset.staging-*"))


def test_lerobot_factory_configures_async_image_writers(monkeypatch, tmp_path: Path) -> None:
    calls: list[tuple[str, dict[str, Any]]] = []

    class LeRobotDataset:
        @staticmethod
        def create(**kwargs):
            calls.append(("create", kwargs))
            return object()

        @staticmethod
        def resume(**kwargs):
            calls.append(("resume", kwargs))
            return object()

    module = types.ModuleType("lerobot.datasets.lerobot_dataset")
    module.LeRobotDataset = LeRobotDataset
    monkeypatch.setitem(sys.modules, "lerobot.datasets.lerobot_dataset", module)
    identity = identity_from_config(CONFIG, "writer_test")
    factory = LeRobotBackendFactory(image_writer_threads=12)

    factory.create(identity, tmp_path / "new")
    factory.resume(identity, tmp_path / "existing")

    assert [kind for kind, _ in calls] == ["create", "resume"]
    assert [kwargs["image_writer_threads"] for _, kwargs in calls] == [12, 12]
