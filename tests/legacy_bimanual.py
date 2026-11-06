"""Synthetic historical bimanual contract for tests of generic collection code.

The deployed camera profile is right-only; these camera IDs are never opened.
"""

from dataclasses import replace
from pathlib import Path

from vlai_l1_runtime.collection.configuration import CollectionConfig, load_collection_config

ROOT = Path(__file__).resolve().parents[1]


def config() -> CollectionConfig:
    right = load_collection_config(ROOT / "configs/collection/right_only.toml")
    left, wrist_right, agent = right.system.cameras.streams
    cameras = replace(
        right.system.cameras,
        streams=(
            replace(left, required_for_collection=True, enabled=True, device_id="test-left"),
            wrist_right,
            replace(agent, device_id="test-agent"),
        ),
    )
    return replace(
        right,
        path=ROOT / "configs/collection/default.toml",
        dataset_root=ROOT / "data/datasets",
        derivative_root=ROOT / "data/derivatives",
        repo_id_prefix="pengyue-polaron/vlai-l1",
        image_writer_threads=12,
        teleoperation_sides=("left", "right"),
        record_camera_roles=("wrist_left", "wrist_right", "agent"),
        leading_stillness=replace(
            right.leading_stillness,
            action_thresholds=right.leading_stillness.action_thresholds * 2,
        ),
        system=replace(right.system, cameras=cameras),
    )
