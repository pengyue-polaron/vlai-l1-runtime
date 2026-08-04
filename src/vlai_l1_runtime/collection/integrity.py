"""VLAI constraints around the shared LeRobot v3 payload validator."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from embodied_ops.datasets.lerobot import (
    LEROBOT_GENERATED_FRAME_COLUMNS,
    validate_lerobot_v3_dataset,
)

from .schema import ACTION_KEY, STATE_KEY


def validate_v3_payloads(
    root: Path,
    *,
    info: dict[str, Any],
    total_episodes: int,
    total_frames: int,
    expected_task: str,
) -> None:
    """Validate shared v3 mechanics plus the canonical VLAI feature contract."""

    validate_lerobot_v3_dataset(
        root,
        info=info,
        expected_episodes=total_episodes,
        expected_frames=total_frames,
        expected_tasks=(expected_task,),
        required_frame_columns=(
            *LEROBOT_GENERATED_FRAME_COLUMNS,
            STATE_KEY,
            ACTION_KEY,
        ),
        required_stat_features=(STATE_KEY, ACTION_KEY),
    )
