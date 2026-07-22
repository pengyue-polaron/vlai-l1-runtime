from __future__ import annotations

import sys
from pathlib import Path

import pytest

from vlai_l1_runtime.panel import L1OperatorPanelAdapter

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/collection/default.toml"


def test_panel_exposes_only_hardware_free_workflows() -> None:
    adapter = L1OperatorPanelAdapter(ROOT, CONFIG)
    catalog = adapter.catalog()

    assert catalog["product"] == {"brand": "VLAI L1", "title": "Operations"}
    assert catalog["readiness"]["collection"] is False
    assert {workflow["id"] for workflow in catalog["workflows"]} == {
        "validate-collection",
        "dataset-doctor",
        "export-v21",
    }
    launch = adapter.build_launch("dataset-doctor", {"experiment": "pick_v1"})
    assert launch.command[:3] == (sys.executable, "-m", "vlai_l1_runtime.cli")
    assert launch.command[-2:] == ("--experiment", "pick_v1")


def test_panel_refuses_live_collection_with_tracked_blockers() -> None:
    adapter = L1OperatorPanelAdapter(ROOT, CONFIG)
    with pytest.raises(RuntimeError, match="teleoperation_uncommissioned"):
        adapter.build_launch("collect", {})
