"""VLAI L1 adapter for the reusable embodied-ops Operator Panel."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

from embodied_ops import validate_experiment_name
from embodied_ops.operator_panel import InputAction, PanelCapabilities, WorkflowLaunch

from .collection.configuration import load_collection_config
from .collection.schema import normalize_task


class L1OperatorPanelAdapter:
    """Expose collection and dataset workflows from tracked readiness."""

    def __init__(self, repo_root: Path, collection_config: Path) -> None:
        self.repo_root = repo_root.expanduser().resolve()
        self.collection_config = load_collection_config(collection_config)
        if self.collection_config.repo_root != self.repo_root:
            raise ValueError("panel repo_root does not own the collection config")
        self.capabilities = PanelCapabilities()
        self.panel_bind = self.collection_config.system.operator_panel.bind
        self.panel_port = self.collection_config.system.operator_panel.port

    def catalog(self) -> dict[str, Any]:
        config = self.collection_config
        workflows = [
            {
                "id": "validate-collection",
                "label": "Validate",
                "eyebrow": "CONFIGURATION",
                "title": "Validate collection contract",
                "description": "Validate tracked collection and System configuration.",
                "submit_label": "Validate",
                "fields": [],
            },
            _experiment_workflow(
                workflow="dataset-doctor",
                label="Doctor",
                eyebrow="DATASET",
                title="Inspect canonical dataset",
                description="Validate metadata, episode ranges, Parquet, and videos.",
                submit_label="Run doctor",
            ),
            _experiment_workflow(
                workflow="export-v21",
                label="Export v2.1",
                eyebrow="DERIVATIVE",
                title="Export LeRobot v2.1",
                description="Create a new v2.1 derivative from canonical v3 data.",
                submit_label="Export",
            ),
        ]
        if config.collection_ready:
            workflows.insert(1, _collect_workflow())
        return {
            "product": {"brand": "VLAI L1", "title": "Operations"},
            "cameras": [],
            "camera_controls": [],
            "readiness": {
                "collection": config.collection_ready,
                "blockers": list(config.collection_blockers),
            },
            "workflows": workflows,
            "registrations": [],
            "configuration_types": [],
            "configuration_groups": [
                {
                    "label": "Tracked configuration",
                    "items": [
                        {
                            "value": self._reference(config.path),
                            "label": "Collection",
                        },
                        {
                            "value": self._reference(config.system.path),
                            "label": "System",
                        },
                    ],
                }
            ],
        }

    def build_launch(self, workflow: str, values: dict[str, Any]) -> WorkflowLaunch:
        if not isinstance(values, dict):
            raise ValueError("workflow values must be an object")
        base = (
            sys.executable,
            "-m",
            "vlai_l1_runtime.cli",
            workflow,
            "--config",
            str(self.collection_config.path),
        )
        if workflow == "validate-collection":
            if values:
                raise ValueError("validate-collection accepts no values")
            return WorkflowLaunch(workflow, workflow, base)
        if workflow in {"dataset-doctor", "export-v21"}:
            if set(values) != {"experiment"}:
                raise ValueError(f"{workflow} requires exactly experiment")
            experiment = validate_experiment_name(_text(values["experiment"], "experiment"))
            return WorkflowLaunch(
                workflow,
                f"{workflow}:{experiment}",
                (*base, "--experiment", experiment),
            )
        if workflow == "collect":
            if not self.collection_config.collection_ready:
                blockers = ", ".join(self.collection_config.collection_blockers)
                raise RuntimeError(f"live collection is unavailable: {blockers}")
            if set(values) != {"experiment", "task", "frames", "decision"}:
                raise ValueError("collect requires experiment, task, frames, and decision")
            experiment = validate_experiment_name(_text(values["experiment"], "experiment"))
            task = normalize_task(values["task"])
            frames = _positive_integer_text(values["frames"], "frames")
            decision = _text(values["decision"], "decision")
            if decision not in {"save", "discard"}:
                raise ValueError("decision must be save or discard")
            return WorkflowLaunch(
                workflow,
                f"collect:{experiment}",
                (
                    *base,
                    "--experiment",
                    experiment,
                    "--task",
                    task,
                    "--frames",
                    str(frames),
                    "--decision",
                    decision,
                ),
                input_actions=(InputAction("enter", "Start recording", "\n", "primary"),),
            )
        raise ValueError(f"unknown operator workflow: {workflow!r}")

    def _reference(self, path: Path) -> str:
        try:
            return path.relative_to(self.repo_root).as_posix()
        except ValueError:
            return str(path)


def _experiment_workflow(
    *,
    workflow: str,
    label: str,
    eyebrow: str,
    title: str,
    description: str,
    submit_label: str,
) -> dict[str, Any]:
    return {
        "id": workflow,
        "label": label,
        "eyebrow": eyebrow,
        "title": title,
        "description": description,
        "submit_label": submit_label,
        "fields": [
            {
                "name": "experiment",
                "label": "Experiment",
                "type": "text",
                "required": True,
                "placeholder": "fruit_placement_v1",
            }
        ],
    }


def _collect_workflow() -> dict[str, Any]:
    return {
        "id": "collect",
        "label": "Collect",
        "eyebrow": "LIVE",
        "title": "Record one episode",
        "description": "Capture paired teleoperation state and commissioned cameras.",
        "submit_label": "Start collection",
        "fields": [
            {
                "name": "experiment",
                "label": "Experiment",
                "type": "text",
                "required": True,
                "placeholder": "fruit_placement_v1",
            },
            {
                "name": "task",
                "label": "Task",
                "type": "text",
                "required": True,
                "placeholder": "place the fruit in the bowl",
            },
            {
                "name": "frames",
                "label": "Frames",
                "type": "text",
                "required": True,
                "default": "300",
            },
            {
                "name": "decision",
                "label": "After capture",
                "type": "select",
                "required": True,
                "default": "save",
                "options": [
                    {"value": "save", "label": "Save episode"},
                    {"value": "discard", "label": "Discard episode"},
                ],
            },
        ],
    }


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{label} must be normalized non-empty text")
    return value


def _positive_integer_text(value: Any, label: str) -> int:
    text = _text(value, label)
    if not text.isascii() or not text.isdigit() or int(text) <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return int(text)
