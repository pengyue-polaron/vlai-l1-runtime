"""VLAI L1 adapter for the reusable embodied-ops Operator Panel."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

from embodied_ops import (
    TaskCatalog,
    load_task_catalog,
    register_task_prompt,
    validate_experiment_name,
)
from embodied_ops.operator_panel import (
    InputAction,
    PanelCapabilities,
    WorkflowLaunch,
    fetch_camera_health,
)

from .collection.configuration import load_collection_config
from .collection.schema import normalize_task


class L1OperatorPanelAdapter:
    """Expose collection and dataset workflows from tracked readiness."""

    def __init__(self, repo_root: Path, collection_config: Path) -> None:
        self.repo_root = repo_root.expanduser().resolve()
        self.collection_config = load_collection_config(collection_config)
        if self.collection_config.repo_root != self.repo_root:
            raise ValueError("panel repo_root does not own the collection config")
        self.capabilities = PanelCapabilities(camera=self, registration=self)
        self.panel_bind = self.collection_config.system.operator_panel.bind
        self.panel_port = self.collection_config.system.operator_panel.port

    def catalog(self) -> dict[str, Any]:
        config = self.collection_config
        prompt_catalogs = self._prompt_catalogs()
        preview_port = config.system.camera_preview.port
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
        if not config.system.teleoperation.blockers:
            workflows.insert(1, _reset_workflow(config.teleoperation_sides))
        if config.collection_ready:
            workflows.insert(
                1,
                _collect_workflow(
                    config.teleoperation_sides,
                    config.record_camera_roles,
                    prompt_catalogs,
                ),
            )
        return {
            "product": {"brand": "VLAI L1", "title": "Operations"},
            "cameras": [
                {
                    "id": stream.role,
                    "label": _camera_label(stream.role),
                    "port": preview_port,
                    "path": f"/{stream.role}.mjpg",
                }
                for stream in config.recording_camera_streams
            ],
            "camera_controls": [
                {
                    "label": "Start cameras",
                    "workflow": "camera",
                    "values": {"action": "start"},
                },
                {
                    "label": "Stop cameras",
                    "workflow": "camera",
                    "values": {"action": "stop"},
                    "tone": "danger",
                    "confirm": "Stop the persistent read-only camera service?",
                },
            ],
            "readiness": {
                "collection": config.collection_ready,
                "blockers": list(config.collection_blockers),
            },
            "workflows": workflows,
            "registrations": [_prompt_registration(prompt_catalogs, self.repo_root)],
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
                },
                {
                    "label": "Registered prompts",
                    "items": [
                        {
                            "value": task.prompt,
                            "label": f"{task.task_id} · {task.distribution.upper()}",
                        }
                        for catalog in prompt_catalogs
                        for task in catalog.tasks
                    ],
                },
            ],
        }

    def camera_health(self) -> dict[str, Any]:
        return fetch_camera_health(self.collection_config.system.camera_preview.port)

    def build_launch(self, workflow: str, values: dict[str, Any]) -> WorkflowLaunch:
        if not isinstance(values, dict):
            raise ValueError("workflow values must be an object")
        if workflow == "camera":
            if set(values) != {"action"} or values["action"] not in {"start", "stop"}:
                raise ValueError("camera workflow requires action start or stop")
            action = str(values["action"])
            return WorkflowLaunch(
                workflow,
                f"camera:{action}",
                (str(self.repo_root / "scripts/camera_service.sh"), action),
            )
        base = (
            sys.executable,
            "-m",
            "vlai_l1_runtime.cli",
            workflow,
            "--config",
            str(self.collection_config.path),
        )
        if workflow in {"validate-collection", "reset"}:
            if values:
                raise ValueError(f"{workflow} accepts no values")
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
            if set(values) != {"experiment", "task"}:
                raise ValueError("collect requires experiment and task")
            experiment = validate_experiment_name(_text(values["experiment"], "experiment"))
            task = normalize_task(values["task"])
            return WorkflowLaunch(
                workflow,
                f"collect:{experiment}",
                (
                    str(self.repo_root / "scripts/collect.sh"),
                    "--config",
                    str(self.collection_config.path),
                    "--experiment",
                    experiment,
                    "--task",
                    task,
                ),
                input_actions=(
                    InputAction("start", "Start recording", "\n", "primary"),
                    InputAction("save", "Save episode", "\n", "primary"),
                    InputAction("reset", "Reset position", "r\n", "quiet"),
                    InputAction("discard", "Discard", "d\n", "danger"),
                    InputAction("quit", "Quit", "q\n", "quiet"),
                ),
            )
        raise ValueError(f"unknown operator workflow: {workflow!r}")

    def register(self, registration: str, values: dict[str, Any]) -> dict[str, Any]:
        if registration != "prompt":
            raise ValueError(f"unknown registration: {registration!r}")
        required = {"catalog", "task_id", "prompt", "distribution"}
        if set(values) != required:
            raise ValueError(
                "prompt registration requires catalog, task_id, prompt, and distribution"
            )
        prompt = normalize_task(values["prompt"])
        target = register_task_prompt(
            self._prompt_catalog_path(_text(values["catalog"], "catalog")),
            task_id=_text(values["task_id"], "task_id"),
            prompt=prompt,
            distribution=_text(values["distribution"], "distribution"),
            repo_root=self.repo_root,
        )
        return {
            "created": self._reference(target),
            "activate": {"panel": "collect", "values": {"task": prompt}},
        }

    def _reference(self, path: Path) -> str:
        try:
            return path.relative_to(self.repo_root).as_posix()
        except ValueError:
            return str(path)

    def _prompt_catalogs(self) -> tuple[TaskCatalog, ...]:
        return tuple(
            load_task_catalog(path, repo_root=self.repo_root)
            for path in sorted((self.repo_root / "configs/tasks").glob("*/catalog.json"))
        )

    def _prompt_catalog_path(self, value: str) -> Path:
        candidate = (self.repo_root / value).resolve()
        allowed = (self.repo_root / "configs/tasks").resolve()
        if (
            not candidate.is_relative_to(allowed)
            or candidate.name != "catalog.json"
            or not candidate.is_file()
        ):
            raise ValueError("prompt catalog must be a catalog.json under configs/tasks")
        load_task_catalog(candidate, repo_root=self.repo_root)
        return candidate


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


def _collect_workflow(
    teleoperation_sides: tuple[str, ...],
    camera_roles: tuple[str, ...],
    prompt_catalogs: tuple[TaskCatalog, ...],
) -> dict[str, Any]:
    side_label = ", ".join(teleoperation_sides)
    camera_label = ", ".join(camera_roles)
    return {
        "id": "collect",
        "label": "Collect",
        "eyebrow": "LIVE",
        "title": "Collect episodes",
        "description": (
            f"Record {side_label} state with {camera_label}; return to Reset after each episode."
        ),
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
                "type": "combobox",
                "required": True,
                "placeholder": "place the fruit in the bowl",
                "help_text": "Select a registered prompt or enter a new normalized task.",
                "options": [
                    {
                        "value": task.prompt,
                        "label": f"{task.task_id} · {task.distribution.upper()}",
                    }
                    for catalog in prompt_catalogs
                    for task in catalog.tasks
                ],
            },
        ],
    }


def _reset_workflow(teleoperation_sides: tuple[str, ...]) -> dict[str, Any]:
    side_label = ", ".join(teleoperation_sides)
    return {
        "id": "reset",
        "label": "Reset",
        "eyebrow": "ROBOT",
        "title": "Reset teleoperation alignment",
        "description": f"Run x_air AdjustPosition on selected sides: {side_label}.",
        "submit_label": "Reset robot",
        "confirm": (
            f"This moves the {side_label} leader/follower arm pair(s). "
            "Confirm the workspace is clear?"
        ),
        "fields": [],
    }


def _prompt_registration(
    catalogs: tuple[TaskCatalog, ...],
    repo_root: Path,
) -> dict[str, Any]:
    return {
        "id": "prompt",
        "label": "Prompts",
        "eyebrow": "TASK REGISTRY",
        "title": "Register a collection prompt",
        "description": "Create one validated prompt record without replacing existing data.",
        "submit_label": "Register prompt",
        "confirm": "Register this prompt in the repository?",
        "fields": [
            {
                "name": "catalog",
                "label": "Catalog",
                "type": "select",
                "required": True,
                "options": [
                    {
                        "value": catalog.path.relative_to(repo_root).as_posix(),
                        "label": catalog.catalog_id,
                    }
                    for catalog in catalogs
                ],
            },
            {
                "name": "prompt",
                "label": "Prompt",
                "type": "text",
                "required": True,
                "placeholder": "place the fruit in the bowl",
            },
            {
                "name": "task_id",
                "label": "Task ID",
                "type": "text",
                "required": True,
                "placeholder": "place_fruit_in_bowl",
                "derive_from": "prompt",
                "transform": "snake_case",
            },
            {
                "name": "distribution",
                "label": "Distribution",
                "type": "select",
                "required": True,
                "default": "train",
                "options": [
                    {"value": "train", "label": "Train"},
                    {"value": "ood", "label": "OOD"},
                ],
            },
        ],
    }


def _camera_label(role: str) -> str:
    return {
        "wrist_left": "Left wrist",
        "wrist_right": "Right wrist",
        "agent": "Agent view",
    }[role]


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{label} must be normalized non-empty text")
    return value
