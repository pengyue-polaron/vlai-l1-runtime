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
    PANEL_CATALOG_SCHEMA_VERSION,
    PanelCapabilities,
    WorkflowLaunch,
    combobox_field,
    fetch_camera_health,
    option,
    order_workflow_forms,
    select_field,
    standard_camera_controls,
    standard_core_workflows,
    standard_panel_product,
    text_field,
)

from .collection.configuration import load_collection_config
from .collection.interaction import L1_COLLECTION_INTERACTION
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
        config_reference = self._reference(config.path)
        collection_options, experiment_options = self._collection_options()
        collection_field = select_field(
            "config",
            "Collection config",
            collection_options,
            default=config_reference,
        )
        dataset_fields = [
            collection_field,
            combobox_field(
                "experiment",
                "Experiment",
                experiment_options,
                placeholder="fruit_placement_v1",
                help_text="Select an existing canonical dataset or enter its exact name.",
                depends_on="config",
            ),
        ]
        workflows = standard_core_workflows(
            hardware_fields=[collection_field],
            collect_fields=[
                collection_field,
                combobox_field(
                    "experiment",
                    "Experiment",
                    experiment_options,
                    placeholder="fruit_placement_v1",
                    help_text=(
                        "Select an existing experiment to append episodes, or type "
                        "a new name to create one."
                    ),
                    depends_on="config",
                ),
                combobox_field(
                    "task",
                    "Task prompt",
                    [
                        option(task.prompt, f"{task.task_id} · {task.distribution.upper()}")
                        for catalog in prompt_catalogs
                        for task in catalog.tasks
                    ],
                    placeholder="place the fruit in the bowl",
                    help_text=(
                        "Select a tracked training prompt or type an exact new prompt. "
                        "One experiment may contain episodes from multiple prompts."
                    ),
                ),
            ],
            reset_fields=[collection_field],
            dataset_fields=dataset_fields,
            reset_confirm=(
                "This moves the selected leader/follower arm pair(s). "
                "Confirm the workspace is clear?"
            ),
        )
        workflows = order_workflow_forms(
            [*workflows, _validate_workflow(collection_options, config_reference)]
        )
        return {
            "schema_version": PANEL_CATALOG_SCHEMA_VERSION,
            "product": standard_panel_product("VLAI L1"),
            "cameras": [
                {
                    "id": stream.role,
                    "label": _camera_label(stream.role),
                    "port": preview_port,
                    "path": f"/{stream.role}.mjpg",
                }
                for stream in config.recording_camera_streams
            ],
            "camera_controls": standard_camera_controls(
                stop_confirm="Stop the persistent read-only camera service?"
            ),
            "workflows": workflows,
            "registrations": [_prompt_registration(prompt_catalogs, self.repo_root)],
            "configuration_types": [],
            "configuration_groups": [
                {
                    "label": "Tracked configuration",
                    "items": [
                        option(config_reference, "Collection"),
                        option(self._reference(config.system.path), "System"),
                    ],
                },
                {
                    "label": "Registered prompts",
                    "items": [
                        option(
                            task.prompt,
                            f"{task.task_id} · {task.distribution.upper()}",
                        )
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
        if workflow in {
            "hardware",
            "validate-collection",
            "reset",
            "dataset-doctor",
            "export-v21",
            "collect",
        }:
            expected_keys = {"config"}
            if workflow in {"dataset-doctor", "export-v21"}:
                expected_keys.add("experiment")
            elif workflow == "collect":
                expected_keys.update(("experiment", "task"))
            if set(values) != expected_keys:
                joined = ", ".join(sorted(expected_keys))
                raise ValueError(f"{workflow} requires exactly {joined}")
            selected_config = self._selected_collection_config(values["config"])
        if workflow in {"hardware", "validate-collection", "reset"}:
            if set(values) != {"config"}:
                raise ValueError(f"{workflow} requires exactly config")
            return WorkflowLaunch(
                workflow,
                workflow,
                (
                    sys.executable,
                    "-m",
                    "vlai_l1_runtime.cli",
                    workflow,
                    "--config",
                    str(selected_config.path),
                ),
            )
        if workflow in {"dataset-doctor", "export-v21"}:
            experiment = validate_experiment_name(_text(values["experiment"], "experiment"))
            return WorkflowLaunch(
                workflow,
                f"{workflow}:{experiment}",
                (
                    sys.executable,
                    "-m",
                    "vlai_l1_runtime.cli",
                    "dataset",
                    "doctor" if workflow == "dataset-doctor" else "export-v21",
                    "--config",
                    str(selected_config.path),
                    experiment,
                ),
            )
        if workflow == "collect":
            if not selected_config.collection_ready:
                blockers = ", ".join(selected_config.collection_blockers)
                raise RuntimeError(f"live collection is unavailable: {blockers}")
            experiment = validate_experiment_name(_text(values["experiment"], "experiment"))
            task = normalize_task(values["task"])
            return WorkflowLaunch(
                workflow,
                f"collect:{experiment}",
                (
                    str(self.repo_root / "scripts/collect.sh"),
                    "--config",
                    str(selected_config.path),
                    "--task",
                    task,
                    experiment,
                ),
                input_actions=L1_COLLECTION_INTERACTION.input_actions,
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

    def _collection_options(self) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
        collection_options = []
        experiment_options = []
        for path in sorted((self.repo_root / "configs/collection").glob("*.toml")):
            config = load_collection_config(path)
            if config.repo_root != self.repo_root:
                raise ValueError("panel repo_root does not own the collection config")
            reference = self._reference(config.path)
            collection_options.append(option(reference, path.stem))
            if not config.dataset_root.is_dir():
                continue
            for dataset in sorted(config.dataset_root.iterdir(), key=lambda item: item.name):
                if not dataset.is_dir() or dataset.name.startswith("."):
                    continue
                try:
                    experiment = validate_experiment_name(dataset.name)
                except ValueError:
                    continue
                experiment_options.append(option(experiment, experiment, depends_value=reference))
        return collection_options, experiment_options

    def _selected_collection_config(self, value: Any):
        reference = _text(value, "config")
        candidate = (self.repo_root / reference).resolve()
        allowed = (self.repo_root / "configs/collection").resolve()
        if not candidate.is_relative_to(allowed) or candidate.suffix != ".toml":
            raise ValueError("config must be a repository TOML under configs/collection")
        if not candidate.is_file():
            raise FileNotFoundError(f"repository config is missing: {candidate}")
        config = load_collection_config(candidate)
        if config.repo_root != self.repo_root:
            raise ValueError("panel repo_root does not own the collection config")
        return config


def _validate_workflow(
    config_options: list[dict[str, str]], config_reference: str
) -> dict[str, Any]:
    return {
        "id": "validate-collection",
        "label": "Config check",
        "eyebrow": "CONFIGURATION",
        "title": "Validate collection contract",
        "description": "Validate tracked collection and System configuration.",
        "submit_label": "Validate",
        "fields": [
            select_field(
                "config",
                "Collection config",
                config_options,
                default=config_reference,
            )
        ],
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
            select_field(
                "catalog",
                "Catalog",
                [
                    option(
                        catalog.path.relative_to(repo_root).as_posix(),
                        catalog.catalog_id,
                    )
                    for catalog in catalogs
                ],
            ),
            text_field("prompt", "Prompt", placeholder="place the fruit in the bowl"),
            {
                **text_field("task_id", "Task ID", placeholder="place_fruit_in_bowl"),
                "derive_from": "prompt",
                "transform": "snake_case",
            },
            select_field(
                "distribution",
                "Distribution",
                [option("train", "Train"), option("ood", "OOD")],
                default="train",
            ),
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
