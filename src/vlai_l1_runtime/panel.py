"""VLAI L1 adapter for the reusable embodied-ops Operator Panel."""

from __future__ import annotations

import json
import math
import sys
from http.client import HTTPConnection
from pathlib import Path
from typing import Any

from embodied_ops import validate_experiment_name
from embodied_ops.operator_panel import InputAction, PanelCapabilities, WorkflowLaunch

from .collection.configuration import load_collection_config
from .collection.schema import normalize_task
from .tasks import TaskCatalog, load_task_catalog, register_task_prompt

_CAMERA_HEALTH_TIMEOUT_S = 0.4
_CAMERA_HEALTH_MAX_BYTES = 64 * 1024


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
        if config.collection_ready:
            workflows.insert(1, _collect_workflow())
        return {
            "product": {"brand": "VLAI L1", "title": "Operations"},
            "cameras": [
                {
                    "id": stream.role,
                    "label": _camera_label(stream.role),
                    "port": preview_port,
                    "path": f"/{stream.role}.mjpg",
                }
                for stream in config.system.cameras.streams
                if stream.enabled
            ],
            "camera_controls": [],
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
        connection = HTTPConnection(
            "127.0.0.1",
            self.collection_config.system.camera_preview.port,
            timeout=_CAMERA_HEALTH_TIMEOUT_S,
        )
        try:
            connection.request("GET", "/healthz", headers={"Cache-Control": "no-store"})
            response = connection.getresponse()
            body = response.read(_CAMERA_HEALTH_MAX_BYTES + 1)
        except (OSError, TimeoutError):
            return _camera_health_unavailable("Camera preview starts with collection.")
        finally:
            connection.close()
        if len(body) > _CAMERA_HEALTH_MAX_BYTES:
            return _camera_health_unavailable("Camera preview health response is too large.")
        if response.status not in {200, 503}:
            return _camera_health_unavailable("Camera preview health request failed.")
        try:
            return _normalize_camera_health(json.loads(body))
        except (TypeError, ValueError, json.JSONDecodeError):
            return _camera_health_unavailable("Camera preview returned invalid health data.")

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


def _normalize_camera_health(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict) or not isinstance(payload.get("ok"), bool):
        raise ValueError("camera health must contain a boolean ok value")
    raw_streams = payload.get("streams")
    if not isinstance(raw_streams, dict):
        raise ValueError("camera health streams must be an object")
    streams: dict[str, dict[str, Any]] = {}
    for role, raw in raw_streams.items():
        if not isinstance(role, str) or not isinstance(raw, dict):
            raise ValueError("camera health stream is invalid")
        ready = raw.get("ready")
        fresh = raw.get("fresh")
        error = raw.get("error")
        if not isinstance(ready, bool) or not isinstance(fresh, bool):
            raise ValueError("camera health readiness values must be booleans")
        if error is not None and not isinstance(error, str):
            raise ValueError("camera health error must be text or null")
        streams[role] = {
            "ready": ready,
            "fresh": fresh,
            "preview_fps": _optional_nonnegative(raw.get("preview_fps")),
            "age_s": _optional_nonnegative(raw.get("age_s")),
            "error": error,
        }
    return {"available": True, "ok": payload["ok"], "streams": streams}


def _optional_nonnegative(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("camera health values must be numeric or null")
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError("camera health values must be finite and non-negative")
    return number


def _camera_health_unavailable(reason: str) -> dict[str, Any]:
    return {"available": False, "ok": False, "streams": {}, "reason": reason}


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{label} must be normalized non-empty text")
    return value


def _positive_integer_text(value: Any, label: str) -> int:
    text = _text(value, label)
    if not text.isascii() or not text.isdigit() or int(text) <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return int(text)
