"""Strict create-only JSON prompt catalogs for collection tasks."""

from __future__ import annotations

import fcntl
import json
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

CATALOG_SCHEMA_VERSION = 1
PROMPT_SCHEMA_VERSION = 1
_IDENTIFIER = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass(frozen=True)
class TaskPrompt:
    task_id: str
    prompt: str
    distribution: str


@dataclass(frozen=True)
class TaskCatalog:
    path: Path
    catalog_id: str
    tasks: tuple[TaskPrompt, ...]


def load_task_catalog(path: Path, *, repo_root: Path) -> TaskCatalog:
    catalog_path = _repository_file(path, repo_root=repo_root, label="task catalog")
    payload = _load_json(catalog_path, label="task catalog")
    _exact_keys(payload, {"schema_version", "id"}, label="task catalog")
    if payload["schema_version"] != CATALOG_SCHEMA_VERSION:
        raise ValueError(f"task catalog schema_version must be {CATALOG_SCHEMA_VERSION}")
    catalog_id = _identifier(payload["id"], label="task catalog id")
    prompt_directory = catalog_path.parent / "prompts"
    if not prompt_directory.is_dir() or prompt_directory.is_symlink():
        raise ValueError("task catalog requires a real prompts directory")
    entries = sorted(path for path in prompt_directory.iterdir() if not path.name.startswith("."))
    invalid = [
        path.name
        for path in entries
        if path.is_symlink() or not path.is_file() or path.suffix != ".json"
    ]
    if invalid:
        raise ValueError(f"task catalog contains unsupported entries: {invalid}")
    ordered = [_load_prompt(path) for path in entries]
    orders = [order for order, _task in ordered]
    tasks = [task for _order, task in ordered]
    if len(orders) != len(set(orders)):
        raise ValueError("task prompt orders must be unique")
    if len({task.task_id for task in tasks}) != len(tasks):
        raise ValueError("task prompt ids must be unique")
    if len({task.prompt for task in tasks}) != len(tasks):
        raise ValueError("task prompt text must be unique")
    ordered.sort(key=lambda item: (item[0], item[1].task_id))
    return TaskCatalog(catalog_path, catalog_id, tuple(task for _order, task in ordered))


def register_task_prompt(
    catalog_path: Path,
    *,
    task_id: str,
    prompt: str,
    distribution: str,
    repo_root: Path,
) -> Path:
    """Atomically create one prompt while preserving every existing record."""

    catalog_path = _repository_file(catalog_path, repo_root=repo_root, label="task catalog")
    candidate = _parse_prompt(
        {
            "schema_version": PROMPT_SCHEMA_VERSION,
            "order": 0,
            "id": task_id,
            "prompt": prompt,
            "distribution": distribution,
        },
        label="new prompt",
    )[1]
    with catalog_path.open("rb") as handle:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        catalog = load_task_catalog(catalog_path, repo_root=repo_root)
        if any(task.task_id == candidate.task_id for task in catalog.tasks):
            raise FileExistsError(f"task id is already registered: {candidate.task_id!r}")
        if any(task.prompt == candidate.prompt for task in catalog.tasks):
            raise ValueError(f"prompt is already registered: {candidate.prompt!r}")
        prompt_directory = catalog_path.parent / "prompts"
        order = (
            max(
                (_load_prompt(path)[0] for path in prompt_directory.glob("*.json")),
                default=0,
            )
            + 10
        )
        payload = {
            "schema_version": PROMPT_SCHEMA_VERSION,
            "order": order,
            "id": candidate.task_id,
            "prompt": candidate.prompt,
            "distribution": candidate.distribution,
        }
        target = prompt_directory / f"{candidate.task_id}.json"
        if target.exists() or target.is_symlink():
            raise FileExistsError(f"prompt file already exists: {target.name}")
        descriptor, staging_name = tempfile.mkstemp(
            prefix=f".{candidate.task_id}.candidate-",
            suffix=".tmp",
            dir=prompt_directory,
        )
        staging = Path(staging_name)
        try:
            os.fchmod(descriptor, 0o644)
            with os.fdopen(descriptor, "w", encoding="utf-8") as staged:
                json.dump(payload, staged, ensure_ascii=False, indent=2)
                staged.write("\n")
                staged.flush()
                os.fsync(staged.fileno())
            _load_prompt(staging)
            os.link(staging, target)
        finally:
            staging.unlink(missing_ok=True)
        registered = load_task_catalog(catalog_path, repo_root=repo_root)
        if candidate not in registered.tasks:
            raise RuntimeError("registered prompt does not match the validated candidate")
        return target


def _load_prompt(path: Path) -> tuple[int, TaskPrompt]:
    prompt = _load_json(path, label=f"prompt {path.name}")
    order, task = _parse_prompt(prompt, label=f"prompt {path.name}")
    if path.suffix == ".json" and path.name != f"{task.task_id}.json":
        raise ValueError("prompt filename must match its task id")
    return order, task


def _parse_prompt(payload: dict[str, Any], *, label: str) -> tuple[int, TaskPrompt]:
    _exact_keys(
        payload,
        {"schema_version", "order", "id", "prompt", "distribution"},
        label=label,
    )
    if payload["schema_version"] != PROMPT_SCHEMA_VERSION:
        raise ValueError(f"{label} schema_version must be {PROMPT_SCHEMA_VERSION}")
    order = payload["order"]
    if isinstance(order, bool) or not isinstance(order, int) or order < 0:
        raise ValueError(f"{label} order must be a non-negative integer")
    task_id = _identifier(payload["id"], label=f"{label} id")
    prompt = payload["prompt"]
    if (
        not isinstance(prompt, str)
        or not prompt
        or prompt != prompt.strip()
        or "\n" in prompt
        or "\r" in prompt
    ):
        raise ValueError(f"{label} prompt must be normalized single-line text")
    distribution = payload["distribution"]
    if distribution not in {"train", "ood"}:
        raise ValueError(f"{label} distribution must be train or ood")
    return order, TaskPrompt(task_id, prompt, distribution)


def _repository_file(path: Path, *, repo_root: Path, label: str) -> Path:
    root = repo_root.resolve()
    candidate = Path(os.path.abspath(os.fspath(path if path.is_absolute() else root / path)))
    allowed = (root / "configs/tasks").resolve()
    if not candidate.is_relative_to(allowed):
        raise ValueError(f"{label} must be a repository file under configs/tasks")
    current = root
    for component in candidate.relative_to(root).parts:
        current /= component
        if current.is_symlink():
            raise ValueError(f"{label} path must not contain symbolic links")
    if not candidate.is_file():
        raise ValueError(f"{label} must be a repository file under configs/tasks")
    return candidate


def _identifier(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or _IDENTIFIER.fullmatch(value) is None:
        raise ValueError(f"{label} must be a lower snake_case identifier")
    return value


def _load_json(path: Path, *, label: str) -> dict[str, Any]:
    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"{label} contains duplicate key: {key!r}")
            result[key] = value
        return result

    try:
        payload = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=reject_duplicates)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid {label} JSON: {exc.msg}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be a JSON object")
    return payload


def _exact_keys(payload: dict[str, Any], expected: set[str], *, label: str) -> None:
    missing = expected - set(payload)
    unknown = set(payload) - expected
    if missing or unknown:
        raise ValueError(
            f"{label} keys differ: missing={sorted(missing)} unknown={sorted(unknown)}"
        )
