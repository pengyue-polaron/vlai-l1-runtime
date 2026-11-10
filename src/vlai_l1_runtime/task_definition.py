"""Pure task/pose contracts; importing or validating never opens hardware."""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from .configuration import _is_loader_validated
from .contracts import feature_names_for_sides

FEATURES = feature_names_for_sides(("right",))
COORDINATE_KIND = "x_air_sdk_joint_degrees_unverified_physical_zero"
POSE_KIND = "vlai_task_start_pose_v1"


def validate_task_id(value):
    if type(value) is not str or re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,63}", value) is None:
        raise ValueError("task id must be a normalized safe identifier")
    return value


def contained_path(root, value, *, must_exist=True):
    if type(value) is not str or not value or "\\" in value:
        raise ValueError("repository-contained relative path required")
    relative = PurePosixPath(value)
    if (
        relative.is_absolute()
        or str(relative) != value
        or any(p in (".", "..") for p in relative.parts)
    ):
        raise ValueError("repository-contained relative path required")
    root = Path(root).resolve()
    path = root.joinpath(*relative.parts)
    if not path.resolve().is_relative_to(root):
        raise ValueError("path escapes repository")
    cursor = root
    for part in relative.parts:
        cursor /= part
        if cursor.is_symlink():
            raise ValueError("symlink is not an immutable task input")
    if must_exist and not path.is_file():
        raise ValueError(f"task input does not exist: {value}")
    return path


def canonical_pose_path(config, task_id):
    validate_task_id(task_id)
    return contained_path(
        config.path.parents[2], f"data/start_poses/{task_id}.json", must_exist=False
    )


def config_sha256(config):
    if not _is_loader_validated(config):
        raise ValueError("loader-validated system configuration required")
    return hashlib.sha256(config.path.read_bytes()).hexdigest()


def validate_joint_pose(config, values):
    if type(values) is not dict or tuple(values) != FEATURES:
        raise ValueError("pose requires exact named eight-joint order")
    limits = config.teleoperation.joint_safety.for_side("right")
    minimum = (*limits.min_deg, -math.degrees(config.follower_feedback.right[-1].position_max_rad))
    maximum = (*limits.max_deg, math.degrees(config.follower_feedback.right[-1].position_max_rad))
    for index, name in enumerate(FEATURES):
        value = values[name]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("pose coordinates must be finite degrees")
        if not minimum[index] <= value <= maximum[index]:
            raise ValueError(f"pose outside tracked bound: {name}")


def validate_stationarity(record):
    for name in (
        "receiver_sequence",
        "source_monotonic_ns",
        "stationary_window_ns",
        "stationary_samples",
    ):
        if type(record.get(name)) is not int or record[name] < 0:
            raise ValueError("pose lacks recorded stationary provenance")
    if (
        record["source_monotonic_ns"] <= 0
        or not 200_000_000 <= record["stationary_window_ns"] <= 10_000_000_000
        or record["stationary_samples"] < 3
    ):
        raise ValueError("pose lacks a complete recorded stationary window")
    for name, limit in (("max_span_deg", 0.05), ("max_drift_deg_s", 0.1)):
        value = record.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= limit:
            raise ValueError("pose failed recorded stationary limits")


def validate_pose_record(config, task_id, record):
    if (
        type(record) is not dict
        or record.get("kind") != POSE_KIND
        or record.get("schema_version") != 1
    ):
        raise ValueError("canonical task start pose required")
    if record.get("task_id") != task_id:
        raise ValueError("start pose task does not match manifest")
    if record.get("position_unit") != "degree" or record.get("coordinate_kind") != COORDINATE_KIND:
        raise ValueError("start pose unit/coordinate frame mismatch")
    if record.get("feature_names") != list(FEATURES):
        raise ValueError("start pose feature order mismatch")
    validate_joint_pose(config, record.get("follower_deg"))
    validate_stationarity(record)
    if record.get("validation_config_sha256") != config_sha256(config):
        raise ValueError("start pose configuration hash mismatch")
    provenance = record.get("provenance")
    if not isinstance(provenance, dict) or provenance.get("mode") not in (
        "captured",
        "historical_import",
    ):
        raise ValueError("start pose provenance required")
    for name in ("source_sha256", "writer_source_sha256"):
        if re.fullmatch(r"[0-9a-f]{64}", str(provenance.get(name, ""))) is None:
            raise ValueError("start pose source hash required")
    return record


@dataclass(frozen=True)
class TaskDefinition:
    id: str
    prompt: str
    distance_cm: float
    start_pose: Path
    expected_checkpoint: dict
    inference_spec: Path
    automatic_policy: Path
    settings: dict
    pose: dict | None
    root: Path

    @classmethod
    def load(cls, config, path, *, require_start_pose=True):
        return load_task(config, path, require_start_pose=require_start_pose)


def load_task(config, manifest_path, *, require_start_pose=True):
    config_sha256(config)
    root = config.path.parents[2].resolve()
    path = Path(manifest_path)
    if path.is_absolute():
        try:
            path = path.relative_to(root)
        except ValueError as error:
            raise ValueError("task manifest must be inside repository") from error
    path = contained_path(root, path.as_posix())
    document = json.loads(path.read_text())
    fields = {
        "kind",
        "id",
        "prompt",
        "distance_cm",
        "start_pose",
        "expected_checkpoint",
        "inference_spec",
        "automatic_policy",
        "settings",
    }
    if type(document) is not dict or set(document) != fields or document["kind"] != "vlai_task_v1":
        raise ValueError("exact task manifest fields required")
    task_id = validate_task_id(document["id"])
    prompt = document["prompt"]
    if type(prompt) is not str or not prompt or prompt != prompt.strip():
        raise ValueError("normalized task prompt required")
    distance = document["distance_cm"]
    if type(distance) not in (int, float) or not math.isfinite(distance) or distance <= 0:
        raise ValueError("positive task distance required")
    checkpoint = document["expected_checkpoint"]
    if (
        type(checkpoint) is not dict
        or set(checkpoint) != {"kind", "path", "deployment_sha256"}
        or checkpoint["kind"] != "pi0"
    ):
        raise ValueError("explicit pi0 checkpoint identity required")
    if (
        type(checkpoint["path"]) is not str
        or not checkpoint["path"].strip()
        or checkpoint["path"] != checkpoint["path"].strip()
    ):
        raise ValueError("checkpoint provenance path required")
    if (
        type(checkpoint["deployment_sha256"]) is not str
        or re.fullmatch(r"[0-9a-f]{64}", checkpoint["deployment_sha256"]) is None
    ):
        raise ValueError("checkpoint deployment sha256 required")
    settings = document["settings"]
    names = {"observation_max_age_s", "producer_timeout_s"}
    if type(settings) is not dict or set(settings) not in (
        names,
        names | {"arrival_tolerance_deg"},
    ):
        raise ValueError("exact task settings required")
    for name, value in settings.items():
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"positive finite task setting required: {name}")
    start = contained_path(root, document["start_pose"], must_exist=False)
    if start != canonical_pose_path(config, task_id):
        raise ValueError("start pose must use canonical per-task path")
    inference = contained_path(root, document["inference_spec"])
    automatic = contained_path(root, document["automatic_policy"])
    pose = None
    if start.exists():
        if not start.is_file():
            raise ValueError("start pose must be a regular file")
        pose = validate_pose_record(config, task_id, json.loads(start.read_text()))
    elif require_start_pose:
        raise ValueError(f"start pose not recorded for task {task_id}")
    return TaskDefinition(
        task_id, prompt, distance, start, checkpoint, inference, automatic, settings, pose, root
    )
