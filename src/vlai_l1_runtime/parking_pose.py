"""Capture and validate the right Follower parking pose without enabling motors."""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime, timezone

from .configuration import MOTOR_NAMES
from .contracts import NamedJointVector, SampleMetadata
from .start_pose import CaptureSpec, PoseCaptureError, StationaryPoseWindow
from .task_definition import (
    COORDINATE_KIND,
    FEATURES,
    config_sha256,
    validate_joint_pose,
    validate_stationarity,
)


def parking_path(config):
    return config.path.parents[2] / "configs/poses/right_follower_park.json"


def load_parking_pose(config):
    record = json.loads(parking_path(config).read_text())
    if (
        type(record) is not dict
        or record.get("kind") != "vlai_parking_pose_v1"
        or record.get("side") != "right"
        or record.get("role") != "follower"
        or record.get("position_unit") != "degree"
        or record.get("coordinate_kind") != COORDINATE_KIND
        or record.get("feature_names") != list(FEATURES)
        or record.get("validation_config_sha256") != config_sha256(config)
    ):
        raise ValueError("parking pose identity, units or configuration mismatch")
    validate_joint_pose(config, record.get("follower_deg"))
    validate_stationarity(record)
    return record


def capture_parking_pose(config):
    """Query only the idle right Follower; never enable, disable or move it."""
    from .teleoperation import lifecycle
    from .teleoperation.can_probe import probe_motor_feedback
    from .teleoperation.feedback_capture import _down, _link, _require_healthy
    from .teleoperation.xair import verify_xair_dependency

    if os.geteuid() != 0:
        raise RuntimeError("idle parking capture requires root")
    dependency = verify_xair_dependency(config)
    endpoint = next(e for e in config.can.endpoints if (e.side, e.role) == ("right", "follower"))
    window = StationaryPoseWindow(config, CaptureSpec())
    record = None

    def observe(sequence, feedback):
        nonlocal record
        if feedback.names != MOTOR_NAMES or not feedback.raw_frames:
            raise PoseCaptureError("complete ordered right Follower feedback required")
        if tuple(f.response_id for f in feedback.raw_frames) != tuple(
            m.receive_id for m in config.motors
        ):
            raise PoseCaptureError("parking feedback motor identity mismatch")
        stamp = min(f.host_receive_monotonic_ns for f in feedback.raw_frames)
        values = dict(zip(FEATURES, map(math.degrees, feedback.position_rad), strict=True))
        record = window.accept(
            NamedJointVector(values, SampleMetadata(sequence, stamp), FEATURES),
            now_ns=time.monotonic_ns(),
        )

    with lifecycle._acquire_lifecycle_lock(config, exclusive=True):
        lifecycle._require_exclusive_control((endpoint.interface,))
        lifecycle._require_interfaces_down((endpoint.interface,))
        lifecycle._require_can_identity(config, endpoint.interface)
        baseline = _link(endpoint.interface)
        health = lifecycle._decode_can_health(endpoint.interface, json.dumps([baseline]))
        if health.state != "STOPPED" or health.txerr or health.rxerr:
            raise RuntimeError(health.detail())
        try:
            lifecycle._configure_can(config, endpoint.interface)
            probe_motor_feedback(
                config,
                endpoint.interface,
                dependency.can_library,
                on_feedback=observe,
                check_health=lambda: _require_healthy(endpoint.interface, baseline),
            )
            if record is None:
                raise PoseCaptureError(
                    "current Follower pose did not settle; parking pose unchanged"
                )
            validate_stationarity(record)
            if time.monotonic_ns() - record["source_monotonic_ns"] > round(
                config.teleoperation.state_timeout_s * 1e9
            ):
                raise PoseCaptureError("parking feedback expired; parking pose unchanged")
        finally:
            _down(endpoint.interface)
    record.update(
        kind="vlai_parking_pose_v1",
        side="right",
        role="follower",
        position_unit="degree",
        coordinate_kind=COORDINATE_KIND,
        feature_names=list(FEATURES),
        validation_config_sha256=config_sha256(config),
        captured_utc=datetime.now(timezone.utc).isoformat(),
        source="idle_right_follower_readonly_feedback",
        interface=endpoint.interface,
        parentdev=endpoint.parentdev,
    )
    path = parking_path(config)
    temporary = path.with_suffix(".tmp")
    try:
        temporary.write_text(json.dumps(record, indent=2) + "\n")
        os.chmod(temporary, 0o644)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return path
