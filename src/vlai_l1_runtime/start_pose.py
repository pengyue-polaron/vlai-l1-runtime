"""Capture a stationary right-side teleoperation pose without commanding motors.

This records SDK-reported joint positions from the existing state datagram.
It never opens CAN, creates a teleoperation owner, or moves toward the pose.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from .configuration import SystemConfig, _is_loader_validated, load_system_config
from .contracts import NamedJointVector
from .task_definition import (
    COORDINATE_KIND,
    FEATURES,
    POSE_KIND,
    canonical_pose_path,
    config_sha256,
    validate_joint_pose,
    validate_pose_record,
    validate_stationarity,
)


class PoseCaptureError(ValueError):
    """A live state stream cannot establish the requested stationary pose."""


@dataclass(frozen=True)
class CaptureSpec:
    settle_ns: int = 200_000_000
    maximum_hold_ns: int = 10_000_000_000
    settle_speed_deg_s: float = 0.1
    settle_span_deg: float = 0.05

    def __post_init__(self):
        for name, expected in (
            ("settle_ns", 200_000_000),
            ("maximum_hold_ns", 10_000_000_000),
            ("settle_speed_deg_s", 0.1),
            ("settle_span_deg", 0.05),
        ):
            value = getattr(self, name)
            if type(value) not in (int, float) or value != expected:
                raise PoseCaptureError("capture limits must match the stationary capture contract")
        if type(self.settle_ns) is not int or type(self.maximum_hold_ns) is not int:
            raise PoseCaptureError("capture time limits must be integer nanoseconds")

    @classmethod
    def load(cls, path):
        record = json.loads(Path(path).read_text())
        if (
            type(record) is not dict
            or set(record) != {"kind", *cls.__dataclass_fields__}
            or record.pop("kind") != "vlai_pose_capture_v1"
        ):
            raise PoseCaptureError("exact pose capture specification required")
        return cls(**record)


class StationaryPoseWindow:
    """Require fresh stationary Follower data; optional Leader is evidence only."""

    def __init__(self, config: SystemConfig, spec: CaptureSpec):
        if not isinstance(config, SystemConfig) or not _is_loader_validated(config):
            raise PoseCaptureError("loader-validated system config required")
        if not isinstance(spec, CaptureSpec):
            raise PoseCaptureError("CaptureSpec required")
        self.config, self.spec = config, spec
        self._last = None
        self._samples = deque()
        self.last_diagnostics = None

    def accept(
        self, follower: NamedJointVector, leader: NamedJointVector | None = None, *, now_ns: int
    ):
        if type(now_ns) is not int or now_ns < 0:
            raise PoseCaptureError("current monotonic time must be nonnegative")
        samples = (("Follower", follower),) + ((("Leader", leader),) if leader is not None else ())
        for role, sample in samples:
            if not isinstance(sample, NamedJointVector) or tuple(sample.feature_names) != FEATURES:
                raise PoseCaptureError(f"right {role} requires exact named eight-joint state")
            stamp = sample.metadata.monotonic_ns
            if stamp > now_ns or now_ns - stamp > round(
                self.config.teleoperation.state_timeout_s * 1e9
            ):
                raise PoseCaptureError(f"right {role} state is stale or future")
            if not all(math.isfinite(value) for value in sample.values.values()):
                raise PoseCaptureError(f"right {role} state must be finite")
        if leader is not None and follower.metadata != leader.metadata:
            raise PoseCaptureError("Leader and Follower must share one SDK state sample")
        metadata = follower.metadata
        if self._last is not None and (
            metadata.source_sequence <= self._last.source_sequence
            or metadata.monotonic_ns <= self._last.monotonic_ns
        ):
            raise PoseCaptureError("right state sequence or timestamp did not increase")
        if self._last is not None and metadata.monotonic_ns - self._last.monotonic_ns > round(
            self.config.teleoperation.state_timeout_s * 1e9
        ):
            raise PoseCaptureError("right state gap exceeded timeout")
        self._last = metadata
        try:
            validate_joint_pose(self.config, dict(follower.values))
        except ValueError as error:
            raise PoseCaptureError(str(error)) from error
        values = {
            "follower": dict(follower.values),
            "leader": dict(leader.values) if leader is not None else None,
        }
        self._samples.append((metadata.monotonic_ns, values))
        cutoff = metadata.monotonic_ns - self.spec.settle_ns
        while len(self._samples) > 2 and self._samples[1][0] <= cutoff:
            self._samples.popleft()
        elapsed_ns = metadata.monotonic_ns - self._samples[0][0]
        if elapsed_ns < self.spec.settle_ns or len(self._samples) < 2:
            return None
        # Evaluate drift over a complete measured window. Prefix-by-prefix
        # differentiation treats one encoder word at10ms as motion and can
        # continually reset an otherwise stationary200ms capture.
        times = [(stamp - metadata.monotonic_ns) / 1e9 for stamp, _ in self._samples]
        center = sum(times) / len(times)
        denominator = sum((t - center) ** 2 for t in times)
        spans, slopes = [], []
        for role in ("follower",):
            for name in FEATURES:
                positions = [sample[role][name] for _, sample in self._samples]
                spans.append(max(positions) - min(positions))
                slopes.append(
                    abs(
                        sum(
                            (t - center) * (q - positions[0])
                            for t, q in zip(times, positions, strict=True)
                        )
                        / denominator
                    )
                )
        self.last_diagnostics = {"max_span_deg": max(spans), "max_drift_deg_s": max(slopes)}
        if max(spans) > self.spec.settle_span_deg or max(slopes) > self.spec.settle_speed_deg_s:
            return None
        return {
            "follower_deg": values["follower"],
            "leader_deg": values["leader"],
            "receiver_sequence": metadata.source_sequence,
            "source_monotonic_ns": metadata.monotonic_ns,
            "stationary_window_ns": elapsed_ns,
            "stationary_samples": len(self._samples),
            **self.last_diagnostics,
        }


def _output_path(config, task_id, output):
    canonical = canonical_pose_path(config, task_id)
    if output is not None and Path(output).absolute() != canonical:
        raise PoseCaptureError("capture output must be the canonical per-task path")
    if canonical.exists() or canonical.is_symlink():
        raise FileExistsError(f"pose output already exists: {canonical}")
    return canonical


def _write_record(config, task_id, record, output):
    validate_pose_record(config, task_id, record)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return record


def capture_pose(
    config: SystemConfig, spec: CaptureSpec, *, task_id: str, output: Path | None = None
):
    """Listen to the existing right state stream, then create one inert pose file."""
    from .teleoperation.xair import XAirStateReceiver

    output = _output_path(config, task_id, output)
    window = StationaryPoseWindow(config, spec)
    deadline = time.monotonic_ns() + spec.maximum_hold_ns
    with XAirStateReceiver(config, sides=("right",)) as receiver:
        while time.monotonic_ns() < deadline:
            sample = receiver.receive(timeout_s=config.teleoperation.state_timeout_s)
            if sample is None:
                raise PoseCaptureError("no fresh right teleoperation state received")
            result = window.accept(*sample, now_ns=time.monotonic_ns())
            if result is not None:
                break
        else:
            raise PoseCaptureError(
                f"right Follower did not become stationary in time: {window.last_diagnostics}"
            )
    record = {
        "schema_version": 1,
        "kind": POSE_KIND,
        "task_id": task_id,
        "status": "recorded_reference_only",
        "automatic_motion_enabled": False,
        "coordinate_kind": COORDINATE_KIND,
        "position_unit": "degree",
        "feature_names": list(FEATURES),
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "source": "existing_x_air_right_state_ipc",
        "source_socket": str(config.teleoperation.state_socket_path),
        "system_config": str(config.path),
        "stationary_spec_kind": "vlai_pose_capture_v1",
        "validation_config_sha256": config_sha256(config),
        "provenance": {
            "mode": "captured",
            "source_sha256": hashlib.sha256(
                json.dumps(result, sort_keys=True, allow_nan=False).encode()
            ).hexdigest(),
            "writer_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "original_config_sha256": config_sha256(config),
        },
        **result,
    }
    return _write_record(config, task_id, record, output)


def import_task_pose(config, task_id, source: Path, *, output: Path | None = None):
    """Assign a historical stationary capture without claiming a new observation."""
    output = _output_path(config, task_id, output)
    source = Path(source)
    if source.is_symlink() or not source.is_file():
        raise PoseCaptureError("historical pose source must be a regular file")
    payload = source.read_bytes()
    original = json.loads(payload)
    if (
        type(original) is not dict
        or original.get("schema_version") != 1
        or original.get("kind") != "right_teleop_start_pose_candidate"
        or original.get("status") != "recorded_reference_only"
        or original.get("automatic_motion_enabled") is not False
        or original.get("source") != "existing_x_air_right_state_ipc"
        or original.get("stationary_spec_kind") != "offline_only_control_handoff_preview_v1"
    ):
        raise PoseCaptureError(
            "original stationary capture schema required; single snapshots cannot be imported"
        )
    if (
        original.get("position_unit") != "degree"
        or original.get("coordinate_kind") != COORDINATE_KIND
        or original.get("feature_names") != list(FEATURES)
    ):
        raise PoseCaptureError("historical pose unit/order/coordinate frame mismatch")
    validate_joint_pose(config, original.get("follower_deg"))
    validate_stationarity(original)
    captured = datetime.fromisoformat(original["captured_utc"])
    if captured.tzinfo is None:
        raise PoseCaptureError("historical capture timestamp must carry timezone")
    record = {
        **original,
        "kind": POSE_KIND,
        "task_id": task_id,
        "validation_config_sha256": config_sha256(config),
        "provenance": {
            "mode": "historical_import",
            "source_path": str(source.resolve()),
            "source_sha256": hashlib.sha256(payload).hexdigest(),
            "writer_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "original_config_sha256": None,
            "original_system_config": original.get("system_config"),
            "imported_utc": datetime.now(timezone.utc).isoformat(),
            "note": "Historical recorded stationary summary; not a fresh capture or current scene verification.",
        },
    }
    return _write_record(config, task_id, record, output)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--stationary-spec", type=Path)
    parser.add_argument("--task-id", required=True)
    args = parser.parse_args(argv)
    config = load_system_config(args.config)
    spec = CaptureSpec.load(
        args.stationary_spec or config.path.parents[2] / "configs/tasks/pose_capture.json"
    )
    record = capture_pose(config, spec, task_id=args.task_id)
    print(
        json.dumps(
            {
                "output": str(canonical_pose_path(config, args.task_id)),
                "status": record["status"],
                "stationary_samples": record["stationary_samples"],
                "follower_deg": record["follower_deg"],
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
