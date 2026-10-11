"""Hardware-free right-Follower, same-capture unloaded delta-wrench diagnostic.

Run with ``python -m vlai_l1_runtime.unloaded_baseline --help``. No persisted
baseline can be imported into another run, posture, or controller.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from dataclasses import asdict
from itertools import pairwise
from pathlib import Path

from .configuration import MOTOR_NAMES, _read_local_regular_file, load_system_config
from .feedback import MotorFeedback, RawMotorFrame
from .feedback_decode import decode_follower_feedback
from .gravity import GravityModel
from .kinematics import SerialChain
from .static_wrench import JointCoordinateMap, OfflineStaticWrench, StaticWrenchPolicy
from .wrench import ARM_JOINT_NAMES, estimate_wrench, propagated_torque_error

ASSUMPTIONS = (
    "Operator declares no external load; cameras, gripper and brackets remain installed.",
    "Identity motor/model coordinates, torque gain one and torque sign remain unverified.",
    "URDF gravity omits camera/bracket payload; finger position and nominal friction are zero.",
    "Raw status acceptance and uncertainty limits are offline diagnostic hypotheses only.",
    "Tare absorbs fixed model/payload/friction/bias errors at one posture in one capture.",
    "Held-out zero-load variation is repeatability, not force accuracy or six-axis calibration.",
    "Host dequeue/kernel timestamps are not synchronized motor acquisition timestamps.",
)
FIELDS = (
    "sequence",
    "host_receive_monotonic_ns",
    "host_receive_realtime_ns",
    "kernel_receive_realtime_ns",
    "can_id",
    "can_fd",
    "payload_hex",
    "rx_overflow_count",
)


def read_artifact(path: Path) -> bytes:
    return _read_local_regular_file(
        path.absolute(), label="offline artifact", max_bytes=64 * 1024 * 1024
    )


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def load_policy(raw: dict, config) -> StaticWrenchPolicy:
    keys = {
        "fit_seconds",
        "validation_seconds",
        "minimum_fit_samples",
        "minimum_validation_samples",
        "maximum_pose_difference_rad",
        "static",
    }
    if set(raw) != keys:
        raise ValueError("unloaded policy requires exact keys")
    windows = (*raw["fit_seconds"], *raw["validation_seconds"])
    if (
        len(raw["fit_seconds"]) != 2
        or len(raw["validation_seconds"]) != 2
        or any(type(v) not in (int, float) or not math.isfinite(v) for v in windows)
        or not 0 <= windows[0] < windows[1] < windows[2] < windows[3]
    ):
        raise ValueError("fit and validation windows must be ordered, finite and disjoint")
    for key in ("minimum_fit_samples", "minimum_validation_samples"):
        if type(raw[key]) is not int or raw[key] < 2:
            raise ValueError("minimum sample counts must be integers >= 2")
    limit = raw["maximum_pose_difference_rad"]
    if type(limit) not in (int, float) or not math.isfinite(limit) or limit <= 0:
        raise ValueError("pose envelope must be positive and finite")
    values = dict(raw["static"])
    if "maximum_pair_age_ns" in values:
        raise ValueError("pair freshness must come from Runtime config")
    for key, value in tuple(values.items()):
        if isinstance(value, list):
            values[key] = tuple(value)
    values["allowed_raw_statuses"] = tuple(frozenset(s) for s in values["allowed_raw_statuses"])
    return StaticWrenchPolicy(
        maximum_pair_age_ns=int(config.teleoperation.state_timeout_s * 1e9), **values
    )


def parse_capture(config, capture: dict, data: bytes):
    """Validate every raw row; assemble fresh eight-motor batches, without TX."""
    endpoint = next(e for e in config.can.endpoints if e.side == "right" and e.role == "follower")
    if (
        capture.get("status") != "passed"
        or capture.get("side") != "right"
        or capture.get("role") != "follower"
        or capture.get("raw_sha256") != digest(data)
        or capture.get("config_sha256") != digest(read_artifact(config.path))
        or (capture.get("interface"), capture.get("parentdev"))
        != (endpoint.interface, endpoint.parentdev)
    ):
        raise ValueError("capture status, digest or configured physical identity mismatch")
    lines = data.decode("utf-8").splitlines()
    if lines[:4] != [
        f"# interface={endpoint.interface}",
        "# status=passed",
        "# diagnostic_only=true",
        "# error=",
    ]:
        raise ValueError("native capture failed or header mismatch")
    offset = 4
    evicted = 0
    retention = capture.get("feedback_retention", "full")
    if retention == "latest":
        prefix = "# retained_latest_evicted="
        if len(lines) < 6 or not lines[4].startswith(prefix):
            raise ValueError("missing latest-retention metadata")
        value = lines[4][len(prefix) :]
        if not value.isascii() or not value.isdecimal():
            raise ValueError("invalid latest-retention eviction count")
        evicted = int(value)
        if (
            type(capture.get("evicted_raw_frames")) is not int
            or capture["evicted_raw_frames"] != evicted
        ):
            raise ValueError("latest-retention report/header mismatch")
        offset = 5
    elif retention != "full":
        raise ValueError("unknown feedback retention mode")
    reader = csv.DictReader(lines[offset:])
    if tuple(reader.fieldnames or ()) != FIELDS:
        raise ValueError("unexpected native CSV fields")
    by_id = {m.receive_id: m for m in config.motors}
    pending, previous_by_id, batches = {}, {}, []
    previous_ns = previous_kernel = 0
    max_gap = max_age = replaced = count = 0
    statuses = Counter()
    timeout = int(config.teleoperation.state_timeout_s * 1e9)
    for sequence, row in enumerate(reader):
        if set(row) != set(FIELDS) or any(v is None for v in row.values()):
            raise ValueError("malformed native CSV row")
        ns = int(row["host_receive_monotonic_ns"])
        kernel = int(row["kernel_receive_realtime_ns"])
        age = int(row["host_receive_realtime_ns"]) - kernel
        motor = by_id.get(int(row["can_id"]))
        if (
            int(row["sequence"]) != sequence
            or ns <= previous_ns
            or kernel <= 0
            or kernel < previous_kernel
            or not 0 <= age <= timeout
            or int(row["rx_overflow_count"]) != 0
            or row["can_fd"] != "1"
            or motor is None
        ):
            raise ValueError("invalid raw identity, timing, sequence, CAN FD or overflow")
        previous_ns, previous_kernel = ns, kernel
        raw = RawMotorFrame(motor.name, motor.receive_id, ns, row["payload_hex"], True)
        # Validate even samples replaced before a complete batch is assembled.
        payload = bytes.fromhex(raw.payload_hex)
        if payload[0] & 15 != motor.send_id:
            raise ValueError("raw slave identity mismatch")
        integers = (
            payload[1] << 8 | payload[2],
            payload[3] << 4 | payload[4] >> 4,
            (payload[4] & 15) << 8 | payload[5],
        )
        if any(n in (0, top) for n, top in zip(integers, (65535, 4095, 4095), strict=True)):
            raise ValueError("raw protocol endpoint observed")
        statuses[bytes.fromhex(raw.payload_hex)[0] >> 4] += 1
        if motor.name in previous_by_id:
            gap = ns - previous_by_id[motor.name]
            max_gap = max(max_gap, gap)
            if gap > timeout:
                raise ValueError("motor response gap exceeds Runtime freshness")
        previous_by_id[motor.name] = ns
        if ns - min(previous_by_id.values()) > timeout:
            raise ValueError("missing fresh motor response")
        if motor.name in pending:
            raise ValueError("duplicate motor before complete batch; refuse hidden raw samples")
        pending[motor.name] = raw
        count += 1
        max_age = max(max_age, age)
        if len(pending) == len(MOTOR_NAMES):
            source = MotorFeedback(
                MOTOR_NAMES,
                ns,
                (0.0,) * 8,
                (0.0,) * 8,
                (0.0,) * 8,
                tuple(pending[n] for n in MOTOR_NAMES),
            )
            batches.append(decode_follower_feedback(config, "right", source).feedback)
            pending = {}
    if not batches:
        raise ValueError("no complete motor batches")
    return batches, {
        "raw_frames": count,
        "complete_batches": len(batches),
        "trailing_partial_motor_count": len(pending),
        "replaced_raw_frames": replaced,
        "maximum_motor_gap_ns": max_gap,
        "maximum_kernel_queue_age_ns": max_age,
        "raw_status_counts": dict(statuses),
        "retention": retention,
        "evicted_raw_frames": evicted,
    }


def evaluate_baseline(estimator, batches, policy: dict):
    """Fit only the early window; motion after fitting permanently invalidates tare."""
    import numpy as np

    first = batches[0].monotonic_ns
    times = [(b.monotonic_ns - first) / 1e9 for b in batches]
    results = [estimator.process(b, side="right") for b in batches]
    if any(r.status == "invalid" for r in results):
        fault = next(r for r in results if r.status == "invalid")
        raise ValueError(f"invalid capture input: {fault.reasons}")
    fit_start, fit_end = policy["fit_seconds"]
    val_start, val_end = policy["validation_seconds"]
    fit_ids = [i for i, t in enumerate(times) if fit_start <= t < fit_end]
    val_ids = [i for i, t in enumerate(times) if val_start <= t <= val_end]
    gap_s = estimator.policy.maximum_sample_gap_ns / 1e9
    for ids, start, end, minimum in (
        (fit_ids, fit_start, fit_end, policy["minimum_fit_samples"]),
        (val_ids, val_start, val_end, policy["minimum_validation_samples"]),
    ):
        if (
            len(ids) < minimum
            or times[ids[0]] - start > gap_s
            or end - times[ids[-1]] > gap_s
            or any(times[b] - times[a] > gap_s for a, b in pairwise(ids))
        ):
            raise ValueError("insufficient sample count or time coverage")
    if any(results[i].external_torque_nm is None for i in fit_ids):
        raise ValueError("baseline window is not fully stationary")
    # Check all eight coordinates: gripper changes also invalidate attached-load tare.
    fit_q = np.array([batches[i].position_rad for i in fit_ids])
    span = np.ptp(fit_q, axis=0)
    envelope = policy["maximum_pose_difference_rad"]
    if np.any(span > envelope):
        raise ValueError("baseline pose moved outside envelope")
    reference = fit_q.mean(axis=0)
    baseline = np.mean([results[i].external_torque_nm for i in fit_ids], axis=0)
    output, accepted, rejections = [], [], Counter()
    invalidated = None
    p = estimator.policy
    ranges = estimator.config.follower_feedback.for_side("right")
    bounds = {
        name: 4 * ranges[i].torque_max_nm / 4095 + p.unmodeled_torque_bound_nm[i]
        for i, name in enumerate(ARM_JOINT_NAMES)
    }
    for i, (t, feedback, result) in enumerate(zip(times, batches, results, strict=True)):
        if not fit_end <= t <= val_end:
            continue
        if invalidated is None:
            if np.any(np.abs(np.array(feedback.position_rad) - reference) > envelope):
                invalidated = "pose envelope exceeded; baseline invalidated for remainder"
            elif result.external_torque_nm is None:
                invalidated = "stationarity lost; baseline invalidated for remainder"
            elif times[i] - times[i - 1] > gap_s:
                invalidated = "sample gap; baseline invalidated for remainder"
        if t < val_start:
            continue
        row = {
            "monotonic_ns": feedback.monotonic_ns,
            "seconds": t,
            "status": "rejected",
            "reason": invalidated,
            "delta_wrench": None,
            "ft_validated": False,
        }
        if invalidated is None:
            try:
                geometry = estimator.chain.evaluate(
                    dict(zip(estimator.chain.joint_names, result.model_positions_rad, strict=True))
                )
                estimate = estimate_wrench(
                    joint_names=ARM_JOINT_NAMES,
                    jacobian=geometry.jacobian,
                    external_joint_torque_nm=dict(
                        zip(
                            ARM_JOINT_NAMES,
                            (np.array(result.external_torque_nm) - baseline).tolist(),
                            strict=True,
                        )
                    ),
                    frame=estimator.chain.root,
                    reference_point=estimator.chain.tip,
                    monotonic_ns=feedback.monotonic_ns,
                    **{
                        key: getattr(p, key)
                        for key in (
                            "characteristic_length_m",
                            "damping_m",
                            "minimum_singular_value_m",
                            "maximum_condition_number",
                            "maximum_residual_nm",
                        )
                    },
                )
                error = propagated_torque_error(estimate, bounds)
                if any(
                    a > b
                    for a, b in zip(
                        error, (*p.maximum_force_error_n, *p.maximum_moment_error_nm), strict=True
                    )
                ):
                    raise ValueError("illustrative propagated error limit exceeded")
                values = (*estimate.force_n, *estimate.moment_nm)
                row.update(
                    status="unvalidated_delta_candidate",
                    reason=None,
                    delta_wrench=values,
                    propagated_error_bound=error,
                )
                accepted.append(values)
            except ValueError as exc:
                row["reason"] = str(exc)
        if row["reason"]:
            rejections[row["reason"]] += 1
        output.append(row)
    report = {
        "ft_validated": False,
        "status": "passed" if not rejections else "rejected",
        "scope": "same capture, same posture only; offline diagnostic",
        "assumptions": ASSUMPTIONS,
        "fit_samples": len(fit_ids),
        "validation_samples": len(output),
        "validation_candidates": len(accepted),
        "rejections": dict(rejections),
        "baseline_model_minus_motor_nm": baseline.tolist(),
        "motor_names": MOTOR_NAMES,
        "reference_position_rad": reference.tolist(),
        "fit_position_span_rad": span.tolist(),
        "baseline_invalidated_reason": invalidated,
        "frame": estimator.chain.root,
        "reference_point": estimator.chain.tip,
        "wrench_order": ["Fx_N", "Fy_N", "Fz_N", "Mx_Nm", "My_Nm", "Mz_Nm"],
        "static_pipeline_counts": dict(Counter(r.status for r in results)),
    }
    if accepted:
        values = np.array(accepted)
        report.update(
            delta_wrench_mean=values.mean(axis=0).tolist(),
            delta_wrench_std=values.std(axis=0).tolist(),
            delta_wrench_abs_max=np.abs(values).max(axis=0).tolist(),
        )
    return report, output, results


def analyze(config_path: Path, capture_path: Path, urdf: Path, policy_path: Path, output: Path):
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"output already exists: {output}")
    config = load_system_config(config_path)
    raw = read_artifact(capture_path / "raw_feedback.csv")
    capture_bytes = read_artifact(capture_path / "report.json")
    capture = json.loads(capture_bytes)
    if "external_load_present" in capture and capture["external_load_present"] is not False:
        raise ValueError("loaded or unknown-load capture cannot establish an unloaded baseline")
    policy_bytes = read_artifact(policy_path)
    policy = json.loads(policy_bytes)
    static_policy = load_policy(policy, config)
    batches, metadata = parse_capture(config, capture, raw)
    # No raw status in partial batches may bypass acceptance.
    allowed = set.intersection(*(set(s) for s in static_policy.allowed_raw_statuses))
    if not set(metadata["raw_status_counts"]) <= allowed:
        raise ValueError("unaccepted raw status in capture")
    model_bytes = read_artifact(urdf)
    names = tuple(f"xarm_right_joint{i}" for i in range(1, 8))
    chain = SerialChain.from_urdf(
        urdf, root="xarm_body_link0", tip="xarm_right_hand_tcp", expected_joint_names=names
    )
    gravity = GravityModel.from_urdf(
        urdf,
        root=chain.root,
        branch="xarm_right_link0",
        expected_joint_names=(*names, "xarm_right_finger_joint1"),
        massless_links=(chain.tip,),
    )
    if chain.urdf_sha256 != digest(model_bytes) or gravity.urdf_sha256 != digest(model_bytes):
        raise ValueError("model changed during read")
    estimator = OfflineStaticWrench(
        config=config,
        side="right",
        chain=chain,
        gravity=gravity,
        mapping=tuple(
            JointCoordinateMap(m, n, 1.0, 0.0, 1.0, 0.0)
            for m, n in zip(ARM_JOINT_NAMES, names, strict=True)
        ),
        auxiliary_positions=(("xarm_right_finger_joint1", 0.0),),
        gravity_m_s2=(0.0, 0.0, -9.80665),
        payloads=(),
        static_friction_torque_nm=(0.0,) * 7,
        policy=static_policy,
        assumptions=ASSUMPTIONS,
    )
    report, rows, results = evaluate_baseline(estimator, batches, policy)
    report.update(
        capture=metadata,
        policy=policy,
        resolved_pair_age_ns=static_policy.maximum_pair_age_ns,
        raw_sha256=digest(raw),
        capture_report_sha256=digest(capture_bytes),
        config_sha256=digest(read_artifact(config.path)),
        urdf_sha256=digest(model_bytes),
        policy_sha256=digest(policy_bytes),
    )
    # Create-only result directory: failures never replace a previous baseline.
    output.mkdir(parents=False, exist_ok=False)
    with (output / "report.json").open("x") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    with (output / "delta_wrench.jsonl").open("x") as handle:
        for row in rows:
            handle.write(json.dumps(row, allow_nan=False) + "\n")
    with (output / "static_results.jsonl").open("x") as handle:
        for result in results:
            handle.write(json.dumps(asdict(result), allow_nan=False) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("config", "capture", "urdf", "policy", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    args = parser.parse_args()
    try:
        report = analyze(args.config, args.capture, args.urdf, args.policy, args.output)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(2, f"unloaded baseline failed: {exc}\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
