"""Hardware-free review of the isolated automatic diagnostic's saved artifacts."""

from __future__ import annotations

import csv
import io
import json
import math
from pathlib import Path

import numpy as np

from .configuration import MOTOR_NAMES, SystemConfig
from .teleoperation.automatic import validate_policy
from .unloaded_baseline import digest, read_artifact


def review_automatic(config: SystemConfig, source: Path) -> dict:
    """Verify integrity and quantify tracking/return differences, never claim FT."""
    report = json.loads(read_artifact(source / "report.json"))
    if (
        report.get("kind") != "isolated_right_follower_automatic_diagnostic"
        or report.get("status") != "passed"
        or report.get("disabled_verified") is not True
        or report.get("config_sha256") != digest(read_artifact(config.path))
        or report.get("mode") not in ("hold", "sweep")
    ):
        raise ValueError("review requires a passed, disabled, matching automatic capture")
    policy = report["policy"]
    validate_policy(config, policy)
    payloads = {}
    for filename, key in (("samples.csv", "samples_sha256"), ("raw_feedback.csv", "raw_sha256")):
        payloads[filename] = read_artifact(source / filename)
        if digest(payloads[filename]) != report.get(key):
            raise ValueError(f"capture hash mismatch: {filename}")
    fields = ["monotonic_ns", "elapsed_seconds"] + [
        prefix + name for prefix in ("q_", "reference_", "torque_") for name in MOTOR_NAMES
    ]
    reader = csv.DictReader(io.StringIO(payloads["samples.csv"].decode()))
    if reader.fieldnames != fields:
        raise ValueError("exact named sample columns required")
    samples = np.array([[float(row[k]) for k in fields] for row in reader])
    if samples.ndim != 2 or len(samples) < 2 or not np.isfinite(samples).all():
        raise ValueError("finite complete sample rows required")
    t = samples[:, 1]
    if (
        np.any(np.diff(samples[:, 0]) <= 0)
        or np.any(np.diff(t) <= 0)
        or t[0] < 0
        or t[0] > config.teleoperation.state_timeout_s
        or np.max(np.diff(t)) > config.teleoperation.state_timeout_s
        or np.max(np.abs((samples[:, 0] - samples[0, 0]) / 1e9 - (t - t[0]))) > 1e-6
    ):
        raise ValueError("nonmonotonic or gapped sample timing")
    duration = (
        policy["hold_test_seconds"]
        if report["mode"] == "hold"
        else policy["hold_seconds"]
        + 4 * policy["cycles"] * (policy["transition_seconds"] + policy["hold_seconds"])
    )
    if abs(t[-1] - duration) > config.teleoperation.state_timeout_s:
        raise ValueError("incomplete diagnostic duration")
    q, reference, torque = samples[:, 2:10], samples[:, 10:18], samples[:, 18:26]
    fixed = list(range(8)) if report["mode"] == "hold" else [0, 1, 2, 3, 4, 5, 7]
    if np.any(reference[:, fixed] != reference[0, fixed]):
        raise ValueError("non-wrist reference changed")
    if np.max(np.abs(reference[:, 6] - reference[0, 6])) > policy["amplitude_deg"] + 1e-9:
        raise ValueError("reference exceeded sweep envelope")

    # Include startup/shutdown status zero; do not pass these through an unloaded
    # baseline parser that correctly requires enabled-only raw feedback.
    raw_text = payloads["raw_feedback.csv"].decode()
    if "# status=passed\n" not in raw_text:
        raise ValueError("raw capture did not pass")
    raw = csv.DictReader(line for line in raw_text.splitlines() if not line.startswith("#"))
    motors = {m.receive_id: m for m in config.motors}
    latest, counts, statuses = {}, dict.fromkeys(MOTOR_NAMES, 0), {n: set() for n in MOTOR_NAMES}
    previous = 0
    first_stamp = 0
    maximum_gap_ns = 0
    frame_count = 0
    for index, row in enumerate(raw):
        motor = motors.get(int(row["can_id"]))
        stamp = int(row["host_receive_monotonic_ns"])
        if index == 0:
            first_stamp = stamp
        b = bytes.fromhex(row["payload_hex"])
        if (
            motor is None
            or int(row["sequence"]) != index
            or stamp <= previous
            or int(row["rx_overflow_count"]) != 0
            or int(row["can_fd"]) != int(config.can.fd)
            or len(b) != 8
            or b[0] & 15 != motor.send_id
            or b[0] >> 4 not in (policy["enabled_raw_status"], policy["disabled_raw_status"])
        ):
            raise ValueError("raw sequence, identity, status or overflow fault")
        if motor.name in latest:
            maximum_gap_ns = max(maximum_gap_ns, stamp - latest[motor.name][0])
        latest[motor.name] = (stamp, b[0] >> 4)
        statuses[motor.name].add(b[0] >> 4)
        counts[motor.name] += 1
        previous = stamp
        frame_count += 1
    if (
        set(latest) != set(MOTOR_NAMES)
        or any(value[1] != policy["disabled_raw_status"] for value in latest.values())
        or maximum_gap_ns > config.teleoperation.state_timeout_s * 1e9
        or any(policy["enabled_raw_status"] not in value for value in statuses.values())
        or first_stamp > samples[0, 0]
        or any(value[0] < samples[-1, 0] for value in latest.values())
    ):
        raise ValueError("missing, stale, never-enabled or unconfirmed-disabled raw motor")

    # Last half of each scheduled hold, without claiming that measured motion
    # was zero or that a repeated command means the same measured posture.
    holds = [(0.0, float(policy["hold_test_seconds"]), "initial")]
    if report["mode"] == "sweep":
        h, transition = policy["hold_seconds"], policy["transition_seconds"]
        holds = [(0.0, float(h), "initial")]
        for leg in range(4 * policy["cycles"]):
            start = h + leg * (transition + h) + transition
            holds.append((start, start + h, ("positive", "return", "negative", "return")[leg % 4]))
    windows = []
    for start, end, label in holds:
        mask = (t >= (start + end) / 2) & (t < end)
        if not mask.any():
            raise ValueError("missing scheduled hold samples")
        windows.append(
            {
                "kind": label,
                "start_seconds": (start + end) / 2,
                "end_seconds": end,
                "samples": int(mask.sum()),
                "mean_q_deg": q[mask].mean(axis=0).tolist(),
                "q_span_deg": np.ptp(q[mask], axis=0).tolist(),
                "mean_reference_deg": reference[mask].mean(axis=0).tolist(),
                "mean_torque_nm": torque[mask].mean(axis=0).tolist(),
                "torque_std_nm": torque[mask].std(axis=0).tolist(),
            }
        )
    returns = []
    for window in windows:
        if window["kind"] == "return":
            returns.append(
                {
                    "start_seconds": window["start_seconds"],
                    "q_difference_deg": (
                        np.array(window["mean_q_deg"]) - windows[0]["mean_q_deg"]
                    ).tolist(),
                    "torque_difference_nm": (
                        np.array(window["mean_torque_nm"]) - windows[0]["mean_torque_nm"]
                    ).tolist(),
                }
            )
    repeat_pairs = []
    for index in range(5, len(windows)):
        current, previous_window = windows[index], windows[index - 4]
        repeat_pairs.append(
            {
                "kind": current["kind"],
                "previous_start_seconds": previous_window["start_seconds"],
                "current_start_seconds": current["start_seconds"],
                "q_difference_deg": (
                    np.array(current["mean_q_deg"]) - previous_window["mean_q_deg"]
                ).tolist(),
                "torque_difference_nm": (
                    np.array(current["mean_torque_nm"]) - previous_window["mean_torque_nm"]
                ).tolist(),
            }
        )
    result = {
        "kind": "automatic_diagnostic_review_v1",
        "status": "passed",
        "meaning": "capture integrity and measurements only; not FT/model acceptance",
        "controller_context": "direct_motor_PD_zero_feedforward",
        "ft_validated": False,
        "external_load_present": report.get("external_load_present"),
        "motor_names": list(MOTOR_NAMES),
        "samples": len(samples),
        "raw_frames": frame_count,
        "raw_frames_per_motor": counts,
        "maximum_motor_gap_ms": maximum_gap_ns / 1e6,
        "duration_seconds": float(t[-1]),
        "maximum_tick_ms": float(np.max(np.diff(t)) * 1000),
        "q_span_deg": np.ptp(q, axis=0).tolist(),
        "reference_span_deg": np.ptp(reference, axis=0).tolist(),
        "maximum_tracking_error_deg": np.max(np.abs(q - reference), axis=0).tolist(),
        "maximum_abs_torque_nm": np.max(np.abs(torque), axis=0).tolist(),
        "hold_windows": windows,
        "same_command_returns": returns,
        "same_approach_repeat_pairs": repeat_pairs,
        "raw_sha256": report["raw_sha256"],
        "samples_sha256": report["samples_sha256"],
    }
    if not math.isfinite(result["maximum_tick_ms"]):
        raise ValueError("invalid review timing")
    return result
