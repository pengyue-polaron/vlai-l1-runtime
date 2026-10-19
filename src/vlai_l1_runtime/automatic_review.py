"""Hardware-free review of the isolated automatic diagnostic's saved artifacts."""

from __future__ import annotations

import csv
import io
import json
import math
from pathlib import Path

import numpy as np

from .automatic_trajectory import is_waypoint_policy, path_offsets, sweep_duration, target_offsets
from .configuration import MOTOR_NAMES, SystemConfig
from .teleoperation.automatic import validate_policy
from .unloaded_baseline import digest, read_artifact


def review_automatic(config: SystemConfig, source: Path, *, recheck_offline_failure=False) -> dict:
    """Verify integrity and quantify tracking/return differences, never claim FT."""
    report = json.loads(read_artifact(source / "report.json"))
    recoverable_review_failure = (
        recheck_offline_failure
        and report.get("status") == "failed"
        and report.get("analysis_status") == "failed"
        and report.get("native_returncode") == 0
        and not report.get("cleanup_error")
        and isinstance(report.get("error"), str)
        and report["error"].startswith("offline review:")
    )
    if (
        report.get("kind") != "isolated_right_follower_automatic_diagnostic"
        or (report.get("status") != "passed" and not recoverable_review_failure)
        or report.get("disabled_verified") is not True
        or report.get("config_sha256") != digest(read_artifact(config.path))
        or report.get("mode") not in ("hold", "sweep", "paired")
    ):
        raise ValueError("review requires a passed, disabled, matching automatic capture")
    policy = report["policy"]
    validate_policy(config, policy)
    multi = is_waypoint_policy(policy)
    if multi and report["mode"] == "paired":
        raise ValueError("waypoint path excludes paired handling")
    points = path_offsets(policy)
    legs = len(points) - 1
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
    duration = policy["hold_test_seconds"] if report["mode"] == "hold" else sweep_duration(policy)
    phases = []
    if report["mode"] == "paired":
        from .paired_protocol import paired_phases

        phases = paired_phases(report, config.teleoperation.state_timeout_s)
        duration = report["completed_paired_seconds"]
    if report.get("completion_kind") == "requested_hold":
        actual = report.get("completed_hold_seconds")
        completions = [
            e for e in report.get("events", []) if e.get("completion_kind") == "requested_hold"
        ]
        if (
            report["mode"] != "hold"
            or report.get("operator_finish_requested") is not True
            or type(actual) not in (int, float)
            or not math.isfinite(actual)
            or not policy["hold_seconds"] <= actual <= duration
            or len(completions) != 1
            or completions[0].get("state") != "completed"
            or completions[0].get("disabled_verified") is not True
            or completions[0].get("elapsed_seconds") != actual
        ):
            raise ValueError("invalid requested hold completion evidence")
        duration = actual
    if abs(t[-1] - duration) > config.teleoperation.state_timeout_s:
        raise ValueError("incomplete diagnostic duration")
    q, reference, torque = samples[:, 2:10], samples[:, 10:18], samples[:, 18:26]
    active_indices = [i for i in range(8) if any(p[i] != 0 for p in points)]
    fixed = (
        list(range(8))
        if report["mode"] == "hold"
        else [i for i in range(8) if i not in active_indices]
    )
    if np.any(reference[:, fixed] != reference[0, fixed]):
        raise ValueError("fixed motor reference changed")
    if np.max(np.abs(reference - reference[0])) > policy["amplitude_deg"] + 1e-9:
        raise ValueError("reference exceeded sweep envelope")
    if multi:
        expected = (
            np.array(
                [
                    target_offsets(policy, float(seconds), hold_only=report["mode"] == "hold")
                    for seconds in t
                ]
            )
            + reference[0]
        )
        if np.max(np.abs(reference - expected)) > 1e-8:
            raise ValueError("reference differs from scheduled waypoint trajectory")
        bounds = config.teleoperation.joint_safety.for_side("right")
        planned = np.asarray(points) + reference[0]
        if np.any(planned[:, :7] < bounds.min_deg) or np.any(planned[:, :7] > bounds.max_deg):
            raise ValueError("waypoint path outside joint bounds")
        if (
            np.any(q[:, :7] < bounds.min_deg)
            or np.any(q[:, :7] > bounds.max_deg)
            or np.any(np.abs(q - reference[0]) > policy["maximum_relative_excursion_deg"])
        ):
            raise ValueError("measured waypoint bounds/excursion exceeded")
    for phase in phases:
        mask = (t >= phase["start_seconds"]) & (t < phase["end_seconds"])
        if phase["phase"].startswith(("awaiting_", "settling_")) and np.any(
            reference[mask] != reference[0]
        ):
            raise ValueError("paired waiting reference changed")
        if phase["phase"].startswith("settling_"):
            h = policy["manual_handling"]
            stable = (t >= phase["end_seconds"] - h["settle_seconds"]) & (t < phase["end_seconds"])
            if not stable.any() or np.any(np.ptp(q[stable], axis=0) > h["settle_span_deg"] + 1e-9):
                raise ValueError("paired settling position span exceeded")

    # Include startup/shutdown status zero; do not pass these through an unloaded
    # baseline parser that correctly requires enabled-only raw feedback.
    raw_text = payloads["raw_feedback.csv"].decode()
    if "# status=passed\n" not in raw_text:
        raise ValueError("raw capture did not pass")
    raw = csv.DictReader(line for line in raw_text.splitlines() if not line.startswith("#"))
    motors = {m.receive_id: m for m in config.motors}
    ranges = dict(zip(MOTOR_NAMES, config.follower_feedback.right, strict=True))
    speed_limits = dict(zip(MOTOR_NAMES, policy["maximum_observed_speed_deg_s"], strict=True))
    enabled_speed = dict.fromkeys(MOTOR_NAMES, 0.0)
    disabled_speed = dict.fromkeys(MOTOR_NAMES, 0.0)
    speed_exceedances = dict.fromkeys(MOTOR_NAMES, 0)
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
        # Match the native decoder's float32 rounding. Motor-reported speed is
        # separate from target speed and position differences. Disabled tails
        # can still move; never mix their peak into enabled-motion diagnostics.
        velocity_code = b[3] << 4 | b[4] >> 4
        speed = abs(
            math.degrees(
                float(
                    np.float32(
                        velocity_code * (2 * ranges[motor.name].velocity_max_rad_s) / 4095
                        - ranges[motor.name].velocity_max_rad_s
                    )
                )
            )
        )
        is_enabled = b[0] >> 4 == policy["enabled_raw_status"]
        peaks = enabled_speed if is_enabled else disabled_speed
        peaks[motor.name] = max(peaks[motor.name], speed)
        if is_enabled and speed > speed_limits[motor.name]:
            speed_exceedances[motor.name] += 1
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
    holds = [(0.0, float(duration), "initial")]
    if report["mode"] in ("sweep", "paired"):
        h, transition = policy["hold_seconds"], policy["transition_seconds"]
        starts = (
            [
                p["start_seconds"]
                for p in phases
                if not p["phase"].startswith(("awaiting_", "settling_"))
            ]
            if phases
            else [0.0]
        )
        holds = []
        for phase_start in starts:
            holds.append((phase_start, phase_start + float(h), "initial"))
            for leg in range(legs * policy["cycles"]):
                start = phase_start + h + leg * (transition + h) + transition
                label = (
                    ("return" if not any(points[leg % legs + 1]) else f"waypoint_{leg % legs + 1}")
                    if multi
                    else ("positive", "return", "negative", "return")[leg % 4]
                )
                holds.append((start, start + h, label))
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
    for window in windows if not phases else []:
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
    for index in range(legs + 1, len(windows)) if not phases else []:
        current, previous_window = windows[index], windows[index - legs]
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
        "source_report_status": report["status"],
        "offline_failure_rechecked": bool(recoverable_review_failure),
        "source_report_sha256": digest(read_artifact(source / "report.json")),
        "meaning": "capture integrity and measurements only; not FT/model acceptance",
        "controller_context": "direct_motor_PD_zero_feedforward",
        "ft_validated": False,
        "external_load_present": report.get("external_load_present"),
        "motor_names": list(MOTOR_NAMES),
        "active_motor": policy["active_motor"],
        "active_motors": [MOTOR_NAMES[i] for i in active_indices],
        "trajectory_kind": policy["kind"],
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
        "raw_maximum_abs_enabled_speed_deg_s": enabled_speed,
        "raw_maximum_abs_disabled_speed_deg_s": disabled_speed,
        "raw_enabled_speed_limit_exceedances": speed_exceedances,
        "hold_windows": windows,
        "same_command_returns": returns,
        "same_approach_repeat_pairs": repeat_pairs,
        "raw_sha256": report["raw_sha256"],
        "samples_sha256": report["samples_sha256"],
        "paired_phases": phases,
    }
    if not math.isfinite(result["maximum_tick_ms"]):
        raise ValueError("invalid review timing")
    return result
