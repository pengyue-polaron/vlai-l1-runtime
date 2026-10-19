"""Offline same-session unloaded/loaded/unloaded motor-torque comparison.

Requires successful stationary-hold release and explicit operator-labelled
windows. Reports posture changes; does not infer force or compensate friction.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
from pathlib import Path

import numpy as np

from .automatic_review import review_automatic
from .configuration import MOTOR_NAMES, load_system_config
from .unloaded_baseline import digest, read_artifact


def compare_windows(samples, windows, policy):
    if set(policy) != {"minimum_samples", "minimum_seconds", "maximum_window_span_deg"}:
        raise ValueError("exact hold review policy required")
    if type(policy["minimum_samples"]) is not int or policy["minimum_samples"] < 2:
        raise ValueError("minimum samples must be an integer >= 2")
    for name in ("minimum_seconds", "maximum_window_span_deg"):
        if (
            type(policy[name]) not in (int, float)
            or not math.isfinite(policy[name])
            or policy[name] <= 0
        ):
            raise ValueError("positive finite hold review policy required")
    samples = np.asarray(samples, dtype=float)
    if (
        samples.ndim != 2
        or samples.shape[1] != 17
        or len(samples) < 2
        or not np.isfinite(samples).all()
        or np.any(np.diff(samples[:, 0]) <= 0)
    ):
        raise ValueError("ordered finite time/q/torque samples required")
    if len(windows) != 3:
        raise ValueError("exact empty_before, loaded, empty_after windows required")
    summaries = []
    previous_end = 0
    for window, label, load in zip(
        windows, ("empty_before", "loaded", "empty_after"), (False, True, False), strict=True
    ):
        if (
            window.get("label") != label
            or window.get("external_load_present") is not load
            or not window.get("operator_statement")
        ):
            raise ValueError("explicit ordered operator load labels required")
        start, end = window.get("start_monotonic_ns"), window.get("end_monotonic_ns")
        if (
            type(start) is not int
            or type(end) is not int
            or start < previous_end
            or end <= start
            or start < samples[0, 0]
            or end > samples[-1, 0]
        ):
            raise ValueError("nonoverlapping windows inside recorded samples required")
        selected = samples[(samples[:, 0] >= start) & (samples[:, 0] <= end)]
        if (
            len(selected) < policy["minimum_samples"]
            or (end - start) / 1e9 < policy["minimum_seconds"]
        ):
            raise ValueError("insufficient settled window samples/duration")
        span = np.ptp(selected[:, 1:9], axis=0)
        if np.any(span > policy["maximum_window_span_deg"]):
            raise ValueError(f"window not position-stable: {label}")
        summaries.append(
            {
                **window,
                "samples": len(selected),
                "mean_monotonic_ns": float(selected[:, 0].mean()),
                "mean_q_deg": selected[:, 1:9].mean(axis=0).tolist(),
                "q_span_deg": span.tolist(),
                "mean_torque_nm": selected[:, 9:17].mean(axis=0).tolist(),
                "torque_std_nm": selected[:, 9:17].std(axis=0).tolist(),
            }
        )
        previous_end = end
    before, loaded, after = summaries
    weight = (loaded["mean_monotonic_ns"] - before["mean_monotonic_ns"]) / (
        after["mean_monotonic_ns"] - before["mean_monotonic_ns"]
    )

    def interpolate(key):
        return (1 - weight) * np.array(before[key]) + weight * np.array(after[key])

    return {
        "kind": "same_session_hold_load_comparison_v1",
        "ft_validated": False,
        "runtime_eligible": False,
        "pose_compensated": False,
        "gravity_or_friction_compensated": False,
        "motor_names": list(MOTOR_NAMES),
        "windows": summaries,
        "interpolated_empty_torque_nm": interpolate("mean_torque_nm").tolist(),
        "loaded_minus_interpolated_empty_torque_nm": (
            np.array(loaded["mean_torque_nm"]) - interpolate("mean_torque_nm")
        ).tolist(),
        "loaded_minus_interpolated_empty_q_deg": (
            np.array(loaded["mean_q_deg"]) - interpolate("mean_q_deg")
        ).tolist(),
        "empty_return_torque_difference_nm": (
            np.array(after["mean_torque_nm"]) - before["mean_torque_nm"]
        ).tolist(),
        "empty_return_q_difference_deg": (
            np.array(after["mean_q_deg"]) - before["mean_q_deg"]
        ).tolist(),
        "meaning": "observed motor-torque response relative to linearly interpolated empty windows; includes posture, hysteresis and model errors, not external FT",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("config", "source", "windows", "policy", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    config = load_system_config(args.config)
    review_automatic(config, args.source)
    report = json.loads(read_artifact(args.source / "report.json"))
    if report["mode"] != "hold":
        raise ValueError("stationary hold capture required")
    window_bytes = read_artifact(args.windows)
    manifest = json.loads(window_bytes)
    if manifest.get("source_raw_sha256") != report["raw_sha256"]:
        raise ValueError("window labels must bind to this raw capture")
    policy = json.loads(read_artifact(args.policy))
    reader = csv.DictReader(io.StringIO(read_artifact(args.source / "samples.csv").decode()))
    fields = ["monotonic_ns"] + [p + n for p in ("q_", "torque_") for n in MOTOR_NAMES]
    result = compare_windows(
        [[float(r[k]) for k in fields] for r in reader], manifest["windows"], policy
    )
    result.update(
        source_raw_sha256=report["raw_sha256"], windows_sha256=digest(window_bytes), policy=policy
    )
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(args.output)


if __name__ == "__main__":
    main()
