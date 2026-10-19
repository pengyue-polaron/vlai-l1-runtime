"""Offline unloaded single-joint hold repeatability, without fitting away loads."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np

from .automatic_review import review_automatic
from .configuration import MOTOR_NAMES, load_system_config
from .unloaded_baseline import digest, read_artifact
from .wrist_baseline_study import validate_context


def compare_holds(review, policy):
    expected = {
        "minimum_hold_samples",
        "maximum_hold_span_deg",
        "maximum_other_joint_delta_deg",
        "maximum_active_joint_delta_deg",
    }
    if set(policy) != expected or any(
        type(v) not in (float, int) or not math.isfinite(v) or v <= 0 for v in policy.values()
    ):
        raise ValueError("exact positive finite repeatability policy required")
    if type(policy["minimum_hold_samples"]) is not int:
        raise ValueError("integer minimum samples required")
    if review["active_motor"] not in ("joint_4", "joint_6", "joint_7"):
        raise ValueError("supported active joint required")
    active = MOTOR_NAMES.index(review["active_motor"])
    fixed = [i for i in range(8) if i != active]
    holds = review["hold_windows"]
    if len(holds) < 5 or (len(holds) - 1) % 4:
        raise ValueError("complete single-joint cycles required")
    for i, h in enumerate(holds):
        label = "initial" if i == 0 else ("positive", "return", "negative", "return")[(i - 1) % 4]
        if h["kind"] != label or h["samples"] < policy["minimum_hold_samples"]:
            raise ValueError("ordered adequately sampled holds required")
        for key in ("mean_q_deg", "mean_torque_nm", "q_span_deg"):
            v = np.asarray(h[key], dtype=float)
            if v.shape != (8,) or not np.isfinite(v).all():
                raise ValueError("finite eight-motor hold vectors required")
        if min(h["q_span_deg"]) < 0 or max(h["q_span_deg"]) > policy["maximum_hold_span_deg"]:
            raise ValueError("hold not position-stable")
    q = np.array([h["mean_q_deg"] for h in holds])
    torque = np.array([h["mean_torque_nm"] for h in holds])

    def pair(i, j):
        return {
            "first_hold": i,
            "second_hold": j,
            "q_difference_deg": (q[j] - q[i]).tolist(),
            "torque_difference_nm": (torque[j] - torque[i]).tolist(),
            "within_pose_tolerance": bool(
                abs(q[j, active] - q[i, active]) <= policy["maximum_active_joint_delta_deg"]
                and np.max(np.abs(q[j, fixed] - q[i, fixed]))
                <= policy["maximum_other_joint_delta_deg"]
            ),
        }

    repeats = [pair(i - 4, i) for i in range(5, len(holds))]
    near = []
    for i, j in itertools.combinations(range(len(holds)), 2):
        # Compare different scheduled path stages, not repeated copies of the
        # same stage. Command stage is not a measured friction-direction label.
        if i > 0 and (i - 1) % 4 == (j - 1) % 4:
            continue
        result = pair(i, j)
        if result["within_pose_tolerance"]:
            near.append(result)
    return {
        "kind": "unloaded_joint_hold_repeatability_v1",
        "active_motor": review["active_motor"],
        "motor_names": list(MOTOR_NAMES),
        "ft_validated": False,
        "runtime_eligible": False,
        "policy": policy,
        "holds": holds,
        "same_scheduled_path_repeat_pairs": repeats,
        "near_pose_different_path_pairs": near,
        "maximum_repeat_torque_delta_nm": max(
            (abs(p["torque_difference_nm"][active]) for p in repeats), default=None
        ),
        "maximum_near_pose_torque_delta_nm": max(
            (abs(p["torque_difference_nm"][active]) for p in near), default=None
        ),
        "meaning": "Descriptive unloaded differences at measured poses and scheduled path stages. Near poses are not identical; these are not friction identification, calibrated uncertainty bounds or external FT. No coefficients fitted or runtime compensation exported.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("config", "source", "context", "policy", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    context_bytes = read_artifact(args.context)
    validate_context(json.loads(context_bytes))
    config = load_system_config(args.config)
    review = review_automatic(config, args.source)
    report = json.loads(read_artifact(args.source / "report.json"))
    if report["mode"] != "sweep":
        raise ValueError("sweep required")
    result = compare_holds(review, json.loads(read_artifact(args.policy)))
    result["provenance"] = {
        k: report[k]
        for k in ("raw_sha256", "samples_sha256", "config_sha256", "native_sha256", "sdk_revision")
    }
    result["provenance"]["operator_context_sha256"] = digest(context_bytes)
    with args.output.open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")
    print(args.output)


if __name__ == "__main__":
    main()
