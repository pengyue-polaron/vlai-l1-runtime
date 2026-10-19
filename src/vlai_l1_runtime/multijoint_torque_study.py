"""Offline window-mean raw torque study; never an instantaneous FT estimator.

Compare fixed ridge baselines using measured pose, window velocity and causal
approach history. Entire transition/hold groups are left out, with train-only
scaling. One recording's grouped scores remain exploratory. No target position,
tracking error, torque history, or future samples enter predictor features.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
from itertools import pairwise
from pathlib import Path

import numpy as np

from .automatic_review import review_automatic
from .automatic_trajectory import is_waypoint_policy, path_offsets
from .configuration import MOTOR_NAMES, load_system_config
from .residual_model import ApproachHistory
from .unloaded_baseline import digest, read_artifact
from .wrench import ARM_JOINT_NAMES
from .wrist_baseline_study import validate_context

KINDS = ("constant", "pose", "pose_velocity", "pose_velocity_history")


def validate_policy(policy):
    fields = {
        "window_seconds",
        "minimum_window_samples",
        "hold_tail_seconds",
        "maximum_hold_span_deg",
        "reversal_excursion_deg",
        "maximum_sample_gap_seconds",
        "ridge_penalty",
        "minimum_training_windows",
        "maximum_nearest_pose_distance",
        "position_box_margin_deg",
        "fixed_joint_margin_deg",
        "gripper_margin_deg",
    }
    if set(policy) != fields or any(
        type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in policy.values()
    ):
        raise ValueError("exact positive finite study policy required")
    for key in ("minimum_window_samples", "minimum_training_windows"):
        if type(policy[key]) is not int or policy[key] < 2:
            raise ValueError("integer sample/window minima required")


def extract_windows(samples, motion, policy):
    """Columns: time seconds, measured q[8] degrees, motor torque[8] Nm.

    Each feature summarizes only the indicated window and its preceding
    measured history. The output is available at window end, not window start.
    """
    validate_policy(policy)
    x = np.asarray(samples, dtype=float)
    if (
        x.ndim != 2
        or x.shape[1] != 17
        or len(x) < 2
        or not np.isfinite(x).all()
        or np.any(np.diff(x[:, 0]) <= 0)
        or x[0, 0] < 0
        or max(np.diff(x[:, 0])) > policy["maximum_sample_gap_seconds"]
    ):
        raise ValueError("finite ordered gap-free time/q/torque required")
    if not is_waypoint_policy(motion) or motion["active_motor"] != "arm":
        raise ValueError("multi-joint waypoint sweep required")
    points = path_offsets(motion)
    if any(p[0] or p[7] for p in points):
        raise ValueError("study requires fixed J1 and gripper references")
    width, tail = policy["window_seconds"], policy["hold_tail_seconds"]
    if tail > motion["hold_seconds"] or any(
        not math.isclose(d / width, round(d / width)) for d in (tail, motion["transition_seconds"])
    ):
        raise ValueError("whole windows and a bounded hold tail required")
    history = ApproachHistory(
        reversal_excursion_rad=(math.radians(policy["reversal_excursion_deg"]),) * 7,
        maximum_gap_ns=round(policy["maximum_sample_gap_seconds"] * 1e9),
    )
    signs = np.array(
        [
            history.update(
                joint_names=ARM_JOINT_NAMES,
                monotonic_ns=1 + round(row[0] * 1e9),
                position_rad=tuple(np.deg2rad(row[1:8]).tolist()),
            )
            for row in x
        ]
    )
    hold, transition = motion["hold_seconds"], motion["transition_seconds"]
    intervals = [(-1, "hold", hold - tail, hold)]
    for group in range((len(points) - 1) * motion["cycles"]):
        start = hold + group * (transition + hold)
        intervals.extend(
            [
                (group, "transition", start, start + transition),
                (group, "hold", start + transition + hold - tail, start + transition + hold),
            ]
        )
    windows, rejected = [], []
    for group, phase, start, end in intervals:
        for block in range(round((end - start) / width)):
            lo = start + block * width
            mask = (x[:, 0] >= lo) & (x[:, 0] < lo + width)
            part = x[mask]
            base = {"group": group, "phase": phase, "start_seconds": lo, "end_seconds": lo + width}
            reason = None
            if len(part) < policy["minimum_window_samples"]:
                reason = "insufficient_samples"
            elif part[-1, 0] - part[0, 0] < width - 2 * policy["maximum_sample_gap_seconds"]:
                reason = "incomplete_window"
            elif (
                phase == "hold"
                and max(np.ptp(part[:, 1:9], axis=0)) > policy["maximum_hold_span_deg"]
            ):
                reason = "hold_not_settled"
            if reason:
                rejected.append({**base, "reason": reason})
                continue
            windows.append(
                {
                    **base,
                    "samples": len(part),
                    "q_deg": part[:, 1:9].mean(axis=0).tolist(),
                    "velocity_deg_s": (
                        (part[-1, 1:8] - part[0, 1:8]) / (part[-1, 0] - part[0, 0])
                    ).tolist(),
                    "history": signs[mask].mean(axis=0).tolist(),
                    "torque_nm": part[:, 9:16].mean(axis=0).tolist(),
                    "torque_std_nm": part[:, 9:16].std(axis=0).tolist(),
                }
            )
    return {"windows": windows, "rejected_windows": rejected}


def features(windows, kind):
    if kind not in KINDS:
        raise ValueError("unknown baseline")
    parts = []
    if kind != "constant":
        parts.append(np.array([w["q_deg"][1:7] for w in windows]))
    if kind in ("pose_velocity", "pose_velocity_history"):
        parts.append(np.array([w["velocity_deg_s"][1:7] for w in windows]))
    if kind == "pose_velocity_history":
        parts.append(np.array([w["history"][1:7] for w in windows]))
    return np.column_stack(parts) if parts else np.zeros((len(windows), 0))


def fit(windows, policy):
    validate_policy(policy)
    if len(windows) < policy["minimum_training_windows"]:
        raise ValueError("insufficient training windows")
    y = np.array([w["torque_nm"] for w in windows])
    models = {}
    for kind in KINDS:
        raw = features(windows, kind)
        center, scale = raw.mean(axis=0), raw.std(axis=0)
        scale = np.where(scale > 1e-9, scale, 1.0)
        design = np.column_stack([np.ones(len(windows)), (raw - center) / scale])
        penalty = np.eye(design.shape[1]) * policy["ridge_penalty"]
        penalty[0, 0] = 0
        coefficient = np.linalg.solve(design.T @ design / len(y) + penalty, design.T @ y / len(y))
        singular = np.linalg.svd(design, compute_uv=False)
        models[kind] = {
            "center": center.tolist(),
            "scale": scale.tolist(),
            "coefficients": coefficient.tolist(),
            "design_rank": int(np.linalg.matrix_rank(design)),
            "feature_count_with_intercept": design.shape[1],
            "design_condition": float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else None,
        }
    return {"models": models, "training_windows": windows, "policy": policy}


def scores(error):
    return (
        {
            "windows": len(error),
            "rmse_nm": np.sqrt(np.mean(error**2, axis=0)).tolist(),
            "maximum_abs_error_nm": np.max(np.abs(error), axis=0).tolist(),
        }
        if len(error)
        else None
    )


def evaluate(frozen, windows):
    train, policy = frozen["training_windows"], frozen["policy"]
    q = np.array([w["q_deg"] for w in train])
    test_q = np.array([w["q_deg"] for w in windows])
    scale = np.maximum(q[:, 1:7].std(axis=0), policy["position_box_margin_deg"])
    distance = np.sqrt(np.sum(((test_q[:, None, 1:7] - q[None, :, 1:7]) / scale) ** 2, axis=2))
    margin = np.array(
        [policy["fixed_joint_margin_deg"]]
        + [policy["position_box_margin_deg"]] * 6
        + [policy["gripper_margin_deg"]]
    )
    support = (distance.min(axis=1) <= policy["maximum_nearest_pose_distance"]) & np.all(
        (test_q >= q.min(axis=0) - margin) & (test_q <= q.max(axis=0) + margin), axis=1
    )
    result = {
        "pose_supported": support.tolist(),
        "nearest_pose_distance": distance.min(axis=1).tolist(),
        "support_meaning": "Geometric proximity only; does not certify velocity/history coverage.",
        "models": {},
    }
    y = np.array([w["torque_nm"] for w in windows])
    for kind, model in frozen["models"].items():
        raw = features(windows, kind)
        design = np.column_stack([np.ones(len(windows)), (raw - model["center"]) / model["scale"]])
        prediction = design @ model["coefficients"]
        error = prediction - y
        result["models"][kind] = {
            "prediction_nm": prediction.tolist(),
            "error_nm": error.tolist(),
            "all_windows_including_extrapolation": scores(error),
            "pose_supported_windows": scores(error[support]),
            "by_phase": {
                phase: scores(error[[w["phase"] == phase for w in windows]])
                for phase in ("transition", "hold")
            },
        }
    return result


def build_study(extracted, provenance, policy):
    if provenance.get("external_load_present") is not False:
        raise ValueError("explicit unloaded training required")
    windows = extracted["windows"]
    groups = sorted({w["group"] for w in windows})
    if len(groups) < 4:
        raise ValueError("at least four complete motion/hold groups required")
    folds = []
    for group in groups:
        training = [w for w in windows if w["group"] != group]
        validation = [w for w in windows if w["group"] == group]
        folds.append(
            {
                "held_out_group": group,
                "windows": validation,
                "evaluation": evaluate(fit(training, policy), validation),
            }
        )
    frozen = fit(windows, policy)
    aggregate = {}
    for kind in KINDS:
        error = np.concatenate([f["evaluation"]["models"][kind]["error_nm"] for f in folds])
        support = np.concatenate([f["evaluation"]["pose_supported"] for f in folds])
        phases = [w["phase"] for f in folds for w in f["windows"]]
        aggregate[kind] = {
            "all_windows_including_extrapolation": scores(error),
            "pose_supported_windows": scores(error[support]),
            "by_phase": {p: scores(error[np.array(phases) == p]) for p in ("transition", "hold")},
        }
    return {
        **frozen,
        "kind": "offline_multijoint_torque_study_v1",
        "provenance": provenance,
        "ft_validated": False,
        "runtime_eligible": False,
        "independent_validation": False,
        "target": "one-second mean unloaded raw motor torque J1-J7 in Nm",
        "limitations": "One capture; group holdouts are exploratory, not independent experiments. Ridge coefficients are not physical parameters. No acceleration, calibrated gravity, mass or wrench claim. Geometric support does not establish history coverage. J1 is a fixed-context output, not an independently excited input.",
        "rejected_windows": extracted["rejected_windows"],
        "grouped_folds": folds,
        "grouped_scores": aggregate,
        "training_evaluation": evaluate(frozen, windows),
    }


def validate_frozen(frozen, provenance, policy):
    if frozen.get("kind") != "offline_multijoint_torque_study_v1" or frozen["policy"] != policy:
        raise ValueError("frozen study/policy mismatch")
    if (
        provenance.get("external_load_present") is not False
        or frozen["provenance"].get("external_load_present") is not False
    ):
        raise ValueError("only explicitly unloaded validation supported")
    for key in ("config_sha256", "native_sha256", "sdk_revision", "tool_context"):
        if not provenance.get(key) or provenance[key] != frozen["provenance"].get(key):
            raise ValueError(f"independent context mismatch: {key}")
    for key in ("raw_sha256", "samples_sha256"):
        if not provenance.get(key) or provenance[key] == frozen["provenance"].get(key):
            raise ValueError("independent validation requires a different recording")


def group_geometric_segments(extracted, motion):
    """Group repeated edges together regardless of direction or cycle.

    Scheduling geometry is used ONLY to define the validation partition, never
    as a predictor. The initial hold retains its own group. Endpoint holds can
    still neighbor another edge; this remains exploratory within-recording CV.
    """
    if not is_waypoint_policy(motion):
        raise ValueError("waypoint geometry required")
    points = path_offsets(motion)
    edges = [tuple(sorted((a, b))) for a, b in pairwise(points)]
    identities = {edge: i for i, edge in enumerate(dict.fromkeys(edges))}
    mapping = {i: identities[edges[i % len(edges)]] for i in range(len(edges) * motion["cycles"])}
    mapping[-1] = -1
    if any(type(w["group"]) is not int or w["group"] not in mapping for w in extracted["windows"]):
        raise ValueError("window group outside scheduled path")
    return {
        "windows": [
            {**w, "temporal_group": w["group"], "group": mapping[w["group"]]}
            for w in extracted["windows"]
        ],
        "rejected_windows": extracted["rejected_windows"],
        "temporal_to_geometric_group": mapping,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("config", "source", "context", "policy", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--frozen-study", type=Path)
    args = parser.parse_args()
    context_bytes = read_artifact(args.context)
    context = json.loads(context_bytes)
    validate_context(context)
    config = load_system_config(args.config)
    review_automatic(config, args.source)
    report = json.loads(read_artifact(args.source / "report.json"))
    if report["mode"] != "sweep":
        raise ValueError("sweep required")
    for key in ("raw_sha256", "samples_sha256"):
        if context.get(key) != report[key]:
            raise ValueError("operator context must bind both recording hashes")
    policy = json.loads(read_artifact(args.policy))
    validate_policy(policy)
    resolution = max(
        math.degrees(2 * m.position_max_rad / 65535) for m in config.follower_feedback.right
    )
    if policy["reversal_excursion_deg"] <= 2 * resolution:
        raise ValueError("history threshold must exceed two encoder codes")
    rows = csv.DictReader(io.StringIO(read_artifact(args.source / "samples.csv").decode()))
    fields = ["elapsed_seconds"] + [p + n for p in ("q_", "torque_") for n in MOTOR_NAMES]
    extracted = extract_windows(
        [[float(r[k]) for k in fields] for r in rows], report["policy"], policy
    )
    provenance = {
        k: report[k]
        for k in ("raw_sha256", "samples_sha256", "config_sha256", "native_sha256", "sdk_revision")
    }
    provenance.update(
        external_load_present=False,
        tool_context=context["tool_context"],
        operator_context_sha256=digest(context_bytes),
        source=str(args.source.absolute()),
    )
    if args.frozen_study:
        frozen_bytes = read_artifact(args.frozen_study)
        frozen = json.loads(frozen_bytes)
        validate_frozen(frozen, provenance, policy)
        result = {
            "kind": "independent_multijoint_torque_validation_v1",
            "provenance": provenance,
            "frozen_study_sha256": digest(frozen_bytes),
            "ft_validated": False,
            "runtime_eligible": False,
            **extracted,
            "evaluation": evaluate(frozen, extracted["windows"]),
        }
    else:
        result = build_study(extracted, provenance, policy)
        grouped = group_geometric_segments(extracted, report["policy"])
        if len({w["group"] for w in grouped["windows"]}) < len(
            {w["group"] for w in extracted["windows"]}
        ):
            geometric = build_study(grouped, provenance, policy)
            result["geometric_grouped_validation"] = {
                "meaning": "Both directions and repeats of each geometric edge are held out together. Same recording, shared neighboring endpoints, not independent validation. Does not select or retune the frozen models.",
                "temporal_to_geometric_group": grouped["temporal_to_geometric_group"],
                "grouped_folds": geometric["grouped_folds"],
                "grouped_scores": geometric["grouped_scores"],
            }
    with args.output.open("x") as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    print(result["kind"], args.output)


if __name__ == "__main__":
    main()
