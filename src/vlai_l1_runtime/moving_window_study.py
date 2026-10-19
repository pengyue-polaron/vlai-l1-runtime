"""Unloaded moving-window torque study. Static states never receive predictions."""

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
from .wrist_baseline_study import validate_context


def validate_policy(policy):
    fields = {
        "window_seconds",
        "minimum_window_samples",
        "minimum_displacement_deg",
        "maximum_total_variation_ratio",
        "maximum_other_joint_span_deg",
        "minimum_training_windows",
        "minimum_windows_per_direction",
        "maximum_feature_condition",
        "position_support_margin_deg",
        "maximum_other_joint_delta_deg",
    }
    if set(policy) != fields or any(
        type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in policy.values()
    ):
        raise ValueError("exact positive finite moving-window policy required")
    for name in (
        "minimum_window_samples",
        "minimum_training_windows",
        "minimum_windows_per_direction",
    ):
        if type(policy[name]) is not int or policy[name] < 2:
            raise ValueError("integer sample/window minimum >= 2 required")
    if policy["maximum_total_variation_ratio"] < 1 or policy["maximum_feature_condition"] < 1:
        raise ValueError("invalid variation/condition limit")


def extract_windows(samples, motion, policy):
    """Input columns: elapsed seconds, eight measured q, eight motor torques."""
    validate_policy(policy)
    x = np.asarray(samples, dtype=float)
    if (
        x.ndim != 2
        or x.shape[1] != 17
        or len(x) < 2
        or not np.isfinite(x).all()
        or np.any(np.diff(x[:, 0]) <= 0)
    ):
        raise ValueError("ordered finite time/q/torque samples required")
    if motion["active_motor"] not in ("joint_4", "joint_6", "joint_7"):
        raise ValueError("supported active joint required")
    active = MOTOR_NAMES.index(motion["active_motor"])
    fixed = [i for i in range(8) if i != active]
    duration = motion["transition_seconds"]
    blocks = duration / policy["window_seconds"]
    if not math.isclose(blocks, round(blocks)) or blocks < 1:
        raise ValueError("whole windows per transition required")
    windows, rejected = [], []
    for leg in range(4 * motion["cycles"]):
        start = motion["hold_seconds"] + leg * (duration + motion["hold_seconds"])
        for b in range(round(blocks)):
            lo = start + b * policy["window_seconds"]
            hi = lo + policy["window_seconds"]
            part = x[(x[:, 0] >= lo) & (x[:, 0] < hi)]
            base = {"cycle": leg // 4 + 1, "leg": leg % 4, "start_seconds": lo, "end_seconds": hi}
            reason = None
            if len(part) < policy["minimum_window_samples"]:
                reason = "insufficient_samples"
            else:
                q = part[:, 1:9]
                delta = q[-1, active] - q[0, active]
                travel = np.abs(np.diff(q[:, active])).sum()
                if abs(delta) < policy["minimum_displacement_deg"]:
                    reason = "insufficient_measured_motion"
                elif travel > policy["maximum_total_variation_ratio"] * abs(delta):
                    reason = "reversal_or_oscillation"
                elif max(np.ptp(q[:, fixed], axis=0)) > policy["maximum_other_joint_span_deg"]:
                    reason = "other_joint_motion"
            if reason:
                rejected.append({**base, "reason": reason})
                continue
            windows.append(
                {
                    **base,
                    "samples": len(part),
                    "q_deg": q.mean(axis=0).tolist(),
                    "direction": 1 if delta > 0 else -1,
                    "displacement_deg": float(delta),
                    "total_variation_deg": float(travel),
                    "mean_speed_deg_s": float(delta / (part[-1, 0] - part[0, 0])),
                    "torque_nm": float(part[:, 9 + active].mean()),
                    "torque_std_nm": float(part[:, 9 + active].std()),
                }
            )
    return {"windows": windows, "rejected_windows": rejected, "static_holds_excluded": True}


def matrix(windows, active, center, kind):
    cols = [np.ones(len(windows))]
    if kind != "constant":
        cols.append(np.array([w["q_deg"][active] - center for w in windows]))
    if kind == "angle_direction":
        cols.append(np.array([w["direction"] for w in windows]))
    return np.column_stack(cols)


def evaluate(study, windows):
    if not windows:
        raise ValueError("moving validation windows required")
    active = MOTOR_NAMES.index(study["active_motor"])
    train, policy = study["training_windows"], study["policy"]
    train_q = np.array([w["q_deg"] for w in train])
    support, diagnostics = [], []
    for w in windows:
        branch = [h["q_deg"][active] for h in train if h["direction"] == w["direction"]]
        q = np.array(w["q_deg"])
        violations = []
        for i in range(8):
            if i == active:
                if not branch:
                    continue
                lo = min(branch) - policy["position_support_margin_deg"]
                hi = max(branch) + policy["position_support_margin_deg"]
            else:
                lo = float(train_q[:, i].min()) - policy["maximum_other_joint_delta_deg"]
                hi = float(train_q[:, i].max()) + policy["maximum_other_joint_delta_deg"]
            if not lo <= q[i] <= hi:
                violations.append(
                    {
                        "motor": MOTOR_NAMES[i],
                        "measured_deg": float(q[i]),
                        "supported_min_deg": lo,
                        "supported_max_deg": hi,
                        "distance_outside_deg": float(max(lo - q[i], q[i] - hi)),
                    }
                )
        support.append(bool(branch) and not violations)
        diagnostics.append({"direction_supported": bool(branch), "pose_violations": violations})
    mask = np.array(support)
    result = {
        "supported_windows": support,
        "support_diagnostics": diagnostics,
        "windows": windows,
        "models": {},
    }
    for kind, model in study["models"].items():
        prediction = matrix(windows, active, study["center_deg"], kind) @ model["coefficients"]
        error = prediction - np.array([w["torque_nm"] for w in windows])
        result["models"][kind] = {
            "prediction_nm": prediction.tolist(),
            "error_nm": error.tolist(),
            "supported_rmse_nm": float(np.sqrt(np.mean(error[mask] ** 2))) if mask.any() else None,
            "supported_maximum_abs_error_nm": float(max(abs(error[mask]))) if mask.any() else None,
        }
    return result


def fit_study(extracted, active_motor, provenance, policy):
    validate_policy(policy)
    if provenance.get("external_load_present") is not False:
        raise ValueError("explicitly unloaded training required")
    active = MOTOR_NAMES.index(active_motor)
    train = [w for w in extracted["windows"] if w["cycle"] == 1]
    validation = [w for w in extracted["windows"] if w["cycle"] > 1]
    if len(train) < policy["minimum_training_windows"] or any(
        sum(w["direction"] == d for w in train) < policy["minimum_windows_per_direction"]
        for d in (-1, 1)
    ):
        raise ValueError("insufficient bidirectional moving training windows")
    center = float(np.mean([w["q_deg"][active] for w in train]))
    study = {
        "kind": "offline_moving_torque_study_v1",
        "active_motor": active_motor,
        "ft_validated": False,
        "runtime_eligible": False,
        "static_supported": False,
        "provenance": provenance,
        "policy": policy,
        "center_deg": center,
        "training_windows": train,
        "models": {},
        "meaning": "Unloaded raw motor torque during measured motion; no acceleration/gravity calibration, load validation or FT claim. Command/error/torque feedback are never predictor features.",
    }
    target = np.array([w["torque_nm"] for w in train])
    for kind in ("constant", "angle", "angle_direction"):
        design = matrix(train, active, center, kind)
        coefficient, _, rank, singular = np.linalg.lstsq(design, target, rcond=None)
        condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else float("inf")
        if rank != design.shape[1] or condition > policy["maximum_feature_condition"]:
            raise ValueError("unidentifiable moving-window features")
        study["models"][kind] = {
            "coefficients": coefficient.tolist(),
            "feature_condition": condition,
        }
    study["temporal_holdout"] = evaluate(study, validation)
    study["rejected_windows"] = extracted["rejected_windows"]
    return study


def validate_frozen(study, active_motor, provenance, policy, *, loaded=False):
    if (
        study.get("kind") != "offline_moving_torque_study_v1"
        or study.get("active_motor") != active_motor
        or study.get("policy") != policy
    ):
        raise ValueError("frozen kind/joint/policy mismatch")
    if (
        provenance.get("external_load_present") is not loaded
        or study["provenance"].get("external_load_present") is not False
    ):
        raise ValueError("unloaded study and explicit validation load label required")
    for key in ("config_sha256", "native_sha256", "sdk_revision", "tool_context"):
        if study["provenance"].get(key) is None or study["provenance"][key] != provenance.get(key):
            raise ValueError(f"independent context mismatch: {key}")
    if study["provenance"].get("raw_sha256") == provenance.get("raw_sha256"):
        raise ValueError("independent validation needs a different recording")


def evaluate_loaded(study, windows):
    result = evaluate(study, windows)
    for model in result["models"].values():
        prediction = model.pop("prediction_nm")
        model.pop("error_nm")
        model.pop("supported_rmse_nm")
        model.pop("supported_maximum_abs_error_nm")
        residual = [w["torque_nm"] - p for w, p in zip(windows, prediction, strict=True)]
        model["unloaded_prediction_nm"] = prediction
        model["supported_motor_torque_residual_nm"] = [
            r if ok else None for r, ok in zip(residual, result["supported_windows"], strict=True)
        ]
        model["outside_support_residual_diagnostic_only_nm"] = [
            None if ok else r for r, ok in zip(residual, result["supported_windows"], strict=True)
        ]
    result["ground_truth_joint_torque_nm"] = None
    result["meaning"] = (
        "Loaded raw motor torque minus frozen unloaded prediction; contains load response and model error, not calibrated external torque. No error against zero or mass accuracy is claimed."
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("config", "source", "context", "policy", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--frozen-study", type=Path)
    parser.add_argument("--loaded-validation", action="store_true")
    args = parser.parse_args()
    if args.loaded_validation and args.frozen_study is None:
        parser.error("loaded validation requires frozen study; loaded fitting is forbidden")
    context_bytes = read_artifact(args.context)
    context = json.loads(context_bytes)
    validate_context(context, loaded=args.loaded_validation)
    config = load_system_config(args.config)
    review_automatic(config, args.source)
    report = json.loads(read_artifact(args.source / "report.json"))
    if report["mode"] != "sweep" or report["policy"]["cycles"] < 2:
        raise ValueError("two-cycle sweep required")
    active = MOTOR_NAMES.index(report["policy"]["active_motor"])
    policy = json.loads(read_artifact(args.policy))
    resolution = math.degrees(2 * config.follower_feedback.right[active].position_max_rad / 65535)
    if policy["minimum_displacement_deg"] <= 2 * resolution:
        raise ValueError("motion threshold must exceed two encoder codes")
    rows = csv.DictReader(io.StringIO(read_artifact(args.source / "samples.csv").decode()))
    fields = ["elapsed_seconds"] + [p + n for p in ("q_", "torque_") for n in MOTOR_NAMES]
    extracted = extract_windows(
        [[float(r[k]) for k in fields] for r in rows], report["policy"], policy
    )
    provenance = {
        k: report[k]
        for k in ("raw_sha256", "samples_sha256", "native_sha256", "config_sha256", "sdk_revision")
    }
    provenance.update(
        external_load_present=args.loaded_validation,
        operator_context_sha256=digest(context_bytes),
        tool_context=context["tool_context"],
        payload=context.get("payload") if args.loaded_validation else None,
    )
    if args.frozen_study:
        frozen_bytes = read_artifact(args.frozen_study)
        frozen = json.loads(frozen_bytes)
        validate_frozen(
            frozen,
            report["policy"]["active_motor"],
            provenance,
            policy,
            loaded=args.loaded_validation,
        )
        study = {
            "kind": "frozen_moving_loaded_response_v1"
            if args.loaded_validation
            else "independent_moving_torque_validation_v1",
            "ft_validated": False,
            "runtime_eligible": False,
            "static_supported": False,
            "provenance": provenance,
            "frozen_study_sha256": digest(frozen_bytes),
            "evaluation": evaluate_loaded(frozen, extracted["windows"])
            if args.loaded_validation
            else evaluate(frozen, extracted["windows"]),
            "rejected_windows": extracted["rejected_windows"],
        }
    else:
        try:
            study = fit_study(extracted, report["policy"]["active_motor"], provenance, policy)
        except ValueError as exc:
            study = {
                "kind": "moving_torque_study_rejected",
                "reason": str(exc),
                "ft_validated": False,
                "runtime_eligible": False,
                "provenance": provenance,
                "policy": policy,
                **extracted,
            }
    with args.output.open("x") as f:
        json.dump(study, f, indent=2, allow_nan=False)
        f.write("\n")
    print(study["kind"], args.output)


if __name__ == "__main__":
    main()
