"""Offline J7 hold-torque study; no wrench calibration or runtime model export.

Fit one cycle and compare the next cycle, or evaluate a frozen study against
another explicitly labelled capture. Loaded captures can only be evaluated
against a frozen unloaded study, never fitted. Each hold has equal weight. Position and
direction are measured, never inferred from the commanded trajectory.
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
from .residual_model import ApproachHistory
from .unloaded_baseline import digest, read_artifact
from .wrench import ARM_JOINT_NAMES


def validate_policy(policy):
    expected = {
        "reversal_excursion_deg",
        "maximum_hold_span_deg",
        "maximum_other_joint_delta_deg",
        "position_support_margin_deg",
        "minimum_hold_samples",
        "maximum_feature_condition",
    }
    if type(policy) is not dict or set(policy) != expected:
        raise ValueError("exact wrist study policy required")
    for name, value in policy.items():
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"positive finite {name} required")
    if type(policy["minimum_hold_samples"]) is not int or policy["minimum_hold_samples"] < 2:
        raise ValueError("at least two hold samples required")
    if policy["maximum_feature_condition"] < 1:
        raise ValueError("invalid feature condition limit")


def validate_context(context, *, loaded=False):
    if (
        context.get("side") != "right"
        or context.get("external_load_present") is not loaded
        or context.get("follower_contact") is not False
        or context.get("tool_context") != "original_gripper_and_camera"
        or "sweep" not in context.get("applies_to_stages", [])
        or not context.get("operator_statement")
    ):
        label = "loaded" if loaded else "unloaded"
        raise ValueError(f"explicit {label}, untouched right-Follower context required")
    if loaded:
        payload = context.get("payload", {})
        mass = payload.get("mass_g")
        if (
            type(mass) not in (int, float)
            or not math.isfinite(mass)
            or mass <= 0
            or payload.get("mass_source") not in ("operator_weighed", "operator_nominal")
            or not payload.get("attachment_description")
            or payload.get("securely_attached") is not True
        ):
            raise ValueError("positive payload mass, source and confirmed attachment required")


def load_holds(config, source, context_path, policy, *, loaded=False):
    validate_policy(policy)
    context_bytes = read_artifact(context_path)
    context = json.loads(context_bytes)
    validate_context(context, loaded=loaded)
    review = review_automatic(config, source)
    capture = json.loads(read_artifact(source / "report.json"))
    if capture["policy"]["active_motor"] != "joint_7":
        raise ValueError("wrist baseline study requires a J7 sweep")
    if capture["mode"] != "sweep" or capture["policy"]["cycles"] < 2:
        raise ValueError("two-cycle sweep required")
    resolution = math.degrees(2 * config.follower_feedback.right[6].position_max_rad / 65535)
    if policy["reversal_excursion_deg"] <= 2 * resolution:
        raise ValueError("direction threshold must exceed encoder quantization")
    rows = list(csv.DictReader(io.StringIO(read_artifact(source / "samples.csv").decode())))
    history = ApproachHistory(
        reversal_excursion_rad=(math.radians(policy["reversal_excursion_deg"]),) * 7,
        maximum_gap_ns=round(config.teleoperation.state_timeout_s * 1e9),
    )
    times, directions = [], []
    for row in rows:
        times.append(float(row["elapsed_seconds"]))
        directions.append(
            history.update(
                joint_names=ARM_JOINT_NAMES,
                monotonic_ns=int(row["monotonic_ns"]),
                position_rad=tuple(
                    math.radians(float(row["q_" + name])) for name in ARM_JOINT_NAMES
                ),
            )[6]
        )
    times, directions = np.array(times), np.array(directions)
    holds = []
    for i, window in enumerate(review["hold_windows"][1:]):
        mask = (times >= window["start_seconds"]) & (times < window["end_seconds"])
        signs = set(directions[mask].tolist())
        if window["samples"] < policy["minimum_hold_samples"]:
            raise ValueError("insufficient hold samples")
        if len(signs) != 1 or 0 in signs:
            raise ValueError("unknown or changing measured approach direction in hold")
        if max(window["q_span_deg"]) > policy["maximum_hold_span_deg"]:
            raise ValueError("scheduled hold was not position-stable")
        holds.append(
            {
                "cycle": i // 4 + 1,
                "leg": i % 4,
                "start_seconds": window["start_seconds"],
                "q_deg": window["mean_q_deg"],
                "direction": signs.pop(),
                "torque_nm": window["mean_torque_nm"][6],
                "torque_std_nm": window["torque_std_nm"][6],
                "all_motor_torque_nm": window["mean_torque_nm"],
                "all_motor_torque_std_nm": window["torque_std_nm"],
            }
        )
    fixed = [0, 1, 2, 3, 4, 5, 7]
    positions = np.array([h["q_deg"] for h in holds])
    if np.any(np.ptp(positions[:, fixed], axis=0) > policy["maximum_other_joint_delta_deg"]):
        raise ValueError("non-wrist posture changed beyond the local study envelope")
    return holds, {
        "config_sha256": capture["config_sha256"],
        "sdk_revision": capture["sdk_revision"],
        "native_sha256": capture["native_sha256"],
        "raw_sha256": capture["raw_sha256"],
        "samples_sha256": capture["samples_sha256"],
        "operator_context_sha256": digest(context_bytes),
        "tool_context": context["tool_context"],
        "external_load_present": loaded,
        "payload": context.get("payload") if loaded else None,
        "source": str(source.absolute()),
    }


def _matrix(holds, center, kind):
    columns = [np.ones(len(holds))]
    if kind != "constant":
        columns.append(np.array([h["q_deg"][6] for h in holds]) - center)
    if kind == "angle_direction":
        columns.append(np.array([h["direction"] for h in holds]))
    return np.column_stack(columns)


def _scores(errors):
    return {
        "hold_count": len(errors),
        "rmse_nm": float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
        "maximum_abs_error_nm": float(np.max(np.abs(errors))) if len(errors) else None,
    }


def evaluate(study, holds):
    training, policy = study["training_holds"], study["policy"]
    fixed = [0, 1, 2, 3, 4, 5, 7]
    train_q = np.array([h["q_deg"] for h in training])
    support = []
    for hold in holds:
        q = np.array(hold["q_deg"])
        branch = [h["q_deg"][6] for h in training if h["direction"] == hold["direction"]]
        margin = policy["position_support_margin_deg"]
        context_limit = policy["maximum_other_joint_delta_deg"]
        support.append(
            bool(
                branch
                and min(branch) - margin <= q[6] <= max(branch) + margin
                and np.all(q[fixed] >= train_q[:, fixed].min(axis=0) - context_limit)
                and np.all(q[fixed] <= train_q[:, fixed].max(axis=0) + context_limit)
            )
        )
    mask = np.array(support, dtype=bool)
    target = np.array([h["torque_nm"] for h in holds])
    models = {}
    for kind, model in study["models"].items():
        predicted = _matrix(holds, study["center_deg"], kind) @ model["coefficients"]
        error = predicted - target
        models[kind] = {
            "supported": _scores(error[mask]),
            "outside_support_diagnostic_only": _scores(error[~mask]),
            "prediction_nm": predicted.tolist(),
            "error_nm": error.tolist(),
        }
    return {"supported_holds": support, "holds": holds, "models": models}


def fit_study(holds, provenance, policy):
    validate_policy(policy)
    if provenance.get("external_load_present", False) is not False:
        raise ValueError("loaded observations cannot train an unloaded study")
    training = [h for h in holds if h["cycle"] == 1]
    validation = [h for h in holds if h["cycle"] > 1]
    if len(training) != 4 or len(validation) < 4:
        raise ValueError("complete separated training and validation cycles required")
    center = float(np.mean([h["q_deg"][6] for h in training]))
    target = np.array([h["torque_nm"] for h in training])
    models = {}
    for kind in ("constant", "angle", "angle_direction"):
        matrix = _matrix(training, center, kind)
        coefficients, _, rank, singular = np.linalg.lstsq(matrix, target, rcond=None)
        if (
            rank != matrix.shape[1]
            or singular[0] / singular[-1] > policy["maximum_feature_condition"]
        ):
            raise ValueError("unidentifiable or ill-conditioned wrist features")
        models[kind] = {
            "coefficients": coefficients.tolist(),
            "feature_condition": float(singular[0] / singular[-1]),
            "training": _scores(matrix @ coefficients - target),
        }
    study = {
        "kind": "offline_wrist_hold_torque_study_v1",
        "ft_validated": False,
        "runtime_eligible": False,
        "target": "joint_7 raw motor torque in Nm; not external torque",
        "split": "first cycle fit; subsequent cycles temporal holdout within the same recording",
        "policy": policy,
        "provenance": provenance,
        "motor_names": list(MOTOR_NAMES),
        "center_deg": center,
        "training_holds": training,
        "models": models,
    }
    study["temporal_holdout"] = evaluate(study, validation)
    return study


def evaluate_loaded(study, holds):
    """Preserve load response; a nonzero residual is not an error against zero."""
    result = evaluate(study, holds)
    for model in result["models"].values():
        prediction = model.pop("prediction_nm")
        model.pop("error_nm")
        model.pop("supported")
        model.pop("outside_support_diagnostic_only")
        model["unloaded_prediction_nm"] = prediction
        residual = [h["torque_nm"] - p for h, p in zip(holds, prediction, strict=True)]
        model["supported_motor_torque_residual_nm"] = [
            r if supported else None
            for r, supported in zip(residual, result["supported_holds"], strict=True)
        ]
        model["outside_support_residual_diagnostic_only_nm"] = [
            None if supported else r
            for r, supported in zip(residual, result["supported_holds"], strict=True)
        ]
    result["interpretation"] = (
        "measured raw motor torque minus frozen unloaded prediction; "
        "includes load response and model error, not calibrated external torque or FT"
    )
    result["ground_truth_joint_torque_nm"] = None
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("config", "source", "context", "policy", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--frozen-study", type=Path)
    parser.add_argument("--loaded-validation", action="store_true")
    args = parser.parse_args()
    if args.loaded_validation and args.frozen_study is None:
        parser.error("--loaded-validation requires --frozen-study; loaded fitting is forbidden")
    policy = json.loads(read_artifact(args.policy))
    holds, provenance = load_holds(
        load_system_config(args.config),
        args.source,
        args.context,
        policy,
        loaded=args.loaded_validation,
    )
    if args.frozen_study is None:
        result = fit_study(holds, provenance, policy)
    else:
        frozen_bytes = read_artifact(args.frozen_study)
        study = json.loads(frozen_bytes)
        if study.get("kind") != "offline_wrist_hold_torque_study_v1" or study["policy"] != policy:
            raise ValueError("frozen study kind/policy mismatch")
        if study["provenance"].get("external_load_present", False) is not False:
            raise ValueError("frozen study must be unloaded")
        for key in ("config_sha256", "sdk_revision", "native_sha256", "tool_context"):
            if study["provenance"][key] != provenance[key]:
                raise ValueError(f"independent capture context mismatch: {key}")
        if study["provenance"]["raw_sha256"] == provenance["raw_sha256"]:
            raise ValueError("independent validation must use another recording")
        result = {
            "kind": (
                "frozen_wrist_loaded_response_v1"
                if args.loaded_validation
                else "frozen_wrist_study_validation_v1"
            ),
            "ft_validated": False,
            "runtime_eligible": False,
            "frozen_study_sha256": digest(frozen_bytes),
            "provenance": provenance,
            "evaluation": (
                evaluate_loaded(study, holds) if args.loaded_validation else evaluate(study, holds)
            ),
        }
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"Offline wrist study saved: {args.output}")


if __name__ == "__main__":
    main()
