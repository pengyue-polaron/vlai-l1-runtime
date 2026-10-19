"""Analyse three sweeps in one owner; labels are explicit and hash-bound."""

import argparse
import csv
import io
import json
import math
from pathlib import Path

import numpy as np

from .automatic_review import review_automatic
from .configuration import MOTOR_NAMES, load_system_config
from .moving_window_study import evaluate, evaluate_loaded, extract_windows, fit_study
from .unloaded_baseline import digest, read_artifact
from .wrist_baseline_study import validate_context


def analyse(samples, phases, motion, policy, provenance, loaded_middle):
    stages = {}
    for phase in phases:
        name = phase["phase"]
        if name.startswith(("awaiting_", "settling_")):
            continue
        selected = samples[
            (samples[:, 0] >= phase["start_seconds"]) & (samples[:, 0] < phase["end_seconds"])
        ].copy()
        selected[:, 0] -= phase["start_seconds"]
        stages[name] = extract_windows(selected, motion, policy)
    try:
        model = fit_study(
            stages["baseline"],
            motion["active_motor"],
            {**provenance, "external_load_present": False},
            policy,
        )
    except ValueError as exc:
        return {
            "kind": "same_owner_paired_moving_study_rejected",
            "status": "rejected",
            "reason": str(exc),
            "ft_validated": False,
            "runtime_eligible": False,
            "static_supported": False,
            "middle_stage_loaded": loaded_middle,
            "provenance": provenance,
            "policy": policy,
            "extraction": stages,
        }
    return {
        "kind": "same_owner_paired_moving_study_v1",
        "status": "analysed",
        "ft_validated": False,
        "runtime_eligible": False,
        "static_supported": False,
        "frozen_before_loaded_acquisition": False,
        "meaning": "Fixed analysis procedure fits only initial unloaded first cycle, checks its second cycle, then evaluates later stages without refitting. All data acquired before analysis; this is not a prospectively frozen independent trial.",
        "middle_stage_loaded": loaded_middle,
        "model": model,
        "middle_evaluation": (evaluate_loaded if loaded_middle else evaluate)(
            model, stages["loaded"]["windows"]
        ),
        "empty_after_evaluation": evaluate(model, stages["empty_after"]["windows"]),
        "extraction": stages,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("config", "source", "context", "policy", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--recheck-offline-failure", action="store_true")
    args = parser.parse_args()
    config = load_system_config(args.config)
    review = review_automatic(
        config, args.source, recheck_offline_failure=args.recheck_offline_failure
    )
    report = json.loads(read_artifact(args.source / "report.json"))
    if report["mode"] != "paired" or report["policy"]["cycles"] < 2:
        raise ValueError("two-cycle paired capture required")
    context_bytes = read_artifact(args.context)
    context = json.loads(context_bytes)
    loaded = context.get("middle_stage_loaded")
    if type(loaded) is not bool or context.get("source_raw_sha256") != report["raw_sha256"]:
        raise ValueError("hash-bound explicit middle load label required")
    if context.get("initial_unloaded") is not True or context.get("final_unloaded") is not True:
        raise ValueError("explicit unloaded end stages required")
    statements = context.get("operator_statements", {})
    if set(statements) != {"initial", "middle", "final"} or any(
        not isinstance(v, str) or not v.strip() for v in statements.values()
    ):
        raise ValueError("actual stage statements required")
    validate_context(context["middle_context"], loaded=loaded)
    policy = json.loads(read_artifact(args.policy))
    active = MOTOR_NAMES.index(report["policy"]["active_motor"])
    resolution = math.degrees(2 * config.follower_feedback.right[active].position_max_rad / 65535)
    if policy["minimum_displacement_deg"] <= 2 * resolution:
        raise ValueError("motion threshold must exceed two encoder codes")
    rows = csv.DictReader(io.StringIO(read_artifact(args.source / "samples.csv").decode()))
    fields = ["elapsed_seconds"] + [p + n for p in ("q_", "torque_") for n in MOTOR_NAMES]
    samples = np.array([[float(r[k]) for k in fields] for r in rows])
    provenance = {
        k: report[k]
        for k in ("raw_sha256", "samples_sha256", "config_sha256", "native_sha256", "sdk_revision")
    }
    provenance["operator_context_sha256"] = digest(context_bytes)
    result = analyse(samples, review["paired_phases"], report["policy"], policy, provenance, loaded)
    result["capture_review"] = {
        key: review[key]
        for key in ("source_report_status", "source_report_sha256", "offline_failure_rechecked")
    }
    with args.output.open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")
    print(args.output)


if __name__ == "__main__":
    main()
