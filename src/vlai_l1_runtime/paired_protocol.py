"""Pure verification of the bounded, single-owner paired sweep protocol."""

import math
from itertools import pairwise


def paired_phases(report, timeout):
    events = [e for e in report.get("events", []) if e.get("state") == "phase"]
    names = ["baseline", "awaiting_attach", "loaded", "awaiting_remove", "empty_after", "complete"]
    handling = report["policy"].get("manual_handling")
    if handling:
        names = [
            "baseline",
            "awaiting_attach",
            "settling_loaded",
            "loaded",
            "awaiting_remove",
            "settling_empty_after",
            "empty_after",
            "complete",
        ]
    if [e.get("phase") for e in events] != names:
        raise ValueError("complete ordered paired phases required")
    if report.get("paired_acknowledgements") != ["attach.ready", "remove.ready"]:
        raise ValueError("paired operator acknowledgements required")
    t = [e.get("elapsed_seconds") for e in events]
    if (
        any(type(v) not in (int, float) or not math.isfinite(v) for v in t)
        or t[0] != 0
        or any(b <= a for a, b in pairwise(t))
    ):
        raise ValueError("ordered finite paired phase timing required")
    policy = report["policy"]
    duration = policy["hold_seconds"] + 4 * policy["cycles"] * (
        policy["transition_seconds"] + policy["hold_seconds"]
    )
    sweeps = (0, 3, 6) if handling else (0, 2, 4)
    if any(abs(t[i + 1] - t[i] - duration) > timeout for i in sweeps):
        raise ValueError("incomplete paired sweep duration")
    if handling and any(
        not handling["settle_seconds"] - 1e-6
        <= t[i + 1] - t[i]
        < handling["settle_timeout_seconds"]
        for i in (2, 5)
    ):
        raise ValueError("invalid paired settling duration")
    actual = report.get("completed_paired_seconds")
    # Phase telemetry has fewer significant digits than final completion.
    # This microsecond tolerance reconciles serialization, not missing motion.
    if (
        type(actual) not in (int, float)
        or not math.isfinite(actual)
        or not actual < policy["hold_test_seconds"]
        or not t[-1] < policy["hold_test_seconds"]
        or not math.isclose(actual, t[-1], rel_tol=0, abs_tol=1e-6)
    ):
        raise ValueError("invalid paired completion/cap")
    completed = [e for e in report["events"] if e.get("completion_kind") == "paired_sweep"]
    if (
        len(completed) != 1
        or completed[0].get("state") != "completed"
        or completed[0].get("disabled_verified") is not True
        or completed[0].get("elapsed_seconds") != actual
    ):
        raise ValueError("paired verified completion required")
    return [
        {"phase": names[i], "start_seconds": t[i], "end_seconds": t[i + 1]}
        for i in range(len(names) - 1)
    ]
