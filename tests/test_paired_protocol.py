import copy

import pytest

from vlai_l1_runtime.paired_protocol import paired_phases


def report():
    names = ["baseline", "awaiting_attach", "loaded", "awaiting_remove", "empty_after", "complete"]
    events = [
        {"state": "phase", "phase": n, "elapsed_seconds": t}
        for n, t in zip(names, [0, 41, 50, 91, 100, 141], strict=True)
    ]
    events.append(
        {
            "state": "completed",
            "completion_kind": "paired_sweep",
            "disabled_verified": True,
            "elapsed_seconds": 141,
        }
    )
    return {
        "events": events,
        "policy": {
            "hold_seconds": 5,
            "transition_seconds": 4,
            "cycles": 1,
            "hold_test_seconds": 200,
        },
        "paired_acknowledgements": ["attach.ready", "remove.ready"],
        "completed_paired_seconds": 141,
    }


def test_valid_protocol_has_three_sweeps_and_two_stationary_waits():
    phases = paired_phases(report(), 0.1)
    assert [p["phase"] for p in phases] == [
        "baseline",
        "awaiting_attach",
        "loaded",
        "awaiting_remove",
        "empty_after",
    ]


def test_phase_serialization_roundoff_is_not_incomplete_acquisition():
    r = report()
    r["completed_paired_seconds"] = r["events"][-1]["elapsed_seconds"] = 141.000000002
    assert len(paired_phases(r, 0.1)) == 5
    r["completed_paired_seconds"] = r["events"][-1]["elapsed_seconds"] = 141.00001
    with pytest.raises(ValueError, match="completion/cap"):
        paired_phases(r, 0.1)


@pytest.mark.parametrize("bad", [None, "missing", "short", "timeout"])
def test_manual_protocol_requires_both_settling_stages(bad):
    r = report()
    r["policy"]["manual_handling"] = {"settle_seconds": 2, "settle_timeout_seconds": 20}
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
    times = [0, 41, 50, 52, 93, 100, 102, 143]
    if bad == "short":
        times[2] = 51
    if bad == "timeout":
        times = [0, 41, 50, 70, 111, 120, 122, 163]
    r["events"] = [
        {"state": "phase", "phase": n, "elapsed_seconds": t}
        for n, t in zip(names, times, strict=True)
    ] + [r["events"][-1]]
    r["events"][-1]["elapsed_seconds"] = r["completed_paired_seconds"] = times[-1]
    if bad == "missing":
        del r["events"][2]
    if bad:
        with pytest.raises(ValueError, match="paired"):
            paired_phases(r, 0.1)
    else:
        assert len(paired_phases(r, 0.1)) == 7


@pytest.mark.parametrize("bad", ["order", "missing_ack", "timing", "cap", "completion", "nan"])
def test_invalid_protocol_rejected(bad):
    r = copy.deepcopy(report())
    if bad == "order":
        r["events"][2]["phase"] = "baseline"
    if bad == "missing_ack":
        r["paired_acknowledgements"] = []
    if bad == "timing":
        r["events"][1]["elapsed_seconds"] = 30
    if bad == "cap":
        r["policy"]["hold_test_seconds"] = 100
    if bad == "completion":
        r["events"][-1]["disabled_verified"] = False
    if bad == "nan":
        r["events"][1]["elapsed_seconds"] = float("nan")
    with pytest.raises(ValueError, match=r"paired|acknowledgements"):
        paired_phases(r, 0.1)
