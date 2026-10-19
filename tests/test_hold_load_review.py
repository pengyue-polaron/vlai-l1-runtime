import copy

import numpy as np
import pytest

from vlai_l1_runtime.hold_load_review import compare_windows

POLICY = {"minimum_samples": 400, "minimum_seconds": 5, "maximum_window_span_deg": 0.1}


def fixture():
    samples, windows = [], []
    for index, (label, loaded) in enumerate(
        zip(("empty_before", "loaded", "empty_after"), (False, True, False), strict=True)
    ):
        start = (index * 10 + 1) * 1_000_000_000
        q = [0] * 6 + [0.2 if loaded else 0, 0]
        tau = [index * 0.01 + (0.12 if loaded else 0)] * 8
        samples.extend([[start + tick * 10_000_000, *q, *tau] for tick in range(601)])
        windows.append(
            {
                "label": label,
                "external_load_present": loaded,
                "operator_statement": "hands clear; load state confirmed",
                "start_monotonic_ns": start,
                "end_monotonic_ns": start + 6_000_000_000,
            }
        )
    return samples, windows


def test_recovers_synthetic_load_response_without_hiding_posture_change():
    samples, windows = fixture()
    result = compare_windows(samples, windows, POLICY)
    assert result["loaded_minus_interpolated_empty_torque_nm"] == pytest.approx([0.12] * 8)
    assert result["loaded_minus_interpolated_empty_q_deg"][6] == pytest.approx(0.2)
    assert result["empty_return_torque_difference_nm"] == pytest.approx([0.02] * 8)
    assert result["pose_compensated"] is False
    assert result["ft_validated"] is False


@pytest.mark.parametrize("bad", ["load", "overlap", "short", "outside", "moving", "nan"])
def test_invalid_measurement_window_rejected(bad):
    samples, windows = fixture()
    samples = np.array(samples)
    if bad == "load":
        windows[1]["external_load_present"] = 1
    elif bad == "overlap":
        windows[1]["start_monotonic_ns"] = windows[0]["start_monotonic_ns"]
    elif bad == "short":
        windows[1]["end_monotonic_ns"] = windows[1]["start_monotonic_ns"] + 1_000_000_000
    elif bad == "outside":
        windows[2]["end_monotonic_ns"] += 1_000_000_000
    elif bad == "moving":
        samples[800, 7] += 1
    else:
        samples[800, 7] = float("nan")
    with pytest.raises(ValueError, match=r"labels|windows|samples|stable|finite"):
        compare_windows(samples, copy.deepcopy(windows), POLICY)
