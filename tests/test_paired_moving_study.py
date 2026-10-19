import json
from pathlib import Path

import numpy as np
import pytest

from vlai_l1_runtime.paired_moving_study import analyse

ROOT = Path(__file__).resolve().parents[1]
POLICY = json.loads((ROOT / "configs/diagnostics/right_moving_window_study.json").read_text())
MOTION = json.loads((ROOT / "configs/diagnostics/right_joint4_paired.json").read_text())


def test_insufficient_baseline_preserves_rejection_and_never_uses_later_load_data():
    samples = np.zeros((400, 17))
    samples[:, 0] = np.arange(400) / 100
    phases = [
        {"phase": name, "start_seconds": start, "end_seconds": start + 1}
        for name, start in [("baseline", 0), ("loaded", 1), ("empty_after", 2)]
    ]
    samples[samples[:, 0] >= 1, 12] = 100
    result = analyse(samples, phases, MOTION, POLICY, {"raw_sha256": "test"}, True)
    assert result["status"] == "rejected"
    assert "insufficient bidirectional" in result["reason"]
    assert set(result["extraction"]) == {"baseline", "loaded", "empty_after"}
    assert "model" not in result
    assert result["runtime_eligible"] is False


def test_paired_analysis_never_fits_loaded_or_final_windows():
    phases = [
        {"phase": n, "start_seconds": s, "end_seconds": e}
        for n, s, e in [
            ("baseline", 0, 109),
            ("awaiting_attach", 109, 110),
            ("loaded", 110, 219),
            ("awaiting_remove", 219, 220),
            ("empty_after", 220, 329),
        ]
    ]
    x = np.zeros((32900, 17))
    x[:, 0] = np.arange(len(x)) / 100
    for phase in phases:
        if phase["phase"].startswith("awaiting"):
            continue
        mask = (x[:, 0] >= phase["start_seconds"]) & (x[:, 0] < phase["end_seconds"])
        t = x[mask, 0] - phase["start_seconds"]
        leg = np.clip(((t - 5) // 13).astype(int), 0, 7) % 4
        u = np.clip(((t - 5) % 13) / 8, 0, 1)
        starts = np.array([0, 0.5, 0, -0.5])[leg]
        ends = np.array([0.5, 0, -0.5, 0])[leg]
        q = starts + (ends - starts) * u
        q[t < 5] = 0
        direction = np.sign(ends - starts)
        x[mask, 4] = 10 + q
        x[mask, 12] = 0.2 + 0.3 * q + 0.4 * direction + (0.3 if phase["phase"] == "loaded" else 0)
    a = analyse(x, phases, MOTION, POLICY, {}, True)
    changed = x.copy()
    changed[(x[:, 0] >= 110) & (x[:, 0] < 219), 12] += 10
    changed[x[:, 0] >= 220, 12] += 5
    b = analyse(changed, phases, MOTION, POLICY, {}, True)
    assert a["model"]["models"] == b["model"]["models"]
    residual = a["middle_evaluation"]["models"]["angle_direction"][
        "supported_motor_torque_residual_nm"
    ]
    assert residual == pytest.approx([0.3] * len(residual))
    assert not a["frozen_before_loaded_acquisition"]
    assert not a["ft_validated"]
