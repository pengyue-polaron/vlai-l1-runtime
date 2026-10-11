from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from vlai_l1_runtime.automatic_review import review_automatic
from vlai_l1_runtime.configuration import MOTOR_NAMES, load_system_config
from vlai_l1_runtime.unloaded_baseline import digest

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_system_config(ROOT / "configs/system/vlai_l1.toml")
POLICY = json.loads((ROOT / "configs/diagnostics/right_automatic_motion.json").read_text())


@pytest.fixture
def capture(tmp_path):
    columns = ["monotonic_ns", "elapsed_seconds"] + [
        prefix + name for prefix in ("q_", "reference_", "torque_") for name in MOTOR_NAMES
    ]
    with (tmp_path / "samples.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for tick in range(500):
            writer.writerow([1_000_000_000 + tick * 10_000_000, tick / 100] + [0] * 24)
    with (tmp_path / "raw_feedback.csv").open("w") as stream:
        stream.write("# status=passed\n")
        writer = csv.writer(stream)
        writer.writerow(
            [
                "sequence",
                "host_receive_monotonic_ns",
                "can_id",
                "can_fd",
                "payload_hex",
                "rx_overflow_count",
            ]
        )
        for tick in range(502):
            for i, motor in enumerate(CONFIG.motors):
                status = 0 if tick in (0, 501) else 1
                payload = bytes([(status << 4) | motor.send_id, 128, 0, 128, 8, 0, 30, 30]).hex()
                writer.writerow(
                    [
                        tick * 8 + i,
                        990_000_000 + tick * 10_000_000 + i,
                        motor.receive_id,
                        1,
                        payload,
                        0,
                    ]
                )
    report = {
        "kind": "isolated_right_follower_automatic_diagnostic",
        "status": "passed",
        "disabled_verified": True,
        "config_sha256": digest(CONFIG.path.read_bytes()),
        "mode": "hold",
        "policy": POLICY,
        "external_load_present": None,
    }
    for filename, key in (("samples.csv", "samples_sha256"), ("raw_feedback.csv", "raw_sha256")):
        report[key] = digest((tmp_path / filename).read_bytes())
    (tmp_path / "report.json").write_text(json.dumps(report))
    return tmp_path


def update_hash(path, filename, key):
    report = json.loads((path / "report.json").read_text())
    report[key] = digest((path / filename).read_bytes())
    (path / "report.json").write_text(json.dumps(report))


def test_complete_integrity_review_has_no_ft_claim(capture):
    result = review_automatic(CONFIG, capture)
    assert result["status"] == "passed"
    assert result["ft_validated"] is False
    assert result["external_load_present"] is None
    assert result["samples"] == 500
    assert result["raw_frames"] == 4016
    assert result["q_span_deg"] == [0] * 8


def test_modified_capture_is_rejected(capture):
    with (capture / "samples.csv").open("a") as stream:
        stream.write("modified\n")
    with pytest.raises(ValueError, match="hash"):
        review_automatic(CONFIG, capture)


def test_incomplete_capture_is_rejected_even_with_valid_hash(capture):
    path = capture / "samples.csv"
    path.write_text("\n".join(path.read_text().splitlines()[:-20]) + "\n")
    update_hash(capture, "samples.csv", "samples_sha256")
    with pytest.raises(ValueError, match="duration"):
        review_automatic(CONFIG, capture)


def test_raw_overflow_is_rejected(capture):
    path = capture / "raw_feedback.csv"
    lines = path.read_text().splitlines()
    lines[4] = lines[4][:-1] + "1"
    path.write_text("\n".join(lines) + "\n")
    update_hash(capture, "raw_feedback.csv", "raw_sha256")
    with pytest.raises(ValueError, match="overflow"):
        review_automatic(CONFIG, capture)


def test_missing_disabled_feedback_is_rejected(capture):
    path = capture / "raw_feedback.csv"
    path.write_text("\n".join(path.read_text().splitlines()[:-8]) + "\n")
    update_hash(capture, "raw_feedback.csv", "raw_sha256")
    with pytest.raises(ValueError, match="disabled"):
        review_automatic(CONFIG, capture)
