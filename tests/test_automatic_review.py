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


@pytest.mark.parametrize("fault", [None, "native", "cleanup", "disabled", "hardware", "analysis"])
def test_explicit_offline_recheck_never_accepts_hardware_failure(capture, fault):
    path = capture / "report.json"
    report = json.loads(path.read_text())
    report.update(
        status="failed",
        analysis_status="failed",
        native_returncode=0,
        error="offline review: previous bug",
    )
    if fault == "native":
        report["native_returncode"] = 1
    elif fault == "cleanup":
        report["cleanup_error"] = "failed down"
    elif fault == "disabled":
        report["disabled_verified"] = False
    elif fault == "hardware":
        report["error"] = "native guard failure"
    elif fault == "analysis":
        del report["analysis_status"]
    path.write_text(json.dumps(report))
    original = path.read_bytes()
    with pytest.raises(ValueError, match="requires a passed"):
        review_automatic(CONFIG, capture)
    if fault is None:
        result = review_automatic(CONFIG, capture, recheck_offline_failure=True)
        assert result["offline_failure_rechecked"] is True
        assert result["source_report_sha256"] == digest(original)
    else:
        with pytest.raises(ValueError, match="requires a passed"):
            review_automatic(CONFIG, capture, recheck_offline_failure=True)
    assert path.read_bytes() == original


@pytest.mark.parametrize("active", [3, 5, 6])
def test_sweep_review_enforces_selected_motor_and_fixed_other_targets(capture, active):
    path = capture / "report.json"
    report = json.loads(path.read_text())
    report["mode"] = "sweep"
    report["policy"].update(
        active_motor=MOTOR_NAMES[active],
        amplitude_deg=0.1,
        transition_seconds=1.0,
        hold_seconds=0.2,
        cycles=1,
    )
    path.write_text(json.dumps(report))
    sample_path = capture / "samples.csv"
    rows = list(csv.reader(sample_path.read_text().splitlines()))
    rows[100][10 + active] = "0.05"

    def save():
        with sample_path.open("w") as f:
            csv.writer(f).writerows(rows)
        update_hash(capture, "samples.csv", "samples_sha256")

    save()
    assert review_automatic(CONFIG, capture)["active_motor"] == MOTOR_NAMES[active]
    rows[100][10 + active] = "0.2"
    save()
    with pytest.raises(ValueError, match="envelope"):
        review_automatic(CONFIG, capture)
    rows[100][10 + active] = "0.05"
    rows[100][10 + (active + 1) % 8] = "0.05"
    save()
    with pytest.raises(ValueError, match="fixed motor"):
        review_automatic(CONFIG, capture)


def test_modified_capture_is_rejected(capture):
    with (capture / "samples.csv").open("a") as stream:
        stream.write("modified\n")
    with pytest.raises(ValueError, match="hash"):
        review_automatic(CONFIG, capture)


@pytest.mark.parametrize("fault", [None, "trajectory", "fixed", "bounds", "paired"])
def test_multijoint_review_checks_complete_path_and_repeated_postures(capture, fault):
    from vlai_l1_runtime.automatic_trajectory import target_offsets

    report_path = capture / "report.json"
    report = json.loads(report_path.read_text())
    p = json.loads((ROOT / "configs/diagnostics/right_multijoint_unloaded_single.json").read_text())
    p.update(
        amplitude_deg=0.001,
        maximum_amplitude_deg=0.001,
        transition_seconds=0.25,
        hold_seconds=0.1,
        cycles=2,
    )
    for point in p["waypoints_deg"]:
        for name in MOTOR_NAMES:
            point[name] *= 0.001 / 16
    report.update(policy=p, mode="paired" if fault == "paired" else "sweep")
    report_path.write_text(json.dumps(report))
    path = capture / "samples.csv"
    rows = list(csv.reader(path.read_text().splitlines()))
    for row in rows[1:]:
        target = list(target_offsets(p, float(row[1])))
        row[2:10] = target
        row[10:18] = target
    if fault == "trajectory":
        rows[20][11] += 0.00001  # Inside envelope but not the planned trajectory.
    elif fault == "fixed":
        rows[20][17] = 0.00001
    elif fault == "bounds":
        rows[20][3] = -100
    with path.open("w") as stream:
        csv.writer(stream).writerows(rows)
    update_hash(capture, "samples.csv", "samples_sha256")
    if fault is not None:
        with pytest.raises(
            ValueError,
            match={
                "trajectory": "scheduled waypoint",
                "fixed": "fixed motor",
                "bounds": "measured waypoint",
                "paired": "excludes paired",
            }[fault],
        ):
            review_automatic(CONFIG, capture)
    else:
        result = review_automatic(CONFIG, capture)
        assert result["active_motors"] == list(MOTOR_NAMES[1:7])
        assert len(result["hold_windows"]) == 15
        assert len(result["same_approach_repeat_pairs"]) == 7
        assert len(result["same_command_returns"]) == 2
        assert result["ft_validated"] is False


def test_handling_review_checks_settling_positions_without_treating_waits_as_sweeps(capture):
    path = capture / "report.json"
    report = json.loads(path.read_text())
    report["mode"] = "paired"
    report["policy"].update(
        amplitude_deg=0.001,
        transition_seconds=0.3,
        hold_seconds=0.05,
        cycles=1,
        hold_test_seconds=10,
    )
    report["policy"]["manual_handling"] = json.loads(
        (ROOT / "configs/diagnostics/right_joint4_paired.json").read_text()
    )["manual_handling"]
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
    report["events"] = [
        {"state": "phase", "phase": n, "elapsed_seconds": t}
        for n, t in zip(names, [0, 1.45, 1.775, 3.775, 5.225, 5.55, 7.55, 9], strict=True)
    ]
    report["events"].append(
        {
            "state": "completed",
            "completion_kind": "paired_sweep",
            "disabled_verified": True,
            "elapsed_seconds": 9,
        }
    )
    report["paired_acknowledgements"] = ["attach.ready", "remove.ready"]
    report["completed_paired_seconds"] = 9
    path.write_text(json.dumps(report))
    sample_path = capture / "samples.csv"
    rows = list(csv.reader(sample_path.read_text().splitlines()))
    for tick, row in enumerate(rows[1:]):
        row[0], row[1] = str(1_000_000_000 + tick * 18_000_000), str(tick * 0.018)
    with sample_path.open("w") as stream:
        csv.writer(stream).writerows(rows)
    update_hash(capture, "samples.csv", "samples_sha256")
    raw_path = capture / "raw_feedback.csv"
    lines = list(csv.reader(raw_path.read_text().splitlines()[1:]))
    for row in lines[1:]:
        tick, motor = divmod(int(row[0]), 8)
        row[1] = str(990_000_000 + tick * 18_000_000 + motor)
    with raw_path.open("w") as stream:
        stream.write("# status=passed\n")
        csv.writer(stream).writerows(lines)
    update_hash(capture, "raw_feedback.csv", "raw_sha256")
    result = review_automatic(CONFIG, capture)
    assert len(result["paired_phases"]) == 7
    assert len(result["hold_windows"]) == 15
    rows[151][6] = ".1"  # q5 disturbance at 2.7 seconds, in settling_loaded.
    with sample_path.open("w") as stream:
        csv.writer(stream).writerows(rows)
    update_hash(capture, "samples.csv", "samples_sha256")
    with pytest.raises(ValueError, match="settling position"):
        review_automatic(CONFIG, capture)


def test_paired_review_keeps_waiting_targets_fixed_and_separates_phases(capture):
    path = capture / "report.json"
    report = json.loads(path.read_text())
    report["mode"] = "paired"
    report["policy"].update(
        amplitude_deg=0.001, transition_seconds=0.3, hold_seconds=0.05, cycles=1
    )
    names = ["baseline", "awaiting_attach", "loaded", "awaiting_remove", "empty_after", "complete"]
    report["events"] = [
        {"state": "phase", "phase": name, "elapsed_seconds": t}
        for name, t in zip(names, [0, 1.45, 1.775, 3.225, 3.55, 5], strict=True)
    ]
    report["events"].append(
        {
            "state": "completed",
            "completion_kind": "paired_sweep",
            "disabled_verified": True,
            "elapsed_seconds": 5,
        }
    )
    report["paired_acknowledgements"] = ["attach.ready", "remove.ready"]
    report["completed_paired_seconds"] = 5
    report["policy"]["hold_test_seconds"] = 6
    path.write_text(json.dumps(report))
    result = review_automatic(CONFIG, capture)
    assert len(result["paired_phases"]) == 5
    assert len(result["hold_windows"]) == 15
    assert result["same_command_returns"] == result["same_approach_repeat_pairs"] == []
    sample_path = capture / "samples.csv"
    rows = list(csv.reader(sample_path.read_text().splitlines()))
    active = MOTOR_NAMES.index(report["policy"]["active_motor"])
    rows[151][10 + active] = "0.0005"  # In the first stationary wait, inside the envelope.
    with sample_path.open("w") as stream:
        csv.writer(stream).writerows(rows)
    update_hash(capture, "samples.csv", "samples_sha256")
    with pytest.raises(ValueError, match="waiting reference"):
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


def test_raw_speed_report_separates_enabled_spike_from_disabled_motion(capture):
    path = capture / "raw_feedback.csv"
    lines = path.read_text().splitlines()
    for tick, code in ((100, 2026), (501, 1900)):
        index = 2 + tick * 8 + 6
        row = lines[index].split(",")
        payload = bytearray.fromhex(row[4])
        payload[3] = code >> 4
        payload[4] = (code & 15) << 4 | payload[4] & 15
        row[4] = payload.hex()
        lines[index] = ",".join(row)
    path.write_text("\n".join(lines) + "\n")
    update_hash(capture, "raw_feedback.csv", "raw_sha256")
    result = review_automatic(CONFIG, capture)
    assert result["raw_maximum_abs_enabled_speed_deg_s"]["joint_7"] == pytest.approx(18.04922055)
    assert result["raw_maximum_abs_disabled_speed_deg_s"]["joint_7"] > 100
    assert result["raw_enabled_speed_limit_exceedances"]["joint_7"] == 1
    assert sum(result["raw_enabled_speed_limit_exceedances"].values()) == 1


def requested_hold_report(capture):
    path = capture / "report.json"
    report = json.loads(path.read_text())
    report["policy"]["hold_test_seconds"] = 600
    report.update(
        completion_kind="requested_hold",
        operator_finish_requested=True,
        completed_hold_seconds=5.0,
        events=[
            {
                "state": "completed",
                "completion_kind": "requested_hold",
                "disabled_verified": True,
                "elapsed_seconds": 5.0,
            }
        ],
    )
    return path, report


def test_review_accepts_verified_explicit_hold_completion(capture):
    path, report = requested_hold_report(capture)
    path.write_text(json.dumps(report))
    result = review_automatic(CONFIG, capture)
    assert result["duration_seconds"] == pytest.approx(4.99)
    assert len(result["hold_windows"]) == 1
    assert result["hold_windows"][0]["end_seconds"] == 5.0


@pytest.mark.parametrize("bad", ["request", "event", "short", "long", "nan", "sweep"])
def test_early_hold_finish_needs_matching_completion_evidence(capture, bad):
    path, report = requested_hold_report(capture)
    if bad == "request":
        report["operator_finish_requested"] = False
    elif bad == "event":
        report["events"][0]["disabled_verified"] = False
    elif bad == "sweep":
        report["mode"] = "sweep"
    else:
        report["completed_hold_seconds"] = {"short": 1, "long": 601, "nan": float("nan")}[bad]
    path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="requested hold"):
        review_automatic(CONFIG, capture)
