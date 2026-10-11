from __future__ import annotations

import copy
import csv
import io
import json
import signal
from pathlib import Path
from types import SimpleNamespace

import pytest

from vlai_l1_runtime.configuration import MOTOR_NAMES, load_system_config
from vlai_l1_runtime.feedback import MotorFeedback, RawMotorFrame
from vlai_l1_runtime.teleoperation import feedback_capture as capture
from vlai_l1_runtime.teleoperation.can_probe import (
    MotorFeedbackProbeError,
    MotorFeedbackProbeResult,
)

CONFIG = Path(__file__).resolve().parents[1] / "configs/system/vlai_l1.toml"


def raw_frames(timestamp):
    return tuple(
        RawMotorFrame(name, 17 + index, timestamp, "0000000000000000", False)
        for index, name in enumerate(MOTOR_NAMES)
    )


@pytest.fixture
def fake_hardware(monkeypatch):
    config = load_system_config(CONFIG)
    calls = []
    links = {
        e.interface: {
            "ifname": e.interface,
            "parentdev": e.parentdev,
            "linkinfo": {
                "info_data": {"state": "STOPPED", "berr_counter": {"tx": 0, "rx": 0}},
                "info_xstats": {"bus_off": 0},
            },
            "stats64": {d: {"errors": 0, "dropped": 0} for d in ("rx", "tx")},
        }
        for e in config.can.endpoints
    }
    lock = io.StringIO()
    monkeypatch.setattr(capture.os, "geteuid", lambda: 0)
    monkeypatch.setattr(
        capture,
        "verify_xair_dependency",
        lambda c: SimpleNamespace(
            revision="test-revision",
            can_library=Path("/not-loaded.so"),
            can_library_sha256="test-hash",
        ),
    )
    monkeypatch.setattr(capture.lifecycle, "_acquire_lifecycle_lock", lambda *a, **k: lock)
    monkeypatch.setattr(capture.lifecycle, "_require_exclusive_control", lambda *a: None)
    monkeypatch.setattr(capture, "_require_services_inactive", lambda *a: None)
    monkeypatch.setattr(capture.lifecycle, "_require_interfaces_down", lambda *a: None)
    monkeypatch.setattr(capture.lifecycle, "_require_can_identity", lambda *a: None)
    monkeypatch.setattr(capture.lifecycle, "_qdisc_drops", lambda *a: 0)
    monkeypatch.setattr(capture, "_link", lambda i: copy.deepcopy(links[i]))

    def configure(c, interface):
        calls.append(("configure", interface))
        links[interface]["linkinfo"]["info_data"]["state"] = "ERROR-ACTIVE"

    def down(interface):
        calls.append(("down", interface))
        links[interface]["linkinfo"]["info_data"]["state"] = "STOPPED"

    def probe(c, interface, library, *, on_feedback, check_health):
        calls.append(("probe", interface))
        for index in range(40):
            check_health()
            on_feedback(
                index + 1,
                MotorFeedback(
                    MOTOR_NAMES,
                    index + 1,
                    (0.0,) * 8,
                    (0.1,) * 8,
                    (0.2,) * 8,
                    raw_frames(index + 1),
                ),
            )
        return MotorFeedbackProbeResult(interface, 40, tuple(range(17, 25)))

    monkeypatch.setattr(capture.lifecycle, "_configure_can", configure)
    monkeypatch.setattr(capture, "_down", down)
    monkeypatch.setattr(capture, "probe_motor_feedback", probe)
    # Any accidental direct subprocess operation in the mocked test is a failure.
    monkeypatch.setattr(capture.subprocess, "run", lambda *a, **k: pytest.fail("hardware touched"))
    return SimpleNamespace(config=config, calls=calls, links=links, lock=lock)


def run(fake, output, sides=("left", "right")):
    return capture.capture_follower_feedback(fake.config, sides=sides, output=output)


def test_complete_capture_owns_only_followers_and_preserves_units(fake_hardware, tmp_path):
    path = run(fake_hardware, tmp_path / "capture")
    report = json.loads(path.read_text())
    assert report["status"] == "passed"
    assert report["ft_validated"] is False
    assert [e["completed_rounds"] for e in report["endpoints"]] == [40, 40]
    assert fake_hardware.calls == [
        ("configure", "can3"),
        ("probe", "can3"),
        ("down", "can3"),
        ("configure", "can2"),
        ("probe", "can2"),
        ("down", "can2"),
    ]
    with (path.parent / "feedback.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 640
    assert rows[0]["motor"] == "joint_1"
    assert rows[7]["motor"] == "gripper"
    assert float(rows[0]["torque_nm"]) == 0.2
    paired = [
        json.loads(line) for line in (path.parent / "raw_feedback.jsonl").read_text().splitlines()
    ]
    assert len(paired) == 80
    assert paired[0]["feedback"]["raw_frames"][0]["response_id"] == 17
    assert fake_hardware.lock.closed


def test_optional_parameter_read_runs_after_probe_and_keeps_the_same_guard(
    fake_hardware, tmp_path, monkeypatch
):
    from vlai_l1_runtime.motor_parameters import ReadRegister
    from vlai_l1_runtime.teleoperation import parameter_probe

    def read(config, interface, motor, *, check_health, on_reading):
        assert fake_hardware.calls[-1] == ("probe", "can2")
        assert interface == "can2"
        assert motor.name == "joint_3"
        assert not fake_hardware.lock.closed
        fake_hardware.calls.append(("parameters", interface))
        for register in ReadRegister:
            check_health()
            on_reading({"motor": motor.name, "register": register.name, "value": 1})

    monkeypatch.setattr(parameter_probe, "read_motor_parameters", read)
    path = capture.capture_follower_feedback(
        fake_hardware.config,
        sides=("right",),
        output=tmp_path / "capture",
        parameter_motor="joint_3",
    )
    report = json.loads(path.read_text())
    assert report["schema_version"] == 3
    assert report["status"] == "passed"
    assert len(report["endpoints"][0]["parameter_reads"]) == 8
    assert len((path.parent / "parameters.jsonl").read_text().splitlines()) == 8
    assert fake_hardware.calls == [
        ("configure", "can2"),
        ("probe", "can2"),
        ("parameters", "can2"),
        ("down", "can2"),
    ]
    assert fake_hardware.lock.closed


def test_parameter_failure_retains_partial_evidence_and_closes_link(
    fake_hardware, tmp_path, monkeypatch
):
    from vlai_l1_runtime.teleoperation import parameter_probe

    def fail(config, interface, motor, *, check_health, on_reading):
        on_reading({"register": "MASTER_ID", "value": 19})
        raise TimeoutError("no parameter reply")

    monkeypatch.setattr(parameter_probe, "read_motor_parameters", fail)
    output = tmp_path / "capture"
    with pytest.raises(TimeoutError, match="no parameter reply"):
        capture.capture_follower_feedback(
            fake_hardware.config, sides=("right",), output=output, parameter_motor="joint_3"
        )
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "failed"
    assert report["endpoints"][0]["probe"]["rounds"] == 40
    assert len((output / "parameters.jsonl").read_text().splitlines()) == 1
    assert fake_hardware.calls[-1] == ("down", "can2")
    assert fake_hardware.lock.closed


def test_failed_state_probe_never_reaches_parameter_transport(fake_hardware, tmp_path, monkeypatch):
    from vlai_l1_runtime.teleoperation import parameter_probe

    def fail(*args, **kwargs):
        raise MotorFeedbackProbeError("missing motor")

    monkeypatch.setattr(capture, "probe_motor_feedback", fail)
    monkeypatch.setattr(
        parameter_probe, "read_motor_parameters", lambda *a, **k: pytest.fail("query after fault")
    )
    with pytest.raises(MotorFeedbackProbeError, match="missing motor"):
        capture.capture_follower_feedback(
            fake_hardware.config,
            sides=("right",),
            output=tmp_path / "capture",
            parameter_motor="joint_3",
        )
    assert fake_hardware.calls[-1] == ("down", "can2")


def test_parameter_option_rejects_bimanual_before_hardware(fake_hardware, tmp_path):
    with pytest.raises(ValueError, match="exactly one Follower"):
        capture.capture_follower_feedback(
            fake_hardware.config,
            sides=("left", "right"),
            output=tmp_path / "capture",
            parameter_motor="joint_3",
        )
    assert not fake_hardware.calls


def test_existing_output_is_not_touched(fake_hardware, tmp_path):
    marker = tmp_path / "keep"
    marker.write_text("original")
    with pytest.raises(FileExistsError):
        run(fake_hardware, tmp_path)
    assert marker.read_text() == "original"
    assert not fake_hardware.calls


@pytest.mark.parametrize(
    "guard", ["_require_exclusive_control", "_require_interfaces_down", "_require_can_identity"]
)
def test_failed_ownership_or_identity_never_configures_links(
    fake_hardware, tmp_path, monkeypatch, guard
):
    def fail(*args):
        raise RuntimeError("guard rejected")

    monkeypatch.setattr(capture.lifecycle, guard, fail)
    with pytest.raises(RuntimeError, match="guard rejected"):
        run(fake_hardware, tmp_path / "capture")
    assert not fake_hardware.calls
    assert fake_hardware.lock.closed


@pytest.mark.parametrize("failure", [MotorFeedbackProbeError("missing motor"), KeyboardInterrupt()])
def test_failure_closes_owned_link_without_querying_other_side(
    fake_hardware, tmp_path, monkeypatch, failure
):
    def fail(*args, **kwargs):
        raise failure

    monkeypatch.setattr(capture, "probe_motor_feedback", fail)
    previous = signal.getsignal(signal.SIGINT)
    with pytest.raises(type(failure)):
        run(fake_hardware, tmp_path / "capture")
    report = json.loads((tmp_path / "capture/report.json").read_text())
    assert report["status"] == "failed"
    assert fake_hardware.calls == [("configure", "can3"), ("down", "can3")]
    assert fake_hardware.lock.closed
    assert signal.getsignal(signal.SIGINT) == previous


def test_partially_configured_link_is_closed(fake_hardware, tmp_path, monkeypatch):
    def fail(*args):
        raise OSError("configuration failed")

    monkeypatch.setattr(capture.lifecycle, "_configure_can", fail)
    with pytest.raises(OSError, match="configuration failed"):
        run(fake_hardware, tmp_path / "capture")
    assert fake_hardware.calls == [("down", "can3")]


def test_nonzero_initial_live_errors_refuse_even_link_cycle(fake_hardware, tmp_path):
    fake_hardware.links["can3"]["linkinfo"]["info_data"]["berr_counter"]["tx"] = 128
    with pytest.raises(RuntimeError, match="txerr=128"):
        run(fake_hardware, tmp_path / "capture")
    assert not fake_hardware.calls


@pytest.mark.parametrize("fault", ["passive", "counter", "qdisc", "rx_drop", "identity"])
def test_each_health_fault_stops_capture(fake_hardware, tmp_path, monkeypatch, fault):
    def fail_probe(config, interface, library, *, check_health, on_feedback):
        link = fake_hardware.links[interface]
        if fault == "passive":
            link["linkinfo"]["info_data"]["state"] = "ERROR-PASSIVE"
        elif fault == "counter":
            link["linkinfo"]["info_xstats"]["bus_off"] += 1
        elif fault == "qdisc":
            monkeypatch.setattr(capture.lifecycle, "_qdisc_drops", lambda *a: 1)
        elif fault == "rx_drop":
            link["stats64"]["rx"]["dropped"] += 1
        else:
            link["parentdev"] = "different-device"
        check_health()

    monkeypatch.setattr(capture, "probe_motor_feedback", fail_probe)
    with pytest.raises(RuntimeError):
        run(fake_hardware, tmp_path / "capture")
    assert fake_hardware.calls == [("configure", "can3"), ("down", "can3")]


def test_cleanup_error_is_reported_and_releases_lock(fake_hardware, tmp_path, monkeypatch):
    def fail(interface):
        raise OSError("cannot close link")

    monkeypatch.setattr(capture, "_down", fail)
    with pytest.raises(RuntimeError, match="cleanup failed"):
        run(fake_hardware, tmp_path / "capture")
    report = json.loads((tmp_path / "capture/report.json").read_text())
    assert report["status"] == "failed"
    assert report["cleanup_errors"]
    assert fake_hardware.lock.closed


def test_out_of_bounds_raw_feedback_is_marked_without_authorizing_motion(
    fake_hardware, tmp_path, monkeypatch
):
    def probe(config, interface, library, *, on_feedback, check_health):
        check_health()
        on_feedback(
            1,
            MotorFeedback(
                MOTOR_NAMES,
                1,
                (0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0),
                (0.0,) * 8,
                (0.0,) * 8,
                raw_frames(1),
            ),
        )
        return MotorFeedbackProbeResult(interface, 1, tuple(range(17, 25)))

    monkeypatch.setattr(capture, "probe_motor_feedback", probe)
    report = json.loads(run(fake_hardware, tmp_path / "capture", ("left",)).read_text())
    assert report["endpoints"][0]["bound_violations"] == ["joint_4"]
    assert report["motion_authorized"] is False
    assert report["ft_validated"] is False


def test_capture_refuses_sdk_values_without_paired_raw_frames(fake_hardware, tmp_path, monkeypatch):
    def probe(config, interface, library, *, on_feedback, check_health):
        on_feedback(1, MotorFeedback(MOTOR_NAMES, 1, (0.0,) * 8, (0.0,) * 8, (0.0,) * 8))

    monkeypatch.setattr(capture, "probe_motor_feedback", probe)
    with pytest.raises(ValueError, match="complete raw feedback"):
        run(fake_hardware, tmp_path / "capture")
    assert fake_hardware.calls == [("configure", "can3"), ("down", "can3")]
    assert json.loads((tmp_path / "capture/report.json").read_text())["status"] == "failed"
