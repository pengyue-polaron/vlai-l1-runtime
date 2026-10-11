from __future__ import annotations

import ctypes
import socket
import subprocess
from dataclasses import FrozenInstanceError, replace
from pathlib import Path

import pytest

from vlai_l1_runtime.configuration import load_system_config
from vlai_l1_runtime.contracts import NamedJointVector, SampleMetadata, feature_names_for_sides
from vlai_l1_runtime.session import CommandSession, SessionError
from vlai_l1_runtime.trajectory import (
    RehearsalState,
    SweepSpec,
    TrajectoryRehearsal,
    WristSweep,
)

CONFIG = load_system_config(Path(__file__).resolve().parents[1] / "configs/system/vlai_l1.toml")
FEATURES = feature_names_for_sides(("right",))
J7 = "right_joint_7.pos"
NS = 1_000_000_000
SPEC = SweepSpec(2.0, 4 * NS, 5 * NS, 2, 1.0, 0.8)


def reference():
    return dict.fromkeys(FEATURES, 0.0) | {"right_joint_4.pos": 35.0, "right_gripper.pos": 1.5}


def plan(**kwargs):
    return WristSweep(CONFIG, side="right", reference_deg=reference(), spec=SPEC, **kwargs)


def feedback(q=None, *, sequence=0, source=NS):
    return NamedJointVector(
        reference() if q is None else q, SampleMetadata(sequence, source), FEATURES
    )


def rehearsal(p=None):
    return TrajectoryRehearsal(
        plan() if p is None else p,
        tracking_error_deg=dict.fromkeys(FEATURES, 0.5),
        tracking_persistence_ns=100_000_000,
    )


def started():
    runner = rehearsal()
    first = runner.step(now_ns=NS, feedback=feedback(), transport_healthy=True)
    assert first.point.position_deg == reference()
    return runner


def test_full_trajectory_and_rehearsal_without_hardware(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("trajectory rehearsal attempted an external resource")

    monkeypatch.setattr(socket, "socket", forbidden)
    monkeypatch.setattr(ctypes, "CDLL", forbidden)
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    p = plan()
    runner = rehearsal(p)
    assert p.duration_ns == 77 * NS
    previous = None
    for i, offset in enumerate(range(0, p.duration_ns + 1, 10_000_000)):
        point = p.at(offset)
        assert set(point.position_deg) == set(FEATURES)
        assert -2 <= point.position_deg[J7] <= 2
        assert abs(point.velocity_deg_s[J7]) <= SPEC.maximum_speed_deg_s
        assert abs(point.acceleration_deg_s2[J7]) <= SPEC.maximum_acceleration_deg_s2
        for name in FEATURES:
            if name != J7:
                assert point.position_deg[name] == reference()[name]
                assert point.velocity_deg_s[name] == point.acceleration_deg_s2[name] == 0
        if previous is not None:
            finite_difference_speed = (point.position_deg[J7] - previous.position_deg[J7]) / 0.01
            assert abs(finite_difference_speed) <= SPEC.maximum_speed_deg_s
        previous = point
        result = runner.step(
            now_ns=NS + offset,
            feedback=feedback(point.position_deg, sequence=i, source=NS + offset),
            transport_healthy=True,
        )
        if offset < p.duration_ns:
            assert result.state is RehearsalState.RUNNING
            assert result.point == point
    assert result.state is RehearsalState.RELEASE_REQUIRED
    assert result.reason == "completed"
    assert result.point is None
    assert (
        runner.acknowledge_release(disabled=True, resources_released=True).state
        is RehearsalState.CLOSED
    )
    assert runner.step(now_ns=999 * NS, feedback=feedback(), transport_healthy=True).point is None


def test_segment_endpoints_have_zero_velocity_and_acceleration():
    p = plan()
    for t in (0, 5, 9, 14, 18, 23, 27, 32, 36, 41, 45, 50, 54, 59, 63, 68, 72, 77):
        point = p.at(t * NS)
        assert point.velocity_deg_s[J7] == point.acceleration_deg_s2[J7] == 0
    for t, target in ((9, 2), (18, 0), (27, -2), (36, 0), (45, 2), (54, 0), (63, -2), (72, 0)):
        assert p.at(t * NS).position_deg[J7] == target


@pytest.mark.parametrize(
    "changes",
    [
        {"amplitude_deg": float("nan")},
        {"amplitude_deg": True},
        {"transition_ns": 0},
        {"hold_ns": 1.5},
        {"cycles": True},
        {"cycles": 0},
        {"maximum_speed_deg_s": 0.9},
        {"maximum_acceleration_deg_s2": 0.7},
        {"amplitude_deg": float("inf")},
    ],
)
def test_invalid_or_over_budget_specs(changes):
    with pytest.raises(ValueError, match=r"must be|exceeds"):
        replace(SPEC, **changes)


def test_exact_identity_bounds_and_immutable_vectors():
    with pytest.raises(ValueError, match="feature mismatch"):
        WristSweep(CONFIG, side="right", reference_deg={"joint_7": 0}, spec=SPEC)
    with pytest.raises(ValueError, match=r"bounds.*joint_7"):
        WristSweep(CONFIG, side="right", reference_deg=reference() | {J7: 89}, spec=SPEC)
    with pytest.raises(ValueError, match=r"bounds.*joint_2"):
        WristSweep(
            CONFIG, side="right", reference_deg=reference() | {"right_joint_2.pos": -11}, spec=SPEC
        )
    q = reference()
    p = WristSweep(CONFIG, side="right", reference_deg=q, spec=SPEC)
    q[J7] = 88
    assert p.at(0).position_deg[J7] == 0
    with pytest.raises(TypeError):
        p.at(0).position_deg[J7] = 88
    with pytest.raises(FrozenInstanceError):
        p.spec = replace(SPEC, amplitude_deg=1)
    with pytest.raises(ValueError, match="configuration"):
        WristSweep(replace(CONFIG, robot_id="fabricated"), side="right", reference_deg=q, spec=SPEC)


@pytest.mark.parametrize("t", [-1, True, 1.5, 77 * NS + 1])
def test_invalid_trajectory_times(t):
    with pytest.raises(ValueError, match=r"time.*must be|past trajectory"):
        plan().at(t)


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        ({"now_ns": NS}, "non_monotonic_host_time"),
        ({"now_ns": NS - 1}, "non_monotonic_host_time"),
        ({"now_ns": NS + 100_000_001}, "feedback_timeout"),
        ({"transport_healthy": False}, "transport_fault"),
        ({"transport_healthy": 1}, "transport_fault"),
        ({"feedback": None}, "wrong_feedback_identity"),
        ({"feedback": feedback(sequence=0, source=NS + 1)}, "non_monotonic_feedback"),
        ({"feedback": feedback(sequence=1, source=NS)}, "non_monotonic_feedback"),
        ({"feedback": feedback(sequence=1, source=NS + 50_000_001)}, "stale_or_future_feedback"),
        (
            {
                "feedback": feedback(
                    reference() | {"right_joint_2.pos": -11}, sequence=1, source=NS + 1
                )
            },
            "joint bounds",
        ),
    ],
)
def test_faults_latch_and_suppress_all_further_targets(changes, reason):
    runner = started()
    args = {
        "now_ns": NS + 50_000_000,
        "feedback": feedback(sequence=1, source=NS + 1),
        "transport_healthy": True,
    } | changes
    result = runner.step(**args)
    assert result.state is RehearsalState.RELEASE_REQUIRED
    assert result.point is None
    assert reason in result.reason
    subsequent = runner.step(
        now_ns=NS + 60_000_000,
        feedback=feedback(sequence=2, source=NS + 60_000_000),
        transport_healthy=True,
    )
    assert subsequent == result


@pytest.mark.parametrize(
    ("source", "q", "reason"),
    [
        (NS - 100_000_001, reference(), "stale_or_future_feedback"),
        (NS + 1, reference(), "stale_or_future_feedback"),
        (NS, reference() | {J7: 1.0}, "initial_reference_mismatch"),
    ],
)
def test_acquisition_rejects_old_or_mismatched_start(source, q, reason):
    runner = rehearsal()
    result = runner.step(now_ns=NS, feedback=feedback(q, source=source), transport_healthy=True)
    assert result.point is None
    assert result.reason == reason


def test_poll_cannot_keep_stale_feedback_alive():
    runner = started()
    assert runner.poll(now_ns=NS + 50_000_000).state is RehearsalState.RUNNING
    assert runner.poll(now_ns=NS + 100_000_000).state is RehearsalState.RUNNING
    result = runner.poll(now_ns=NS + 100_000_001)
    assert result.reason == "feedback_timeout"
    assert result.point is None


def test_tracking_excursions_reset_and_gripper_is_also_guarded():
    runner = started()
    feature = "right_gripper.pos"
    for i, bad in enumerate((True, False, True, True, True), start=1):
        q = reference() | ({feature: 3.0} if bad else {})
        stamp = NS + i * 50_000_000
        result = runner.step(
            now_ns=stamp, feedback=feedback(q, sequence=i, source=stamp), transport_healthy=True
        )
        assert (result.point is None) is (i == 5)
    assert result.reason == f"persistent_tracking_error:{feature}"


@pytest.mark.parametrize(("disabled", "released"), [(False, True), (True, False), (True, 1)])
def test_stop_acknowledgement_requires_both_disable_and_release(disabled, released):
    runner = started()
    assert runner.stop().point is None
    result = runner.acknowledge_release(disabled=disabled, resources_released=released)
    assert result.state is RehearsalState.RELEASE_FAILED
    assert runner.step(now_ns=NS + 1, feedback=feedback(), transport_healthy=True).point is None
    with pytest.raises(ValueError, match="not expected"):
        runner.acknowledge_release(disabled=True, resources_released=True)


def test_rehearsal_does_not_enable_production_command_readiness():
    assert plan().command_blockers == CONFIG.command_blockers
    session = CommandSession(CONFIG)
    session.connect_read_only(now_ns=NS)
    with pytest.raises(SessionError, match="readiness"):
        session.acquire_command(lease_id="rehearsal", measured=feedback(), now_ns=NS)


def test_feedback_must_have_the_selected_side_identity():
    left = feature_names_for_sides(("left",))
    vector = NamedJointVector(dict.fromkeys(left, 0), SampleMetadata(0, NS), left)
    result = rehearsal().step(now_ns=NS, feedback=vector, transport_healthy=True)
    assert result.reason == "wrong_feedback_identity"
    assert result.point is None


def test_final_sample_outside_tracking_tolerance_is_not_success():
    p = plan()
    runner = rehearsal(p)
    for i, offset in enumerate(range(0, p.duration_ns + 1, 100_000_000)):
        q = dict(p.at(offset).position_deg)
        if offset == p.duration_ns:
            q[J7] += 1
        result = runner.step(
            now_ns=NS + offset,
            feedback=feedback(q, sequence=i, source=NS + offset),
            transport_healthy=True,
        )
    assert result.reason == "final_tracking_error"
    assert result.point is None


def test_tracking_uses_feedback_source_time_instead_of_later_host_time():
    # Delayed feedback must be compared with the trajectory at acquisition.
    p = plan()
    runner = TrajectoryRehearsal(
        p, tracking_error_deg=dict.fromkeys(FEATURES, 0.001), tracking_persistence_ns=10_000_000
    )
    start = NS
    for i, offset in enumerate(range(0, 15 * NS, 10_000_000)):
        source = start + offset - 40_000_000
        q = p.at(max(0, source - start)).position_deg
        result = runner.step(
            now_ns=start + offset,
            feedback=feedback(q, sequence=i, source=source),
            transport_healthy=True,
        )
        assert result.state is RehearsalState.RUNNING
