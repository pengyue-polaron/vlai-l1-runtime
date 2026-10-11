from __future__ import annotations

import ctypes
import socket
import subprocess
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from vlai_l1_runtime.configuration import MOTOR_NAMES, load_system_config
from vlai_l1_runtime.feedback import MotorFeedback, RawMotorFrame
from vlai_l1_runtime.gravity import GravityModel
from vlai_l1_runtime.kinematics import SerialChain
from vlai_l1_runtime.static_wrench import (
    JointCoordinateMap,
    OfflineStaticWrench,
    StaticWrenchPolicy,
)
from vlai_l1_runtime.wrench import ARM_JOINT_NAMES

CONFIG = Path(__file__).resolve().parents[1] / "configs/system/vlai_l1.toml"


@pytest.fixture(autouse=True)
def forbid_hardware(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("offline estimator attempted hardware/process access")

    original = ctypes.CDLL

    def libc_only(name, *args, **kwargs):
        if name is not None:
            forbidden()
        return original(name, *args, **kwargs)

    monkeypatch.setattr(ctypes, "CDLL", libc_only)
    monkeypatch.setattr(socket, "socket", forbidden)
    monkeypatch.setattr(subprocess, "Popen", forbidden)


@pytest.fixture
def estimator(tmp_path):
    axes = ("0 0 1", "0 1 0", "1 0 0", "0 1 0", "0 0 1", "1 0 0", "0 1 0")
    xyzs = ("0.1 0 0.2", "0.2 0 0", "0.1 0.1 0", "0.2 0 0", "0 0.1 0.1", "0.1 0 0", "0.1 0.1 0")
    xml = ['<robot name="test"><link name="base"/>']
    for i, (axis, xyz) in enumerate(zip(axes, xyzs, strict=True), 1):
        parent = "base" if i == 1 else f"link{i - 1}"
        xml.append(f'''<link name="link{i}"><inertial>
            <origin xyz="0.04 0.02 0.03" rpy="0 0 0"/><mass value="0.3"/>
            </inertial></link><joint name="joint_{i}" type="revolute">
            <parent link="{parent}"/><child link="link{i}"/>
            <origin xyz="{xyz}" rpy="0 0 0"/><axis xyz="{axis}"/>
            <limit lower="-3" upper="3"/></joint>''')
    xml.append("""<link name="tcp"/><joint name="tool" type="fixed"><parent link="link7"/>
        <child link="tcp"/><origin xyz="0.1 0.03 0.08" rpy="0 0 0"/></joint></robot>""")
    path = tmp_path / "model.urdf"
    path.write_text("".join(xml))
    model = GravityModel.from_urdf(
        path,
        root="base",
        branch="link1",
        expected_joint_names=ARM_JOINT_NAMES,
        massless_links=("tcp",),
    )
    chain = SerialChain.from_urdf(
        path, root="base", tip="tcp", expected_joint_names=ARM_JOINT_NAMES
    )
    policy = StaticWrenchPolicy(
        minimum_stationary_ns=200_000_000,
        maximum_sample_gap_ns=150_000_000,
        maximum_pair_age_ns=2_000_000,
        maximum_motor_skew_ns=1_000_000,
        maximum_velocity_rad_s=(0.05,) * 8,
        maximum_position_span_rad=(0.001,) * 8,
        allowed_raw_statuses=(frozenset({0}),) * 8,
        characteristic_length_m=0.2,
        damping_m=0.0,
        minimum_singular_value_m=1e-6,
        maximum_condition_number=1000.0,
        maximum_residual_nm=0.1,
        unmodeled_torque_bound_nm=(0.0,) * 7,
        maximum_force_error_n=(10.0,) * 3,
        maximum_moment_error_nm=(1.0,) * 3,
    )
    return OfflineStaticWrench(
        config=load_system_config(CONFIG),
        side="right",
        chain=chain,
        gravity=model,
        mapping=tuple(JointCoordinateMap(n, n, 1.0, 0.0, 1.0, 0.0) for n in ARM_JOINT_NAMES),
        auxiliary_positions=(),
        gravity_m_s2=(0.0, 0.0, -9.80665),
        payloads=(),
        static_friction_torque_nm=(0.0,) * 7,
        policy=policy,
        assumptions=("synthetic model and synthetic calibration only",),
    )


def snapshot(estimator, i, *, q=0.3, dq=0.0, status=0, force=(2.0, -1.0, 3.0, 0.1, 0.2, -0.1)):
    ranges = estimator.config.follower_feedback.right

    def encode(value, maximum, top):
        return round((value + maximum) * top / (2 * maximum))

    p_int = encode(q, ranges[0].position_max_rad, 65535)
    quantized = float(
        np.float32(p_int * (2 * ranges[0].position_max_rad) / 65535 - ranges[0].position_max_rad)
    )
    positions = dict.fromkeys(ARM_JOINT_NAMES, quantized)
    g = estimator.gravity.evaluate(positions, gravity_m_s2=estimator.gravity_vector, payloads=())
    jacobian = np.array(estimator.chain.evaluate(positions).jacobian)
    torque = np.array(g.holding_effort) - jacobian.T @ force
    timestamp = 1_000_000_000 + i * 100_000_000
    frames = []
    for j, (name, limits) in enumerate(zip(MOTOR_NAMES, ranges, strict=True)):
        v_int = encode(dq, limits.velocity_max_rad_s, 4095)
        t_int = encode(float(torque[j]) if j < 7 else 0.0, limits.torque_max_nm, 4095)
        assert 0 < t_int < 4095
        payload = bytes(
            [
                (status << 4) | (j + 1),
                p_int >> 8,
                p_int & 255,
                v_int >> 4,
                ((v_int & 15) << 4) | (t_int >> 8),
                t_int & 255,
                30,
                31,
            ]
        )
        frames.append(
            RawMotorFrame(name, 17 + j, timestamp - 1_000_000 + j * 10_000, payload.hex(), True)
        )
    return MotorFeedback(MOTOR_NAMES, timestamp, (0.0,) * 8, (0.0,) * 8, (0.0,) * 8, tuple(frames))


def ready(estimator):
    results = [estimator.process(snapshot(estimator, i), side="right") for i in range(3)]
    assert [r.status for r in results[:2]] == ["warming_up", "warming_up"]
    return results[-1]


def test_complete_raw_to_static_wrench_pipeline_and_quantization_bound(estimator):
    result = ready(estimator)
    assert result.status == "numerical_candidate"
    assert result.ft_validated is False
    assert result.assumptions == estimator.assumptions
    actual = np.array((*result.estimate.force_n, *result.estimate.moment_nm))
    expected = np.array([2.0, -1.0, 3.0, 0.1, 0.2, -0.1])
    assert np.all(np.abs(actual - expected) <= np.array(result.propagated_error_bound) + 1e-12)
    assert max(abs(v) for v in result.gravity_torque_nm) > 1


def test_coordinate_mapping_preserves_power_and_distinguishes_offset_from_torque_gain():
    mapping = JointCoordinateMap("joint_1", "model_j1", -2.0, 0.4, 1.5, 0.1)
    q, dq, effort = mapping.transform(0.2, 0.3, 0.9)
    assert q == pytest.approx(0)
    assert effort * dq == pytest.approx(1.5 * (0.9 - 0.1) * 0.3)
    with pytest.raises(ValueError, match="nonzero"):
        replace(mapping, position_scale=0.0)


@pytest.mark.parametrize(
    "fault", ["side", "status", "bounds", "timestamp", "age", "skew", "endpoint"]
)
def test_integrity_faults_latch_and_never_reuse_last_estimate(estimator, fault):
    assert ready(estimator).estimate is not None
    frame = snapshot(estimator, 3)
    side = "right"
    if fault == "side":
        side = "left"
    elif fault == "status":
        frame = snapshot(estimator, 3, status=8)
    elif fault == "bounds":
        frame = snapshot(estimator, 3, q=-0.1)
    elif fault == "timestamp":
        frame = snapshot(estimator, 2)
    else:
        raw = frame.raw_frames[0]
        if fault == "age":
            raw = replace(raw, host_receive_monotonic_ns=frame.monotonic_ns - 5_000_000)
        elif fault == "skew":
            raw = replace(raw, host_receive_monotonic_ns=frame.monotonic_ns - 1_999_999)
        else:
            raw = replace(raw, payload_hex=raw.payload_hex[:10] + "00" + raw.payload_hex[12:])
            raw = replace(raw, payload_hex=raw.payload_hex[:9] + "0" + raw.payload_hex[10:])
        frame = replace(frame, raw_frames=(raw, *frame.raw_frames[1:]))
    rejected = estimator.process(frame, side=side)
    assert rejected.status == "invalid"
    assert rejected.estimate is None
    assert estimator.process(snapshot(estimator, 4), side="right").estimate is None


def test_motion_and_gaps_require_a_new_stationary_window(estimator):
    assert ready(estimator).estimate is not None
    assert (
        estimator.process(snapshot(estimator, 3, dq=0.2), side="right").status == "not_stationary"
    )
    assert estimator.process(snapshot(estimator, 4), side="right").status == "warming_up"
    assert estimator.process(snapshot(estimator, 5), side="right").status == "warming_up"
    assert estimator.process(snapshot(estimator, 6), side="right").estimate is not None
    assert estimator.process(snapshot(estimator, 9), side="right").status == "warming_up"
    assert estimator.process(snapshot(estimator, 10), side="right").status == "warming_up"


def test_position_drift_rejected_even_when_velocity_field_says_stationary(estimator):
    assert ready(estimator).estimate is not None
    result = estimator.process(snapshot(estimator, 3, q=0.31), side="right")
    assert result.status == "not_stationary"
    assert result.estimate is None


def test_uncertain_torques_reject_the_candidate(estimator):
    estimator.policy = replace(estimator.policy, unmodeled_torque_bound_nm=(5.0,) * 7)
    result = ready(estimator)
    assert result.status == "rejected"
    assert result.estimate is None
    assert "propagated torque error" in result.reasons[0]


def test_bad_condition_rejects_candidate_without_latching_input_fault(estimator):
    estimator.policy = replace(estimator.policy, maximum_condition_number=1.0)
    result = ready(estimator)
    assert result.status == "rejected"
    assert result.estimate is None
    assert "condition" in result.reasons[0]
