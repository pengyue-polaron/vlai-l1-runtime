from __future__ import annotations

import ctypes
import math
import socket
import subprocess
from dataclasses import replace

import numpy as np
import pytest

from vlai_l1_runtime.gravity import GravityModel, PointMass
from vlai_l1_runtime.kinematics import SerialChain
from vlai_l1_runtime.wrench import ARM_JOINT_NAMES, estimate_wrench


@pytest.fixture(autouse=True)
def forbid_hardware(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("gravity model attempted hardware/process access")

    cdll = ctypes.CDLL

    def libc_only(name, *args, **kwargs):
        if name is not None:
            forbidden()
        return cdll(name, *args, **kwargs)

    monkeypatch.setattr(ctypes, "CDLL", libc_only)
    monkeypatch.setattr(socket, "socket", forbidden)
    monkeypatch.setattr(subprocess, "Popen", forbidden)


def link(name, mass, com):
    return f'''<link name="{name}"><inertial>
      <origin xyz="{com}" rpy="0.1 0.2 0.3"/><mass value="{mass}"/>
    </inertial></link>'''


def joint(name, kind, parent, child, xyz, axis="0 1 0", mimic=""):
    return f'''<joint name="{name}" type="{kind}">
      <parent link="{parent}"/><child link="{child}"/>
      <origin xyz="{xyz}" rpy="0 0 0"/>
      <axis xyz="{axis}"/><limit lower="-3" upper="3"/>{mimic}
    </joint>'''


def model_file(tmp_path, *, fingers=False):
    parts = ['<robot name="synthetic"><link name="base"/>', link("arm", 2, "0.5 0 0")]
    parts.append(joint("j1", "revolute", "base", "arm", "0 0 0"))
    parts.append('<link name="tcp"/>')
    parts.append(joint("tool", "fixed", "arm", "tcp", "1 0 0"))
    if fingers:
        parts.extend(
            [
                link("finger1", 0.3, "0.05 0.01 0.02"),
                link("finger2", 0.2, "0.03 -0.01 0.01"),
                joint("slide1", "prismatic", "arm", "finger1", "1 0.1 0", "1 0 0"),
                joint(
                    "slide2",
                    "prismatic",
                    "arm",
                    "finger2",
                    "1 -0.1 0",
                    "-1 0 0",
                    '<mimic joint="slide1" multiplier="0.5" offset="0.02"/>',
                ),
            ]
        )
    parts.append("</robot>")
    path = tmp_path / "model.urdf"
    path.write_text("".join(parts))
    names = ("j1", "slide1") if fingers else ("j1",)
    return path, GravityModel.from_urdf(
        path, root="base", branch="arm", expected_joint_names=names, massless_links=("tcp",)
    )


def test_pendulum_holding_sign_and_payload_are_analytic(tmp_path):
    _, model = model_file(tmp_path)
    q = 0.3
    result = model.evaluate({"j1": q}, gravity_m_s2=(0.0, 0.0, -10.0), payloads=())
    assert result.holding_effort == pytest.approx((-10 * math.cos(q),))
    assert result.potential_energy_j == pytest.approx(-10 * math.sin(q))
    assert result.effort_units == ("Nm",)
    payload = PointMass("load", "tcp", 0.5, (0.2, 0.0, 0.0))
    loaded = model.evaluate({"j1": q}, gravity_m_s2=(0.0, 0.0, -10.0), payloads=(payload,))
    assert loaded.holding_effort == pytest.approx((-16 * math.cos(q),))
    assert loaded.total_mass_kg == 2.5
    np.testing.assert_allclose(np.sum(loaded.mass_contributions, axis=0), loaded.holding_effort)


def test_payload_difference_equals_negative_external_gravity_wrench(tmp_path):
    path, model = model_file(tmp_path)
    q, gravity = {"j1": 0.4}, (3.0, 2.0, -9.0)
    chain = SerialChain.from_urdf(path, root="base", tip="tcp", expected_joint_names=("j1",))
    geometry = chain.evaluate(q)
    empty = model.evaluate(q, gravity_m_s2=gravity, payloads=())
    loaded = model.evaluate(
        q, gravity_m_s2=gravity, payloads=(PointMass("load", "tcp", 0.7, (0.0, 0.0, 0.0)),)
    )
    tau_ext = np.array(geometry.jacobian).T @ np.r_[0.7 * np.array(gravity), np.zeros(3)]
    np.testing.assert_allclose(np.array(loaded.holding_effort) - empty.holding_effort, -tau_ext)


def test_finger_branch_and_mimic_derivatives_match_potential_energy(tmp_path):
    _, model = model_file(tmp_path, fingers=True)
    gravity = (3.0, 2.0, -9.0)
    q = {"j1": 0.3, "slide1": 0.04}
    result = model.evaluate(q, gravity_m_s2=gravity, payloads=())
    assert result.mass_names == ("arm", "finger1", "finger2")
    assert result.effort_units == ("Nm", "N")
    assert result.total_mass_kg == 2.5
    for i, name in enumerate(model.joint_names):
        delta = 1e-6
        up = model.evaluate(q | {name: q[name] + delta}, gravity_m_s2=gravity, payloads=())
        down = model.evaluate(q | {name: q[name] - delta}, gravity_m_s2=gravity, payloads=())
        assert result.holding_effort[i] == pytest.approx(
            (up.potential_energy_j - down.potential_energy_j) / (2 * delta), abs=1e-8
        )
    zero = model.evaluate(q | {"j1": 0.0}, gravity_m_s2=gravity, payloads=())
    assert zero.holding_effort[1] == pytest.approx(-0.6)


def test_rotated_mount_and_inertial_origin_rotation(tmp_path):
    path, model = model_file(tmp_path)
    baseline = model.evaluate({"j1": 0.3}, gravity_m_s2=(2.0, 3.0, -9.0), payloads=())
    # Rotate the entire subtree around root Z and rotate gravity by the same amount.
    j = replace(
        model.joints[0], geometry=replace(model.joints[0].geometry, rpy=(0.0, 0.0, math.pi / 2))
    )
    rotated = replace(model, joints=(j, *model.joints[1:])).evaluate(
        {"j1": 0.3}, gravity_m_s2=(-3.0, 2.0, -9.0), payloads=()
    )
    assert rotated.holding_effort == pytest.approx(baseline.holding_effort)
    # Inertial RPY rotates the inertia tensor, not the COM translation.
    path.write_text(path.read_text().replace('rpy="0.1 0.2 0.3"', 'rpy="1.0 2.0 3.0"'))
    changed = GravityModel.from_urdf(
        path, root="base", branch="arm", expected_joint_names=("j1",), massless_links=("tcp",)
    )
    assert (
        changed.evaluate({"j1": 0.3}, gravity_m_s2=(2.0, 3.0, -9.0), payloads=()).holding_effort
        == baseline.holding_effort
    )


@pytest.mark.parametrize(
    ("before", "after", "match"),
    [
        ('value="2"', 'value="-2"', "positive"),
        ('xyz="0.5 0 0"', 'xyz="nan 0 0"', "finite"),
        ('<mass value="2"/>', "", "missing mass"),
        ('<link name="tcp"/>', '<link name="tcp"><visual/></link>', "empty frame"),
        ('joint="slide1"', 'joint="missing"', "mimic target"),
        ('joint="slide1"', 'joint="slide2"', "cyclic"),
        ('joint="slide1"', 'joint="j1"', "same moving joint type"),
        ('multiplier="0.5"', 'multiplier="inf"', "finite"),
        ('axis xyz="0 1 0"', 'axis xyz="0 0 0"', "axis"),
    ],
)
def test_invalid_mass_topology_and_mimic_rejected(tmp_path, before, after, match):
    path, _ = model_file(tmp_path, fingers=True)
    path.write_text(path.read_text().replace(before, after, 1))
    with pytest.raises(ValueError, match=match):
        GravityModel.from_urdf(
            path,
            root="base",
            branch="arm",
            expected_joint_names=("j1", "slide1"),
            massless_links=("tcp",),
        )


def test_missing_massless_declaration_and_missing_coordinates_rejected(tmp_path):
    path, model = model_file(tmp_path)
    with pytest.raises(ValueError, match="mass/COM"):
        GravityModel.from_urdf(
            path, root="base", branch="arm", expected_joint_names=("j1",), massless_links=()
        )
    for q in ({}, {"j1": True}, {"j1": float("nan")}, {"j1": 0.0, "gripper": 0.0}):
        with pytest.raises(ValueError, match="exact independent"):
            model.evaluate(q, gravity_m_s2=(0.0, 0.0, -10.0), payloads=())


def test_limits_reported_and_invalid_payloads_rejected(tmp_path):
    _, model = model_file(tmp_path)
    result = model.evaluate({"j1": 4.0}, gravity_m_s2=(0.0, 0.0, -10.0), payloads=())
    assert result.outside_model_limits == ("j1",)
    assert result.holding_effort == pytest.approx((-10 * math.cos(4),))
    for payload, message in (
        (PointMass("arm", "tcp", 1.0, (0.0, 0.0, 0.0)), "unique"),
        (PointMass("load", "unknown", 1.0, (0.0, 0.0, 0.0)), "attachment"),
    ):
        with pytest.raises(ValueError, match=message):
            model.evaluate({"j1": 0.0}, gravity_m_s2=(0.0, 0.0, -10.0), payloads=(payload,))
    with pytest.raises(ValueError, match="nonfinite"):
        model.evaluate({"j1": 0.0}, gravity_m_s2=(1e308, 1e308, -1e308), payloads=())


def test_seven_joint_gravity_subtraction_recovers_synthetic_six_axis_load(tmp_path):
    axes = ("0 0 1", "0 1 0", "1 0 0", "0 1 0", "0 0 1", "1 0 0", "0 1 0")
    offsets = ("0.1 0 0.2", "0.2 0 0", "0.1 0.1 0", "0.2 0 0", "0 0.1 0.1", "0.1 0 0", "0.1 0.1 0")
    parts = ['<robot name="seven"><link name="base"/>']
    for i, (axis, xyz) in enumerate(zip(axes, offsets, strict=True), 1):
        parts.append(link(f"link{i}", 0.5, "0.04 0.02 0.03"))
        parts.append(
            joint(
                f"joint_{i}",
                "revolute",
                "base" if i == 1 else f"link{i - 1}",
                f"link{i}",
                xyz,
                axis,
            )
        )
    parts.extend(
        ['<link name="tcp"/>', joint("tool", "fixed", "link7", "tcp", "0.1 0.03 0.08"), "</robot>"]
    )
    path = tmp_path / "seven.urdf"
    path.write_text("".join(parts))
    model = GravityModel.from_urdf(
        path,
        root="base",
        branch="link1",
        expected_joint_names=ARM_JOINT_NAMES,
        massless_links=("tcp",),
    )
    q = dict.fromkeys(ARM_JOINT_NAMES, 0.3)
    gravity = model.evaluate(q, gravity_m_s2=(0.0, 0.0, -9.80665), payloads=())
    chain = SerialChain.from_urdf(
        path, root="base", tip="tcp", expected_joint_names=ARM_JOINT_NAMES
    )
    geometry = chain.evaluate(q)
    expected = np.array([2.0, 3.0, -1.0, 0.1, -0.2, 0.3])
    motor = np.array(gravity.holding_effort) - np.array(geometry.jacobian).T @ expected
    external = np.array(gravity.holding_effort) - motor
    estimate = estimate_wrench(
        joint_names=ARM_JOINT_NAMES,
        jacobian=geometry.jacobian,
        external_joint_torque_nm=dict(zip(ARM_JOINT_NAMES, external, strict=True)),
        frame=geometry.frame,
        reference_point=geometry.reference_point,
        monotonic_ns=1,
        characteristic_length_m=0.2,
        damping_m=0.0,
        minimum_singular_value_m=1e-6,
        maximum_condition_number=1000.0,
        maximum_residual_nm=1e-8,
    )
    np.testing.assert_allclose((*estimate.force_n, *estimate.moment_nm), expected, atol=1e-12)
