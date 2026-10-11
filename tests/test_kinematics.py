from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

from vlai_l1_runtime.kinematics import SerialChain
from vlai_l1_runtime.wrench import ARM_JOINT_NAMES, estimate_wrench


def write_chain(tmp_path, *, axes, xyzs, rpys=None):
    rpys = rpys or ["0 0 0"] * len(axes)
    elements = ['<robot name="synthetic"><link name="base"/>']
    parent = "base"
    names = []
    for index, (axis, xyz, rpy) in enumerate(zip(axes, xyzs, rpys, strict=True), 1):
        name = f"joint_{index}"
        child = f"link{index}"
        names.append(name)
        elements.append(f'''
        <link name="{child}"/>
        <joint name="{name}" type="revolute">
          <parent link="{parent}"/><child link="{child}"/>
          <origin xyz="{xyz}" rpy="{rpy}"/><axis xyz="{axis}"/>
          <limit lower="-3.14" upper="3.14"/>
        </joint>''')
        parent = child
    elements.append(f'''
    <link name="tcp"/>
    <joint name="tool" type="fixed">
      <parent link="{parent}"/><child link="tcp"/>
      <origin xyz="0.15 0.03 0.08" rpy="0.1 0.2 0.3"/>
    </joint></robot>''')
    path = tmp_path / "model.urdf"
    path.write_text("".join(elements))
    chain = SerialChain.from_urdf(path, root="base", tip="tcp", expected_joint_names=tuple(names))
    return path, chain


def seven_joint_chain(tmp_path):
    return write_chain(
        tmp_path,
        axes=["0 0 1", "0 1 0", "1 0 0", "0 1 0", "0 0 1", "1 0 0", "0 1 0"],
        xyzs=["0.1 0 0.2", "0.2 0 0", "0.1 0.1 0", "0.2 0 0", "0 0.1 0.1", "0.1 0 0", "0.1 0.1 0"],
        rpys=["0.1 0.2 0.3", "0.2 0.1 -0.1", "0 0 0", "0.1 0 0", "0 0.2 0", "0 0 0.1", "0 0 0"],
    )


def test_single_joint_analytic_position_axis_and_jacobian(tmp_path):
    _, chain = write_chain(tmp_path, axes=["0 0 2"], xyzs=["0 0 1"])
    geometry = chain.evaluate({"joint_1": math.pi / 2})
    np.testing.assert_allclose(geometry.position_m, (-0.03, 0.15, 1.08), atol=1e-14)
    np.testing.assert_allclose(
        np.array(geometry.jacobian)[:, 0], (-0.15, -0.03, 0, 0, 0, 1), atol=1e-14
    )


def test_joint_origin_rotation_already_encodes_mount_offset(tmp_path):
    _, chain = write_chain(tmp_path, axes=["-1 0 0"], xyzs=["0 0 0"], rpys=[f"{math.pi / 2} 0 0"])
    geometry = chain.evaluate({"joint_1": 0.0})
    np.testing.assert_allclose(geometry.position_m, (0.15, -0.08, 0.03), atol=1e-14)
    np.testing.assert_allclose(np.array(geometry.jacobian)[3:, 0], (-1, 0, 0), atol=1e-14)


def test_full_geometric_jacobian_matches_finite_differences(tmp_path):
    _, chain = seven_joint_chain(tmp_path)
    q = dict(zip(chain.joint_names, (0.1, -0.3, 0.4, 0.5, -0.2, 0.3, 0.1), strict=True))
    geometry = chain.evaluate(q)
    jacobian = np.asarray(geometry.jacobian)
    epsilon = 1e-6
    for index, name in enumerate(chain.joint_names):
        plus = chain.evaluate(q | {name: q[name] + epsilon})
        minus = chain.evaluate(q | {name: q[name] - epsilon})
        velocity = (np.array(plus.position_m) - minus.position_m) / (2 * epsilon)
        rotation_dot = (np.array(plus.rotation) - minus.rotation) / (2 * epsilon)
        skew = rotation_dot @ np.array(geometry.rotation).T
        angular = (skew[2, 1], skew[0, 2], skew[1, 0])
        np.testing.assert_allclose(jacobian[:, index], (*velocity, *angular), atol=1e-9)


def test_tool_point_shift_changes_linear_jacobian_and_wrench_moment(tmp_path):
    path, chain = seven_joint_chain(tmp_path)
    hand = SerialChain.from_urdf(
        path, root="base", tip="link7", expected_joint_names=chain.joint_names
    )
    q = dict.fromkeys(chain.joint_names, 0.3)
    tool_geometry, hand_geometry = chain.evaluate(q), hand.evaluate(q)
    offset = np.array(tool_geometry.position_m) - hand_geometry.position_m
    hand_j, tool_j = np.array(hand_geometry.jacobian), np.array(tool_geometry.jacobian)
    for index in range(7):
        np.testing.assert_allclose(
            tool_j[:3, index], hand_j[:3, index] + np.cross(hand_j[3:, index], offset)
        )
    wrench_at_tool = np.array([2, 3, 4, 0.1, 0.2, 0.3])
    wrench_at_hand = wrench_at_tool.copy()
    wrench_at_hand[3:] += np.cross(offset, wrench_at_tool[:3])
    np.testing.assert_allclose(tool_j.T @ wrench_at_tool, hand_j.T @ wrench_at_hand)


def test_model_to_wrench_pipeline_recovers_known_load(tmp_path):
    _, chain = seven_joint_chain(tmp_path)
    geometry = chain.evaluate(dict.fromkeys(chain.joint_names, 0.3))
    expected = np.array([2.0, 3.0, -1.0, 0.1, -0.2, 0.3])
    torque = np.asarray(geometry.jacobian).T @ expected
    estimate = estimate_wrench(
        joint_names=ARM_JOINT_NAMES,
        jacobian=geometry.jacobian,
        external_joint_torque_nm=dict(zip(ARM_JOINT_NAMES, torque, strict=True)),
        frame=geometry.frame,
        reference_point=geometry.reference_point,
        monotonic_ns=1,
        characteristic_length_m=0.2,
        damping_m=0.0,
        minimum_singular_value_m=1e-6,
        maximum_condition_number=1000,
        maximum_residual_nm=1e-8,
    )
    np.testing.assert_allclose((*estimate.force_n, *estimate.moment_nm), expected, atol=1e-12)


def test_outside_limits_is_reported_without_clamping(tmp_path):
    _, chain = write_chain(tmp_path, axes=["0 0 1"], xyzs=["0 0 0"])
    narrow = replace(chain, joints=(replace(chain.joints[0], lower_rad=0.0), chain.joints[1]))
    actual = narrow.evaluate({"joint_1": -0.1})
    assert actual.outside_model_limits == ("joint_1",)
    assert actual.position_m != narrow.evaluate({"joint_1": 0.0}).position_m


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True, "0.1"])
def test_invalid_joint_values_rejected(tmp_path, value):
    _, chain = write_chain(tmp_path, axes=["0 0 1"], xyzs=["0 0 0"])
    with pytest.raises(ValueError, match="finite"):
        chain.evaluate({"joint_1": value})


def test_wrong_joint_names_rejected(tmp_path):
    path, chain = write_chain(tmp_path, axes=["0 0 1"], xyzs=["0 0 0"])
    with pytest.raises(ValueError, match="exact expected"):
        SerialChain.from_urdf(path, root="base", tip="tcp", expected_joint_names=("wrong",))
    with pytest.raises(ValueError, match="exactly"):
        chain.evaluate({"wrong": 0.0})


@pytest.mark.parametrize(
    ("before", "after", "match"),
    [
        ('axis xyz="0 0 1"', 'axis xyz="0 0 0"', "axis"),
        ('rpy="0 0 0"', 'rpy="nan 0 0"', "finite"),
        ('type="revolute"', 'type="prismatic"', "supported"),
        ('lower="-3.14"', 'lower="4"', "limits"),
        ('<axis xyz="0 0 1"/>', "", "missing axis"),
        ('<robot name="synthetic">', '<robot name="synthetic"><link name="base"/>', "unique"),
        ('<parent link="base"/>', '<parent link="tcp"/>', "acyclic"),
        ('<parent link="base"/>', '<parent link="missing"/>', "unknown"),
    ],
)
def test_malformed_models_rejected(tmp_path, before, after, match):
    path, chain = write_chain(tmp_path, axes=["0 0 1"], xyzs=["0 0 0"])
    text = path.read_text()
    assert before in text
    path.write_text(text.replace(before, after, 1))
    with pytest.raises(ValueError, match=match):
        SerialChain.from_urdf(path, root="base", tip="tcp", expected_joint_names=chain.joint_names)
