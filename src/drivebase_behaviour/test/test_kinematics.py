import math
import pathlib
import subprocess

import numpy as np
import pytest

from drivebase_behaviour.kinematics import (
    extract_planar_arm,
    forward_kinematics,
    load_chain,
)

PITCH_JOINTS = ["arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex"]


@pytest.fixture(scope="module")
def urdf_xml():
    repo = pathlib.Path(__file__).resolve().parents[3]
    xacro = repo / "src/drivebase_description/urdf/drivebase.urdf.xacro"
    return subprocess.run(
        ["xacro", str(xacro), "use_sim:=true", "use_arm:=true",
         "wheel_mu1:=1.0", "wheel_mu2:=0.6"],
        capture_output=True, text=True, check=True,
    ).stdout


@pytest.fixture(scope="module")
def chain(urdf_xml):
    return load_chain(urdf_xml, "arm_base_link", "arm_gripper_frame_link")


def test_chain_finds_the_five_movable_joints_in_order(chain):
    assert chain.joint_names == [
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll",
    ]


def test_fk_at_zero_is_a_valid_transform(chain):
    t = forward_kinematics(chain, {})
    assert t.shape == (4, 4)
    assert t[3, :] == pytest.approx([0, 0, 0, 1])
    # Rotation block must be orthonormal.
    r = t[:3, :3]
    assert (r @ r.T) == pytest.approx(np.eye(3), abs=1e-9)


def test_pan_rotates_the_tip_about_the_base_z_axis(chain):
    at_zero = forward_kinematics(chain, {})[:3, 3]
    quarter = forward_kinematics(
        chain, {"arm_shoulder_pan": math.pi / 2})[:3, 3]
    # Radius from the pan axis is preserved; height is preserved.
    assert math.hypot(*quarter[:2]) == pytest.approx(
        math.hypot(*at_zero[:2]), abs=1e-9)
    assert quarter[2] == pytest.approx(at_zero[2], abs=1e-9)


def test_planar_link_lengths_are_physically_sensible(chain):
    arm = extract_planar_arm(chain)
    # The SO-101 is a desktop arm: every segment is between 3 and 25 cm.
    for length in (arm.l1, arm.l2, arm.l3):
        assert 0.03 < length < 0.25
    # Total reach must be enough to get from a 260 mm mount to the ground,
    # which STATUS.md measured as a 219 mm drop.
    assert arm.l1 + arm.l2 + arm.l3 > 0.219


def test_planar_model_reproduces_full_fk(chain):
    """The planar model is only useful if it agrees with real FK.

    For any pitch-joint triple, the planar (r, z) prediction must match the
    radius and height the full 4x4 chain produces. This is the test that
    catches a wrong angle offset, which no amount of eyeballing the URDF will.
    """
    arm = extract_planar_arm(chain)
    for t1, t2, t3 in [
        (0.0, 0.0, 0.0),
        (0.3, -0.4, 0.2),
        (-0.5, 0.6, -0.3),
        (1.0, -1.0, 0.5),
    ]:
        values = dict(zip(PITCH_JOINTS, (t1, t2, t3)))
        tip = forward_kinematics(chain, values)[:3, 3]
        r_actual = math.hypot(tip[0], tip[1])
        z_actual = tip[2]

        r_model = arm.origin_r + (
            arm.l1 * math.cos(arm.a1 + t1)
            + arm.l2 * math.cos(arm.a2 + t1 + t2)
            + arm.l3 * math.cos(arm.a3 + t1 + t2 + t3)
        )
        z_model = arm.origin_z + (
            arm.l1 * math.sin(arm.a1 + t1)
            + arm.l2 * math.sin(arm.a2 + t1 + t2)
            + arm.l3 * math.sin(arm.a3 + t1 + t2 + t3)
        )
        assert r_model == pytest.approx(r_actual, abs=1e-6)
        assert z_model == pytest.approx(z_actual, abs=1e-6)
