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


def test_pan_rotates_the_tip_about_the_pan_axis(chain):
    """Panning preserves radius and height — about the pan axis, which is
    38.8 mm off the base origin, not about the origin itself.

    Tolerance is 1e-5 rather than the 1e-9 you would expect from exact
    geometry: the URDF's CAD export writes pi as 3.14159, which tilts the pan
    axis 2.65e-6 rad off base z and moves the tip ~1e-6 m over a pan. That is
    a property of the model, not of this code.
    """
    arm = extract_planar_arm(chain)
    ax, ay = arm.axis_xy

    at_zero = forward_kinematics(chain, {})[:3, 3]
    r_zero = math.hypot(at_zero[0] - ax, at_zero[1] - ay)

    for angle in (math.pi / 2, 1.0, -0.7):
        panned = forward_kinematics(chain, {"arm_shoulder_pan": angle})[:3, 3]
        assert math.hypot(panned[0] - ax, panned[1] - ay) == pytest.approx(
            r_zero, abs=1e-5)
        assert panned[2] == pytest.approx(at_zero[2], abs=1e-5)


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
        r_actual, z_actual = arm.to_planar(tip)
        r_model, z_model = arm.tip(t1, t2, t3)

        assert r_model == pytest.approx(r_actual, abs=1e-5)
        assert z_model == pytest.approx(z_actual, abs=1e-5)


def test_planar_model_is_independent_of_pan(chain):
    """The whole point of the planar reduction: pan changes where the plane
    points, never where the tip sits within it. If (r, z) drifted with pan,
    the IK would need a different solution per pan angle.
    """
    arm = extract_planar_arm(chain)
    pitch = (0.4, -0.6, 0.3)
    expected = arm.tip(*pitch)

    for pan in (0.0, 0.8, -1.2, math.pi / 2):
        values = dict(zip(PITCH_JOINTS, pitch))
        values["arm_shoulder_pan"] = pan
        tip = forward_kinematics(chain, values)[:3, 3]
        assert arm.to_planar(tip, pan) == pytest.approx(expected, abs=1e-5)
