"""Forward kinematics for the SO-101, read from the URDF.

PURE except for numpy and urdf_parser_py. No rclpy.

WHY THIS DERIVES RATHER THAN HARDCODES. The SO-101's joint origins come from a
CAD export: link offsets are not aligned to any axis (the elbow sits at
-0.11257, -0.028, 0) and the rpy values carry 1e-16 noise. Reading three link
lengths off that by hand is a transcription error waiting to happen, and a
wrong length fails silently as an arm that reaches slightly past the litter.

So: parse the chain, compose the transforms, and let extract_planar_arm derive
the planar parameters by evaluating the result. The IK then solves against
numbers that provably match the model it will command.
"""

import math
from dataclasses import dataclass

import numpy as np
from urdf_parser_py.urdf import URDF


@dataclass(frozen=True)
class Joint:
    name: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    movable: bool


@dataclass(frozen=True)
class Chain:
    joints: list[Joint]

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.joints if j.movable]


@dataclass(frozen=True)
class PlanarArm:
    """The three pitch joints reduced to a planar 3R arm.

    Tip position in the pan plane, for pitch values (t1, t2, t3):
        r = origin_r + l1*cos(a1+t1) + l2*cos(a2+t1+t2) + l3*cos(a3+t1+t2+t3)
        z = origin_z + l1*sin(a1+t1) + l2*sin(a2+t1+t2) + l3*sin(a3+t1+t2+t3)

    The a_i are the zero-pose angles of each link, which absorb the CAD frame
    rotations so the IK never has to know about them.
    """

    l1: float
    l2: float
    l3: float
    a1: float
    a2: float
    a3: float
    origin_r: float
    origin_z: float


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle_to_matrix(axis: tuple[float, float, float],
                          angle: float) -> np.ndarray:
    a = np.array(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm == 0.0:
        return np.eye(3)
    a = a / norm
    k = np.array([[0.0, -a[2], a[1]],
                  [a[2], 0.0, -a[0]],
                  [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _transform(rotation: np.ndarray,
               translation: tuple[float, float, float]) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def load_chain(urdf_xml: str, base_link: str, tip_link: str) -> Chain:
    robot = URDF.from_xml_string(urdf_xml)

    child_to_joint = {j.child: j for j in robot.joints}

    # Walk up from the tip: the URDF is a tree, so each link has exactly one
    # parent joint, which makes this unambiguous. Walking down would require
    # searching every branch.
    reversed_joints: list[Joint] = []
    link = tip_link
    while link != base_link:
        if link not in child_to_joint:
            raise ValueError(f"{tip_link} does not descend from {base_link}")
        j = child_to_joint[link]
        origin_xyz = tuple(j.origin.xyz) if j.origin else (0.0, 0.0, 0.0)
        origin_rpy = tuple(j.origin.rpy) if j.origin else (0.0, 0.0, 0.0)
        reversed_joints.append(Joint(
            name=j.name,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=tuple(j.axis) if j.axis else (0.0, 0.0, 1.0),
            movable=j.type in ("revolute", "continuous", "prismatic"),
        ))
        link = j.parent

    reversed_joints.reverse()
    return Chain(joints=reversed_joints)


def forward_kinematics(
    chain: Chain, joint_values: dict[str, float],
) -> np.ndarray:
    """4x4 transform from the chain's base link to its tip. Joints absent from
    joint_values are treated as zero."""
    result = np.eye(4)
    for j in chain.joints:
        result = result @ _transform(
            _rpy_to_matrix(*j.origin_rpy), j.origin_xyz)
        if j.movable:
            angle = float(joint_values.get(j.name, 0.0))
            result = result @ _transform(
                _axis_angle_to_matrix(j.axis, angle), (0.0, 0.0, 0.0))
    return result


def extract_planar_arm(chain: Chain) -> PlanarArm:
    """Derive the planar 3R parameters by evaluating FK at the zero pose.

    The three pitch joints have parallel axes, so the tip traces a plane that
    rotates rigidly with shoulder_pan. Measuring each link as the vector
    between consecutive joint origins - in (radius, height) coordinates -
    yields lengths and zero-pose angles that reproduce full FK exactly. The
    test asserts precisely that.
    """
    names = chain.joint_names
    pan, lift, elbow, flex = names[0], names[1], names[2], names[3]

    def origin_of(joint_name: str) -> np.ndarray:
        """Position of a joint's frame origin, in the base link."""
        result = np.eye(4)
        for j in chain.joints:
            result = result @ _transform(
                _rpy_to_matrix(*j.origin_rpy), j.origin_xyz)
            if j.name == joint_name:
                return result[:3, 3]
            if j.movable:
                result = result @ _transform(np.eye(3), (0.0, 0.0, 0.0))
        raise ValueError(f"joint {joint_name} not in chain")

    def tip() -> np.ndarray:
        return forward_kinematics(chain, {})[:3, 3]

    def planar(p: np.ndarray) -> tuple[float, float]:
        return (math.hypot(p[0], p[1]), p[2])

    _ = pan  # the pan joint defines the plane; it contributes no link length

    p0 = planar(origin_of(lift))
    p1 = planar(origin_of(elbow))
    p2 = planar(origin_of(flex))
    p3 = planar(tip())

    def link(a: tuple[float, float],
             b: tuple[float, float]) -> tuple[float, float]:
        dr, dz = b[0] - a[0], b[1] - a[1]
        return (math.hypot(dr, dz), math.atan2(dz, dr))

    l1, a1 = link(p0, p1)
    l2, a2 = link(p1, p2)
    l3, a3 = link(p2, p3)

    return PlanarArm(
        l1=l1, l2=l2, l3=l3, a1=a1, a2=a2, a3=a3,
        origin_r=p0[0], origin_z=p0[1],
    )
