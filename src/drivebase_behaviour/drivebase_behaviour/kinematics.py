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

    Tip position in the pan plane, for planar angles (q1, q2, q3):
        r = origin_r + l1*cos(a1+q1) + l2*cos(a2+q1+q2) + l3*cos(a3+q1+q2+q3)
        z = origin_z + l1*sin(a1+q1) + l2*sin(a2+q1+q2) + l3*sin(a3+q1+q2+q3)

    The a_i are the zero-pose angles of each link, which absorb the CAD frame
    rotations so the IK never has to know about them.

    The q_i are *planar* angles, positive from r toward z. Joint commands are
    q_i = sense * t_i, because this arm's pitch joints turn the other way (see
    `sense`). `tip` takes joint values and applies that itself; anything
    solving in planar angles must convert before commanding the arm.

    WHAT (r, z) MEANS, because two offsets make the obvious reading wrong.

    `r` is NOT `hypot(x, y)` in `arm_base_link`, for two independent reasons:

    1. The pan axis does not pass through the base origin. It sits 38.8 mm
       along +x, so a radius measured from the origin is not conserved when
       the arm pans, and the IK would aim at a moving target.
    2. The three pitch joints run in a plane 18.3 mm to one side of the pan
       axis, while the tip comes back to the axis plane at the wrist (the
       gripper is deliberately centred). An unsigned `hypot` mixes those two
       planes together and no set of link lengths can reconcile them.

    So `r` is the *signed projection* onto `radial_xy` — the in-plane radial
    direction, taken from the pan axis toward the zero-pose tip — measured
    from `axis_xy`. Projecting rather than taking a magnitude is what drops
    the constant lateral offset, which is legitimate precisely because it is
    constant: every pitch axis is parallel to it, and a rotation about an axis
    cannot change displacement along that axis. `z` is plain base-link height.

    Use `to_planar` to convert a base-link point rather than reconstructing
    this by hand.
    """

    l1: float
    l2: float
    l3: float
    a1: float
    a2: float
    a3: float
    origin_r: float
    origin_z: float
    axis_xy: tuple[float, float]
    radial_xy: tuple[float, float]
    sense: float
    """+1 if a positive joint command raises the planar angle, -1 if it lowers
    it. Derived from the joint axes, not assumed: it is -1 on this arm."""
    pan_sense: float
    """+1 if a positive pan command rotates the plane counter-clockwise seen
    from +z, -1 otherwise. Also -1 on this arm — the pan axis points down."""

    def to_planar(self, point, pan: float = 0.0) -> tuple[float, float]:
        """Base-link (x, y, z) to planar (r, z), for the arm panned to `pan`.

        `pan` matters because r is a projection onto the radial direction, and
        panning turns that direction. Projecting rather than taking
        `hypot(dx, dy)` is what discards the pitch chain's constant lateral
        offset; see the class docstring. For a point already on the pan plane
        the two agree, but the joint origins are not on it.
        """
        angle = self.pan_sense * pan
        c, s = math.cos(angle), math.sin(angle)
        ux = self.radial_xy[0] * c - self.radial_xy[1] * s
        uy = self.radial_xy[0] * s + self.radial_xy[1] * c
        dx = point[0] - self.axis_xy[0]
        dy = point[1] - self.axis_xy[1]
        return (dx * ux + dy * uy, point[2])

    def bearing(self, point) -> float:
        """The pan command that turns the plane to face `point`.

        The inverse of the pan half of `to_planar`: the angle is measured from
        the pan *axis*, not the base origin, and `pan_sense` converts a
        counter-clockwise bearing into a joint command.
        """
        dx = point[0] - self.axis_xy[0]
        dy = point[1] - self.axis_xy[1]
        zero = math.atan2(self.radial_xy[1], self.radial_xy[0])
        return self.pan_sense * _wrap(math.atan2(dy, dx) - zero)

    def from_planar(self, r: float, z: float,
                    pan: float = 0.0) -> tuple[float, float, float]:
        """Planar (r, z) back to a base-link point, for the arm at `pan`.

        The exact inverse of `to_planar`, and only well defined in that
        direction: `to_planar` discards the component perpendicular to the
        plane, so this returns the one point on the plane that maps back. That
        is not a limitation in practice — the tip lies on the plane to within
        the model's own 1e-6 noise, because the gripper is centred on the pan
        axis.
        """
        angle = self.pan_sense * pan
        c, s = math.cos(angle), math.sin(angle)
        ux = self.radial_xy[0] * c - self.radial_xy[1] * s
        uy = self.radial_xy[0] * s + self.radial_xy[1] * c
        return (self.axis_xy[0] + r * ux, self.axis_xy[1] + r * uy, z)

    def tip(self, t1: float, t2: float, t3: float) -> tuple[float, float]:
        """Planar (r, z) of the tip, for the three pitch *joint* values."""
        q1, q2, q3 = self.sense * t1, self.sense * t2, self.sense * t3
        r = self.origin_r + (
            self.l1 * math.cos(self.a1 + q1)
            + self.l2 * math.cos(self.a2 + q1 + q2)
            + self.l3 * math.cos(self.a3 + q1 + q2 + q3)
        )
        z = self.origin_z + (
            self.l1 * math.sin(self.a1 + q1)
            + self.l2 * math.sin(self.a2 + q1 + q2)
            + self.l3 * math.sin(self.a3 + q1 + q2 + q3)
        )
        return (r, z)


def _wrap(angle: float) -> float:
    """To (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


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

    See PlanarArm for why those coordinates are anchored on the pan axis and
    taken as a signed projection.
    """
    names = chain.joint_names
    pan, lift, elbow, flex = names[0], names[1], names[2], names[3]

    def frame_of(joint_name: str) -> np.ndarray:
        """A joint's 4x4 frame at the zero pose, in the base link."""
        result = np.eye(4)
        for j in chain.joints:
            result = result @ _transform(
                _rpy_to_matrix(*j.origin_rpy), j.origin_xyz)
            if j.name == joint_name:
                return result
        raise ValueError(f"joint {joint_name} not in chain")

    def origin_of(joint_name: str) -> np.ndarray:
        return frame_of(joint_name)[:3, 3]

    def axis_of(joint_name: str) -> np.ndarray:
        """A joint's rotation axis at the zero pose, in the base link."""
        f = frame_of(joint_name)
        axis = next(j.axis for j in chain.joints if j.name == joint_name)
        return f[:3, :3] @ np.array(axis, dtype=float)

    def tip() -> np.ndarray:
        return forward_kinematics(chain, {})[:3, 3]

    # The pan joint contributes no link length; it fixes where the plane is
    # hinged, which is the one thing the old origin-anchored version got wrong.
    axis_point = origin_of(pan)
    axis_xy = (float(axis_point[0]), float(axis_point[1]))

    tip_at_zero = tip()
    radial = np.array([tip_at_zero[0] - axis_xy[0],
                       tip_at_zero[1] - axis_xy[1]])
    norm = float(np.linalg.norm(radial))
    if norm == 0.0:
        raise ValueError("zero-pose tip lies on the pan axis; radial "
                         "direction is undefined")
    radial_xy = (float(radial[0] / norm), float(radial[1] / norm))

    def planar(p: np.ndarray) -> tuple[float, float]:
        dx, dy = p[0] - axis_xy[0], p[1] - axis_xy[1]
        return (dx * radial_xy[0] + dy * radial_xy[1], p[2])

    p0 = planar(origin_of(lift))
    p1 = planar(origin_of(elbow))
    p2 = planar(origin_of(flex))
    p3 = planar(tip_at_zero)

    def link(a: tuple[float, float],
             b: tuple[float, float]) -> tuple[float, float]:
        dr, dz = b[0] - a[0], b[1] - a[1]
        return (math.hypot(dr, dz), math.atan2(dz, dr))

    l1, a1 = link(p0, p1)
    l2, a2 = link(p1, p2)
    l3, a3 = link(p2, p3)

    # Which way a positive joint command turns the arm within the plane. The
    # plane's own positive sense is r toward z, which is a rotation about
    # e_r x e_z; the pitch joints on this arm turn about +y, the opposite way,
    # so a positive command *decreases* the planar angle. Getting this
    # backwards is invisible at the zero pose and wrong everywhere else.
    plane_normal = np.array([radial_xy[1], -radial_xy[0], 0.0])
    senses = [float(np.dot(axis_of(name), plane_normal))
              for name in (lift, elbow, flex)]
    if not all(abs(abs(s) - 1.0) < 1e-3 for s in senses):
        raise ValueError(
            f"pitch axes are not perpendicular to the arm plane: {senses}")
    if not (all(s > 0 for s in senses) or all(s < 0 for s in senses)):
        raise ValueError(
            f"pitch joints do not share a rotation sense: {senses}")

    pan_dot = float(np.dot(axis_of(pan), np.array([0.0, 0.0, 1.0])))
    if abs(abs(pan_dot) - 1.0) > 1e-3:
        raise ValueError(
            f"pan axis is not vertical in the base link: {pan_dot}")

    return PlanarArm(
        l1=l1, l2=l2, l3=l3, a1=a1, a2=a2, a3=a3,
        origin_r=p0[0], origin_z=p0[1],
        axis_xy=axis_xy, radial_xy=radial_xy,
        sense=math.copysign(1.0, senses[0]),
        pan_sense=math.copysign(1.0, pan_dot),
    )
