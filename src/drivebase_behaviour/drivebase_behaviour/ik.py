"""Closed-form inverse kinematics for the SO-101.

PURE. No rclpy, no numpy.

WHY CLOSED FORM. shoulder_lift, elbow_flex and wrist_flex have parallel axes,
so once shoulder_pan aims the plane the problem is a planar 3R arm. Fixing the
approach direction removes the one redundant degree of freedom and what remains
is the textbook 2R solve. Tens of microseconds, no solver, no MoveIt on a Pi
that is already sharing cores with YOLO.

The approach angle is fixed by the caller rather than chosen here: coming down
vertically onto litter is what the gripper geometry wants, and making it a
parameter keeps that a decision of the grasp sequence rather than of the maths.

TWO CONVENTIONS THIS MODULE DOES NOT INVENT, both from `PlanarArm`:

- `approach_angle` is a *planar* angle — the direction of the last link in the
  (r, z) plane, positive from r toward z. Straight down is -pi/2. It is not a
  joint value and does not carry the arm's sign flip.
- Planar angles are converted to joint commands with `arm.sense`, which is -1
  on this arm. Skipping that is invisible at the zero pose and wrong
  everywhere else; see PROGRESS.md, "The arm's planar frame".
"""

import math
from dataclasses import dataclass

from drivebase_behaviour.kinematics import PlanarArm

# From so101_macro.xacro; test_ik.py asserts they still match the URDF.
# wrist_roll and gripper are not solved for - roll is held near zero to protect
# the sensor pod harness, and the gripper is a grasp command, not a pose.
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


@dataclass(frozen=True)
class IkSolution:
    shoulder_pan: float
    shoulder_lift: float
    elbow_flex: float
    wrist_flex: float


def _within(name: str, value: float,
            limits: dict[str, tuple[float, float]]) -> bool:
    low, high = limits[name]
    return low <= value <= high


def solve(
    arm: PlanarArm,
    target_xyz: tuple[float, float, float],
    approach_angle: float,
    limits: dict[str, tuple[float, float]],
) -> IkSolution | None:
    """Returns None for anything unreachable or outside a joint limit.

    Never clamps. A clamped solution aims the arm somewhere other than the
    commanded point while reporting success, which on this robot means closing
    the gripper on empty ground and reporting a pickup.

    `target_xyz` is in `arm_base_link`. Its height is taken as given and its
    bearing sets the pan; any component off the resulting plane is dropped by
    the projection, so a point the arm cannot lie in is silently solved as its
    nearest in-plane point. That is the correct behaviour here - the plane is
    two-dimensional and the tip is centred on it - but it means the caller
    owns the target's accuracy, not this function.
    """
    pan = arm.bearing(target_xyz)
    if not _within("shoulder_pan", pan, limits):
        return None

    r, z = arm.to_planar(target_xyz, pan)

    # Back off the last link along the approach direction to find where the
    # wrist_flex joint must sit. That reduces 3R to 2R.
    wrist_r = r - arm.origin_r - arm.l3 * math.cos(approach_angle)
    wrist_z = z - arm.origin_z - arm.l3 * math.sin(approach_angle)

    distance = math.hypot(wrist_r, wrist_z)
    if distance > arm.l1 + arm.l2 or distance < abs(arm.l1 - arm.l2):
        return None

    # Law of cosines. The clamp guards floating-point drift at full extension
    # only - the reachability test above has already rejected genuine misses,
    # so this cannot mask an out-of-range target.
    cos_elbow = (
        distance * distance - arm.l1 * arm.l1 - arm.l2 * arm.l2
    ) / (2.0 * arm.l1 * arm.l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))

    # Both elbow branches are exact solutions of the same target, so trying the
    # second is not a relaxation - it reaches the commanded point just as
    # precisely. Elbow-down is preferred because the other branch folds the arm
    # back over itself, which on a front-mounted arm means driving the elbow
    # toward the chassis; it is accepted only when elbow-down breaks a limit.
    for elbow_interior in (-math.acos(cos_elbow), math.acos(cos_elbow)):
        solution = _finish(arm, wrist_r, wrist_z, elbow_interior,
                           approach_angle, pan, limits)
        if solution is not None:
            return solution
    return None


def _finish(
    arm: PlanarArm,
    wrist_r: float,
    wrist_z: float,
    elbow_interior: float,
    approach_angle: float,
    pan: float,
    limits: dict[str, tuple[float, float]],
) -> IkSolution | None:
    """One elbow branch, in planar angles, checked against the limits."""
    beta = math.atan2(
        arm.l2 * math.sin(elbow_interior),
        arm.l1 + arm.l2 * math.cos(elbow_interior),
    )
    link1_angle = math.atan2(wrist_z, wrist_r) - beta

    # Undo the zero-pose offsets absorbed into a1/a2/a3 by extract_planar_arm,
    # turning absolute link directions back into per-joint planar angles.
    q1 = link1_angle - arm.a1
    q2 = (link1_angle + elbow_interior) - arm.a2 - q1
    q3 = approach_angle - arm.a3 - q1 - q2

    # Planar angles to joint commands. arm.sense is -1 here.
    lift, elbow, flex = arm.sense * q1, arm.sense * q2, arm.sense * q3

    for name, value in [("shoulder_lift", lift),
                        ("elbow_flex", elbow),
                        ("wrist_flex", flex)]:
        if not _within(name, value, limits):
            return None

    return IkSolution(
        shoulder_pan=pan,
        shoulder_lift=lift,
        elbow_flex=elbow,
        wrist_flex=flex,
    )
