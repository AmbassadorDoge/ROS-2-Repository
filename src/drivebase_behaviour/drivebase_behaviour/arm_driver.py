"""Publishes the six arm joint positions, and sequences timed poses.

Thin on purpose: the six gz JointPositionController plugins (and the Feetech
bus on hardware) accept a position and hold it, so there is nothing to close a
loop around here. Timing is open-loop because the descent is open-loop - by the
time the gripper is at grab range the target is occluded and inside the ToF's
minimum range. See the design doc §6.
"""

from rclpy.node import Node
from std_msgs.msg import Float64

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class ArmDriver:
    def __init__(self, node: Node, wrist_roll_limit: float) -> None:
        self.node = node
        # Software limit, tighter than the URDF's +-2.84 rad. A harness
        # carrying the pod's camera and ToF cable through wrist_roll will not
        # survive its full travel; the URDF limit describes the servo, not the
        # cable.
        self.wrist_roll_limit = abs(wrist_roll_limit)
        self.publishers = {
            j: node.create_publisher(Float64, f"/arm/{j}/position", 10)
            for j in JOINTS
        }
        self._steps: list[tuple[dict[str, float], float]] = []
        self._step_index = 0
        self._step_deadline = 0.0
        self.sequence_done = True

    def go_to(self, pose: dict[str, float]) -> None:
        for joint, value in pose.items():
            if joint not in self.publishers:
                continue
            if joint == "wrist_roll":
                value = max(-self.wrist_roll_limit,
                            min(self.wrist_roll_limit, value))
            message = Float64()
            message.data = float(value)
            self.publishers[joint].publish(message)

    def start_sequence(
        self, steps: list[tuple[dict[str, float], float]],
    ) -> None:
        self._steps = list(steps)
        self._step_index = -1
        self._step_deadline = 0.0
        self.sequence_done = not self._steps

    def update(self, now: float) -> None:
        if self.sequence_done:
            return
        if self._step_index >= 0 and now < self._step_deadline:
            return

        self._step_index += 1
        if self._step_index >= len(self._steps):
            self.sequence_done = True
            return

        pose, hold = self._steps[self._step_index]
        self.go_to(pose)
        self._step_deadline = now + hold


def _pose_of(solution) -> dict[str, float]:
    return {
        "shoulder_pan": solution.shoulder_pan,
        "shoulder_lift": solution.shoulder_lift,
        "elbow_flex": solution.elbow_flex,
        "wrist_flex": solution.wrist_flex,
        "wrist_roll": 0.0,
    }


def grasp_sequence(
    grasp,
    approach,
    search_pose: dict[str, float],
    gripper_open: float,
    gripper_closed: float,
    hold_seconds: float,
) -> list[tuple[dict[str, float], float]]:
    """Descend vertically onto the target, close, lift vertically, return.

    BOTH POSES ARE SOLVED BY IK. `approach` is the solution for a point
    directly above `grasp`, so the move between them is a straight vertical
    line and the jaws come down around the litter rather than across it.

    This used to build the approach by subtracting a constant from
    shoulder_lift, which reads like a vertical offset and is not one. Measured
    at the centre of the grasp band: that perturbation raised the tip 56 mm but
    also swung it 105 mm OUTWARD, so the "descent" was a diagonal rake that
    swept the open jaws sideways through the litter. In sim it launched a 15 g
    can 1.6 m across the field, which looked like a physics or collision fault
    and was neither.

    Pre-grasp comes first with the gripper already open: opening it after
    arriving would sweep the jaws through the litter just as surely.
    """
    reach = _pose_of(grasp)
    clear = _pose_of(approach)

    return [
        (dict(clear, gripper=gripper_open), hold_seconds),
        (dict(reach, gripper=gripper_open), hold_seconds),
        (dict(reach, gripper=gripper_closed), hold_seconds),
        (dict(clear, gripper=gripper_closed), hold_seconds),
        (dict(search_pose, gripper=gripper_closed), hold_seconds),
    ]
