#!/usr/bin/env python3
"""Drive the arm through the poses the grasp uses, holding each one to be shot.

Produces the figures behind the arm-workspace notes. Run it with the GUI up,
and grab the window from the HOST while it holds each pose:

    bash scripts/dev.sh bash -c \
      'ros2 launch drivebase_sim sim.launch.py headless:=false arm:=true & \
       sleep 32; python3 /ws/scripts/capture_arm_poses.py'
    # meanwhile, on the host:
    import -window root -display :1 shot.png

DO NOT USE /gui/screenshot. It replies `data: true` and writes no file - the
service is registered whether or not anything is behind it, so the success
reply means only that the request was accepted. Another silent one.

Timing is fixed and printed so a host-side capture loop can line up with it.
Prints achieved joint values alongside each pose, so a picture and its numbers
cannot drift apart.
"""

import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
OUT = "/ws/runs/shots"
HOLD_SECONDS = 16.0

# lift/elbow/wrist_flex from ik.solve; see scratchpad poses.py in the notes.
POSES = [
    ("1-search", "search pose - where the arm rides while hunting",
     dict(shoulder_pan=0.0, shoulder_lift=-1.4, elbow_flex=0.0,
          wrist_flex=0.4, wrist_roll=0.0, gripper=1.2)),
    ("2-pregrasp-old", "OLD pre-grasp: lift - 0.35, tip at r=0.243",
     dict(shoulder_pan=0.0, shoulder_lift=1.259, elbow_flex=0.036,
          wrist_flex=-0.124, wrist_roll=0.0, gripper=1.2)),
    ("3-pregrasp-new", "NEW pre-grasp: IK 60 mm above target, tip at r=0.137",
     dict(shoulder_pan=0.0, shoulder_lift=1.086, elbow_flex=0.568,
          wrist_flex=-0.133, wrist_roll=0.0, gripper=1.2)),
    ("4-grasp-centre", "grasp at band centre, r=0.137, cmd lift +1.609",
     dict(shoulder_pan=0.0, shoulder_lift=1.609, elbow_flex=0.036,
          wrist_flex=-0.124, wrist_roll=0.0, gripper=1.2)),
    ("5-grasp-edge", "grasp at outer band edge, r=0.193, cmd lift +1.716",
     dict(shoulder_pan=0.0, shoulder_lift=1.716, elbow_flex=-0.529,
          wrist_flex=0.334, wrist_roll=0.0, gripper=1.2)),
]


class Poser(Node):
    def __init__(self) -> None:
        super().__init__("arm_pose_capture")
        self.pubs = {j: self.create_publisher(Float64, f"/arm/{j}/position", 10)
                     for j in JOINTS}
        self.latest: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._on, 10)
        self.pose: dict[str, float] = {}
        self.create_timer(0.2, self._send)

    def _on(self, message: JointState) -> None:
        self.latest = message

    def _send(self) -> None:
        for joint, value in self.pose.items():
            message = Float64()
            message.data = float(value)
            self.pubs[joint].publish(message)


def gz(service: str, reqtype: str, request: str) -> None:
    subprocess.run(
        ["gz", "service", "-s", service, "--reqtype", reqtype,
         "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request],
        capture_output=True, text=True)


def settle(node: Poser, seconds: float) -> None:
    end = time.time() + seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    subprocess.run(["mkdir", "-p", OUT], check=False)
    rclpy.init()
    node = Poser()

    # Side-on, low and close: shoulder_lift's angle is the whole point, and it
    # is unreadable from the default three-quarter view.
    gz("/gui/move_to", "gz.msgs.StringMsg", 'data: "drivebase"')
    gz("/gui/follow", "gz.msgs.StringMsg", 'data: "drivebase"')
    gz("/gui/follow/offset", "gz.msgs.Vector3d", "x: 0.25, y: -1.05, z: 0.22")
    settle(node, 3.0)

    for name, caption, pose in POSES:
        node.pose = pose
        print(f"HOLDING {name}", flush=True)
        settle(node, HOLD_SECONDS)

        state = node.latest
        achieved = ""
        if state is not None and "arm_shoulder_lift" in state.name:
            got = state.position[state.name.index("arm_shoulder_lift")]
            grip = state.position[state.name.index("arm_gripper")]
            achieved = (f"  lift cmd {pose['shoulder_lift']:+.3f} -> "
                        f"got {got:+.3f} | gripper cmd {pose['gripper']:+.3f} "
                        f"-> got {grip:+.3f}")
        print(f"{name:16s} {caption}{achieved}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
