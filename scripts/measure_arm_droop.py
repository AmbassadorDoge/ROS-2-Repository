#!/usr/bin/env python3
"""Commanded vs achieved arm pose, at the pose the grasp actually uses.

    bash scripts/dev.sh bash -c \
      'ros2 launch drivebase_sim sim.launch.py headless:=true arm:=true & \
       sleep 30; python3 /ws/scripts/measure_arm_droop.py'

WHY THIS EXISTS. PROGRESS.md records that the arm droops up to 0.20 rad from
its commanded pose when extended, and does not establish whether that is a
torque limit or a starved controller. The answer decides a hardware question:
at the grasp pose shoulder_lift is commanded to +1.609 against a limit of
+1.745, so a 0.20 rad shortfall is larger than the whole remaining margin. If
the droop is proportional steady-state error it is free to fix in the gains; if
it is a torque limit the arm mount has to come down.

The default pose is the IK solution for the centre of the grasp band at the
50 mm grasp plane - the deepest, most extended thing the mission ever asks for.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 1.609,
    "elbow_flex": 0.036,
    "wrist_flex": -0.124,
    "wrist_roll": 0.0,
    "gripper": 1.2,
}
SETTLE_SECONDS = 22.0


class DroopProbe(Node):
    def __init__(self) -> None:
        super().__init__("arm_droop_probe")
        self.publishers_by_joint = {
            j: self.create_publisher(Float64, f"/arm/{j}/position", 10)
            for j in POSE
        }
        self.latest: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        # Re-sent every tick: a volatile publisher with no subscriber yet drops
        # silently, and discovery is not instant. See coordinator_node.py.
        self.create_timer(0.2, self._command)

    def _on_state(self, message: JointState) -> None:
        self.latest = message

    def _command(self) -> None:
        for joint, value in POSE.items():
            message = Float64()
            message.data = float(value)
            self.publishers_by_joint[joint].publish(message)


def main() -> int:
    rclpy.init()
    node = DroopProbe()
    deadline = time.time() + SETTLE_SECONDS
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    state = node.latest
    if state is None:
        print("no /joint_states received")
        return 1

    print(f"{'joint':16s} {'commanded':>10s} {'achieved':>10s} {'error':>9s}")
    worst = 0.0
    for joint, value in POSE.items():
        name = f"arm_{joint}"
        if name not in state.name:
            continue
        achieved = state.position[state.name.index(name)]
        error = achieved - value
        worst = max(worst, abs(error))
        print(f"{joint:16s} {value:+10.3f} {achieved:+10.3f} {error:+9.3f}")

    print(f"\nworst |error| = {worst:.3f} rad")
    print("shoulder_lift margin to its +1.745 limit at this pose: 0.136 rad")
    print("torque limit or gains? cmd_max is 10.0 against an effort limit of "
          "10 - if the error is well inside that, it is the gains.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
