"""Drive toward litter_can_a and report what sim_litter_detector says.

Task 11's acceptance check. Needs a running sim, and must share its container
invocation - each scripts/dev.sh call is a fresh container, so launch and query
cannot be split across two:

    scripts/dev.sh bash -c 'source /opt/ros/jazzy/setup.bash &&
        source /ws/install/setup.bash &&
        ros2 launch drivebase_sim sim.launch.py headless:=true &
        sleep 30 && python3 /ws/scripts/verify_litter_detector.py'

One long-lived node: `ros2 topic echo --once` intermittently gives up before
discovering a low-rate publisher (PROGRESS.md trap 1).

CAVEAT ON hue_report: it samples whatever frame arrived last, which is not
necessarily a frame with litter in it. Reading its histogram as "the litter's
hue" is only valid when the run ends with the can in view.
"""
import json
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

import cv2


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_detector")
        self.targets = []
        self.directions = []
        self.frames = 0
        self.last_frame = None
        self.bridge = CvBridge()

        self.create_subscription(String, "/vision/target",
                                 lambda m: self.targets.append(m.data), 10)
        self.create_subscription(String, "/vision/direction",
                                 lambda m: self.directions.append(m.data), 10)
        self.create_subscription(Image, "/pod_camera/image_raw",
                                 self.on_image, qos_profile_sensor_data)

        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm = {
            n: self.create_publisher(Float64, f"/arm/{n}/position", 10)
            for n in ("shoulder_pan", "shoulder_lift", "elbow_flex",
                      "wrist_flex")
        }

    def on_image(self, msg):
        self.frames += 1
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self, **joints):
        for name, value in joints.items():
            m = Float64()
            m.data = float(value)
            self.arm[name].publish(m)

    def drive(self, seconds, x=0.0, z=0.0):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            t = Twist()
            t.linear.x = x
            t.angular.z = z
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd.publish(Twist())


def summarise(label, node):
    print(f"\n=== {label} ===")
    print(f"camera frames seen: {node.frames}")
    if not node.targets:
        print("NO /vision/target MESSAGES AT ALL")
        return
    print(f"target msgs: {len(node.targets)}, "
          f"directions: {sorted(set(node.directions))}")
    latest = json.loads(node.targets[-1])
    print("latest:", json.dumps(latest, indent=2)[:600])
    detected = [json.loads(t) for t in node.targets]
    n_det = sum(1 for d in detected if d.get("detected"))
    print(f"detected=true in {n_det}/{len(detected)} messages")


def hue_report(node):
    """What hue does the rendered litter actually land on?"""
    frame = node.last_frame
    if frame is None:
        print("\nno camera frame captured")
        return
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1] >= 120
    val = hsv[:, :, 2] >= 60
    strong = hsv[:, :, 0][sat & val]
    print(f"\n=== hue of saturated pixels (n={strong.size}) ===")
    if strong.size:
        hist = np.bincount(strong, minlength=180)
        top = np.argsort(hist)[::-1][:8]
        for h in top:
            if hist[h]:
                print(f"  hue {h:3d}: {hist[h]} px")
        near_zero = int(hist[:11].sum())
        near_180 = int(hist[170:].sum())
        print(f"  in 0..10: {near_zero} px, in 170..179: {near_180} px")


def main():
    rclpy.init()
    node = Verifier()

    node.spin(12.0)
    summarise("at spawn (nothing in view expected)", node)

    node.targets.clear()
    node.directions.clear()
    # Search pose from PROGRESS.md: pod looks down and forward.
    node.pose(shoulder_pan=0.0, shoulder_lift=-1.4, elbow_flex=0.0,
              wrist_flex=0.4)
    node.spin(5.0)

    # litter_can_a is at (1.8, 0.5); the robot spawns near the origin.
    node.drive(9.0, x=0.30, z=0.06)
    node.spin(3.0)
    summarise("after driving toward litter_can_a", node)
    hue_report(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
