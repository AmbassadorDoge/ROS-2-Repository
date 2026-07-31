#!/usr/bin/env python3
"""Drive a list of arm poses in a running sim and record pod geometry + ToF.

    ./scripts/dev.sh bash -c '
      ros2 launch drivebase_sim sim.launch.py headless:=true rviz:=false &
      sleep 45
      python3 scripts/pod_sweep.py poses.json out.json --shots 0 3
    '

WHY THIS EXISTS AS A FILE. Task 5 measured 210 poses and threw its harness
away, so Task 5b had to rebuild it before it could re-measure anything. Each
scripts/dev.sh call is a fresh container, so launch-and-query must be a single
invocation; relaunching Gazebo per pose is what makes a sweep take hours under
llvmpipe instead of minutes. One launch, many poses.

Every figure it prints is measured. The ground-range PREDICTION is computed
from tf alone - pod height over the negated ray z - and never sees the ToF, so
prediction-vs-measurement is an independent check that the beam terminated on
the ground plane and not on the robot's own castings.

Pose file: JSON list of {"name", "lift", "elbow", "wrist", "pan"?, "roll"?}.
"""

from __future__ import annotations

import json
import math
import sys

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, JointState, Range
from std_msgs.msg import Float64

# base_link rides at wheel_radius; the wheels put the ground 0.060 m below it.
GROUND_BELOW_BASE = 0.060

# The gpu_lidar's <max> in gazebo.xacro. A ray that misses everything comes
# back as exactly this, which is how "pointing at the sky" is recognised.
TOF_MAX = 4.0

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
          "wrist_roll", "gripper")


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Sweep(Node):
    def __init__(self) -> None:
        super().__init__("pod_sweep")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.cmd = {j: self.create_publisher(Float64, f"/arm/{j}/position", 10)
                    for j in JOINTS}

        # /tof/pod is published by laserscan_to_range with
        # qos_profile_sensor_data, i.e. BEST_EFFORT. A RELIABLE subscriber is
        # incompatible with it and receives NOTHING - rclpy logs one warning and
        # then stays quiet forever, which reads exactly like a dead sensor.
        # (docs/superpowers/plans/PROGRESS.md claims this publisher is RELIABLE.
        # Measured here: it is not.) Depth 50 so a whole dwell fits in history.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.tof: list[float] = []
        self.create_subscription(Range, "/tof/pod", self._on_tof, sensor_qos)

        self.image: Image | None = None
        self.create_subscription(Image, "/pod_camera/image_raw",
                                 self._on_image, sensor_qos)

        # /joint_states carries arm_-prefixed names, not the command-topic ones.
        self.joints: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joints, 10)

    def _on_tof(self, msg: Range) -> None:
        self.tof.append(float(msg.range))

    def _on_image(self, msg: Image) -> None:
        self.image = msg

    def _on_joints(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = float(pos)

    def spin(self, seconds: float) -> None:
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def command(self, pose: dict) -> None:
        for joint, key in (("shoulder_pan", "pan"), ("shoulder_lift", "lift"),
                           ("elbow_flex", "elbow"), ("wrist_flex", "wrist"),
                           ("wrist_roll", "roll"), ("gripper", "grip")):
            self.cmd[joint].publish(Float64(data=float(pose.get(key, 0.0))))

    def lookup(self, child: str):
        tf = self.buffer.lookup_transform("base_link", child,
                                          rclpy.time.Time())
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z]), quat_to_matrix(tf.transform.rotation)


def measure(node: Sweep, pose: dict, settle: float, dwell: float) -> dict:
    node.command(pose)
    node.spin(settle)
    node.tof.clear()
    node.image = None
    node.spin(dwell)

    grip, _ = node.lookup("arm_gripper_frame_link")
    tof_p, tof_R = node.lookup("pod_tof_link")
    cam_p, cam_R = node.lookup("pod_camera_optical_frame")

    ray = tof_R[:, 0]              # gpu_lidar fires along its link's +X
    view = cam_R[:, 2]             # optical +Z is the camera's view axis
    pod_height = float(tof_p[2]) + GROUND_BELOW_BASE

    # tf only - deliberately no ToF input, so it can contradict the ToF.
    if ray[2] < -1e-6:
        predicted = pod_height / -ray[2]
        hit_x = float(tof_p[0] + predicted * ray[0])
    else:
        predicted = None           # ray goes up: nothing but sky to hit
        hit_x = None

    samples = list(node.tof)
    result = {
        "name": pose.get("name", ""),
        "cmd": {k: pose.get(k, 0.0) for k in ("pan", "lift", "elbow", "wrist")},
        "act": {k: round(node.joints.get(f"arm_{j}", float("nan")), 4)
                for k, j in (("lift", "shoulder_lift"), ("elbow", "elbow_flex"),
                             ("wrist", "wrist_flex"), ("pan", "shoulder_pan"),
                             ("roll", "wrist_roll"), ("grip", "gripper"))},
        "grip_x": round(float(grip[0]), 4),
        "grip_y": round(float(grip[1]), 4),
        "grip_z_above_ground": round(float(grip[2]) + GROUND_BELOW_BASE, 4),
        "pod_xyz": [round(float(v), 4) for v in tof_p],
        "pod_height": round(pod_height, 4),
        "ray": [round(float(v), 4) for v in ray],
        "view_axis": [round(float(v), 4) for v in view],
        "boresight_deg": round(math.degrees(math.acos(
            float(np.clip(np.dot(ray, view), -1, 1)))), 4),
        "predicted_range": None if predicted is None else round(predicted, 4),
        "ground_hit_x": None if hit_x is None else round(hit_x, 4),
        "tof_n": len(samples),
        "tof_mean": round(float(np.mean(samples)), 4) if samples else None,
        "tof_min": round(float(np.min(samples)), 4) if samples else None,
        "tof_max": round(float(np.max(samples)), 4) if samples else None,
    }
    if result["tof_mean"] is not None and predicted:
        result["err_pct"] = round(
            100.0 * (result["tof_mean"] - predicted) / predicted, 2)
    return result


def save_shot(node: Sweep, path: str) -> bool:
    if node.image is None:
        return False
    import cv2
    msg = node.image
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, -1)
    if msg.encoding in ("rgb8", "rgba8"):
        buf = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, buf)
    return True


def main() -> int:
    poses = json.load(open(sys.argv[1]))
    out_path = sys.argv[2]
    shots = set(sys.argv[3:])

    rclpy.init()
    node = Sweep()

    # Discovery race: publishers and tf need a moment before the first pose,
    # or pose 0 is measured against a tf tree that is not there yet.
    node.spin(8.0)

    results = []
    for i, pose in enumerate(poses):
        row = measure(node, pose, settle=float(pose.get("settle", 6.0)),
                      dwell=float(pose.get("dwell", 3.0)))
        name = pose.get("name", str(i))
        if name in shots or str(i) in shots:
            path = f"/ws/runs/pod_sweep/{name}.png"
            row["shot"] = path if save_shot(node, path) else "NO IMAGE"
        results.append(row)
        print(json.dumps(row), flush=True)

    json.dump(results, open(out_path, "w"), indent=1)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
