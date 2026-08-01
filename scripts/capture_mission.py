#!/usr/bin/env python3
"""Save annotated pod-camera frames through a live mission, for documentation.

    bash scripts/dev.sh bash -c '<launch the stack> ; \
      python3 /ws/scripts/capture_mission.py --seconds 200'

Writes /ws/runs/shots/mission/NNN_<state>.jpg at 1 Hz, each frame annotated
with what the detector reported for it - box, centroid, horizontal error and
direction. The point is to show the coordinator's actual input rather than a
description of it.

State comes from /behaviour_state if the coordinator publishes it, and
otherwise from the detection itself, so this still produces something useful
against a bare sim.
"""

import argparse
import json
import pathlib
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String

OUT = pathlib.Path("/ws/runs/shots/mission")


class MissionCapture(Node):
    def __init__(self, period: float) -> None:
        super().__init__("mission_capture")
        OUT.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.detection: dict | None = None
        self.tof: float = float("nan")
        self.index = 0
        self.last_saved = 0.0
        self.period = period

        self.create_subscription(Image, "/pod_camera/image_raw",
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(String, "/vision/target", self._on_target, 10)
        self.create_subscription(Range, "/tof/pod", self._on_tof,
                                 qos_profile_sensor_data)

    def _on_target(self, message: String) -> None:
        try:
            self.detection = json.loads(message.data)
        except json.JSONDecodeError:
            self.detection = None

    def _on_tof(self, message: Range) -> None:
        self.tof = message.range

    def _on_image(self, message: Image) -> None:
        now = time.time()
        if now - self.last_saved < self.period:
            return
        self.last_saved = now

        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        height, width = frame.shape[:2]
        detection = self.detection or {}
        detected = bool(detection.get("detected"))

        # Frame centre: the quantity the base and the arm are both driving to.
        cv2.line(frame, (width // 2, 0), (width // 2, height), (90, 90, 90), 1)
        cv2.line(frame, (0, height // 2), (width, height // 2), (90, 90, 90), 1)

        label = "NO TARGET"
        if detected:
            box = detection.get("bbox_pixels", {})
            x1, y1 = int(box.get("x1", 0)), int(box.get("y1", 0))
            x2, y2 = int(box.get("x2", 0)), int(box.get("y2", 0))
            centre = detection.get("center_normalized", {})
            cx = int(centre.get("x", 0.5) * width)
            cy = int(centre.get("y", 0.5) * height)
            error = float(detection.get("horizontal_error", 0.0))

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 220, 255), -1)
            # The error is horizontal because the image is upright. Draw it.
            cv2.arrowedLine(frame, (width // 2, cy), (cx, cy), (0, 220, 255), 2,
                            tipLength=0.25)
            label = (f"{detection.get('direction', '?')}  err={error:+.3f}  "
                     f"h={(y2 - y1) / height:.3f}  tof={self.tof:.3f}")

        cv2.rectangle(frame, (0, 0), (width, 24), (0, 0, 0), -1)
        cv2.putText(frame, label, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 220, 0) if detected else (120, 120, 120), 1, cv2.LINE_AA)

        state = "seen" if detected else "none"
        path = OUT / f"{self.index:03d}_{state}.jpg"
        cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        self.index += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--period", type=float, default=1.0)
    arguments = parser.parse_args()

    rclpy.init()
    node = MissionCapture(arguments.period)
    deadline = time.time() + arguments.seconds
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"saved {node.index} frames to {OUT}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
