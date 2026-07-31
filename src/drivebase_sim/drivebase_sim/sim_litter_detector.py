"""Stand-in for trash_vision, for simulation only.

trash_vision cannot run here: it loads NCNN weights from a hardcoded home
directory and opens a TCP camera stream. This node segments the saturated red
litter models in test_field.sdf by hue instead.

THE INTERFACE IS THE POINT. Topic names, message types and the exact JSON key
layout match trash_vision's publish_target/publish_no_target byte for byte, so
the coordinator cannot tell which one it is talking to and hardware swaps the
real detector in by simply not launching this.

It is not a perception result. It proves the loop runs, not that detection
works.
"""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SimLitterDetector(Node):
    def __init__(self) -> None:
        super().__init__("sim_litter_detector")

        self.declare_parameter("image_topic", "/pod_camera/image_raw")
        self.declare_parameter("deadband", 0.12)
        # Smallest blob worth reporting, as a fraction of frame area. Below
        # this the centroid is dominated by noise and the bbox height - which
        # the ToF fallback depends on - is meaningless.
        self.declare_parameter("min_area_fraction", 0.0004)
        # Red straddles the hue origin, so this range WRAPS when low > high:
        # see _hue_mask. The litter models are pure red (hue 0) and the field's
        # barriers are orange (hue ~13 in OpenCV's 0-179 units), so the upper
        # bound is what keeps the barriers out of the mask.
        self.declare_parameter("hue_low", 170)
        self.declare_parameter("hue_high", 10)
        self.declare_parameter("saturation_min", 120)
        self.declare_parameter("value_min", 60)

        self.deadband = float(self.get_parameter("deadband").value)
        self.min_area_fraction = float(
            self.get_parameter("min_area_fraction").value)
        self.hue_low = int(self.get_parameter("hue_low").value)
        self.hue_high = int(self.get_parameter("hue_high").value)
        self.saturation_min = int(self.get_parameter("saturation_min").value)
        self.value_min = int(self.get_parameter("value_min").value)

        self.bridge = CvBridge()
        self.frame_number = 0

        self.target_publisher = self.create_publisher(
            String, "/vision/target", 10)
        self.direction_publisher = self.create_publisher(
            String, "/vision/direction", 10)

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Simulated litter detector started")

    def _hue_mask(self, hsv: np.ndarray) -> np.ndarray:
        """Threshold on hue, handling the wrap at red.

        OpenCV packs hue into 0-179 and pure red sits at 0, so a lit red
        object scatters to both ends of the scale - a naive inRange(0, 10)
        keeps the bright half and silently drops the shaded half, which moves
        the centroid rather than emptying the mask. When hue_low > hue_high
        the range is read as wrapping and taken as the union of both ends.
        """
        low = np.array([0, self.saturation_min, self.value_min])
        high = np.array([179, 255, 255])

        if self.hue_low <= self.hue_high:
            low[0], high[0] = self.hue_low, self.hue_high
            return cv2.inRange(hsv, low, high)

        lower_end = cv2.inRange(
            hsv,
            np.array([0, self.saturation_min, self.value_min]),
            np.array([self.hue_high, 255, 255]),
        )
        upper_end = cv2.inRange(
            hsv,
            np.array([self.hue_low, self.saturation_min, self.value_min]),
            np.array([179, 255, 255]),
        )
        return cv2.bitwise_or(lower_end, upper_end)

    def on_image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.frame_number += 1

        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._hue_mask(hsv)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.publish_no_target()
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area_fraction * width * height:
            self.publish_no_target()
            return

        x, y, w, h = cv2.boundingRect(largest)
        x1, y1, x2, y2 = float(x), float(y), float(x + w), float(y + h)

        center_x = ((x1 + x2) / 2.0) / width
        center_y = ((y1 + y2) / 2.0) / height
        horizontal_error = center_x - 0.5

        if horizontal_error < -self.deadband:
            direction = "LEFT"
        elif horizontal_error > self.deadband:
            direction = "RIGHT"
        else:
            direction = "CENTER"

        # Key layout copied from trash_vision's publish_target. Do not
        # "improve" it - the coordinator's JSON adapter parses both.
        target = {
            "detected": True,
            "frame_number": self.frame_number,
            "confidence": 1.0,
            "bbox_pixels": {
                "x1": round(x1, 1), "y1": round(y1, 1),
                "x2": round(x2, 1), "y2": round(y2, 1),
            },
            "center_normalized": {
                "x": round(center_x, 4), "y": round(center_y, 4),
            },
            "floor_point_normalized": {
                "x": round(center_x, 4), "y": round(y2 / height, 4),
            },
            "horizontal_error": round(horizontal_error, 4),
            "direction": direction,
        }

        target_message = String()
        target_message.data = json.dumps(target)
        self.target_publisher.publish(target_message)

        direction_message = String()
        direction_message.data = direction
        self.direction_publisher.publish(direction_message)

    def publish_no_target(self) -> None:
        target_message = String()
        target_message.data = json.dumps({
            "detected": False,
            "frame_number": self.frame_number,
        })
        self.target_publisher.publish(target_message)

        direction_message = String()
        direction_message.data = "NO_TARGET"
        self.direction_publisher.publish(direction_message)


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    node = SimLitterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
