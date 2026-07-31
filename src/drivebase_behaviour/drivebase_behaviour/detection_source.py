"""Adapters that turn whatever the detector publishes into a Detection.

The coordinator never sees JSON and never imports a message type. Swapping
trash_vision's std_msgs/String for drivebase_msgs/LitterDetection is a
parameter change here and nothing at all anywhere else.
"""

from rclpy.node import Node
from std_msgs.msg import String

from drivebase_behaviour.detection import Detection, parse_target_json


class StringJsonDetectionSource:
    """Today's interface: JSON in a std_msgs/String.

    Stamps on ARRIVAL, because std_msgs/String carries no header. Under load
    this misattributes latency - the state machine's staleness check is what
    stops that turning into servoing at a target that has already moved.
    """

    def __init__(self, node: Node, topic: str) -> None:
        self.node = node
        self.latest: Detection | None = None
        node.create_subscription(String, topic, self._on_message, 10)

    def _on_message(self, message: String) -> None:
        now = self.node.get_clock().now().nanoseconds * 1e-9
        parsed = parse_target_json(message.data, now)
        if parsed is not None:
            self.latest = parsed


class TypedDetectionSource:
    """The proposed interface. Uses the header stamp, so latency is real."""

    def __init__(self, node: Node, topic: str) -> None:
        from drivebase_msgs.msg import LitterDetection

        self.node = node
        self.latest: Detection | None = None
        node.create_subscription(LitterDetection, topic, self._on_message, 10)

    def _on_message(self, message) -> None:
        stamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1e-9
        )
        self.latest = Detection(
            stamp_seconds=stamp,
            detected=message.detected,
            frame_number=message.frame_number,
            confidence=message.confidence,
            center_x=message.center_x,
            center_y=message.center_y,
            floor_x=message.floor_x,
            floor_y=message.floor_y,
            horizontal_error=message.horizontal_error,
            bbox=(message.bbox_x1, message.bbox_y1,
                  message.bbox_x2, message.bbox_y2),
        )


def make_detection_source(node: Node, interface: str, topic: str):
    if interface == "typed":
        return TypedDetectionSource(node, topic)
    if interface == "json_string":
        return StringJsonDetectionSource(node, topic)
    raise ValueError(
        f"unknown detection_interface '{interface}'; "
        "expected 'json_string' or 'typed'")
