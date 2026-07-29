"""Collapse simulated ultrasonic cones into sensor_msgs/Range.

Gazebo Harmonic has no sonar sensor type, so each HC-SR04 is simulated as a
narrow gpu_lidar spanning the sensor's cone. A real ultrasonic rangefinder
reports one number: the nearest echo anywhere in that cone. This node reproduces
that by taking the minimum valid ray per scan.

Sim-only. On hardware the HC-SR04 driver publishes sensor_msgs/Range directly,
which is what lets Nav2's range_sensor_layer config transfer between the two
without modification.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from sensor_msgs.msg import LaserScan, Range


class LaserScanToRange(Node):
    def __init__(self) -> None:
        super().__init__("laserscan_to_range")

        self.declare_parameter(
            "sensor_names",
            [
                "front_left",
                "front_midleft",
                "front_center",
                "front_midright",
                "front_right",
            ],
        )
        self.declare_parameter("scan_topic_template", "/ultrasonic/{name}/scan")
        self.declare_parameter("range_topic_template", "/ultrasonic/{name}")
        self.declare_parameter("frame_template", "ultrasonic_{name}_link")

        sensor_names: list[str] = list(
            self.get_parameter("sensor_names").value
        )
        scan_template: str = str(
            self.get_parameter("scan_topic_template").value
        )
        range_template: str = str(
            self.get_parameter("range_topic_template").value
        )
        self.frame_template: str = str(
            self.get_parameter("frame_template").value
        )

        # Named *_by_name to avoid shadowing Node.publishers / Node.subscriptions.
        self.publishers_by_name: dict[str, Publisher] = {}
        self.subscriptions_by_name: dict[str, Subscription] = {}

        for name in sensor_names:
            self.publishers_by_name[name] = self.create_publisher(
                Range, range_template.format(name=name), qos_profile_sensor_data
            )
            # Default-arg binding, not closure capture: a closure over `name`
            # would leave every callback pointing at the last loop value.
            self.subscriptions_by_name[name] = self.create_subscription(
                LaserScan,
                scan_template.format(name=name),
                lambda msg, sensor_name=name: self.on_scan(msg, sensor_name),
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            f"Converting {len(sensor_names)} ultrasonic cones to Range: "
            f"{', '.join(sensor_names)}"
        )

    def on_scan(self, scan: LaserScan, sensor_name: str) -> None:
        message = Range()

        # Reuse the simulator's stamp rather than now() so the reading stays
        # correctly ordered under use_sim_time; restamping here would break
        # tf lookups in the costmap.
        message.header.stamp = scan.header.stamp
        message.header.frame_id = self.frame_template.format(name=sensor_name)

        message.radiation_type = Range.ULTRASOUND
        message.min_range = scan.range_min
        message.max_range = scan.range_max
        message.field_of_view = scan.angle_max - scan.angle_min

        # inf means no return within max_range, nan means an invalid ray. Both
        # must be filtered before min() or a single bad ray poisons the result.
        valid = [
            r
            for r in scan.ranges
            if not math.isnan(r) and not math.isinf(r)
            and scan.range_min <= r <= scan.range_max
        ]

        # No echo reads as max_range, matching how HC-SR04 drivers report a
        # timeout. Publishing inf here would make the costmap treat the sensor
        # as broken rather than clear.
        message.range = min(valid) if valid else scan.range_max

        self.publishers_by_name[sensor_name].publish(message)


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    node = LaserScanToRange()
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
