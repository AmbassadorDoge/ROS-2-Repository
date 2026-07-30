"""Fill in position covariance on simulated GPS fixes.

`gz.msgs.NavSat` has no covariance field, so ros_gz_bridge publishes NavSatFix
messages with `position_covariance` all zeros and `covariance_type` UNKNOWN.
robot_localization weights GPS by exactly that covariance, and zeros read as
"infinitely confident" - the filter then snaps hard to every fix, including the
bad ones, which is the opposite of what a 2.5 m CEP receiver deserves.

Sim-only shim. Real u-blox drivers derive covariance from HDOP and publish it on
/gps/fix directly, which is why this node republishes onto that name: the
canonical topic carries the same content in both worlds, so nothing downstream
needs to know which is which.

    /gps/fix_raw  (bridge, no covariance)  ->  /gps/fix  (covariance filled)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


class GpsCovarianceShim(Node):
    def __init__(self) -> None:
        super().__init__("gps_covariance_shim")

        # Defaults match the noise actually injected in gazebo.xacro. If you
        # change the sensor noise there, change these too - a filter told the
        # wrong uncertainty is worse than one told none.
        self.declare_parameter("horizontal_stddev", 2.5)
        self.declare_parameter("vertical_stddev", 5.0)

        horizontal = float(self.get_parameter("horizontal_stddev").value)
        vertical = float(self.get_parameter("vertical_stddev").value)

        # NavSatFix wants variances, not standard deviations.
        self.covariance = [
            horizontal ** 2, 0.0, 0.0,
            0.0, horizontal ** 2, 0.0,
            0.0, 0.0, vertical ** 2,
        ]

        self.publisher = self.create_publisher(
            NavSatFix, "/gps/fix", qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            NavSatFix, "/gps/fix_raw", self.on_fix, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Filling GPS covariance: horizontal {horizontal} m, "
            f"vertical {vertical} m"
        )

    def on_fix(self, message: NavSatFix) -> None:
        message.position_covariance = self.covariance
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.publisher.publish(message)


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    node = GpsCovarianceShim()
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
