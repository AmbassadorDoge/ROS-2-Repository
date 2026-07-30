"""Local EKF: fuses wheel odometry and IMU to publish odom -> base_footprint.

Runs alongside the simulation or real hardware — it only consumes /odom and
/imu/data, so it does not care which is producing them.

    ros2 launch drivebase_localization localization.launch.py use_sim_time:=true

Output is /odometry/filtered plus the odom -> base_footprint transform, which is
what makes the robot appear to move in RViz. Nothing publishes map -> odom yet;
that is the global GPS-corrected filter, a later phase.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    ekf_config = PathJoinSubstitution([
        FindPackageShare("drivebase_localization"), "config", "ekf_local.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running against the simulator.",
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            # Must match the top-level key in ekf_local.yaml, or every parameter
            # is silently ignored and the filter runs on defaults.
            name="ekf_local",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        ),
    ])
