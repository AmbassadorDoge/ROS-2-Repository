"""Display the drivebase URDF in RViz with interactive joint sliders.

Development tool only — no hardware or simulation involved. Use it to check
that frame placement and wheel geometry match the mechanical design before
committing to either.

    ros2 launch drivebase_description view_model.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("drivebase_description")

    xacro_file = PathJoinSubstitution([pkg, "urdf", "drivebase.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "view_model.rviz"])

    # ParameterValue(..., value_type=str) is required — without it the URDF
    # is passed as a YAML-parsed value and robot_state_publisher rejects it.
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # Stands in for real encoder feedback so the wheel joints have
        # positions to publish; replaced by the motor driver on hardware.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
        ),
    ])
