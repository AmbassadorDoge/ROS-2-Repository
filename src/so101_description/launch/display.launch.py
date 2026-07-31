"""Visualize the SO-101 arm in RViz without controllers.

Joint values come from joint_state_publisher_gui sliders. Use this for quick
model/TF inspection only; for the ros2_control demo use bringup.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("so101_description"), "urdf", "so101.urdf.xacro",
            ]),
            " use_ros2_control:=false",
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Start joint_state_publisher_gui"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("so101_description"), "rviz", "so101.rviz",
            ])],
        ),
    ])
