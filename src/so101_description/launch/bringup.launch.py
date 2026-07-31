"""Bring up the SO-101 arm with ros2_control (mock hardware simulation).

Starts robot_state_publisher, the controller_manager with mock hardware, and
spawns joint_state_broadcaster + arm_controller. RViz is optional (rviz:=true).
There is exactly one robot_state_publisher and one controller_manager here;
do not combine with display.launch.py, which runs its own state publisher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("so101_description"), "urdf", "so101.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("so101_description"), "config", "so101_controllers.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Start RViz"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # Jazzy: controller_manager reads robot_description from the topic
        # published by robot_state_publisher (parameter passing is deprecated).
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_yaml],
            remappings=[("~/robot_description", "/robot_description")],
            output="both",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("so101_description"), "rviz", "so101.rviz",
            ])],
            condition=IfCondition(rviz),
        ),
    ])
