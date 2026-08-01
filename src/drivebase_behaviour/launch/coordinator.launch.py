"""Bring up the behaviour coordinator.

    ros2 launch drivebase_behaviour coordinator.launch.py use_sim_time:=true

Expects drivebase_sim (or the real robot) and drivebase_navigation to already
be running. This launch file publishes no transforms and starts no Nav2 nodes.

robot_description is passed as a parameter rather than read from a topic: the
coordinator needs the arm's geometry before it can accept a grasp, and a node
that starts without it would fail at the worst possible moment.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    params = PathJoinSubstitution([
        FindPackageShare("drivebase_behaviour"), "config", "coordinator.yaml",
    ])
    xacro_file = PathJoinSubstitution([
        FindPackageShare("drivebase_description"),
        "urdf", "drivebase.urdf.xacro",
    ])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_sim:=true", " use_arm:=true"]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Set true when running against the simulator.",
        ),
        Node(
            package="drivebase_behaviour",
            executable="coordinator",
            name="behaviour_coordinator",
            output="screen",
            parameters=[
                params,
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description,
                },
            ],
        ),
    ])
