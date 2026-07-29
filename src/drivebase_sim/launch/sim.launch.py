"""Bring up the drivebase in Gazebo Harmonic.

    ros2 launch drivebase_sim sim.launch.py
    ros2 launch drivebase_sim sim.launch.py headless:=true

Publishes the same interface the real robot will expose - /odom, /joint_states,
/imu/data, /gps/fix, /ultrasonic/* as sensor_msgs/Range - so EKF and Nav2 config
developed here transfers to hardware unchanged.

Deliberately does NOT publish odom -> base_footprint. That transform belongs to
robot_localization; until the EKF lands, the tf tree is rooted at base_footprint
and the robot will not appear to move in RViz. That is expected, not a bug.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Must match the -name passed to `create` below, and the /model/<name>/...
# paths in config/bridge.yaml. Changing it means changing all three.
ENTITY_NAME = "drivebase"


def generate_launch_description() -> LaunchDescription:
    description_pkg = FindPackageShare("drivebase_description")
    sim_pkg = FindPackageShare("drivebase_sim")

    world_file = PathJoinSubstitution([sim_pkg, "worlds", "test_field.sdf"])
    bridge_config = PathJoinSubstitution([sim_pkg, "config", "bridge.yaml"])
    xacro_file = PathJoinSubstitution(
        [description_pkg, "urdf", "drivebase.urdf.xacro"]
    )

    headless = LaunchConfiguration("headless")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_sim:=true"]),
        value_type=str,
    )

    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run without the Gazebo GUI. Sensors still render, so "
                        "this needs a working GPU either way.",
        ),

        # -r starts unpaused. -s is server-only; without it gz_args opens the GUI.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_launch]),
            launch_arguments={"gz_args": [world_file, " -r"]}.items(),
            condition=UnlessCondition(headless),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_launch]),
            launch_arguments={"gz_args": [world_file, " -r -s"]}.items(),
            condition=IfCondition(headless),
        ),

        # Publishes the fixed transforms (sensor frames, wheel offsets) and
        # consumes /joint_states from the simulator for the wheel joints.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
            }],
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", ENTITY_NAME,
                "-topic", "/robot_description",
                # Spawned clear of the ground so settling is visible; dropping
                # it exactly at wheel height can wedge it in the plane.
                "-z", "0.15",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{
                "config_file": bridge_config,
                "use_sim_time": True,
            }],
            output="screen",
        ),

        Node(
            package="drivebase_sim",
            executable="laserscan_to_range",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])
