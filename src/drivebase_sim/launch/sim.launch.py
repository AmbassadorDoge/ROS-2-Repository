"""Bring up the drivebase in Gazebo Harmonic.

    ros2 launch drivebase_sim sim.launch.py
    ros2 launch drivebase_sim sim.launch.py rviz:=true
    ros2 launch drivebase_sim sim.launch.py gps:=true
    ros2 launch drivebase_sim sim.launch.py headless:=true ekf:=false

Publishes the same interface the real robot will expose - /odom, /joint_states,
/imu/data, /gps/fix, /ultrasonic/* as sensor_msgs/Range - so EKF and Nav2 config
developed here transfers to hardware unchanged.

The local EKF runs by default and owns odom -> base_footprint. Gazebo's DiffDrive
transform is deliberately left unbridged so there is exactly one publisher of it
(see gazebo.xacro). Run with ekf:=false to see raw wheel odometry alone - the
robot then sits still in RViz, since nothing is estimating where it is.

drivebase_localization owns map -> odom too: a static identity by default, or the
GPS-corrected global EKF with gps:=true. Nothing here publishes transforms.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
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
    rviz_config = PathJoinSubstitution([sim_pkg, "rviz", "sim.rviz"])
    xacro_file = PathJoinSubstitution(
        [description_pkg, "urdf", "drivebase.urdf.xacro"]
    )

    headless = LaunchConfiguration("headless")
    with_rviz = LaunchConfiguration("rviz")
    with_ekf = LaunchConfiguration("ekf")
    use_gps = LaunchConfiguration("gps")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_sim:=true"]),
        value_type=str,
    )

    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    # Gazebo rewrites package:// mesh URIs to model:// and then resolves them
    # against GZ_SIM_RESOURCE_PATH, which ROS does not populate. Without this the
    # arm's meshes fail to load with "Unable to find file
    # [model://so101_description/...]" and the arm is invisible - though still
    # fully present in physics, which makes it a confusing thing to chase.
    #
    # The path must contain the directory that *holds* the package folder, hence
    # the trailing "..".
    resource_path = [
        PathJoinSubstitution([FindPackageShare("so101_description"), ".."]),
        ":",
        PathJoinSubstitution([FindPackageShare("drivebase_description"), ".."]),
    ]

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),

        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run without the Gazebo GUI. Sensors still render, so "
                        "this needs a working GPU either way.",
        ),
        DeclareLaunchArgument(
            "ekf", default_value="true",
            description="Run the local EKF. Without it nothing publishes "
                        "odom -> base_footprint and RViz shows a stationary robot.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Also open RViz with the sim view.",
        ),
        DeclareLaunchArgument(
            "gps", default_value="false",
            description="Add navsat_transform + the global EKF so GPS bounds "
                        "drift. Makes short-range accuracy worse, not better.",
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

        # Fills covariance the gz NavSat message cannot carry. Always on, not
        # gated behind gps:=true, so /gps/fix means the same thing either way.
        Node(
            package="drivebase_sim",
            executable="gps_covariance",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare("drivebase_localization"),
                "launch", "localization.launch.py",
            ])]),
            launch_arguments={
                "use_sim_time": "true",
                "use_gps": use_gps,
            }.items(),
            condition=IfCondition(with_ekf),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(with_rviz),
        ),
    ])
