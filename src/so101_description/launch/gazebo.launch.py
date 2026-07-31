"""Simulate the SO-101 arm in Gazebo Sim (Harmonic) with ros2_control.

The controller_manager runs inside the Gazebo process (gz_ros2_control
plugin), so no separate ros2_control_node is started here — do not combine
with bringup.launch.py. The arm's URDF root link is named "world", which
libsdformat converts into a fixed attachment to the Gazebo world frame.

Args:
  gui:=true    start the Gazebo GUI client (default false: server only,
               suitable for headless machines like this Pi)
  rviz:=true   start RViz (default false)
"""

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("so101_description"), "urdf", "so101.urdf.xacro",
            ]),
            " use_gazebo:=true",
        ]),
        value_type=str,
    )

    gz_sim_launch = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="Start the Gazebo GUI client"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Start RViz"),

        # Let Gazebo resolve model://so101_description/... mesh URIs
        # (sdformat's conversion of package:// URIs).
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("so101_description"), ".."]),
        ),

        # Gazebo Sim: empty world, running (-r); -s = server only when gui:=false.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={"gz_args": "-r -s -v1 empty.sdf"}.items(),
            condition=UnlessCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={"gz_args": "-r -v1 empty.sdf"}.items(),
            condition=IfCondition(gui),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": True}],
        ),

        # Spawn the arm from the /robot_description topic.
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "/robot_description", "-name", "so101"],
            output="both",
        ),

        # Bridge Gazebo sim time to ROS /clock.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
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
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(rviz),
        ),
    ])
