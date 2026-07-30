"""State estimation for the drivebase.

    # dead reckoning only
    ros2 launch drivebase_localization localization.launch.py use_sim_time:=true

    # GPS-corrected
    ros2 launch drivebase_localization localization.launch.py \\
        use_sim_time:=true use_gps:=true

Consumes /odom, /imu/data and (with GPS) /gps/fix, so it does not care whether
the simulator or real hardware is producing them.

THIS PACKAGE OWNS BOTH TRANSFORMS IN THE LOCALIZATION CHAIN

    map -> odom              ekf_global, or a static identity when use_gps:=false
    odom -> base_footprint   ekf_local, always

Keeping both here means there is exactly one owner of each. drivebase_navigation
deliberately publishes neither - two publishers of map -> odom produce
localization that looks subtly and inconsistently wrong rather than failing.

WHAT use_gps ACTUALLY BUYS

The local filter drifts ~0.5 m over a 5.5 m path (measured). The NEO-6M carries
2.5 m of noise. Over short distances GPS is therefore WORSE than dead reckoning,
and enabling it will look like a regression. It earns its place over tens of
metres, where local drift keeps growing and GPS error does not.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gps = LaunchConfiguration("use_gps")

    pkg = FindPackageShare("drivebase_localization")

    def config(name):
        return PathJoinSubstitution([pkg, "config", name])

    common = {"use_sim_time": use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Set true when running against the simulator.",
        ),
        DeclareLaunchArgument(
            "use_gps", default_value="false",
            description="Run navsat_transform + the global EKF so GPS bounds "
                        "drift. Off by default because it makes short-range "
                        "accuracy worse, not better - see the module docstring.",
        ),

        # ---- always: local filter, odom -> base_footprint ----
        # `name` must match the top-level key in ekf_local.yaml or every parameter
        # is silently ignored and the filter runs on defaults.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            output="screen",
            parameters=[config("ekf_local.yaml"), common],
        ),

        # ---- without GPS: map and odom are the same frame ----
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
            parameters=[common],
            condition=UnlessCondition(use_gps),
        ),

        # ---- with GPS: global filter owns map -> odom ----
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global",
            output="screen",
            parameters=[config("ekf_global.yaml"), common],
            # Renamed so it does not collide with the local filter's output.
            # Nav2's controller must keep using /odometry/filtered, the smooth
            # one; this jumps whenever GPS corrects it.
            remappings=[("odometry/filtered", "odometry/global")],
            condition=IfCondition(use_gps),
        ),

        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[config("navsat_transform.yaml"), common],
            remappings=[
                # The node's internal topic name is `imu`, NOT `imu/data`. Getting
                # this wrong is silent and total: navsat_transform needs a GPS fix,
                # an odometry estimate AND an IMU heading before it can compute its
                # transform, so with the IMU missing it re-derives its datum on
                # every single fix (measured: 415 times) and never publishes
                # /odometry/gps at all. The global EKF then runs as a second
                # dead-reckoning filter with no GPS correction, which looks like it
                # is working.
                ("imu", "imu/data"),
                # Its odometry input is the GLOBAL estimate, closing the loop
                # described in navsat_transform.yaml.
                ("odometry/filtered", "odometry/global"),
            ],
            condition=IfCondition(use_gps),
        ),
    ])
