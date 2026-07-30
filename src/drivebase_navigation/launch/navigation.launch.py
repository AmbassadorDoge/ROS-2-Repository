"""Nav2 for the drivebase: planner, RPP controller, ultrasonic costmaps.

    ros2 launch drivebase_navigation navigation.launch.py use_sim_time:=true

Expects drivebase_localization to be running: it owns BOTH map -> odom and
odom -> base_footprint. This launch file publishes no transforms at all, so
there is exactly one publisher of each. Also expects the /ultrasonic/* Range
topics. Send goals from RViz's "2D Goal Pose" tool, or:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 1.6}}}}"

WHY THIS DOES NOT USE nav2_bringup/navigation_launch.py
-------------------------------------------------------
That launch file hardcodes ten lifecycle nodes - including route_server and
docking_server - with no way to exclude them, and the lifecycle manager aborts
the *entire* bringup if any single node fails to configure. This robot has no
charging dock, so docking_server fails with "Charging dock plugins not given!"
and takes navigation down with it.

The alternative was inventing dock and route configuration for hardware that does
not exist. An explicit node list is more honest and easier to reason about. The
cost is that this list needs review when upgrading Nav2, which is an acceptable
trade for a project pinned to Jazzy through the competition.

VELOCITY CHAIN (each hop made explicit rather than relying on defaults)

    controller_server ─┐
                       ├─> /cmd_vel_nav ─> velocity_smoother
    behavior_server   ─┘                        │
                                                v
                                       /cmd_vel_smoothed
                                                │
                                                v
                                       collision_monitor
                                                │
                                                v
                                          /cmd_vel  ─> robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Order matters: the lifecycle manager transitions these in sequence, and
# bt_navigator must come up after the servers it calls into.
LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "bt_navigator",
    "waypoint_follower",
]

# Keeps nodes on the namespaced tf topics rather than absolute /tf.
TF_REMAP = [("/tf", "tf"), ("/tf_static", "tf_static")]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    params = PathJoinSubstitution([
        FindPackageShare("drivebase_navigation"), "config", "nav2_params.yaml",
    ])
    common = {"use_sim_time": use_sim_time}

    def nav_node(package, executable, extra_remaps=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[params, common],
            remappings=list(TF_REMAP) + list(extra_remaps),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Set true when running against the simulator.",
        ),
        nav_node("nav2_controller", "controller_server",
                 [("cmd_vel", "cmd_vel_nav")]),
        nav_node("nav2_planner", "planner_server"),
        nav_node("nav2_behaviors", "behavior_server",
                 [("cmd_vel", "cmd_vel_nav")]),
        # Explicit input remap. Upstream relies on this node's default topic
        # name lining up with the controller's output; naming it here means the
        # chain cannot silently break if that default changes.
        nav_node("nav2_velocity_smoother", "velocity_smoother",
                 [("cmd_vel", "cmd_vel_nav")]),
        # In/out topics come from nav2_params.yaml, not remaps.
        nav_node("nav2_collision_monitor", "collision_monitor"),
        nav_node("nav2_bt_navigator", "bt_navigator"),
        nav_node("nav2_waypoint_follower", "waypoint_follower"),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[common, {
                "autostart": True,
                "node_names": LIFECYCLE_NODES,
                "bond_timeout": 10.0,
            }],
        ),
    ])
