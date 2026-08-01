"""The coordinator: ticks the state machine and wires it to ROS.

Deliberately thin. Every decision lives in state_machine.py, every piece of
geometry in localisation.py and ik.py, all of which are pure and tested without
a simulator. This file only gathers inputs, applies outputs, and owns the one
thing that cannot be pure: talking to Nav2, tf and the wheels.

VELOCITY OWNERSHIP. During APPROACHING this node publishes to /cmd_vel_nav -
the same topic controller_server uses - and it cancels the Nav2 goal BEFORE
transitioning. controller_server then stops publishing, so there is exactly one
publisher at any instant, and the approach still inherits velocity_smoother and
collision_monitor. Writing /cmd_vel directly would skip both, losing ultrasonic
collision safety exactly when the robot is closest to something.
"""

import math
from dataclasses import replace

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Range

from drivebase_behaviour.arm_driver import ArmDriver, grasp_sequence
from drivebase_behaviour.detection_source import make_detection_source
from drivebase_behaviour.ik import JOINT_LIMITS, solve
from drivebase_behaviour.kinematics import extract_planar_arm, load_chain
from drivebase_behaviour.localisation import (
    fallback_range_from_bbox,
    pixel_to_unit_ray,
    point_from_range,
    range_is_plausible,
)
from drivebase_behaviour.state_machine import (
    Config,
    Inputs,
    State,
    StateMachine,
)

ARM_JOINT_ORDER = (
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
)


class Coordinator(Node):
    def __init__(self) -> None:
        super().__init__("behaviour_coordinator")

        self._declare_parameters()
        self.config = Config(
            deadband=self.p("deadband"),
            confirm_frames=int(self.p("confirm_frames")),
            confirm_frames_fallback=int(self.p("confirm_frames_fallback")),
            grasp_min=self.p("grasp_min"),
            grasp_max=self.p("grasp_max"),
            lost_timeout=self.p("lost_timeout"),
            approach_timeout=self.p("approach_timeout"),
            confirm_timeout=self.p("confirm_timeout"),
            detection_stale_after=self.p("detection_stale_after"),
            min_confidence=self.p("min_confidence"),
        )
        self.machine = StateMachine(self.config)

        flat = list(self.get_parameter("waypoints").value)
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        if not self.waypoints:
            raise RuntimeError("no waypoints configured")
        self.waypoint_index = -1

        self.detections = make_detection_source(
            self,
            str(self.get_parameter("detection_interface").value),
            str(self.get_parameter("detection_topic").value),
        )

        self.camera_info: CameraInfo | None = None
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info, qos_profile_sensor_data)

        self.tof: Range | None = None
        self.create_subscription(
            Range, str(self.get_parameter("tof_topic").value),
            self._on_tof, qos_profile_sensor_data)

        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav_goal_handle = None
        self.nav_succeeded = False
        self.nav_aborted = False

        self.arm = ArmDriver(self, self.p("wrist_roll_limit"))
        self.planar_arm = self._load_arm_model()

        self.stored_grasp_point: tuple[float, float, float] | None = None
        self._last_logged_state = State.IDLE

        self.arm.go_to(self._pose("search_pose"))
        self.machine.start()

        period = 1.0 / self.p("tick_hz")
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Coordinator started with {len(self.waypoints)} waypoints")

    # ---------- parameters -------------------------------------------------

    def _declare_parameters(self) -> None:
        defaults = {
            "detection_interface": "json_string",
            "detection_topic": "/vision/target",
            "camera_info_topic": "/pod_camera/camera_info",
            "tof_topic": "/tof/pod",
            "camera_optical_frame": "pod_camera_optical_frame",
            "static_frame": "odom",
            "arm_base_frame": "arm_base_link",
            "tick_hz": 10.0,
            "waypoints": [2.0, 0.0, 4.0, 1.5, 2.0, 3.0, 0.0, 1.5],
            "loop_patrol": True,
            "min_confidence": 0.4,
            "deadband": 0.12,
            "detection_stale_after": 0.5,
            "confirm_frames": 5,
            "confirm_frames_fallback": 10,
            "tof_min_range": 0.04,
            "tof_max_range": 4.0,
            "fallback_reference_height_px": 60.0,
            "fallback_reference_range_m": 1.0,
            # Camera-range window; see config/coordinator.yaml for the unit
            # caveat behind these.
            "grasp_min": 0.199,
            "grasp_max": 0.333,
            "workspace_radius": 0.40,
            "lost_timeout": 2.0,
            "approach_timeout": 30.0,
            "confirm_timeout": 8.0,
            "approach_linear_speed": 0.22,
            "approach_min_linear_speed": 0.06,
            "approach_angular_gain": 1.2,
            "approach_max_angular_speed": 0.8,
            "search_pose": [0.0, -1.4, 0.0, 0.4, 0.0, 0.0],
            "confirm_pose": [0.0, 0.9, 0.46, -1.40, 0.0, 0.0],
            "stow_pose": [0.0, -1.20, 1.55, 1.10, 0.0, 0.0],
            "gripper_open": 1.2,
            "gripper_closed": 0.0,
            "grasp_lift_offset": 0.35,
            "grasp_hold_seconds": 1.5,
            "wrist_roll_limit": 1.0,
            "approach_angle": -1.5708,
            "robot_description": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _pose(self, name: str) -> dict[str, float]:
        values = list(self.get_parameter(name).value)
        return dict(zip(ARM_JOINT_ORDER, values))

    # ---------- setup ------------------------------------------------------

    def _load_arm_model(self):
        """Reads the arm's geometry from /robot_description.

        Blocking on purpose: without the URDF there is no IK, and starting
        anyway would mean discovering that at the moment the arm is asked to
        grasp something.
        """
        urdf = str(self.get_parameter("robot_description").value)
        if not urdf:
            raise RuntimeError(
                "robot_description parameter is empty; the coordinator cannot "
                "derive arm kinematics. Pass it from the launch file.")
        chain = load_chain(
            urdf,
            str(self.get_parameter("arm_base_frame").value),
            "arm_gripper_frame_link",
        )
        return extract_planar_arm(chain)

    # ---------- subscriptions ---------------------------------------------

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _on_tof(self, message: Range) -> None:
        self.tof = message

    # ---------- the tick ---------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_point(self):
        """Camera ray times ToF range, in the camera optical frame.

        Returns (point, used_fallback) or (None, False).
        """
        detection = self.detections.latest
        if detection is None or not detection.detected:
            return None, False
        if self.camera_info is None:
            return None, False

        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            return None, False

        u = detection.center_x * self.camera_info.width
        v = detection.center_y * self.camera_info.height
        ray = pixel_to_unit_ray(u, v, fx, fy, cx, cy)

        used_fallback = False
        range_m = self.tof.range if self.tof is not None else float("nan")
        if not range_is_plausible(
            range_m, self.p("tof_min_range"), self.p("tof_max_range")
        ):
            bbox_height = detection.bbox[3] - detection.bbox[1]
            range_m = fallback_range_from_bbox(
                bbox_height,
                float(self.camera_info.height),
                self.p("fallback_reference_height_px"),
                self.p("fallback_reference_range_m"),
            )
            used_fallback = True
            if math.isnan(range_m):
                return None, False

        return point_from_range(ray, range_m), used_fallback

    def _tick(self) -> None:
        now = self._now()
        self.arm.update(now)

        point, used_fallback = self._target_point()
        range_m = (
            math.sqrt(sum(c * c for c in point)) if point else float("nan")
        )
        range_valid = point is not None

        detection = self.detections.latest
        if detection is not None and used_fallback:
            # Carry the fallback flag into the gate, which demands more frames
            # for it.
            detection = replace(detection, range_is_fallback=True)

        in_workspace = (
            range_valid and range_m <= self.p("workspace_radius")
        )

        outputs = self.machine.tick(Inputs(
            now=now,
            detection=detection,
            range_m=range_m,
            range_valid=range_valid,
            in_workspace=in_workspace,
            nav_goal_active=self.nav_goal_handle is not None,
            nav_goal_succeeded=self.nav_succeeded,
            nav_goal_aborted=self.nav_aborted,
            arm_sequence_done=self.arm.sequence_done,
        ))
        self.nav_succeeded = False
        self.nav_aborted = False

        # Log every transition. Without this the mission is unobservable: the
        # states that matter (APPROACHING, CONFIRMING) can come and go between
        # two waypoint lines and leave no trace, which is exactly what made the
        # first end-to-end run so hard to read.
        if outputs.state is not self._last_logged_state:
            self.get_logger().info(
                f"{self._last_logged_state.value} -> {outputs.state.value} "
                f"(range={range_m:.3f} valid={range_valid} "
                f"in_workspace={in_workspace})")
            self._last_logged_state = outputs.state

        if outputs.cancel_nav_goal:
            self._cancel_nav_goal()
        if outputs.send_next_waypoint:
            self._send_next_waypoint()
        if outputs.store_grasp_point and point is not None:
            self._store_and_grasp(point)
        if outputs.start_stow:
            self.arm.start_sequence(
                [(self._pose("search_pose"), self.p("grasp_hold_seconds"))])

        if outputs.state is State.NAVIGATING and self.arm.sequence_done:
            # Re-assert the search pose every tick rather than once at startup.
            #
            # Publishing it in __init__ does not work: the publisher is created
            # microseconds earlier, discovery has not connected the gz
            # JointPositionController subscriber yet, and a volatile publisher
            # with no subscriber silently drops the message. Nothing re-sent it,
            # so the arm sat in its URDF stow pose for the whole patrol, the pod
            # camera pointed nowhere useful, and the detector reported
            # `detected: false` for every frame of a run that otherwise looked
            # healthy. Measured: joints held at stow (-1.202/1.600/1.102)
            # instead of search (-1.4/0.0/0.4).
            #
            # Position controllers hold their last command, so re-sending is
            # idempotent and this self-heals from any dropped message.
            self.arm.go_to(self._pose("search_pose"))
        elif outputs.state is State.APPROACHING:
            self._servo(detection)
        elif outputs.state in (State.CONFIRMING, State.PICKING, State.STOWING):
            self.cmd_publisher.publish(Twist())
            if outputs.state is State.CONFIRMING:
                self._aim_arm(detection)

    # ---------- actions ----------------------------------------------------

    def _servo(self, detection) -> None:
        """Arcs, never point turns.

        linear.x never drops to zero while correcting heading. RPP and the
        power budget both prefer arcs over point turns; in-place rotation is
        also the peak-current maneuver on hardware.
        """
        command = Twist()
        if detection is None or not detection.detected:
            command.linear.x = self.p("approach_min_linear_speed")
            self.cmd_publisher.publish(command)
            return

        error = detection.horizontal_error
        angular = -self.p("approach_angular_gain") * error
        limit = self.p("approach_max_angular_speed")
        command.angular.z = max(-limit, min(limit, angular))

        # Slow down when badly off heading, but never stop.
        straightness = max(0.0, 1.0 - abs(error) / 0.5)
        fast, slow = (self.p("approach_linear_speed"),
                      self.p("approach_min_linear_speed"))
        command.linear.x = slow + (fast - slow) * straightness

        self.cmd_publisher.publish(command)

    def _aim_arm(self, detection) -> None:
        """Bearing first. A single-point ToF only means anything once it is
        pointed at the target - reading range before centring measures the
        ground.

        Aims from confirm_pose, NOT search_pose as the plan had it. search_pose
        puts the beam on the ground 1.168 m ahead of base_link, well outside the
        0.287-0.481 m grasp window, so the range gate could never have passed
        from there. confirm_pose lands the beam at x = 0.392 m, mid-window.
        Both figures measured; see docs/arm_workspace.md §3.
        """
        if detection is None or not detection.detected:
            return
        pose = self._pose("confirm_pose")
        pose["shoulder_pan"] = (
            pose.get("shoulder_pan", 0.0)
            - detection.horizontal_error * 1.0
        )
        low, high = JOINT_LIMITS["shoulder_pan"]
        pose["shoulder_pan"] = max(low, min(high, pose["shoulder_pan"]))
        self.arm.go_to(pose)

    def _store_and_grasp(self, point) -> None:
        """Freeze the target in a static frame, then grasp open-loop.

        Everything from here is blind: the jaws occlude the target and it is
        inside the ToF's minimum range. See the design doc §6.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("arm_base_frame").value),
                str(self.get_parameter("camera_optical_frame").value),
                rclpy.time.Time(),
            )
        except tf2_ros.TransformException as error:
            self.get_logger().warning(f"tf lookup failed, skipping grasp: {error}")
            self.arm.start_sequence([])
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        target = _apply_transform(point, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

        solution = solve(
            self.planar_arm, target, self.p("approach_angle"), JOINT_LIMITS)
        if solution is None:
            self.get_logger().warning(
                f"target {target} unreachable, abandoning this piece")
            self.arm.start_sequence([])
            return

        self.stored_grasp_point = target
        self.arm.start_sequence(grasp_sequence(
            solution,
            self._pose("search_pose"),
            self.p("gripper_open"),
            self.p("gripper_closed"),
            self.p("grasp_lift_offset"),
            self.p("grasp_hold_seconds"),
        ))

    def _send_next_waypoint(self) -> None:
        self.waypoint_index += 1
        if self.waypoint_index >= len(self.waypoints):
            if not bool(self.get_parameter("loop_patrol").value):
                self.get_logger().info("Patrol complete")
                return
            self.waypoint_index = 0

        x, y = self.waypoints[self.waypoint_index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(
            self.get_parameter("static_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning("Nav2 action server unavailable")
            return

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(f"Waypoint {self.waypoint_index}: ({x}, {y})")

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.nav_aborted = True
            return
        self.nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        status = future.result().status
        self.nav_goal_handle = None
        # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus.
        if status == 4:
            self.nav_succeeded = True
        else:
            self.nav_aborted = True

    def _cancel_nav_goal(self) -> None:
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None


def _apply_transform(point, translation, quaternion):
    x, y, z, w = quaternion
    px, py, pz = point
    # q * p * q^-1, expanded.
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    rx = px + w * tx + (y * tz - z * ty)
    ry = py + w * ty + (z * tx - x * tz)
    rz = pz + w * tz + (x * ty - y * tx)
    return (rx + translation[0], ry + translation[1], rz + translation[2])


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    node = Coordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
