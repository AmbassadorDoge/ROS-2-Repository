"""Drive a repeatable scripted path and log every pose estimate time-aligned.

A measurement rig, not a controller. It exists because an earlier shell-based
harness produced numbers that could not be trusted, in five distinct ways:

  * `ros2 topic pub &` + `kill` left publishers alive to compete, so in one
    circuit test only the FIRST turn took effect while the script believed it was
    driving a square.
  * Each sample spawned separate `ros2 topic echo` processes ~1 s apart, so
    ground truth and the estimates were never read at the same instant and their
    difference was dominated by sampling skew.
  * Missing readings silently became 0.0, fabricating errors that happened to
    equal distance-from-origin.
  * Legs were timed with wall-clock sleep while the simulation ran slower than
    realtime, making leg length unpredictable.
  * Turns were open-loop, and only ~65% of a commanded rotation is achieved -
    varying 1.53x to 1.94x for the same command.

This node removes all five: one process owns the single /cmd_vel publisher, one
timer samples all cached subscriptions at the same instant, deadlines are in sim
time, and legs terminate on measured progress rather than elapsed time.

CLOSED LOOP ON GROUND TRUTH IS DELIBERATE. Every run then drives the same real
path, so differences between runs are estimator behaviour rather than path
variation. That would be cheating in a controller; here it is the control.

    ros2 run drivebase_sim scripted_drive --ros-args \\
        -p path_name:=square10 -p output_csv:=/tmp/run.csv -p use_sim_time:=true
"""

import math
import pathlib

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Named paths. Each begins with `orient`, a 90 deg turn onto +y, because driving
# straight out of the origin runs into the barrier at x=3 and measures wheel slip
# instead of drift. That prologue turn is tagged separately from `turn` so the
# analysis can exclude it - it is common to every path, so it cancels when
# comparing runs.
#
# All three stay inside x in [-10, 0], y in [0, 40], which is clear of every
# obstacle in test_field.sdf (they all sit at x > 1).
QUARTER = math.pi / 2

# All three cover ~20 m of translation and differ only in counted rotation, so
# the difference between them isolates turn-induced error.
PATHS = {
    # 20 m, 0 deg counted rotation.
    "straight20": [("orient", QUARTER), ("straight", 20.0), ("pause", 3.0)],

    # 20 m, 360 deg counted rotation.
    "square5": [("orient", QUARTER)] + [
        leg for _ in range(4)
        for leg in (("straight", 5.0), ("pause", 1.5), ("turn", QUARTER),
                    ("pause", 1.5))
    ],

    # 20 m, 720 deg counted rotation.
    "square2x8": [("orient", QUARTER)] + [
        leg for _ in range(8)
        for leg in (("straight", 2.5), ("pause", 1.5), ("turn", QUARTER),
                    ("pause", 1.5))
    ],

    # Short shakedown path for verifying the rig itself.
    "smoke": [("orient", QUARTER), ("straight", 5.0), ("pause", 2.0),
              ("turn", QUARTER), ("pause", 2.0)],

    # Diagnostic: two rotations back to back with NO translation between them.
    # Distinguishes "only the first turn ever works" from "driving forward
    # breaks subsequent rotation".
    "twoturns": [("orient", QUARTER), ("pause", 2.0),
                 ("turn", QUARTER), ("pause", 2.0)],
}

TURN_LEGS = ("turn", "orient")


def yaw_of(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_diff(a: float, b: float) -> float:
    """Shortest signed difference a - b, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Pose2D:
    __slots__ = ("x", "y", "yaw")

    def __init__(self, x=math.nan, y=math.nan, yaw=math.nan):
        self.x, self.y, self.yaw = x, y, yaw

    @property
    def valid(self) -> bool:
        return not math.isnan(self.x)


class ScriptedDrive(Node):
    def __init__(self) -> None:
        super().__init__("scripted_drive")

        self.declare_parameter("path_name", "smoke")
        self.declare_parameter("output_csv", "/tmp/scripted_drive.csv")
        self.declare_parameter("linear_speed", 0.45)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("sample_period", 0.25)
        # Abort if commanded motion produces no measurable movement for this long.
        # This is the exact failure the old harness hid: it kept "driving" while
        # the robot stood still. A rig that cannot detect that is worthless.
        self.declare_parameter("stall_timeout", 12.0)

        path_name = str(self.get_parameter("path_name").value)
        if path_name not in PATHS:
            raise SystemExit(
                f"unknown path_name '{path_name}'; have {sorted(PATHS)}"
            )
        self.legs = PATHS[path_name]
        self.path_name = path_name

        self.csv_path = pathlib.Path(str(self.get_parameter("output_csv").value))
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.stall_timeout = float(self.get_parameter("stall_timeout").value)

        self.truth = Pose2D()
        self.local = Pose2D()
        self.glob = Pose2D()
        self.wheel = Pose2D()

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        # Ground truth from Gazebo's OdometryPublisher: the model's true world
        # pose, not a simulated sensor.
        self.create_subscription(
            Odometry, "/gz/ground_truth", self.on_truth, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", lambda m: self.on_odom(m, self.local), 10
        )
        self.create_subscription(
            Odometry, "/odometry/global", lambda m: self.on_odom(m, self.glob), 10
        )
        self.create_subscription(
            Odometry, "/odom", lambda m: self.on_odom(m, self.wheel), 10
        )

        self.leg_index = -1          # -1 until the first truth arrives
        self.leg_start = Pose2D()
        self.leg_start_time = None
        self.turn_accumulated = 0.0
        self.turn_reference = 0.0
        self.cum_distance = 0.0
        self.cum_rotation = 0.0
        self.previous_truth = Pose2D()
        self.stall_reference = Pose2D()
        self.stall_since = None
        self.rows = []
        self.finished = False
        self.aborted = None

        self.create_timer(0.05, self.control_tick)
        self.create_timer(
            float(self.get_parameter("sample_period").value), self.sample_tick
        )
        self.get_logger().info(
            f"path '{path_name}': {len(self.legs)} legs -> {self.csv_path}"
        )

    # ---------- inputs ----------

    def on_truth(self, message: Odometry) -> None:
        p = message.pose.pose.position
        o = message.pose.pose.orientation
        self.truth = Pose2D(p.x, p.y, yaw_of(o.x, o.y, o.z, o.w))

    def on_odom(self, message: Odometry, target: Pose2D) -> None:
        p = message.pose.pose.position
        o = message.pose.pose.orientation
        target.x, target.y = p.x, p.y
        target.yaw = yaw_of(o.x, o.y, o.z, o.w)

    # ---------- driving ----------

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def begin_leg(self, index: int) -> None:
        self.leg_index = index
        self.leg_start = Pose2D(self.truth.x, self.truth.y, self.truth.yaw)
        self.leg_start_time = self.now_seconds()
        self.turn_accumulated = 0.0
        self.turn_reference = self.truth.yaw
        self.stall_reference = Pose2D(self.truth.x, self.truth.y, self.truth.yaw)
        self.stall_since = self.leg_start_time
        if index < len(self.legs):
            kind, target = self.legs[index]
            self.get_logger().info(
                f"leg {index}/{len(self.legs) - 1}: {kind} {target:.3f}"
            )

    def control_tick(self) -> None:
        if self.finished or not self.truth.valid:
            return

        # Accumulate true path length and rotation for the whole run.
        if self.previous_truth.valid:
            self.cum_distance += math.hypot(
                self.truth.x - self.previous_truth.x,
                self.truth.y - self.previous_truth.y,
            )
            self.cum_rotation += abs(
                angle_diff(self.truth.yaw, self.previous_truth.yaw)
            )
        self.previous_truth = Pose2D(self.truth.x, self.truth.y, self.truth.yaw)

        if self.leg_index < 0:
            self.begin_leg(0)
            return

        if self.leg_index >= len(self.legs):
            self.complete()
            return

        kind, target = self.legs[self.leg_index]
        command = Twist()
        done = False

        if kind == "straight":
            travelled = math.hypot(
                self.truth.x - self.leg_start.x, self.truth.y - self.leg_start.y
            )
            done = travelled >= target
            command.linear.x = 0.0 if done else self.linear_speed
            self.check_stall(moving=not done, rotating=False)

        elif kind in TURN_LEGS:
            # Unwrapped accumulation: a single atan2 difference cannot represent
            # more than pi, and reading it per tick keeps large turns correct.
            self.turn_accumulated += abs(
                angle_diff(self.truth.yaw, self.turn_reference)
            )
            self.turn_reference = self.truth.yaw
            done = self.turn_accumulated >= target
            command.angular.z = 0.0 if done else self.angular_speed
            self.check_stall(moving=False, rotating=not done)

        elif kind == "pause":
            done = (self.now_seconds() - self.leg_start_time) >= target
            self.stall_since = self.now_seconds()   # standing still is expected

        self.publisher.publish(command)

        if done:
            self.publisher.publish(Twist())     # explicit zero; DiffDrive latches
            self.begin_leg(self.leg_index + 1)

    def check_stall(self, moving: bool, rotating: bool) -> None:
        progress = math.hypot(
            self.truth.x - self.stall_reference.x,
            self.truth.y - self.stall_reference.y,
        )
        turned = abs(angle_diff(self.truth.yaw, self.stall_reference.yaw))
        if (moving and progress > 0.05) or (rotating and turned > 0.02):
            self.stall_reference = Pose2D(
                self.truth.x, self.truth.y, self.truth.yaw
            )
            self.stall_since = self.now_seconds()
            return
        if self.now_seconds() - self.stall_since > self.stall_timeout:
            self.aborted = (
                f"stalled on leg {self.leg_index} "
                f"({self.legs[self.leg_index][0]}): commanded motion produced no "
                f"measurable movement for {self.stall_timeout:.0f}s"
            )
            self.complete()

    # ---------- output ----------

    def sample_tick(self) -> None:
        if self.finished or self.leg_index < 0:
            return
        # Every REQUIRED source must be live before the first row, or the leading
        # rows carry nan purely from startup ordering and the "missing data"
        # warning cries wolf on every run. /odometry/global is deliberately not
        # required - it only exists when localization runs with use_gps:=true, and
        # its absence is a valid configuration rather than a fault.
        if not (self.truth.valid and self.local.valid and self.wheel.valid):
            return
        kind = (
            self.legs[self.leg_index][0]
            if self.leg_index < len(self.legs) else "done"
        )
        self.rows.append((
            f"{self.now_seconds():.3f}", str(self.leg_index), kind,
            f"{self.truth.x:.4f}", f"{self.truth.y:.4f}", f"{self.truth.yaw:.5f}",
            f"{self.local.x:.4f}", f"{self.local.y:.4f}", f"{self.local.yaw:.5f}",
            f"{self.glob.x:.4f}", f"{self.glob.y:.4f}", f"{self.glob.yaw:.5f}",
            f"{self.wheel.x:.4f}", f"{self.wheel.y:.4f}", f"{self.wheel.yaw:.5f}",
            f"{self.cum_distance:.4f}", f"{self.cum_rotation:.5f}",
        ))

    def complete(self) -> None:
        if self.finished:
            return
        self.finished = True
        self.publisher.publish(Twist())

        header = ("t,leg,leg_type,true_x,true_y,true_yaw,"
                  "loc_x,loc_y,loc_yaw,glob_x,glob_y,glob_yaw,"
                  "odom_x,odom_y,odom_yaw,cum_dist,cum_rot")
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self.csv_path.open("w") as handle:
            handle.write(f"# path={self.path_name}\n")
            handle.write(header + "\n")
            for row in self.rows:
                handle.write(",".join(row) + "\n")

        # Only the required sources: columns 3-8 (truth, local) and 12-14
        # (wheel). The global filter's columns are legitimately nan whenever
        # localization runs without use_gps.
        gaps = sum(
            1 for row in self.rows
            if any(v == "nan" for v in row[3:9] + row[12:15])
        )
        if self.aborted:
            self.get_logger().error(f"ABORTED: {self.aborted}")
        self.get_logger().info(
            f"{'ABORTED' if self.aborted else 'complete'}: {len(self.rows)} rows, "
            f"{gaps} with missing data, "
            f"{self.cum_distance:.2f} m travelled, "
            f"{math.degrees(self.cum_rotation):.0f} deg rotated"
        )
        if gaps:
            self.get_logger().warning(
                f"{gaps} rows have nan - a topic was not publishing; "
                "treat this run as suspect"
            )


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    node = ScriptedDrive()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.complete()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
