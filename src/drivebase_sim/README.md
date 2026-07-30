# drivebase_sim

Gazebo Harmonic simulation of the drivebase. Exposes the same ROS interface the
real robot will, so EKF and Nav2 configuration developed here transfers to
hardware unchanged.

## Run

```bash
colcon build --packages-select drivebase_description drivebase_sim
source install/setup.bash
ros2 launch drivebase_sim sim.launch.py
```

`headless:=true` skips the GUI. Sensors still render either way, so a working GPU
is needed regardless (software rendering via `LIBGL_ALWAYS_SOFTWARE=1` works but
is slow).

## Interface

| Topic | Type | Rate |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | in |
| `/odom` | `nav_msgs/Odometry` | 50 Hz |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz |
| `/gps/fix` | `sensor_msgs/NavSatFix` | 5 Hz |
| `/ultrasonic/{front_left,front_midleft,front_center,front_midright,front_right}` | `sensor_msgs/Range` | ~3 Hz |
| `/joint_states` | `sensor_msgs/JointState` | ~1 kHz (see below) |

## Gotchas

**`/cmd_vel` is plain `Twist`.** Jazzy Nav2's velocity chain publishes unstamped
`Twist` (`enable_stamped_cmd_vel` defaults to false), so the bridge matches that.

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # no stamped flag
```

A type mismatch here is **silent** — no error on either side. `/cmd_vel` will just
show two types in `ros2 topic info /cmd_vel` and the robot will ignore everything
while Nav2 publishes happily at 20 Hz. Check that first if the robot won't move.

**DiffDrive latches the last command.** Stop publishing and the robot keeps
driving forever. Send an explicit zero to stop it. (The real Pico firmware must
implement a command timeout instead — see `docs/power_budget.md` §4.)

**`odom` → `base_footprint` comes from the EKF**, which this launch file starts by
default. Gazebo's own DiffDrive transform is left unbridged so there is exactly
one publisher. Run with `ekf:=false` and the robot sits still in RViz while Gazebo
shows it driving — that is the EKF's absence, not a bug.

Add `rviz:=true` to open RViz with both the EKF pose (green) and raw wheel
odometry (orange) trailing behind. The gap that opens between them during turns
is the skid-steer scrub bias, measured at **1.53×** — see
`drivebase_localization/config/ekf_local.yaml`.

**`/joint_states` publishes at ~1 kHz.** Gazebo's `JointStatePublisher` has no
rate limit and ignores `<update_rate>`. Harmless: it only drives wheel-spin
visuals, and nothing reads wheel angle.

**Three places are coupled by the entity name** `drivebase`: `ENTITY_NAME` in
`launch/sim.launch.py`, `gz_prefix` in `drivebase_description/urdf/gazebo.xacro`,
and the `gz_topic_name` values in `config/bridge.yaml`. Renaming one without the
others yields a sim that runs and publishes nothing.

## Deliberately simplified

No camera in sim — the vision pipeline is developed against real footage, and
synthetic imagery would not help the YOLO model.

Ultrasonics are narrow `gpu_lidar` cones, because Gazebo Harmonic has no sonar
sensor type. `laserscan_to_range` collapses each cone to its nearest return,
matching what a real HC-SR04 reports. Rate is capped at 3 Hz to reflect the real
constraint that HC-SR04s cross-talk and must fire sequentially.
