# Litter-collection robot

Autonomous outdoor litter collection: a four-wheel skid-steer base with an SO-101
arm, navigating without a map or LIDAR. ROS 2 **Jazzy**, Gazebo **Harmonic**,
Raspberry Pi 5.

**Start with [`docs/STATUS.md`](docs/STATUS.md)** — current state, measured
figures, known issues, and what to do next.

## Packages

| Package | |
|---|---|
| `drivebase_description` | URDF, frame tree, Gazebo plugins and sensors |
| `so101_description` | SO-101 arm, vendored upstream and wrapped as a mountable macro |
| `drivebase_sim` | Gazebo world, bridge, sensor shims, measurement rig |
| `drivebase_localization` | Local + global EKF, `navsat_transform` |
| `drivebase_navigation` | Nav2: RPP controller, ultrasonic costmaps |
| `trash_vision` | YOLO litter detector (separate author) |

## Run it

```bash
colcon build && source install/setup.bash
ros2 launch drivebase_sim sim.launch.py rviz:=true
ros2 launch drivebase_navigation navigation.launch.py use_sim_time:=true   # 2nd terminal
```

Then set a goal with RViz's *2D Goal Pose*, or drive manually with
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

Needs a real GPU. Software rendering works but starves the control loops.

## Sensing

No LIDAR and no prior map. Obstacle avoidance is built entirely from five
forward-facing ultrasonic cones feeding a rolling Nav2 costmap; GPS bounds
long-range drift; a camera handles litter detection and final approach.

## Documentation

- [`docs/STATUS.md`](docs/STATUS.md) — state, decisions, next steps, gotchas
- [`docs/localization_accuracy.md`](docs/localization_accuracy.md) — measured drift, with limitations stated
- [`docs/power_budget.md`](docs/power_budget.md) — loads, batteries, motor spec, safety

## One thing to know before building on this

**Turns, not distance, dominate position error** — roughly 0.35 m per 90°, while
20 m of straight driving costs essentially nothing. Two turns put the base ~0.7 m
off, which is far larger than a piece of litter. The arm has to be aimed by the
camera closing its own loop, never by driving to a coordinate.
