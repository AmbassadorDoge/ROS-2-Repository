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

Everything runs in the ROS 2 Jazzy container. `scripts/dev.sh` builds the image
on first use and wires up the GPU and display:

```bash
bash scripts/dev.sh colcon build --symlink-install
bash scripts/dev.sh ros2 launch drivebase_sim sim.launch.py rviz:=true
bash scripts/dev.sh ros2 launch drivebase_navigation navigation.launch.py use_sim_time:=true   # 2nd terminal
```

Then set a goal with RViz's *2D Goal Pose*, or drive manually with
`bash scripts/dev.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

`bash scripts/dev.sh` with no arguments gives an interactive shell with the
workspace sourced. Build artifacts live in Docker volumes, not in the repo —
`bash scripts/dev.sh --reset` drops them.

Needs a real GPU; `scripts/dev.sh` passes `/dev/dri` through and falls back to
llvmpipe only when there is none. Software rendering works but starves the
control loops, so no timing figure measured under it is worth believing.

The pure-geometry parts (kinematics, detection, localisation) need no ROS at
all — see `scripts/dev-native.sh`.

## Sensing

No LIDAR and no prior map. Obstacle avoidance is built entirely from five
forward-facing ultrasonic cones feeding a rolling Nav2 costmap; GPS bounds
long-range drift; a camera handles litter detection and final approach.

## Documentation

- [`docs/STATUS.md`](docs/STATUS.md) — state, decisions, next steps, gotchas
- [`docs/localization_accuracy.md`](docs/localization_accuracy.md) — measured drift, with limitations stated
- [`docs/power_budget.md`](docs/power_budget.md) — loads, batteries, motor spec, safety
- [`docs/bom.md`](docs/bom.md) — parts to order, with the specs that are easy to get wrong

## One thing to know before building on this

**The base cannot be trusted to position the arm** — the arm has to be aimed by
the camera closing its own loop, never by driving to a coordinate.

Not because of odometry drift, as it turns out. An earlier version of this note
quoted ~0.35 m of error per 90° turn; that figure was measuring a Gazebo contact
bug in which the body slid instead of rotating, and once the wheel-slip model
was fitted a full five-turn, 20 m square came in at 0.035 m total. The
conclusion stands on firmer ground anyway: GPS bounds error at 1.4–5.6 m, the
simulated IMU is better than anything you can buy, and no contact model here has
been validated against hardware. Litter is far smaller than any of those.
