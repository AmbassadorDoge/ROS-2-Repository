# Project status and handoff

Autonomous litter-collection robot. ROS 2 **Jazzy**, Gazebo **Harmonic**,
Raspberry Pi 5 target.

Last updated 2026-07-31. Everything below was verified by running it, not by
reading code — figures quoted here have a measurement behind them.

---

## Where things stand

| Area | State |
|---|---|
| Robot description (URDF) | Done, matches measured chassis |
| SO-101 arm | Mounted, reaches ground, provisional stow pose |
| SO-101 arm control | `ros2_control` done — mock and Gazebo, trajectory verified |
| Gazebo simulation | Done — drives, senses, obstacles |
| Local EKF (`odom` → `base_footprint`) | Done, verified against ground truth |
| Global EKF + GPS (`map` → `odom`) | Done, plumbing verified |
| Nav2 | Done — reaches goals, routes around obstacles |
| Measurement rig | Done, produced the accuracy numbers |
| Parts list | Done — `bom.md`, pack decision made, nothing ordered |
| **Behaviour coordinator** | **In progress** — pure modules and FSM done, node not wired |
| **Pico firmware** | **Not started** |
| **Real sensor drivers** | **Not started** |
| **Battery monitor** | **Not started** |
| `trash_vision` restructuring | Not started (partner's package) |

Work sits on `experimental/behaviour-coordinator`.

### Arm control, added 2026-07-31

Ported from Christopher's `URDF-arm` branch on
`robot-curiosity/ROS-2-Repository` and re-based onto our macro — his version
used bare joint names and mesh collision, ours needs `arm_` prefixing and
boxes. `experimental/so101-ros2-control` on his repo carries the adaptation
back to him.

`joint_trajectory_controller` over all six joints, two backends behind
`use_gazebo`: `mock_components/GenericSystem` and `gz_ros2_control`. Verified
both ways — controllers active, six command interfaces claimed, and a
`FollowJointTrajectory` goal on `arm_shoulder_pan` reached 0.6 rad.

**The prefix is load-bearing.** The macro namespaces every joint and the
drivebase instantiates it as `prefix="arm_"`. A controller config with bare
joint names spawns, reports `active`, and moves nothing — no error either side.
There is one `so101_controllers.yaml` and it is prefixed; see that package's
README.

The dev image now carries `controller-manager`,
`joint-trajectory-controller`, `gz-ros2-control` and `ros2controlcli`.

---

## Quick start

```bash
colcon build && source install/setup.bash

# simulation, RViz, local EKF
ros2 launch drivebase_sim sim.launch.py rviz:=true

# add navigation, in a second terminal
ros2 launch drivebase_navigation navigation.launch.py use_sim_time:=true

# drive manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`sim.launch.py` flags: `headless`, `rviz`, `ekf`, `gps`, `arm`, `wheel_mu2`.

Needs a real GPU. Software rendering works (`LIBGL_ALWAYS_SOFTWARE=1`) but runs
well under real time and starves the control loops.

---

## Measured figures

| Quantity | Value | Source |
|---|---|---|
| Turn cost, local EKF | **~0.35 m per 90°** | `localization_accuracy.md` |
| Straight-line cost | negligible over 20 m | same |
| Effective-track factor | **1.742×** (stdev 0.025, n=6) | same |
| Global (GPS) error | 1.4–5.6 m, bounded | same |
| Ultrasonic vs world geometry | 2.732 m vs 2.75 m predicted | verified in sim |
| Arm reach below its mount | **219 mm** | measured in sim |
| Gripper height at full extension | **41 mm** above ground | measured in sim |
| Motion power draw | ~62 W pavement, ~105 W grass | `power_budget.md` |

**The single most important consequence:** turns, not distance, dominate
position error, and a couple of turns puts the base ~0.7 m off. Litter is far
smaller than that, so **the arm must be aimed by the camera, never by driving to
a coordinate.** The existing detector's LEFT/RIGHT/CENTER output is the right
shape for that.

---

## Known issues

### 1. Simulated robot cannot rotate in place more than once per run

**Simulation artifact only. Not a prediction about the hardware.**

First rotation of a run succeeds; every later one leaves the body stationary
while the wheel joints keep spinning (measured: `odom_yaw` +279°, `true_yaw`
0.0°). The command path, bridge, DiffDrive plugin and joint actuation all work —
it is the tyre/ground contact solver failing to convert wheel spin into body
rotation.

Ruled out: commanded angular velocity (0.5/0.8/1.2 rad/s), the arm
(`arm:=false` fails too), lateral friction (`wheel_mu2` 0.6/0.3/0.15), and
translation between turns.

**Also ruled out: software rendering.** Reproduced 2026-07-30 on an AMD
Radeon 680M at real-time factor 0.999, `square5`, in `runs/gpu_square5.csv`.
The prologue `orient` turn succeeded; the first counted `turn` leg then
stalled and the rig aborted it after 12 s. Over that leg `true_yaw` moved
1.57057 → 1.57080 rad (0.013°) while `odom_yaw` moved 0.403 rad (23°), and
`true_y` slid 4.6416 → 4.4860 m. The body slides instead of rotating. This is
contact solver behaviour, not a starved one.

**`twoturns` narrows it further** (`runs/gpu_twoturns.csv`): two rotations
back to back with no translation between them, and the second still stalls.
So it is "only the first rotation of a run ever works", not "driving forward
breaks subsequent rotation".

#### Cause found 2026-07-30: `wheel_mu2` was never being read

gz-sim's default physics engine is **dartsim**, which takes a single friction
coefficient and does not implement the anisotropic `mu1`/`mu2` split. The
`<mu1>`/`<mu2>` pair in `gazebo.xacro` maps to ODE's friction element, so the
lateral value is silently discarded. **That is why sweeping `wheel_mu2` across
0.6/0.3/0.15 changed nothing — the parameter had no effect to have.** The
robot was running effectively isotropic at `mu1` 1.0 against a ground plane at
0.9, which is the "isotropic high friction makes a skid-steer refuse to
rotate" case the xacro comment itself warns about.

Confirmed by lowering the coefficient that *is* read. At `wheel_mu1:=0.35`
`wheel_mu2:=0.35`, `twoturns` completes (182° rotated, scrub down from 1.65 m
to 0.42 m) and a full `square5` completes all four corners — 21.24 m, 457°
rotated, `runs/gpu_square5_mu035.csv`.

**This is a diagnosis, not an adopted fix.** The URDF default is deliberately
left at `mu1` 1.0, because 0.35 changes how much the tyres scrub and so
invalidates every measured figure in `localization_accuracy.md`. Same run
measured the wheel-odometry over-report at **1.594×**, against the documented
1.742× at `mu1` 1.0 — the figures move with friction. `sim.launch.py` now
exposes `wheel_mu1` so this is reproducible; picking a value means
re-measuring the drift figures against it, and 0.35 has no hardware
justification behind it.

Fix candidates, cheapest first. Candidate 1 remains the right one — it models
slip properly rather than blunting friction globally, which is what lowering
`mu1` does:

1. **`gz-sim-wheel-slip-system`** — Gazebo's own slip model, with
   `slip_compliance_lateral` / `slip_compliance_longitudinal`. Built for exactly
   this. Try first.
2. **Copy Clearpath's Husky** — the canonical 4-wheel skid-steer Gazebo model,
   public URDF, friction and contact parameters already tuned by people who hit
   this problem. Fastest path to something that works.
3. **Contact tuning** — `<kp>`, `<kd>`, `<maxVel>`, `<minDepth>` on the wheel
   surfaces plus solver iterations. Fiddly but standard for skid steer.
4. **`ros2_control` + `gz_ros2_control` with effort-limited joints** — the real
   fix. `DiffDrive` commands joint velocity with unlimited torque, so contacts
   simply slip and friction force is independent of commanded speed, which is why
   raising angular velocity changed nothing. Effort limits also match how the
   real robot works (Pico PID → motor torque), so this is needed eventually
   regardless.

   **Substantially de-risked 2026-07-31.** `gz_ros2_control` is now in the dev
   image and proven to load under this Jazzy/Harmonic pairing — it runs
   `controller_manager` inside the Gazebo process for the arm. What is left is
   a second `<ros2_control>` system over the wheels with *velocity* interfaces
   and effort limits, plus `diff_drive_controller` replacing the gz DiffDrive
   plugin. That is now config work rather than a stack-integration gamble,
   which moves this from "most expensive candidate" to a serious contender
   against candidate 1.

**Blocks:** multi-turn drift measurement, and tuning any rotation-heavy Nav2
behaviour (spin recovery, `rotate_to_heading`). **Does not affect:** geometry,
sensor models, straight-line results, the 0.35 m/turn figure (measured on turns
that did work), or Nav2 obstacle avoidance (RPP arcs rather than point-turns).

### 2. Nav2 stalled ~30 s in one run of two

Recovered via its behaviour tree and reached the goal. Control loop was also
running 3.5–11 Hz against a 20 Hz target under software rendering, so this may
just be a starved controller. **Re-check on a real GPU before tuning anything.**

### 3. `core.158` is still in git history

A 24 MB core dump from the initial commit. `git commit --amend` on that commit
clears it while history is shallow, but rewrites history — do it before anyone
else clones.

---

## Decisions worth not relitigating

- **RPP, not MPPI.** MPPI rolls out 2000 trajectories × 56 steps per cycle; the
  Pi 5 already shares cores with YOLO. RPP also prefers arcs over point turns,
  which the power budget and the turn-drift number both independently favour.
- **No `nav2_bringup/navigation_launch.py`.** It hardcodes ten lifecycle nodes
  including `docking_server`, aborts the whole bringup if any fails, and offers
  no way to exclude them. This robot has no charging dock. An explicit node list
  beats inventing config for hardware that does not exist.
- **`drivebase_localization` owns both transforms.** `map` → `odom` and
  `odom` → `base_footprint`. `drivebase_navigation` publishes none, so each has
  exactly one publisher.
- **Yaw comes from the IMU, never the wheels.** Skid steer rotates by scrubbing;
  wheel yaw is biased, not just noisy. Measured 1.742× over-report.
- **Arm collision geometry is boxes, not meshes.** 0.5–2.6 MB STLs would make
  Gazebo compute mesh-mesh contacts every step.
- **Meshes are vendored, not fetched at build time.** This has to still build at
  competition time; a live GitHub dependency is a reproducibility risk.

---

## Gotchas found the hard way

Every one of these failed **silently** — no error on either side.

| Symptom | Cause |
|---|---|
| Robot ignores all velocity commands | `/cmd_vel` carried two message types. Nav2 Jazzy publishes plain `Twist`; `enable_stamped_cmd_vel` defaults false. **Check `ros2 topic info /cmd_vel` first.** |
| Arm invisible but present in physics | Gazebo rewrites `package://` to `model://` and needs `GZ_SIM_RESOURCE_PATH` |
| GPS fixes scattered ~800 km | gz navsat applies horizontal noise in **degrees**, vertical in **metres** |
| `navsat_transform` re-derives datum forever | It subscribes to `imu`, **not** `imu/data` |
| Global estimate walks off at an angle | `magnetic_declination_radians` must be **0.0 in sim** — the simulated IMU has no magnetometer |
| Filter snaps to every GPS fix | `gz.msgs.NavSat` carries no covariance; bridged `NavSatFix` arrives all zeros |
| Nav2 bringup aborts entirely | `collision_monitor` and `docking_server` unconfigured |
| Robot "drives a square" but goes straight | backgrounded `ros2 topic pub` surviving `kill` — use `scripted_drive` |
| Outer ultrasonics read a constant 0.08 m | They were staring at the front wheels; sensor band raised to 200 mm |
| Sweeping `wheel_mu2` changes nothing at all | dartsim takes one friction coefficient and ignores the `mu1`/`mu2` split. The lateral value is discarded on load, with no warning. See issue 1 |
| Gazebo starts, no window ever opens, `create` loops "Requesting list of world names", `/clock` never published | gz-transport discovers peers over UDP multicast. Under `--net=host` on a host with no multicast route it finds nothing. Server and GUI processes both stay alive, so it does not look like a crash. `GZ_IP=127.0.0.1` — set in `scripts/dev.sh` |
| Container GUI dies with "Authorization required, but no authorization protocol specified" | Xwayland authorises local clients by peer UID and a Hyprland session has no Xauthority file to forward. A root container is refused. The container must run as the host UID — `scripts/dev.sh` passes `--user` |
| `ros2 run` cannot find an executable that colcon clearly installed | The repo is on an NTFS/fuseblk mount that silently drops the executable bit, so console scripts install non-executable. `build/`, `install/`, `log/` are Docker volumes for this reason |

---

## Next steps, in order

1. **Order parts.** The actual critical path — everything in hardware weeks waits
   on lead times. Full list in `bom.md`. The pack decision is made (power tool
   battery, 18–20 V), which is what unblocks the motor order: **24 V motors,
   120–155 RPM rated at 24 V**. Two remaining easy mistakes: the USB-C bank must
   do **5 V/5 A** or the Pi caps peripheral current and can starve the camera,
   and **get the 9-DOF IMU (BNO055 / ICM-20948), not the MPU6050** — a 6-DOF
   part cannot provide absolute heading, and heading is the whole error budget.
   Everything except the motors is voltage-independent and orderable now.
2. **Chassis + arm bracket.** The arm needs a mount on the front face at
   **260 mm** above ground, which may not be in the current print. Also needs a
   pocket for the battery adapter plate.
3. **Pico firmware.** Encoder counting in PIO (~25,000 edges/s across four wheels
   — Linux userspace GPIO will drop counts), wheel velocity PID, and a
   **command timeout**. The sim proved this is needed: gz DiffDrive latches the
   last command and drives forever when commands stop.
4. **Pi-side serial node**, then real sensor drivers (GPS/NMEA, IMU, HC-SR04).
5. **`trash_vision` restructuring** — split the camera into its own node, replace
   JSON-in-`String` with a typed message, fix the 1000 Hz timer that blocks the
   executor.
6. **Behaviour coordinator** sequencing nav → detect → pickup. Underway, and
   further along than its position here suggests: detection parsing,
   pixel-to-point localisation, arm kinematics, closed-form IK and the mission
   state machine are all written and unit-tested (67 tests, 0.45 s), with the
   pure modules deliberately free of `rclpy` so they test without a simulator.
   What remains is the ROS shell — arm driver, detection sources, the
   coordinator node itself, then an end-to-end sim run. Tasks 13–17 of
   `docs/superpowers/plans/2026-07-30-behaviour-coordinator.md`. The arm-side
   dependency cleared when `ros2_control` landed.

The sim rotation bug sits below all of these. It blocks one measurement, not the
robot.
