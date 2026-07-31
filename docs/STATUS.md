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

`sim.launch.py` flags: `headless`, `rviz`, `ekf`, `gps`, `arm`, `litter_detector`,
`wheel_mu1`, `wheel_mu2`, `wheel_slip`, `slip_lateral`, `slip_longitudinal`.

Needs a real GPU. Software rendering works (`LIBGL_ALWAYS_SOFTWARE=1`) but runs
well under real time and starves the control loops.

---

## Measured figures

| Quantity | Value | Source |
|---|---|---|
| Turn cost, local EKF | **~0.007 m per 90°** (was 0.35 — that measured the contact bug) | `localization_accuracy.md` |
| Straight-line cost | negligible over 20 m | same |
| Effective-track factor | **2.112×** (stdev 0.008, n=10) | same |
| Global (GPS) error | 1.4–5.6 m, bounded | same |
| Ultrasonic vs world geometry | 2.732 m vs 2.75 m predicted | verified in sim |
| Arm reach below its mount | **219 mm** | measured in sim |
| Gripper height at full extension | **41 mm** above ground | measured in sim |
| Motion power draw | ~62 W pavement, ~105 W grass | `power_budget.md` |

**The single most important consequence:** **the arm must be aimed by the
camera, never by driving to a coordinate.** The existing detector's
LEFT/RIGHT/CENTER output is the right shape for that.

The reasoning behind that changed on 2026-07-31 even though the conclusion did
not. It used to rest on turns costing ~0.35 m each; that figure was measuring
the contact bug below, and with the bug fixed a five-turn 20 m square drifts
0.035 m. It now rests on GPS error of 1.4–5.6 m, a simulated IMU better than any
real part, and no contact model here having been checked against hardware.
Litter is far smaller than any of those.

---

## Known issues

### 1. ~~Simulated robot cannot rotate in place more than once per run~~ — FIXED 2026-07-31

The body used to stay stationary while the wheel joints kept spinning (measured:
`odom_yaw` +279°, `true_yaw` 0.0°). Ruled out along the way: commanded angular
velocity, the arm, `wheel_mu2`, translation between turns, and software
rendering.

**Cause.** gz-sim's default physics engine is **dartsim**, which takes a single
friction coefficient and does not implement the anisotropic `mu1`/`mu2` split.
The `<mu1>`/`<mu2>` pair in `gazebo.xacro` maps to ODE's friction element, so
the lateral value was discarded on load with no warning. That is why sweeping
`wheel_mu2` across 0.6/0.3/0.15 changed nothing — the parameter had no effect to
have. The robot ran effectively isotropic at `mu1` 1.0 against a 0.9 ground
plane, the exact case the xacro comment warns makes a skid-steer refuse to
rotate.

**Fix: `gz-sim-wheel-slip-system`** — candidate 1, and it was the right call.
It supplies the anisotropy dartsim throws away, modelling slip properly instead
of blunting friction globally the way lowering `mu1` does. On by default
(`wheel_slip:=true`), with `slip_lateral` 1.0 and `slip_longitudinal` 0.0
exposed and sweepable. Lateral compliance is what lets the tyres scrub through a
point turn; longitudinal near zero keeps them gripping for drive.

Results, `mu1` left at 1.0 throughout:

| | Before | After |
|---|---|---|
| `twoturns` | aborts on the first turn | completes, 181°, **0.10 m** scrub |
| `square5` | aborts at the first corner | completes, 20.3 m, 453° |
| Effective-track factor | 1.742× (stdev 0.025, n=6) | **2.112×** (stdev 0.008, n=10) |
| Local EKF error, full square | not measurable | **0.035 m** |

For comparison the `wheel_mu1:=0.35` workaround got `twoturns` through at 0.42 m
of scrub; the slip model does it at 0.10 m without touching `mu1`.

**Every figure in `localization_accuracy.md` was re-measured against this**, and
two of them moved. The effective-track factor went *up* 21%, because the wheels
now scrub freely rather than the body sliding. And the turn-cost figure turned
out to be measuring the bug: what was recorded as 0.35 m of error per 90° was
largely the body sliding 0.156 m during a turn it never completed.

That also retires the old claim that the track factor was "stable across
lateral-friction values from 0.15 to 0.6." It was stable because those values
were being discarded.

`runs/slip_square5.csv` and `runs/slip_square5_b.csv` are the two runs behind
the numbers above.

**Also fixed: `scripts/analyse_drift.py` was silently discarding every turn.**
It diffed yaw endpoints, which cannot represent a rotation past π, so it carried
a `wheel_delta > 2.8` guard — and the new over-report pushed all ten turns past
it. It reported "no usable turns found" for a run where every turn was clean. It
now accumulates the wrapped step-to-step difference, which has no such ceiling.

**Still not validated against hardware.** `slip_lateral` 1.0 has no more
physical justification behind it than `mu1` 0.35 did; it is a value that makes
the simulation behave like a skid-steer instead of a sledge. Re-measure once
there are real tyres on real ground.

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
  wheel yaw is biased, not just noisy. Measured 2.112× over-report — and that
  figure moved 21% when the contact model was fixed, which is itself the
  argument: it is a property of the tyre model, not a constant to trust.
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
