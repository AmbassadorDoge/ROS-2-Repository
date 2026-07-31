# Behaviour coordinator — progress

Resume point for `2026-07-30-behaviour-coordinator.md`.
Last updated 2026-07-30. **Phase 1 complete. Next: Phase 2 (Tasks 6-10).**

Branch: **`feature/behaviour-coordinator`** (nothing pushed; `master` untouched)

---

## Done

| Task | Commit | Result |
|---|---|---|
| Design spec | `80885d9` | Approved |
| Implementation plan | `77ad22c` | 17 tasks, 5 phases |
| Container tooling | `f450191` | `scripts/dev.sh` + `scripts/docker/Dockerfile` |
| **1** Sensor pod frames | `408c840` | 27 links / 26 joints, +3/+3 over baseline |
| **2** Camera + ToF sensors | `e1f3d2b` | Both publish real data under llvmpipe |
| **3** Bridge camera/ToF/arm | `fecfdab` | **Arm moves from ROS** |
| **4** Litter objects | `84ce48f` | 3 non-static models, poses stable |
| **5** Measure arm workspace | `6baed36` | `docs/arm_workspace.md`, 210 poses |
| **5b** Pod aim investigation | `b9820a5` | Pod stays `rpy 0 0 0`; confirm pose found |

Plan corrections found by executing it: `50454aa`, `b696145`, `20936ee`,
`ca92441`, `2dbf7c3`.

Phase 2 so far: **6** `drivebase_msgs` (`bb23167`), **7** detection adapter
(`881a7c5`), **8** pixel-to-point localisation (`a595fed`), **9** URDF-derived
kinematics — 39 tests green.

## Next

**Task 10: closed-form IK** — the last of Phase 2. Solve the planar 3R arm
from `extract_planar_arm`. **Read "The arm's planar frame" below first**; the
three offsets there are exactly what a naive IK gets wrong, and Task 9 lost a
session to two of them. Run with `./scripts/dev-native.sh python -m pytest ...`.

Execution mode agreed: **subagent-driven**, one agent per task, reviewed
between tasks.

## Measured figures — use these, do not re-derive

| Parameter | Value | Source |
|---|---|---|
| `grasp_range` | **0.397 m** | lowest gripper pose, z = 0.0354 m |
| `grasp_min` | **0.287 m** | nearest x with gripper under 0.060 m |
| `grasp_max` | **0.481 m** | furthest x with gripper under 0.060 m |
| Search pose | pan 0, lift **−1.4**, elbow **0.0**, wrist_flex **0.4** | ToF 1.08–1.14 m of ground; gripper tucked to x = 0.267 m |
| Confirm pose | pan 0, lift **0.9**, elbow **0.46**, wrist_flex **−1.40** | ToF 0.2716 m, beam lands x = 0.392 m, mid grasp window |
| `wrist_roll` limit | **not measured** | harness routing; no simulator analogue. The plan's ±1.0 rad is still a guess and is *not* endorsed. |
| Camera intrinsics | fx = fy = 493.7925, cx = 320, cy = 240, 640×480 | `camera_info`, = 320/tan(1.15/2) |

Reach reproduces `STATUS.md`: 220 mm below mount (vs 219), gripper 40.1 mm above
ground (vs 41). The sensor pod did not disturb the arm's kinematics.

---

## Environment

**Two machines now. Check which one you are on before trusting a command.**

### Arch Linux workstation (current, from Task 9 onward)

No Docker, no ROS 2, no Gazebo, and no passwordless sudo. Pure-logic work runs
through **`./scripts/dev-native.sh <command>`**, which provisions a Python
3.12 venv at `~/.venvs/drivebase-dev` (numpy, pytest, urdf_parser_py, xacro,
and `ament_index_python` from source — it is not on PyPI) and a minimal ament
index at `~/.cache/drivebase-dev/ament` so xacro can resolve
`$(find drivebase_description)` without a colcon install tree.

```bash
./scripts/dev-native.sh python -m pytest src/drivebase_behaviour/test -v
```

**It covers geometry only** — anything importing `rclpy`, any launch file,
`colcon`, and all of Gazebo still need the container. A
`ModuleNotFoundError` on a ROS package is that boundary, not a bug.

Two things this cost, worth not rediscovering:

- **The repo is on an NTFS/fuseblk mount that drops the executable bit.** A
  venv created *inside* the repo produces console scripts that will not run
  (`Permission denied` on `.venv/bin/xacro`), and new scripts land as `100644`
  — `git update-index --chmod=+x` is how `dev-native.sh` got its mode.
- **Docker and ROS were offered but need a password**, which an agent session
  cannot supply. Install commands are in the handoff notes if the container
  path is wanted back.

### Apple Silicon Mac (Tasks 1–8)

No native ROS 2 or Gazebo either; everything ran through
**`./scripts/dev.sh <command>`**, which mounts the repo at `/ws` in
`drivebase-dev:jazzy-pod` (built once, on top of the pre-existing
`drivebase-dev:jazzy`, adding `ros-jazzy-urdfdom-py`).

Workspace built clean there: `./scripts/dev.sh colcon build --symlink-install`,
6 packages, ~3 s. **Not re-verified on Arch** — there is no colcon here.

### Software rendering

Docker on macOS has no GPU passthrough, so Gazebo falls back to llvmpipe.
**Measured: it works** — the camera publishes full 640×480 frames headless.

- **Trust:** geometry, tf, kinematics, frame layout, whether data flows.
- **Do not trust:** anything in Hz, or "how long did it take". Re-check on a
  real GPU. `STATUS.md` known issue #2 may be nothing but a starved controller.

---

## Verified facts worth not rediscovering

**Camera intrinsics are correct end to end.** `camera_info` reports
`fx = fy = 493.7925`, `cx = 320`, `cy = 240` at 640×480 — exactly
`320 / tan(1.15/2)`, computed independently. Task 8's `test_localisation.py`
derives `FX` from that same formula, so the geometry chain will agree.

**The arm is commandable.** `shoulder_pan` −0.00018 → 0.60008 rad on command,
other five joints holding stow. This was the gate for all of Phases 2-4.

**`/tof/pod` reports `radiation_type = 1` (INFRARED).**

### Three traps, all silent

1. **`ros2 topic echo --once` lies.** Each CLI call is a fresh node and
   intermittently gives up before discovering a low-rate publisher — it
   reported a *working* bridge as broken. Verify with one long-lived `rclpy`
   subscriber spinning ~25 s instead.
2. **`std_msgs/msg/Double` does not exist.** The gz side is `gz.msgs.Double`,
   so the guess is natural and the resulting topic carries nothing, with no
   error anywhere. Correct ROS type is `std_msgs/Float64`.
3. **Each `scripts/dev.sh` call is a fresh container.** Background processes do
   not survive between calls; launch-then-query must be one invocation.

4. **`/tof/pod` is BEST_EFFORT** (`qos_profile_sensor_data`). A **RELIABLE**
   subscriber gets one warning and then silence. An earlier version of this
   file said the opposite; it cost an agent a whole sweep.
5. **The arm droops up to 0.20 rad** from its commanded pose when extended
   (worst at lift 0.8 / elbow −1.2; the deepest grasp pose converges within
   0.010 rad). Always measure achieved tf — never assume commanded ==
   achieved. **Task 10's IK must not assume the arm reaches what it is told.**
   Whether this is a torque limit or a starved controller under llvmpipe is
   not established.
6. **`/joint_states` uses `arm_`-prefixed names**, not the command-topic names.

### The arm's planar frame — three offsets, all silent

Task 9 reduces the three pitch joints to a planar 3R arm. The reduction is
exact, but only in coordinates anchored the right way. Each of these is
invisible at the zero pose and wrong everywhere else, which is the worst
possible failure shape — the first WIP passed its zero-pose case and failed
the rest.

1. **The pan axis is 38.8 mm off the base origin** (`+x` in `arm_base_link`).
   A radius measured as `hypot(x, y)` from the origin is not conserved when
   the arm pans, so an IK built on it aims at a target that moves when it
   turns.
2. **The pitch chain runs 18.3 mm to one side of the pan plane** (y =
   −0.0183), while the tip returns to y ≈ 0 at the wrist — the gripper is
   deliberately centred. So `hypot` mixes two different planes and *no* set
   of link lengths reconciles them. `r` must be a **signed projection** onto
   the radial direction, which drops the lateral offset legitimately: the
   pitch axes are parallel to it, and rotation about an axis cannot change
   displacement along it.
3. **A positive joint command turns the arm the *negative* way in the plane.**
   The pitch joints turn about `+y`, while the plane's own positive sense
   (r toward z) is a rotation about `−y`. `PlanarArm.sense` is `−1`, derived
   from the axes rather than assumed, and `pan_sense` is `−1` too because the
   pan axis points *down*.

`PlanarArm.to_planar(point, pan)` does all three. Do not hand-roll it — and
note it takes the pan angle, because `r` is a projection and panning turns the
direction it projects onto.

**Tolerances are 1e-5, not 1e-9, and that is the model's fault, not the
code's.** The CAD export writes pi as `3.14159`, which tilts the pan axis
2.65e-6 rad off vertical and moves the tip ~1e-6 m over a pan.

**Cross-checks against measurements already in this file** (all consistent,
all in the same direction): mount height 0.260 m exactly; lowest tip 224.9 mm
below the mount vs **219–220 mm measured in sim**; grasp radius 0.389 m vs
**0.397 m measured**. The model is ideal commanded kinematics and the sim
figures include the **droop of up to 0.20 rad** noted above, so the model
reading a few mm *further* is expected. Task 10's IK inherits this: it solves
where the arm is told to go, never where it ends up.

### Settled: the pod's aim

Investigated at length in Task 5b; **do not reopen without reading
`docs/arm_workspace.md` §3.**

The pod's axis is 87.2° off the gripper's approach direction, and that is
**correct as mounted**. Aiming it at the gripper makes the ToF read a constant
0.058 m — the casting of `arm_gripper_link`, which sits 0.0619 m away along −Y.
The pod is at the *root* of a 160 mm gripper, so any aim at the grasp point
looks down its length. Clearing the castings by 60 mm costs 80 mm of
displacement off the grasp axis; mounting outboard was tried and failed too.

The grasp window *is* viewable — from pan 0, lift 0.9, elbow 0.46,
wrist_flex −1.40. Task 5's claim that no pose could see it was wrong; it never
paired strongly negative `wrist_flex` with positive `shoulder_lift`.

**Consequence for the coordinator:** at the confirm pose the gripper is *not*
over the target (gripper x = 0.572, beam x = 0.392). Seeing the grasp zone and
holding the jaws above it are mutually exclusive on this arm. `CONFIRMING`
ranges and images the target from where it can see; the descent is open-loop
from the stored point, exactly as spec §6 requires.

### Cosmetic, deliberately not fixed

- `tof_to_range` logs "Converting 1 ultrasonic cones" for an infrared sensor.
- `Range.field_of_view` is `0.0` for the pod ToF (single ray, `angle_max ==
  angle_min`). Defensible, but a consumer that renders a cone or divides by it
  gets a degenerate value.
- Gazebo emits `/tof/pod/scan/points` for every `gpu_lidar`. Unused,
  deliberately unbridged.

---

## Still true from the design

`src/trash_vision/` is **out of scope** — another author's package. The typed
message and node changes are a proposal in §9 of the spec, not work to do here.

Figures in `config/coordinator.yaml` marked `# MEASURED` are placeholders until
Task 5 produces them. Do not ship them as real.
