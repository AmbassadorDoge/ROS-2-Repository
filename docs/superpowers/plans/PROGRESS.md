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

## Next

**Phase 2: Tasks 6-10** — `drivebase_msgs`, detection adapter, localisation,
URDF-derived kinematics, closed-form IK. **All pure logic: no simulator, no
GPU, fast to test.** Run with `./scripts/dev.sh python3 -m pytest ...`.

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

Apple Silicon Mac. No native ROS 2 or Gazebo. **Everything runs through
`./scripts/dev.sh <command>`**, which mounts the repo at `/ws` in
`drivebase-dev:jazzy-pod` (built once, on top of the pre-existing
`drivebase-dev:jazzy`, adding `ros-jazzy-urdfdom-py`).

Workspace builds clean: `./scripts/dev.sh colcon build --symlink-install`,
6 packages, ~3 s.

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
