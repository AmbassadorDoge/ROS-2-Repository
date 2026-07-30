# Behaviour coordinator — progress

Resume point for `2026-07-30-behaviour-coordinator.md`.
Last updated 2026-07-30, paused mid-Phase 1.

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

Plan corrections found by executing it: `50454aa`, `b696145`, `20936ee`.

## Next

**Task 4: litter objects in `test_field.sdf`.** Then 5 (measure arm
workspace), and Phase 2 (Tasks 6-10, pure logic — no simulator needed).

Execution mode agreed: **subagent-driven**, one agent per task, reviewed
between tasks.

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

### Open item for Task 5

**The ToF can see the arm's own gripper** — 0.062-0.071 m at the stow pose,
which is not ground. The pod points along the wrist axis with the jaws in front
of it. Any search-pose candidate reading under ~0.3 m is looking at the robot.
Reject those; if *every* candidate is occluded, the pod's mounting `rpy` needs
a pitch offset. A ToF that sees the gripper during `CONFIRMING` would confirm a
grasp on the robot's own hand.

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
