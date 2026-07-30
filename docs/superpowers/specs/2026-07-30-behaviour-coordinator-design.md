# Behaviour coordinator — design

Sequences the mission: patrol → detect → approach → grasp → resume. This is
step 6 of `docs/STATUS.md`'s next-steps list, brought forward because steps 1-4
are blocked on hardware lead times.

Status: **design agreed, not implemented.** Figures marked *(to measure)* are
placeholders that must be replaced with values measured in simulation before
the code depending on them is trusted.

---

## 1. What forced this design

Three findings, none of which were in `STATUS.md`, shaped everything below.

**There is no camera in the simulation.** `drivebase.urdf.xacro` defines
`camera_link` and a correctly-oriented `camera_optical_frame`, but
`gazebo.xacro` declares only the IMU, NavSat and five ultrasonic `gpu_lidar`s,
and `bridge.yaml` bridges no image topic. `trash_vision`'s `detector_node.py`
opens `cv2.VideoCapture("tcp://127.0.0.1:8888")` — a hardware stream — and
loads weights from `/home/christopherg/best_ncnn_model`. So the detect and
approach half of the mission had nothing to consume in simulation, and the real
detector cannot run there at all.

**The arm is not commandable from ROS.** `gazebo.xacro` mounts six
`gz-sim-joint-position-controller-system` plugins on `/arm/<joint>/position`,
but those are *gz transport* topics and `bridge.yaml` contains no `ROS_TO_GZ`
entries for them. The arm holds its stow pose and nothing in ROS can move it.

**Sensing moved to the end effector.** The original design had a
chassis-mounted camera, bearing-only sensing, a flat-ground homography for
range, and the *base* visually servoing to centre the target. That plan ran
directly into known issue #1 — the base cannot rotate in place more than once
per run. Mounting camera and a time-of-flight rangefinder on the arm replaces
it: a pixel gives a ray, the ToF gives scalar distance along that ray, and the
product is a Cartesian point. `shoulder_pan` then does the aiming instead of
the base.

That last change is the important one, and its consequence is worth stating
plainly:

> **The base no longer has to aim.** It only has to park the litter somewhere
> inside the arm's workspace — a ~200 mm target rather than a ~40 mm one. That
> is a tolerance the base can hit even with 0.35 m of turn drift. The rotation
> bug leaves the critical path for pickup.

It also removes the flat-ground assumption entirely, which was the weakest part
of the original design: it assumed level pavement and would have degraded on
grass or any slope.

---

## 2. Decisions

| Decision | Rationale |
|---|---|
| Hand-rolled explicit FSM, tick-driven | No new dependency on a Pi already sharing cores with YOLO. Matches this repo's stated preference for explicit node lists over framework magic (see the `nav2_bringup` rationale in `navigation.launch.py`). Transition table is pure Python, so it unit-tests without spinning ROS. |
| Waypoint patrol as the top-level loop | Deterministic and demoable; matches how a competition run is scored. Vision interrupts the patrol; pickup resumes it at the same waypoint. |
| Approach drives `/cmd_vel_nav`, not `/cmd_vel` | Cancelling the Nav2 goal stops `controller_server` publishing, so exactly one publisher exists at any instant. Reuses the existing `velocity_smoother` → `collision_monitor` chain, so approach keeps smoothing and ultrasonic collision safety. |
| Approach steers in arcs, never point-turns | Maintains a nonzero forward velocity while correcting heading. Independently favoured by the RPP choice and the power budget, and it sidesteps known issue #1. |
| Eye-in-hand only, plus an arm search pose | One camera, one calibration, one image pipeline. The chassis mount stays in the URDF as the documented fallback if detection-at-range proves to be the weak link. |
| Sensor pod on `arm_wrist_link` (before `wrist_roll`) | Camera never rolls, so the image up-vector is fixed. Harness crosses four moving joints rather than five. Parallax to the jaws is acceptable because the final descent is open-loop regardless. |
| Closed-form planar IK | `shoulder_lift`, `elbow_flex` and `wrist_flex` all pitch in the `shoulder_pan` plane — a 3R planar arm. Fixing the approach direction to vertical collapses it to a 2R solve. Tens of lines of trig, microseconds to run, testable against forward kinematics. |
| Grasp descent is open-loop from a stored point | See §6. Non-negotiable, not a simplification. |
| `trash_vision` is not modified | Partner's package. This document contains a proposal (§9); the code change is theirs to make. |

### Decisions reversed during design

**Scripted joint waypoints → closed-form IK.** Scripted poses with
`shoulder_pan` aimed by bearing was correct for bearing-only sensing. Given a
true Cartesian point, scripted poses cannot use it — the arm would reach the
same place regardless of how far away the litter actually is. Reaching a
commanded point is IK by definition.

**Flat-ground homography → direct ray × range.** Superseded by the ToF. The
homography survives only as a degraded fallback (§5).

---

## 3. Packages

| Package | Change |
|---|---|
| `drivebase_msgs` | **New**, `ament_cmake`. Holds the proposed typed `LitterDetection.msg`. Python packages cannot generate messages, which is why this is separate. |
| `drivebase_behaviour` | **New**, `ament_python`. Coordinator node, detection adapter, target localisation, IK, arm sequencer, launch, params. |
| `drivebase_description` | Sensor pod: camera + ToF as children of `arm_wrist_link`. Camera and ToF sensors in `gazebo.xacro`. Arm search pose. |
| `drivebase_sim` | Bridge entries (image, camera_info, ToF scan, six arm joints), litter objects in the world, `sim_litter_detector` shim. |
| `trash_vision` | **Untouched.** |

---

## 4. Frames and the sensor pod

Existing geometry, confirmed from the URDF rather than assumed:

| Quantity | Value | Source |
|---|---|---|
| `base_link` height above ground | 0.060 m | `base_footprint_to_base_link`, = `wheel_radius` |
| `front_face_x` | 0.215 m | `chassis_length / 2`, `chassis_length = 0.430` |
| Arm mount above ground | 0.260 m | `arm_mount_z` 0.200 + 0.060 |
| Chassis camera (unused, retained) | 0.310 m above ground, x = 0.215 | `camera_joint` |

New frames, both fixed children of `arm_wrist_link`:

- `pod_camera_link`, with `pod_camera_optical_frame` following the ROS optical
  convention (z forward, x right, y down), exactly as the existing
  `camera_optical_frame` does.
- `pod_tof_link`.

**Co-location is a hard requirement, not a preference.** A 20 mm camera-to-ToF
separation at 80 mm range is a large angular disagreement: the ToF can be
reading past the target while the camera reports it centred. The pod must place
both apertures as close as the physical parts allow, and the residual offset
must be recorded as extrinsics in the URDF — not assumed to be zero.

The camera→arm-base transform is already available for free: Gazebo's
`JointStatePublisher` is configured with no `<joint_name>` filter, so it
publishes all six arm joints, and `robot_state_publisher` builds the arm's tf
chain from them live.

ToF is simulated the way the ultrasonics are — a `gpu_lidar` with a single
sample and a narrow cone — and collapsed to `sensor_msgs/Range` by the existing
`laserscan_to_range` node.

**`wrist_roll` must be software-limited during pickup.** The URDF allows
−2.744…+2.841 rad (±160°). A harness carrying camera and ToF cable through that
joint will not survive its full travel. The limit belongs in the coordinator's
parameters, and the value is a hardware decision *(to measure)*.

---

## 5. Target localisation

```
ray   = K⁻¹ · [u, v, 1]              # pixel → unit ray in pod_camera_optical_frame
point = ray · range                   # ToF scalar distance along that ray
p_odom = tf(pod_camera_optical_frame → odom) · point
```

`K` comes from `camera_info`, published by the Gazebo camera sensor and bridged.

**Pointing order is bearing-first.** A single-point ToF returns a meaningful
number only when it is actually aimed at the target. The loop centres with the
camera, *then* trusts the range. Reading range before centring measures ground
distance, not target distance.

**ToF fallback.** Time-of-flight sensors are worst on exactly what litter is
made of: clear plastic, shiny film, dark matte surfaces. A VL53-class part will
drop out or read long on a clear bottle. The coordinator rejects a range that
is absent, below the sensor minimum, or outside the arm's workspace, and falls
back to a bbox-height distance estimate — crude, and indifferent to surface
finish. A fallback-derived point is flagged in the `Detection` record so the
confirmation gate can demand more consecutive frames before committing.

---

## 6. Why the grasp is open-loop

As the gripper descends, the target is occluded by the jaws and leaves the
frame; at grab range the target is also inside a VL53-class sensor's minimum
range (~30–40 mm). **Visual servoing cannot run through the descent.**

So the coordinator captures the 3D point at the moment of confirmation,
transforms it into `odom` — a static frame, so base creep during the ~2 s grasp
is absorbed rather than ignored — and executes the descent from that stored
point.

This makes the confirmation gate the load-bearing safety check of the whole
design. It is not a formality.

---

## 7. State machine

```
IDLE ──start──> NAVIGATING ──confirmed detection──> APPROACHING
                    ^  |                                 |
        resume      |  | waypoint reached                | inside workspace
        patrol      |  v                                 v
                    +──── STOWING <── PICKING <──── CONFIRMING
                    |                                    |
                    +────────── lost / timeout ──────────+
```

| State | Behaviour | Exit |
|---|---|---|
| `IDLE` | Arm to search pose. Waits for start. | On start → `NAVIGATING` |
| `NAVIGATING` | Sends waypoint *i* to `NavigateToPose`; arm holds search pose; watches detections throughout. | Confirmed target → cancel goal → `APPROACHING`. Goal reached → next waypoint. Goal aborted → log, next waypoint. |
| `APPROACHING` | Publishes servo `Twist` to `/cmd_vel_nav`, always with nonzero forward velocity (arcs). Arm holds the search pose. The base is responsible only for coarse alignment and closing range — it drives until the target is inside the arm's workspace, and no further. | In workspace → `CONFIRMING`. Target lost > `lost_timeout` → resume patrol. Timeout → resume patrol. |
| `CONFIRMING` | Base velocity zero. **The arm does the centring, not the base:** `shoulder_pan` (and `wrist_flex` for elevation) drives `horizontal_error` toward zero, bearing-first, and only then is the ToF range trusted (§5). Requires *N* consecutive frames: detected, `\|horizontal_error\| ≤ deadband`, range valid and inside `[grasp_min, grasp_max]`. Stores the resulting point in `odom`. | Pass → `PICKING`. Fail/timeout → `APPROACHING` once, then resume patrol. |
| `PICKING` | IK to the stored point, vertical approach, close gripper, lift. Base commanded to zero velocity throughout. | Always → `STOWING` |
| `STOWING` | Arm to search pose. | → `NAVIGATING`, resuming at waypoint *i* |

Every state carries a timeout whose fallback is resuming the patrol. **A failed
pickup loses one piece of litter; it must never wedge the mission.**

---

## 8. Detection adapter

The coordinator never sees JSON. An internal frozen `Detection` dataclass is
produced by one of two interchangeable sources, selected by a
`detection_interface` parameter:

- `StringJsonDetectionSource` (**default**) — subscribes `/vision/target`
  (`std_msgs/String`), parses today's JSON payload, stamps on receipt.
- `TypedDetectionSource` — subscribes `/vision/detection`
  (`drivebase_msgs/LitterDetection`), uses the header stamp.

Swapping is a parameter change. The coordinator's tests do not know which
exists.

**A limitation worth stating rather than hiding:** `std_msgs/String` carries no
timestamp, so the JSON source must stamp on arrival. Under load this
misattributes latency and the coordinator can servo on a stale detection. This
is the strongest single argument for the typed message.

---

## 9. Proposal for `trash_vision` (not implemented here)

Offered for the package's author to accept or reject.

**Add a header and make the continuous error primary.** `LEFT`/`RIGHT`/`CENTER`
was the right shape for steering a base. For an eye-in-hand arm it is lossy —
the arm uses the proportional error directly, and bucketing it into thirds
discards the precision the ToF just bought. `horizontal_error` is already in the
JSON payload, so this is a re-emphasis rather than new work.

```
# drivebase_msgs/LitterDetection.msg
std_msgs/Header header          # stamped at capture, not at publish

bool    detected
uint32  frame_number
float32 confidence

float32 bbox_x1                 # pixels
float32 bbox_y1
float32 bbox_x2
float32 bbox_y2

float32 center_x                # normalised 0..1
float32 center_y
float32 floor_x
float32 floor_y

float32 horizontal_error        # PRIMARY: continuous, -0.5 .. +0.5

uint8 DIRECTION_NO_TARGET = 0   # derived from horizontal_error, kept for
uint8 DIRECTION_LEFT      = 1   # compatibility with existing consumers
uint8 DIRECTION_CENTER    = 2
uint8 DIRECTION_RIGHT     = 3
uint8 direction
```

Three other issues observed in `detector_node.py`, listed for the author:

1. `create_timer(0.001, self.process_frame)` runs YOLO inference inside a
   1 kHz timer callback. Nothing else in that executor gets scheduled.
   Capture belongs in its own node or thread.
2. `model_path` defaults to `/home/christopherg/best_ncnn_model` and
   `mapping_upload_url` to a hardcoded LAN address. Neither is portable.
3. `cv2.VideoCapture` on a TCP stream means the node cannot run against a
   simulated camera at all.

---

## 10. Testing

**Unit, no ROS spin:**

- Ray-times-range localisation against hand-computed points.
- Closed-form IK against forward kinematics — solve, then FK the solution and
  assert it returns the commanded point. Includes unreachable points and
  joint-limit violations.
- FSM transition table driven by synthetic `Detection` sequences: target lost
  mid-approach, confidence flapping at the threshold, ToF dropout, every
  timeout path.
- JSON adapter against malformed, truncated and missing-field payloads.

**Integration, in simulation:**

`sim_litter_detector` — a colour-blob detector reading the simulated camera and
publishing the *exact same* `/vision/target` and `/vision/direction` interface
as `trash_vision`. The real YOLO detector cannot run in simulation (hardware
model path, TCP stream), so this shim is how the loop gets exercised. Because
the interface is identical, hardware swaps the real detector in with no
coordinator change.

Litter objects (small, brightly coloured) get added to `test_field.sdf`, clear
of the existing obstacles at x > 1.

---

## 11. Phasing

Each phase is independently verifiable, and each leaves the tree working.

1. **Sim plumbing.** Sensor pod frames, camera + ToF sensors, bridge entries
   for image/`camera_info`/ToF/six arm joints, litter objects in the world.
   *Verify:* image topic renders, `ros2 topic pub` moves an arm joint.
   *Measure here:* arm forward reach, workspace bounds, search pose, and the
   `grasp_*` figures left open in §12.
2. **`drivebase_msgs` + localisation + IK + adapter.** Pure logic, fully
   unit-tested, no coordinator yet.
3. **`sim_litter_detector`.** Publishes the current String interface.
   *Verify:* detections appear when litter is in view.
4. **Coordinator FSM + launch.** *Verify:* a full patrol → detect → approach →
   grasp → resume cycle in simulation.
5. **Docs.** Update `STATUS.md` — state table, new measured figures, and the
   fact that the rotation bug no longer blocks pickup.

---

## 12. Open — must be measured, not assumed

| Item | Why it is not filled in |
|---|---|
| `grasp_range`, `grasp_min`, `grasp_max` | Depend on the arm's *forward* reach. `STATUS.md` records the 219 mm vertical drop and 41 mm gripper height, never the horizontal extent. |
| Arm search pose | The current stow (`shoulder_lift −1.20`, `elbow_flex 1.55`, `wrist_flex 1.10`) is folded up and likely sees little ground ahead. |
| IK link lengths L1, L2, L3 | Must be derived from the xacro joint origins at build time, never hardcoded. |
| `wrist_roll` software limit | A harness routing decision, made when the arm bracket is finalised. |
| Camera↔ToF extrinsics | Depends on the physical pod. Must not be assumed zero (§4). |
| Camera FOV and resolution | Should match the real part once chosen, so simulation and hardware agree. |

---

## 13. Risks

**Cabling.** Camera plus ToF on the end effector means a harness through four
moving joints. Route it before the arm bracket is finalised, not after.

**ToF on real litter.** §5's fallback is a mitigation, not a fix. If clear
bottles prove to be a large fraction of the target set, this needs revisiting
with real hardware data.

**Search coverage.** A single eye-in-hand camera close to the ground has a
narrow view of ground far ahead — hunting with a magnifying glass. If
detection-at-range is the weak link, the chassis camera mount at 0.310 m is
already in the URDF and this is the reason it was kept.

**Sim fidelity of the grasp.** Simulated contact between a box-collision
gripper and small objects is unreliable. A successful grasp in Gazebo is weaker
evidence than a successful approach.
