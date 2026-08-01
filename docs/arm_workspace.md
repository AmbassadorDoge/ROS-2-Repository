# Arm workspace, measured

Measured in Gazebo Harmonic under llvmpipe software rendering, via tf
`base_link` -> `arm_gripper_frame_link`, driving `/arm/<joint>/position`
(`std_msgs/Float64`) and reading `/tof/pod` and `/pod_camera/image_raw`.
Ground is `base_link` z + 0.060.

Every number below came from a run. Task 5 commanded 210 poses across five
simulator sessions; Task 5b added 154 more across five further sessions, under
three different pod mount configurations. The tables are the subset that bounds
each figure. Nothing here is interpolated or hand-solved. Timing figures are
deliberately absent — software rendering makes them meaningless (see the plan's
Global Constraints).

**What Task 5b re-measured, and what carried over.** §3 is rewritten: every ToF
figure Task 5 recorded there was taken before the pod's aim had been
investigated, and its conclusion that no pose could see the grasp window was
wrong. §1's gripper positions and §2's reach figures carried over untouched —
rotating a sensor does not move an arm — and §1's one ToF entry (the stow pose)
was re-taken. §5 (droop), §6 (agreement with `STATUS.md`) and the camera-to-ToF
offset finding in §4 do not depend on pod orientation and carried over.

`shoulder_pan`, `wrist_roll` and `gripper` were held at 0 for every pose in
these tables, so only the three pitch joints are listed.

---

## 1. Gripper position by joint pose

`cmd` is what was published; `act` is what `/joint_states` reported after
settling. The arm does not always reach its target — see §5.

### Deepest reach, and the ground-contact envelope

| shoulder_lift | elbow_flex | wrist_flex | act lift/elbow/wrist | x (m) | z above ground (m) |
|---|---|---|---|---|---|
| 1.74 | −0.2 | 0.0 | 1.745 / −0.208 / 0.001 | 0.3973 | **0.0354** |
| 1.74 | −0.3 | 0.0 | 1.745 / −0.297 / 0.003 | 0.4231 | 0.0372 |
| 1.74 | −0.1 | 0.05 | 1.745 / −0.097 / 0.049 | 0.3570 | 0.0376 |
| 1.74 | 0.0 | 0.0 | 1.745 / −0.006 / −0.002 | 0.3385 | 0.0399 |
| 1.74 | −0.4 | 0.0 | 1.745 / −0.387 / 0.006 | 0.4488 | 0.0413 |
| 1.74 | −0.5 | 0.0 | 1.745 / −0.477 / 0.009 | 0.4741 | 0.0476 |
| 1.74 | 0.3 | −0.2 | 1.745 / 0.236 / −0.208 | 0.3013 | 0.0493 |
| 1.74 | −0.6 | 0.2 | 1.745 / −0.574 / 0.204 | 0.4704 | 0.0494 |
| 1.74 | 0.2 | 0.0 | (run 5) | 0.2874 | 0.0539 |
| 1.74 | −0.7 | 0.3 | 1.745 / −0.668 / 0.305 | 0.4809 | **0.0553** |
| 1.74 | 0.5 | −0.4 | 1.745 / 0.426 / −0.407 | 0.2799 | 0.0605 |
| 1.74 | −0.8 | 0.4 | 1.745 / −0.764 / 0.404 | 0.4910 | 0.0622 |
| 1.60 | 0.0 | 0.0 | 1.658 / −0.008 / 0.002 | 0.3678 | 0.0459 |
| 1.40 | 0.0 | 0.0 | (run 5) | 0.4131 | 0.0613 |

The last two rows of the 1.74 block are the first poses to break 0.060 m at
each end; they are what pins `grasp_min` and `grasp_max`.

### Further out and further in (context for the envelope)

| shoulder_lift | elbow_flex | wrist_flex | x (m) | z above ground (m) |
|---|---|---|---|---|
| 1.74 | −0.9 | 0.5 | 0.5005 | 0.0703 |
| 1.74 | −1.0 | 0.6 | 0.5093 | 0.0793 |
| 1.74 | −1.2 | 0.8 | 0.5240 | 0.0998 |
| 1.74 | 0.7 | −0.6 | 0.2606 | 0.0761 |
| 1.74 | 0.9 | −0.8 | 0.2443 | 0.0955 |
| 1.74 | 1.1 | −1.0 | 0.2316 | 0.1182 |
| 1.74 | 1.69 | −1.6 | 0.2213 | 0.1974 |
| 1.1 | −1.2 | 0.0 | 0.6926 | 0.3689 |
| 0.0 | 0.0 | 0.0 | 0.6124 | 0.4092 |

`x = 0.693` (lift 1.1, elbow −1.2, wrist 0.0) is the furthest forward the
gripper frame reached in any run, but at 0.369 m up it is nowhere near litter.

### Search-pose candidates (arm raised, pod looking forward)

| shoulder_lift | elbow_flex | wrist_flex | grip x (m) | grip z above ground (m) | pod height (m) |
|---|---|---|---|---|---|
| −1.4 | 0.0 | 0.4 | 0.2740 | 0.6817 | 0.5439 |
| −1.4 | 0.0 | 0.5 | 0.2897 | 0.6718 | 0.5437 |
| −1.2 | 0.0 | 0.2 | 0.3264 | 0.6951 | 0.5615 |
| −1.4 | 0.4 | 0.0 | 0.3480 | 0.6589 | 0.5281 |
| −1.0 | 0.0 | 0.0 | 0.3837 | 0.6935 | 0.5660 |

### Reference: the current stow pose

| pose | grip x (m) | grip z above ground (m) | ToF (m) |
|---|---|---|---|
| lift −1.20, elbow 1.55, wrist 1.10 | 0.3122 | 0.2322 | 0.070 |

0.070 m is the arm's own structure, not ground — see §3. (Task 5 recorded 0.066
for the same pose; 0.070 is the Task 5b re-measurement.)

---

## 2. Derived figures

| Parameter | Value | How it was obtained |
|---|---|---|
| `grasp_range` | **0.397 m** | x of the pose that puts the gripper lowest: lift 1.74 / elbow −0.2 / wrist 0.0, z = 0.0354 m |
| `grasp_min` | **0.287 m** | nearest x still under 0.060 m: lift 1.74 / elbow 0.2 / wrist 0.0, z = 0.0539 m. The next step in (elbow 0.5, wrist −0.4) reached x = 0.280 but z = 0.0605, over the line |
| `grasp_max` | **0.481 m** | furthest x still under 0.060 m: lift 1.74 / elbow −0.7 / wrist 0.3, z = 0.0553 m. The next step out (elbow −0.8, wrist 0.4) reached x = 0.491 but z = 0.0622 |
| Search pose | **pan 0, lift −1.4, elbow 0.0, wrist_flex 0.4, wrist_roll 0, gripper 0** | ToF read 1.138 m (50 samples) against a tf-only predicted ground intersection of 1.118 m, at x = 1.168 m ahead of `base_link`. Gripper tucked to x = 0.267 m. See §3 for why this is ground and not the robot, and for why Task 5's 1.080 m differs |
| Confirm pose | **pan 0, lift 0.9, elbow 0.46, wrist_flex −1.40, wrist_roll 0, gripper 0** | ToF read 0.2716 m (50 samples) against a tf-only prediction of 0.2715 m, putting the beam on the ground at x = 0.392 m — inside the 0.287–0.481 m grasp window. Found in Task 5b; §3 has the three cross-checks and the caveat that the gripper is not over the target |
| `wrist_roll` limit | **not measured** | It is a harness-routing decision and there is no harness yet, so simulation cannot produce it. The URDF hard limit is −2.74385…+2.84121 rad. The plan's provisional ±1.0 rad is still a guess and is not endorsed here. One thing *is* measured: the pod hangs off `arm_wrist_link`, which is upstream of `wrist_roll`, so `pod_tof_link` and `pod_camera_optical_frame` do not move when `wrist_roll` turns. Task 5b confirmed it from the other side: sweeping `wrist_roll` across −2.5…+2.5 rad moved the ToF reading by under 2 mm |

Lowest point reached anywhere: **35.4 mm** above ground, i.e. **224.6 mm**
below the arm mount (mount is 0.260 m above ground).

Usable ground-grasp span: **0.287 m to 0.481 m** from `base_link`, a 194 mm
window. The chassis front face is at x = 0.215 m, so the window starts 72 mm
in front of the robot.

---

## 3. The pod's aim: what was tried, and what it can actually see

Task 5 measured that the pod's axis is 87.2 deg away from the gripper's
approach direction, and recommended rotating the mount to close that gap.
Task 5b applied that rotation and re-measured. **The rotation is wrong and has
been reverted.** The 87.2 deg is real, but it is what makes the sensor usable,
not what breaks it.

### Aiming at the gripper blinds the sensor

`arm_wrist_link`'s +X does not point at the gripper. Static geometry from tf
with `arm_wrist_link` as the parent, unchanged from Task 5 and re-confirmed
here (it is fixed geometry, so it does not depend on pod orientation):

| child | xyz in `arm_wrist_link` (m) | distance (m) | unit vector |
|---|---|---|---|
| `arm_gripper_frame_link` | 0.00790, −0.15923, 0.01832 | 0.16047 | 0.0492, −0.9922, 0.1142 |
| `pod_tof_link` | 0.01000, 0.00000, 0.01500 | 0.01803 | — |
| `pod_camera_optical_frame` | 0.01000, 0.00000, 0.03000 | 0.03162 | — |

Two mount rotations were built and driven in the simulator:

| pod joint `rpy` | pod xyz | angle to gripper | poses driven | ToF returned |
|---|---|---|---|---|
| `0 −0.1145 −1.5213` | 0.010 / 0.015 | **0.02 deg** | 31 | **0.058 m**, always |
| `0 0.0402 −1.6374` | 0.030 / 0.040 | 11.06 deg | 44 | **0.089 m**, always |
| `0 0 0` (kept) | 0.010 / 0.015 | 87.18 deg | 29 | see below |

The first row is exactly what the plan prescribed, and its angle check passes:
`pod_tof_joint` and `pod_camera_joint` both came out at **0.024 deg** to the
gripper. The aim is correct. The sensor is still blind.

Both failing figures are constant to within ±3 mm across poses whose geometry
is completely different — arm folded, arm extended, gripper at ground level,
gripper 0.68 m up. Anything rigidly attached to `arm_wrist_link` would behave
exactly like that, and nothing else would.

The decisive check is the sky test. These poses point the ray **upward**, where
an unobstructed beam must return the sensor's `<max>` of 4.000 m:

| `rpy` / pod xyz | pose (lift / elbow / wrist) | ray z | ToF returned |
|---|---|---|---|
| `0 −0.1145 −1.5213` / 0.010, 0.015 | −1.4 / 0.0 / 0.4 | +0.842 | 0.057 m |
| `0 −0.1145 −1.5213` / 0.010, 0.015 | −1.4 / 0.0 / 0.4, gripper open | +0.822 | 0.058 m |
| `0 0.0402 −1.6374` / 0.030, 0.040 | −1.4 / 0.0 / 0.4 | +0.899 | 0.089 m |
| `0 0.0402 −1.6374` / 0.030, 0.040 | −1.4 / 0.0 / 0.4, gripper open | +0.898 | 0.091 m |
| `0 0 0` / 0.010, 0.015 | −0.10 / −1.45 / −1.10 | +0.925 | **4.000 m** |
| `0 0 0` / 0.010, 0.015 | −1.00 / −1.55 / −0.10 | +0.988 | **4.000 m** |
| `0 0 0` / 0.010, 0.015 | 0.50 / −1.65 / −1.50 | +0.829 | **4.000 m** |
| `0 0 0` / 0.010, 0.015 | 0.40 / −1.55 / −1.50 | +0.841 | **4.000 m** |

Aimed at the gripper, the beam reports 57 mm of open sky. The camera frame saved
at that pose shows the arm's own castings filling about 70% of it
(`runs/pod_sweep/sky_r0.0.png`). It is not a grazing incidence or a marginal
case: at that mount `wrist_roll` was swept from −2.5 to +2.5 rad and the gripper
opened and closed, 11 combinations, and the reading never left 0.057–0.059 m.

**Why.** The pod sits at the root of a 160 mm gripper. Any aim that points at
the grasp point looks lengthwise down the gripper body. Ray-casting the visual
meshes (`scripts/pod_occlusion.py`, exact triangle geometry from the same STLs
the `gpu_lidar` renders) says clearing the castings by 60 mm costs 80 mm of
displacement from the grasp axis at the jaws — the two quantities are the same
quantity. There is no rotation that fixes it, and moving the mount outboard
does not either: the second row above is that experiment.

At `rpy 0 0 0` the beam clears the nearest casting by **69.5 deg**.

### Task 5's "no arm pose sees the grasp zone" was wrong

This is the correction that matters. Task 5 concluded that the grasp window
could not be viewed from any pose. It can. Task 5 drove 210 poses and simply
never drove the corner where it happens — strongly negative `wrist_flex`
(−1.25 to −1.60) against positive `shoulder_lift`, which none of its sweeps
paired.

`scripts/pod_aim_search.py` found it by sweeping the full commanded joint space
offline. That is arithmetic, not measurement, so it is only usable as a screen:
its FK reproduces the ray each of the 44 driven poses actually reported to
**1.0e-3**, and every pose it proposed was then driven for real. The numbers
below are the driven ones.

### Confirm poses — measured

Ground at `base_link` z + 0.060. "Predicted" is `pod_height / −ray_z` from tf
alone, with no reference to the ToF.

| cmd lift/elbow/wrist | act lift/elbow/wrist | pod height | ray | predicted | **measured ToF** | err | ground hit x | grip x | grip z |
|---|---|---|---|---|---|---|---|---|---|
| 0.9 / 0.46 / −1.40 | 0.997 / 0.466 / −1.385 | 0.2707 | −0.078, 0, −0.997 | 0.2715 | **0.2716** | **+0.02%** | **0.3919** | 0.5718 | 0.2605 |
| 1.0 / 0.36 / −1.35 | 1.034 / 0.365 / −1.346 | 0.2682 | −0.053, 0, −0.999 | 0.2685 | 0.2688 | +0.10% | 0.4088 | 0.5821 | 0.2618 |
| 0.9 / 0.66 / −1.55 | 0.941 / 0.654 / −1.543 | 0.2753 | −0.054, 0, −0.999 | 0.2757 | 0.2747 | −0.36% | 0.3783 | 0.5521 | 0.2689 |
| 0.8 / 0.66 / −1.55 | 0.848 / 0.662 / −1.541 | 0.2857 | +0.031, 0, −0.999 | 0.2859 | 0.2852 | −0.23% | 0.4098 | 0.5600 | 0.2928 |
| 1.1 / 0.36 / −1.35 | 1.126 / 0.357 / −1.347 | 0.2559 | −0.137, 0, −0.991 | 0.2584 | 0.2580 | −0.14% | 0.3781 | 0.5715 | 0.2362 |
| 0.9 / 0.80 / −1.60 | 0.896 / 0.779 / −1.597 | 0.2804 | −0.078, 0, −0.997 | 0.2813 | 0.2812 | −0.03% | 0.3580 | 0.5389 | 0.2701 |
| 1.0 / 0.46 / −1.40 | 1.084 / 0.459 / −1.387 | 0.2600 | −0.155, 0, −0.988 | 0.2632 | 0.2626 | −0.21% | 0.3634 | 0.5619 | 0.2373 |
| 1.1 / 0.20 / −1.25 | 1.129 / 0.210 / −1.248 | 0.2593 | −0.091, 0, −0.996 | 0.2604 | 0.2593 | −0.43% | 0.4093 | 0.5919 | 0.2469 |

All 26 confirm candidates driven agreed with the tf-only prediction to better
than **0.7%**, and 21 of them landed inside the 0.287-0.481 m window.

**Chosen confirm pose: pan 0, lift 0.9, elbow 0.46, wrist_flex −1.40,
wrist_roll 0, gripper 0.** The beam lands at **x = 0.392 m**, near the middle
of the 0.287-0.481 m grasp window, measuring **0.2716 m** against a tf-only
prediction of 0.2715 m.

Three cross-checks, the same three Task 5 used:

1. Measured vs tf-only predicted: **+0.02%**. Repeated in a second session:
   predicted 0.2693, measured 0.2690.
2. Rays with `ray_z > 0` return exactly **4.000 m** (four poses, table above).
3. The camera frame at this pose is open ground; the arm's shadow occupies the
   top-left ~15% and no casting appears. `runs/pod_sweep/C_a_L0.9.png`.

One thing this pose is not: it does not hold the gripper over the target. The
gripper sits at x = 0.572 while the beam lands at x = 0.392, so the arm is
reaching past what it is looking at. That is forced — the pod is 87 deg off the
gripper, so seeing the grasp zone and holding the jaws above it are mutually
exclusive on this arm. `CONFIRMING` can range and image the target from here,
but the approach to grasp is a separate motion, which is consistent with §5:
the grasp is open-loop after a camera-aimed approach.

### Search pose — re-measured

| pose (lift / elbow / wrist) | act | ray | pod height | predicted | **measured ToF** | err | ground hit x | grip x |
|---|---|---|---|---|---|---|---|---|
| −1.4 / 0.0 / 0.4 | −1.476 / 0.001 / 0.413 | 0.873, 0, −0.487 | 0.5450 | 1.1182 | **1.1382** | +1.79% | 1.168 | **0.2670** |
| −1.2 / 0.0 / 0.2 | −1.237 / 0.024 / 0.214 | 0.841, 0, −0.541 | 0.5627 | 1.0404 | 1.0539 | +1.29% | 1.113 | 0.3222 |

**Chosen search pose: pan 0, lift −1.4, elbow 0.0, wrist_flex 0.4, wrist_roll 0,
gripper 0** — unchanged from Task 5, and it survives because the pod's 87 deg
offset swings it forward and down when the arm is raised. 1.138 m of ground,
inside the 0.8-1.5 m the plan asked for, gripper tucked to x = 0.267 m, tighter
than the stow pose's 0.316 m, so the Nav2 footprint does not widen. The camera
frame shows open ground with two of the world's litter cans in it and the arm
confined to the left ~12%: `runs/pod_sweep/search_proven.png`.

Task 5 recorded 1.080 m against a prediction of 1.062 m for this pose; this run
got 1.138 against 1.118. The difference is droop — Task 5's session settled at a
slightly different achieved `shoulder_lift` — and both runs are internally
consistent to within 2%, which is what the check is for. The pose is unchanged;
the range it returns is repeatable only to about 5%.

### The stow pose still reads the robot

| pose | act | ray | predicted | measured ToF |
|---|---|---|---|---|
| lift −1.20, elbow 1.55, wrist 1.10 | −1.193 / 1.611 / 1.100 | −0.999, 0, −0.053 | 7.374 | **0.070 m** |

The ray points backward over the chassis and the predicted ground intersection
is 7 m behind the robot. 0.070 m is the arm's own structure. Unchanged from
Task 5, and the reason nothing should read `/tof/pod` at stow.

---

## 4. Camera and ToF, other measured facts

- **Camera and ToF are boresighted but 15 mm apart**, along the wrist's +Z,
  i.e. perpendicular to the shared viewing axis. §4 of the design spec warns
  against assuming the offset is zero; it is not, and it is lateral, so at
  0.08 m the two disagree by about 10.6°.
- **The pod image is rotated 90° with respect to the world.** The optical
  frame itself is correct — optical +Z is the view axis, +X is image right,
  +Y is image down, all confirmed against `arm_wrist_link` — but the wrist
  link's +Z (image up) lies along the base's ±Y, so the horizon runs
  vertically in every frame. Pixel-to-ray projection through
  `pod_camera_optical_frame` is unaffected; anything that assumes "up in the
  image is up in the world" is not.
- **Ranges run 5–9 mm long** against the ground model. Solving the ground
  model backwards from the measured ranges puts `base_link` at 0.065–0.069 m
  above ground rather than the nominal 0.060 m. The ToF's configured noise is
  zero-mean with 0.005 m stddev, so noise alone does not account for it. Left
  unexplained rather than absorbed into the figures.

---

## 5. The arm does not hold its commanded pose

`/joint_states` disagreed with the commanded position at most poses. The error
tracks how much torque the pose demands: worst was **0.20 rad** at
lift 0.8 / elbow −1.2 (arm straight out, maximum moment); the deepest grasp
pose, lift 1.74 / elbow 0 / wrist 0, converged to within 0.010 rad.

Consequences:

- Every x and z in §1 is measured tf, so it is where the gripper **actually
  went** for that command, droop included. It is not FK of the commanded pose.
- The droop is repeatable: the pose `lift 1.74 / elbow 0 / wrist 0` was run in
  two separate sessions and returned x = 0.3378 / 0.3385 and z = 0.0401 /
  0.0399 — under 1 mm apart.
- A closed-form IK (Task 9) that assumes commanded == achieved will be off by
  up to 0.2 rad in the extended poses. This is why the grasp is open-loop
  after a camera-aimed approach rather than a coordinate solve.

Whether this is a real torque limit or the Gazebo position controller being
starved under llvmpipe is not established here. `docs/STATUS.md` known issue #2
records a similar suspicion for Nav2.

---

## 6. Agreement with `docs/STATUS.md`

| STATUS.md | This measurement | Verdict |
|---|---|---|
| Arm reach below its mount: 219 mm | 220 mm at lift 1.74 / elbow 0 / wrist 0; **225 mm** at lift 1.74 / elbow −0.2 / wrist 0 | Confirmed, and slightly conservative — the extra 5 mm needs a small negative elbow |
| Gripper height at full extension: 41 mm | **40.1 mm** at lift 1.74 / elbow 0 / wrist 0 | Confirmed |

Both figures reproduce, which also means the sensor pod added in Task 1 did
not disturb the arm's kinematics.

`docs/STATUS.md` records nothing about horizontal extent, which is what §12 of
the design spec said. §2 above now fills it.

---

## 7. Mount height: 260 mm as built, 224 mm as a candidate

Added 2026-08-01. **The URDF default is still 0.260 and nothing here has been
adopted.** `arm_mount_height` is now a xacro arg, so both columns come from one
model:

```bash
bash scripts/dev-native.sh python scripts/measure_arm_workspace.py            # both
bash scripts/dev-native.sh python scripts/measure_arm_workspace.py --mount 0.224
```

### Why 224 mm

The reach floor is **224.4 mm below the mount**, wherever the mount is. At
260 mm the gripper therefore bottoms out 35.6 mm above the ground and *cannot
touch it at all* — so flat litter is not merely hard to pick up, it is outside
the workspace. `litter_wrapper` in the test field is 12 mm tall.

What actually governs tolerance is **depth below the mount, not mount height**.
Radial tolerance collapses as the target approaches the reach floor, so the two
numbers trade against each other directly:

| Grasp height | Band at 260 mm | Band at 224 mm |
|---|---|---|
| 0 mm (ground) | out of reach | 32 mm |
| **12 mm** (the wrapper) | **out of reach** | **115 mm** |
| 20 mm | out of reach | 144 mm |
| 35.4 mm | out of reach | 184 mm |
| 50 mm (a can) | 123 mm | 211 mm |
| 70 mm | 181 mm | 237 mm |

`shoulder_lift` margin to its +1.745 limit, at band centre, goes from
**+0.136 rad** (260 mm, grasping at 50 mm) to **+0.448**. At 224 mm the arm
stops working at the end of its travel everywhere it can grasp, which is the
condition §5 above is really describing.

Raising the grasp height in software is *geometrically identical* to lowering
the mount by the same amount — but it is capped by the litter, and it cannot
help the 12 mm wrapper at all. That is the whole argument for the bracket.

### What does not break

- **Ultrasonic band.** 24.0 mm of clearance at 224 mm (60.0 mm at 260 mm). The
  arm base extends 2.4 mm below its mount point and 69.6 mm above it, so it does
  not reach down into the band. Below ~215 mm this stops being true.
- **Patrol.** The coordinator parks at `search_pose`, whose tip is 651 mm up at
  224 mm — nowhere near the sensors. (`stow_pose` does sit in front of the band,
  but that is already true at 260 mm and the coordinator never commands it.)
- **Ground clearance and wheels.** Unaffected; the arm works ahead of the front
  face and above the 120 mm wheel tops.

### Section 5, and the measurements that change

**These are ideal commanded kinematics.** The figures below are FK, not driven
poses, so they do not include the droop of §5 — and that difference is not
negligible at the two calibrated poses. Both discrepancies with the recorded
sim figures run the way droop predicts, which is the cross-check:

| | Recorded in sim | FK here (260 mm) | Direction |
|---|---|---|---|
| `search_pose` ToF range | 1.08–1.14 m | 1.021 m | Droop tilts the arm back; beam lands further out |
| `confirm_pose` ToF range | 0.2716 m | 0.284 m | Droop reaches further down; beam lands nearer |

So **re-measure in sim before adopting any of this** — do not paste the FK
numbers into `coordinator.yaml`.

Every figure the coordinator is calibrated on, and what it becomes:

| Where | Value | At 260 mm | At 224 mm (FK) |
|---|---|---|---|
| `coordinator.yaml` `grasp_min` / `grasp_max` | camera range window | 0.199 / 0.333 | re-measure; FK band is 0.237–0.253 at a 12 mm grasp |
| `coordinator.yaml` `grasp_height` | grasp plane | 0.050 | 0.012–0.020 becomes reachable |
| `coordinator.yaml` `confirm_pose` | pose | `[0, 0.9, 0.46, −1.40, 0, 0]` | re-run `scripts/pod_aim_search.py` — the beam intercept moves 1.5 mm but the pose's *height* above the target changes by 36 mm |
| `coordinator.yaml` `search_pose` | pose | `[0, −1.4, 0, 0.4, 0, 0]` | ToF ground intercept 1.064 m → 1.008 m |
| `coordinator_node.py` `approach_bbox_height_min` | 0.22 | tuned at 260 mm | the pod rides 36 mm lower, so apparent size at a given range changes |
| §1, §3 of this document | driven tf | as recorded | all of it |

### Putting it back

`arm_mount_height` defaults to **0.260**, so reverting is deleting an override,
not editing geometry:

1. Drop any `arm_mount_height:=0.224` from launch files and command lines. The
   default restores the built configuration; no URDF edit is involved.
2. `coordinator.yaml` is the only other file that would carry adopted values.
   Restore `grasp_height: 0.050`, `grasp_min: 0.199`, `grasp_max: 0.333` and
   both poses from the table above.
3. Re-run `bash scripts/dev-native.sh python scripts/verify_pod_orientation.py`
   and the 67 unit tests. Neither depends on mount height, so both passing after
   a revert confirms nothing else drifted.

Nothing in §1–§6 was re-measured for 224 mm. Those sections remain the record of
the **260 mm** build, and are not superseded by this one.

---

## Limitations

Simulation only, and under software rendering. Geometry, tf and kinematics are
trustworthy there; nothing in this document is phrased in Hz or in elapsed
time.

Contact between a box-collision gripper and a 66 mm can is unreliable in
Gazebo, so §2 bounds the **reach**, not the grasp success rate. No object was
picked up to produce these numbers.

The ToF figures are simulated returns from a single-ray `gpu_lidar` against
plain Gazebo surfaces. A real VL53-class part against a crumpled wrapper or a
clear bottle will not behave like this (design spec §5, §13).

`shoulder_pan` was held at 0 throughout, so the workspace measured here is the
vertical plane y = 0 only. Lateral reach is unmeasured. `wrist_roll` was held at
0 except in the 11 poses of the occlusion sweep in §3.

The confirm pose in §3 was found by an offline sweep of the commanded joint
space (`scripts/pod_aim_search.py`) rather than by driving every pose. That
sweep is arithmetic on a forward-kinematic model, and it does not model droop.
It is used only to propose candidates; all 26 it proposed were then driven, and
only the driven numbers appear above. It follows that a better confirm pose may
exist between the 0.05 rad grid points it searched.
