# Arm workspace, measured

Measured in Gazebo Harmonic under llvmpipe software rendering, via tf
`base_link` -> `arm_gripper_frame_link`, driving `/arm/<joint>/position`
(`std_msgs/Float64`) and reading `/tof/pod` and `/pod_camera/image_raw`.
Ground is `base_link` z + 0.060.

Every number below came from a run. 210 poses were commanded across five
simulator sessions; the tables are the subset that bounds each figure. Nothing
here is interpolated or hand-solved. Timing figures are deliberately absent —
software rendering makes them meaningless (see the plan's Global Constraints).

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
| lift −1.20, elbow 1.55, wrist 1.10 | 0.3122 | 0.2322 | 0.066 |

0.066 m is the arm's own structure, not ground — see §3.

---

## 2. Derived figures

| Parameter | Value | How it was obtained |
|---|---|---|
| `grasp_range` | **0.397 m** | x of the pose that puts the gripper lowest: lift 1.74 / elbow −0.2 / wrist 0.0, z = 0.0354 m |
| `grasp_min` | **0.287 m** | nearest x still under 0.060 m: lift 1.74 / elbow 0.2 / wrist 0.0, z = 0.0539 m. The next step in (elbow 0.5, wrist −0.4) reached x = 0.280 but z = 0.0605, over the line |
| `grasp_max` | **0.481 m** | furthest x still under 0.060 m: lift 1.74 / elbow −0.7 / wrist 0.3, z = 0.0553 m. The next step out (elbow −0.8, wrist 0.4) reached x = 0.491 but z = 0.0622 |
| Search pose | **pan 0, lift −1.4, elbow 0.0, wrist_flex 0.4, wrist_roll 0, gripper 0** | ToF read 1.080 m (13 samples, 1.072–1.091) against a predicted ground intersection of 1.062 m at x = 1.107 m ahead of `base_link`. See §3 for why this is ground and not the robot |
| `wrist_roll` limit | **not measured** | It is a harness-routing decision and there is no harness yet, so simulation cannot produce it. The URDF hard limit is −2.74385…+2.84121 rad. The plan's provisional ±1.0 rad is still a guess and is not endorsed here. One thing *is* measured: the pod hangs off `arm_wrist_link`, which is upstream of `wrist_roll`, so `pod_tof_link` and `pod_camera_optical_frame` do not move when `wrist_roll` turns |

Lowest point reached anywhere: **35.4 mm** above ground, i.e. **224.6 mm**
below the arm mount (mount is 0.260 m above ground).

Usable ground-grasp span: **0.287 m to 0.481 m** from `base_link`, a 194 mm
window. The chassis front face is at x = 0.215 m, so the window starts 72 mm
in front of the robot.

---

## 3. The sensor pod points the wrong way

This is the most important thing the sweep found, and it is a design fault,
not a measurement artefact.

**Measured static geometry**, from tf with `arm_wrist_link` as the parent
(printed once per session, identical every time):

| child | xyz in `arm_wrist_link` (m) | distance (m) | unit vector |
|---|---|---|---|
| `arm_gripper_frame_link` | 0.00790, −0.15923, 0.01832 | 0.16047 | 0.0492, −0.9922, 0.1142 |
| `pod_tof_link` | 0.01000, 0.00000, 0.01500 | 0.01803 | — |
| `pod_camera_optical_frame` | 0.01000, 0.00000, 0.03000 | 0.03162 | — |

The gpu_lidar fires along its link's **+X**, and `pod_camera_optical_frame`'s
+Z (the view axis) was measured to be the same direction in `base_link` at
every one of the 210 poses — the two agree to 4 decimal places, so camera and
ToF are boresighted.

The gripper, meanwhile, sits along the wrist's **−Y**. The angle between the
pod's axis and the direction of the gripper is **87.2°**. The pod looks
sideways relative to the thing the arm is about to grab.

### What that costs

At **every** pose that puts the gripper near the ground, the pod axis points
*backward* over the robot (base-frame ray x ≈ −0.98) and the ToF reads the
robot's own structure:

| pose (lift / elbow / wrist) | grip z above ground | ToF ray in `base_link` | ToF mean |
|---|---|---|---|
| 1.74 / 0.0 / 0.0 | 0.0399 | −0.986, 0.000, 0.166 | 0.150 m |
| 1.74 / −0.2 / 0.0 | 0.0354 | −1.000, 0.000, −0.033 | 0.176 m |
| 1.74 / −0.7 / 0.3 | 0.0553 | −0.982, 0.000, −0.188 | 0.238 m |
| 1.74 / 0.3 / −0.2 | 0.0493 | −0.980, 0.000, 0.202 | 0.119 m |
| 1.74 / 0.9 / −0.8 | 0.0955 | −0.984, 0.000, 0.179 | 0.054 m |
| stow | 0.2322 | −0.999, 0.000, −0.038 | 0.066 m |

Every one is under 0.30 m, and none is ground: the predicted ground
intersection for those rays is behind the robot or non-existent. The camera
frames saved at the stow pose and at lift 1.74 / elbow 0 / wrist 0 show the
arm's own castings filling the whole image and about 40% of it respectively. This is exactly the failure the plan warned about — a CONFIRMING
state that trusted `/tof/pod` here would confirm a grasp on the robot's own
hand at 0.15 m, every time, regardless of what is in the jaws.

### But the search pose is fine

The plan's blockquote allowed for the possibility that *every* candidate is
occluded. It is not. Because the pod is 87° off the gripper, raising the arm
(negative `shoulder_lift`) swings the pod forward and down onto open ground:

| pose (lift / elbow / wrist) | ToF ray | pod height | predicted ground range | measured ToF | ground hit x |
|---|---|---|---|---|---|
| −1.4 / 0.0 / 0.4 | 0.859, 0.000, −0.512 | 0.5439 | 1.0625 | **1.0800** | 1.107 |
| −1.4 / 0.0 / 0.5 | 0.797, 0.000, −0.604 | 0.5437 | 0.8999 | 0.9077 | 0.912 |
| −1.2 / 0.0 / 0.2 | 0.832, 0.000, −0.556 | 0.5615 | 1.0109 | 1.0234 | 1.080 |
| −1.4 / 0.4 / 0.0 | 0.814, 0.000, −0.581 | 0.5281 | 0.9085 | 0.9201 | 0.997 |
| −1.0 / 0.0 / 0.0 | 0.793, 0.000, −0.609 | 0.5660 | 0.9288 | 0.9375 | 1.025 |
| −1.4 / −0.4 / 0.8 | 0.914, 0.000, −0.405 | 0.5232 | 1.2912 | 1.3166 | 1.304 |
| −1.4 / 0.4 / −0.3 | 0.957, 0.000, −0.292 | 0.5307 | 1.8201 | 1.8689 | 1.996 |

"Predicted ground range" is `pod_height / −ray_z`, computed from tf alone with
no reference to the ToF. Measured and predicted agree to **within 2%** in
every row, which is the evidence that the beam terminates on the ground plane
and not on the robot. Two independent cross-checks back it: rays with
`ray_z > 0` returned exactly 4.000 m (the sensor's `<max>`, i.e. sky), and the
camera frame saved at the chosen pose shows open ground with two of the
world's litter cans in it and the arm confined to the left ~15% of the image.

**Chosen: pan 0, lift −1.4, elbow 0.0, wrist_flex 0.4** — 1.080 m of ground,
inside the 0.8–1.5 m the plan asked for, and it tucks the gripper to
x = 0.274 m, *tighter* than the current stow pose's 0.312 m, so it does not
widen the Nav2 footprint. `lift −1.0, elbow 0.0, wrist 0.0` is the fallback:
0.9375 m, a completely unobstructed camera frame, but the gripper sits at
x = 0.384 m, 69 mm further out than stow.

### Recommendation

Give the pod joints a mounting rotation so the axis follows the gripper. From
the measured approach unit (0.0492, −0.9922, 0.1142) in `arm_wrist_link`,
solving `Rz(yaw)·Ry(pitch)·x̂ = u`:

```
pitch = −asin(0.1142)                  = −0.1145 rad
yaw   = atan2(−0.9922, 0.0492)/cos(pitch) → −1.5213 rad
```

giving `rpy="0 -0.1145 -1.5213"` on `pod_camera_joint` and `pod_tof_joint`.
That is arithmetic on measured vectors, not a measurement — it needs its own
sweep to confirm, and it will move the search pose, because the whole
lift/elbow/wrist mapping in §1 rotates with it. **Do not apply it and keep the
search pose above.**

Whichever way it is resolved, the coordinator must not read `/tof/pod` during
CONFIRMING with the pod as currently mounted.

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

`wrist_roll` and `shoulder_pan` were held at 0 throughout, so the workspace
measured here is the vertical plane y = 0 only. Lateral reach is unmeasured.
