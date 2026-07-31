# Behaviour Coordinator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a mission coordinator that patrols waypoints, interrupts on a
litter detection, drives the base until the litter is inside the arm's
workspace, then aims and grasps it with an eye-in-hand camera + ToF sensor.

**Architecture:** A hand-rolled, tick-driven finite state machine in a single
`rclpy` node. All decision logic, geometry and kinematics live in pure-Python
modules with no ROS imports, so they unit-test without spinning a node. The node
is a thin shell that wires topics and tf to those modules. Detections arrive
through an adapter so today's JSON-in-`String` interface can be swapped for a
typed message by changing one parameter.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8), Python 3.12, `rclpy`,
`numpy`, `urdf_parser_py`, `opencv-python`, `pytest`, colcon workspace rooted at
the repo root.

**Design spec:** `docs/superpowers/specs/2026-07-30-behaviour-coordinator-design.md`
Read it before starting. This plan implements it and does not restate its
reasoning.

## Global Constraints

- **Branch:** all work lands on `feature/behaviour-coordinator`.
- **EVERY command runs through `scripts/dev.sh`.** The development machine is
  an Apple Silicon Mac with no native ROS 2 or Gazebo; everything executes in
  the `drivebase-dev:jazzy-pod` container with the repo mounted at `/ws`.
  Where a step below writes `colcon build ...`, run
  `./scripts/dev.sh colcon build ...`. Where it writes `python3 -m pytest ...`,
  run `./scripts/dev.sh python3 -m pytest ...`. A bare `ros2` or `colcon` on
  the host will fail with "command not found".
- **Software rendering: trust geometry, never trust timing.** Docker on macOS
  has no GPU passthrough, so Gazebo's camera and `gpu_lidar` fall back to
  llvmpipe. They do work — verified, images arrive — but well under real time.
  Results about frame layout, tf, kinematics, reach, detection and state
  transitions are valid. Any figure phrased in Hz, or "how long did it take",
  is **not**, and must be re-checked on a real GPU. `docs/STATUS.md` already
  records a Nav2 stall that may be nothing more than a starved controller.
- **Long sim launches:** allow ~20-25 s for Gazebo to come up. (Measured in
  Task 2: pod topics were listable in under 10 s even under llvmpipe.)
- **Never verify a topic with `ros2 topic echo --once`.** Measured in Task 3:
  each CLI invocation is a fresh node, and it intermittently fails to discover
  a low-rate publisher before giving up — reporting
  `does not appear to be published yet` for topics that are demonstrably
  publishing. It reported a *working* bridge as broken. To check that data
  flows, use one long-lived `rclpy` node that subscribes to everything and
  spins for ~25 s. Every "verify the topic publishes" step below means that,
  not `echo --once`.
- **One container per `scripts/dev.sh` call.** Background processes do not
  survive between calls, so anything that launches the simulator and then
  queries it must be a single
  `./scripts/dev.sh bash -c '... & sleep 25; ...; kill %1'` invocation.
- **Always `colcon build --symlink-install`, never a bare `colcon build`.**
  Mixing the two corrupts the build tree: `ament_cmake_python` leaves a real
  directory where the symlink build then wants a symlink, and the failure is
  not confined to the package you rebuilt — it aborted `drivebase_sim` too.
  Recover with `rm -rf build/<pkg> install/<pkg>` and rebuild.
- **ROS 2 Jazzy / Gazebo Harmonic.** No package that is not in the Jazzy
  binary index.
- **No new heavy runtime dependencies.** The Pi 5 shares cores with YOLO. No
  MoveIt, no py_trees, no SMACH.
- **`src/trash_vision/` is out of scope. Do not modify any file under it.**
  It belongs to another author. The proposed changes live in §9 of the spec.
- **Pure logic must not import `rclpy`.** Modules under
  `drivebase_behaviour/` named `detection.py`, `localisation.py`,
  `kinematics.py`, `ik.py` and `state_machine.py` import only the standard
  library and `numpy`. This is what makes them testable. A test that needs
  `rclpy.init()` is a sign the boundary was crossed.
- **Comments explain WHY, not WHAT.** Match the existing style in
  `drivebase_sim/drivebase_sim/laserscan_to_range.py` — heavy on rationale,
  silent on the obvious. Do not add docstrings that restate a function name.
- **Never hardcode a figure that can be derived.** Arm link lengths come from
  the URDF at runtime. Measured values go in a params YAML, not in source.
- **Python style:** 4-space indent, matching the existing sources.
  **Do not add `test_flake8.py` / `test_pep257.py` stubs.** `drivebase_sim`
  declares `ament_flake8` and `ament_pep257` as test deps without any lint test
  files, and that declared-but-unused pattern is this repo's convention for its
  own packages — only `trash_vision` (another author's, from `ros2 pkg create`
  defaults) has them. Adding the stubs would fail the build immediately on
  ament's single-quote preference and on `D103` for every test function, and
  `D103` cannot be satisfied without violating the "no docstrings that restate
  a function name" rule above. Match the repo; do not chase the linter.
- **Frames:** `pod_camera_optical_frame` follows the ROS optical convention
  (z forward, x right, y down), matching the existing `camera_optical_frame`.

## Measured constants already established

Do not re-derive these; they are confirmed from the URDF.

| Quantity | Value | Source |
|---|---|---|
| `base_link` above ground | 0.060 m | `base_footprint_to_base_link` = `wheel_radius` |
| `front_face_x` | 0.215 m | `chassis_length / 2`, `chassis_length = 0.430` |
| Arm mount above ground | 0.260 m | `arm_mount_z` 0.200 + 0.060 |
| Arm chain | `arm_base_link` →(`shoulder_pan`)→ `arm_shoulder_link` →(`shoulder_lift`)→ `arm_upper_arm_link` →(`elbow_flex`)→ `arm_lower_arm_link` →(`wrist_flex`)→ `arm_wrist_link` →(`wrist_roll`)→ `arm_gripper_link` | `so101_macro.xacro` |
| Joint limits (rad) | `shoulder_pan` ±1.91986, `shoulder_lift` ±1.74533, `elbow_flex` ±1.69, `wrist_flex` ±1.65806, `wrist_roll` −2.74385…+2.84121, `gripper` −0.174533…+1.74533 | `so101_macro.xacro` |
| Stow pose | `shoulder_pan` 0.0, `shoulder_lift` −1.20, `elbow_flex` 1.55, `wrist_flex` 1.10, `wrist_roll` 0.0, `gripper` 0.0 | `drivebase.urdf.xacro:49-54` |

## File Structure

```
src/drivebase_msgs/                       NEW (ament_cmake)
  CMakeLists.txt
  package.xml
  msg/LitterDetection.msg                 proposed typed interface

src/drivebase_behaviour/                  NEW (ament_python)
  package.xml
  setup.py
  resource/drivebase_behaviour
  drivebase_behaviour/
    __init__.py
    detection.py          Detection dataclass + JSON parsing        PURE
    localisation.py       pixel -> ray -> 3D point                  PURE
    kinematics.py         URDF chain load, FK, planar extraction    PURE
    ik.py                 closed-form planar IK                     PURE
    state_machine.py      FSM transition table                      PURE
    detection_source.py   ROS adapters (String / typed)
    arm_driver.py         publishes the six joint positions
    coordinator_node.py   the node: tf, Nav2 client, servo, ticks
  config/
    coordinator.yaml      params, incl. measured figures
  launch/
    coordinator.launch.py
  test/
    test_detection.py  test_localisation.py  test_kinematics.py
    test_ik.py  test_state_machine.py

src/drivebase_description/urdf/
  drivebase.urdf.xacro    MODIFY  sensor pod frames, search pose
  gazebo.xacro            MODIFY  camera + ToF sensors

src/drivebase_sim/
  config/bridge.yaml      MODIFY  image, camera_info, ToF, 6 arm joints
  worlds/test_field.sdf   MODIFY  litter objects
  launch/sim.launch.py    MODIFY  ToF Range shim instance
  drivebase_sim/
    laserscan_to_range.py MODIFY  radiation_type parameter
    sim_litter_detector.py NEW    colour-blob shim
  setup.py                MODIFY  new entry point

docs/
  STATUS.md               MODIFY  state table, measured figures
  arm_workspace.md        NEW     Phase 1 measurements
```

---

# Phase 1 — Simulation plumbing

Nothing in later phases can be verified until the simulator has a camera, a
ToF, a commandable arm and something to pick up.

---

### Task 1: Sensor pod frames

**Files:**
- Modify: `src/drivebase_description/urdf/drivebase.urdf.xacro` — **inside** the
  `<xacro:if value="${use_arm}">` block, immediately after the closing
  `</xacro:so101>` and **before** the block's `</xacro:if>`.

> Not after the closing `</xacro:if>`. Placing it there compiles, but emits
> `pod_camera_link` unconditionally with `arm_wrist_link` as its parent — a
> link that does not exist when `use_arm:=false` — producing a broken tree.
> Step 3 below is what catches this.

**Interfaces:**
- Consumes: nothing
- Produces: tf frames `pod_camera_link`, `pod_camera_optical_frame`,
  `pod_tof_link`, all fixed children of `arm_wrist_link`. Xacro properties
  `pod_camera_xyz`, `pod_camera_rpy`, `pod_tof_xyz`.

- [ ] **Step 1: Add the pod frames**

Insert inside the `<xacro:if value="${use_arm}">` block that mounts the arm,
after the `<xacro:so101 .../>` call. The snippet below is written at 2-space
indent; re-indent it to match the 4-space indent the block's existing children
use.

```xml
  <!-- Sensor pod: camera + ToF, eye-in-hand.

       Mounted on arm_wrist_link, which is BEFORE wrist_roll in the chain. The
       camera therefore never rolls, so the image up-vector is fixed and the
       harness crosses four moving joints rather than five. See
       docs/superpowers/specs/2026-07-30-behaviour-coordinator-design.md §4.

       CO-LOCATION IS LOAD-BEARING. A 20 mm camera-to-ToF separation at 80 mm
       range is a large angular disagreement - the ToF can be reading past the
       target while the camera reports it centred. These offsets are a
       placeholder until the physical pod exists; they must be remeasured, and
       must never be assumed zero. -->
  <xacro:property name="pod_camera_xyz" value="0.010 0.0 0.030"/>
  <xacro:property name="pod_camera_rpy" value="0 0 0"/>
  <xacro:property name="pod_tof_xyz"    value="0.010 0.0 0.015"/>

  <link name="pod_camera_link">
    <xacro:sensor_inertia/>
  </link>
  <joint name="pod_camera_joint" type="fixed">
    <parent link="arm_wrist_link"/>
    <child  link="pod_camera_link"/>
    <origin xyz="${pod_camera_xyz}" rpy="${pod_camera_rpy}"/>
  </joint>

  <!-- ROS optical convention: z forward, x right, y down. Image pipelines and
       the ray projection in drivebase_behaviour/localisation.py both assume
       it. Same rotation the chassis camera_optical_joint uses. -->
  <link name="pod_camera_optical_frame"/>
  <joint name="pod_camera_optical_joint" type="fixed">
    <parent link="pod_camera_link"/>
    <child  link="pod_camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

  <link name="pod_tof_link">
    <xacro:sensor_inertia/>
  </link>
  <joint name="pod_tof_joint" type="fixed">
    <parent link="arm_wrist_link"/>
    <child  link="pod_tof_link"/>
    <origin xyz="${pod_tof_xyz}" rpy="0 0 0"/>
  </joint>
```

- [ ] **Step 2: Verify the xacro expands and the URDF is valid**

```bash
cd "$(git rev-parse --show-toplevel)"
xacro src/drivebase_description/urdf/drivebase.urdf.xacro use_sim:=true \
  use_arm:=true wheel_mu1:=1.0 wheel_mu2:=0.6 > /tmp/pod.urdf
check_urdf /tmp/pod.urdf | head -40
grep -c "pod_camera_optical_frame" /tmp/pod.urdf
```

Expected: `check_urdf` reports a valid tree with no loops; grep returns `2`
(the link and the joint's child reference).

- [ ] **Step 3: Verify the pod is parented to the wrist, not the base**

```bash
python3 - <<'PY'
from urdf_parser_py.urdf import URDF
r = URDF.from_xml_file('/tmp/pod.urdf')
for j in r.joints:
    if j.child in ('pod_camera_link', 'pod_tof_link'):
        print(j.name, '->', j.parent)
        assert j.parent == 'arm_wrist_link', j.parent
print('OK')
PY
```

Expected: both print `-> arm_wrist_link`, then `OK`.

- [ ] **Step 4: Commit**

```bash
git add src/drivebase_description/urdf/drivebase.urdf.xacro
git commit -m "Add eye-in-hand sensor pod frames on arm_wrist_link"
```

---

### Task 2: Camera and ToF sensors in Gazebo

**Files:**
- Modify: `src/drivebase_description/urdf/gazebo.xacro` — **inside** the
  `<xacro:if value="${use_arm}">` block, not before `</robot>`. The pod only
  exists when the arm does; declaring these unconditionally references links
  that are absent under `use_arm:=false`. Step 3 catches it.

**Interfaces:**
- Consumes: frames from Task 1
- Produces: gz topics `/pod_camera/image`, `/pod_camera/camera_info`,
  `/tof/pod/scan`

- [ ] **Step 1: Add both sensors**

Append inside the `<xacro:if value="${use_arm}">` block (the pod only exists
when the arm does):

```xml
  <!-- ==================================================================
       SENSOR POD
       Eye-in-hand camera and ToF. Both need gz-sim-sensors-system, which is a
       WORLD plugin and already declared in drivebase_sim's test_field.sdf - a
       sensor with no matching world system stays silent with no error.
       ================================================================== -->
  <gazebo reference="pod_camera_link">
    <sensor name="pod_camera" type="camera">
      <always_on>1</always_on>
      <!-- 15 Hz, not 30. The detector is the bottleneck on hardware and a
           faster camera would only queue frames the Pi cannot process. -->
      <update_rate>15</update_rate>
      <topic>/pod_camera/image</topic>
      <gz_frame_id>pod_camera_optical_frame</gz_frame_id>
      <camera>
        <!-- 1.15 rad ~ 66 deg horizontal. Matches a Pi Camera v3 class part;
             keep this in step with whatever is actually bought, so the
             intrinsics the coordinator reads from camera_info transfer. -->
        <horizontal_fov>1.15</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>10.0</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>

  <!-- ToF simulated exactly as the ultrasonics are: Gazebo Harmonic has no
       time-of-flight sensor type, so it is a gpu_lidar with a single ray.
       laserscan_to_range collapses it to sensor_msgs/Range, which is what a
       real VL53-class driver publishes directly.

       ONE sample, not five: a ToF returns a single distance along its axis,
       unlike a sonar which reports the nearest echo anywhere in a cone. -->
  <gazebo reference="pod_tof_link">
    <sensor name="pod_tof" type="gpu_lidar">
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>${show_sensor_rays}</visualize>
      <topic>/tof/pod/scan</topic>
      <gz_frame_id>pod_tof_link</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>0.0</max_angle>
          </horizontal>
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>0.0</max_angle>
          </vertical>
        </scan>
        <!-- VL53L1X-class limits. min 0.04 m is why the grasp descent must be
             open-loop: at grab range the target is inside this. -->
        <range>
          <min>0.04</min>
          <max>4.0</max>
          <resolution>0.001</resolution>
        </range>
        <noise type="gaussian"><mean>0.0</mean><stddev>0.005</stddev></noise>
      </lidar>
    </sensor>
  </gazebo>
```

- [ ] **Step 2: Verify the xacro still expands**

```bash
cd "$(git rev-parse --show-toplevel)"
xacro src/drivebase_description/urdf/drivebase.urdf.xacro use_sim:=true \
  use_arm:=true wheel_mu1:=1.0 wheel_mu2:=0.6 > /tmp/pod.urdf && echo EXPANDS
grep -c "pod_camera\|pod_tof" /tmp/pod.urdf
```

Expected: `EXPANDS`, grep count > 0.

- [ ] **Step 3: Verify `arm:=false` still expands (the pod is arm-gated)**

```bash
xacro src/drivebase_description/urdf/drivebase.urdf.xacro use_sim:=true \
  use_arm:=false wheel_mu1:=1.0 wheel_mu2:=0.6 > /tmp/noarm.urdf && echo EXPANDS
grep -c "pod_camera" /tmp/noarm.urdf || echo "0 (correct)"
```

Expected: `EXPANDS`, and no `pod_camera` — the pod must not appear without the arm.

- [ ] **Step 4: Verify the sensors publish in Gazebo**

```bash
colcon build --symlink-install --packages-select drivebase_description drivebase_sim
source install/setup.bash
ros2 launch drivebase_sim sim.launch.py headless:=true &
sleep 25
gz topic -l | grep -E "pod_camera|tof"
kill %1
```

Expected: `/pod_camera/image`, `/pod_camera/camera_info`, `/tof/pod/scan` listed.

- [ ] **Step 5: Commit**

```bash
git add src/drivebase_description/urdf/gazebo.xacro
git commit -m "Add eye-in-hand camera and ToF sensors to the sim"
```

---

### Task 3: Bridge the camera, ToF and arm joints

The arm's six `JointPositionController` plugins listen on *gz transport*
topics. Without these entries nothing in ROS can move the arm.

**Files:**
- Modify: `src/drivebase_sim/config/bridge.yaml`
- Modify: `src/drivebase_sim/drivebase_sim/laserscan_to_range.py`
- Modify: `src/drivebase_sim/launch/sim.launch.py`

**Interfaces:**
- Consumes: gz topics from Task 2
- Produces: ROS topics `/pod_camera/image_raw` (`sensor_msgs/Image`),
  `/pod_camera/camera_info` (`sensor_msgs/CameraInfo`), `/tof/pod`
  (`sensor_msgs/Range`), and `/arm/<joint>/position` (`std_msgs/Float64`) for
  each of `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`,
  `wrist_roll`, `gripper`.

- [ ] **Step 1: Add the bridge entries**

Append to `src/drivebase_sim/config/bridge.yaml`:

```yaml
# Eye-in-hand camera. camera_info carries the intrinsics that
# drivebase_behaviour/localisation.py needs to turn a pixel into a ray - the
# coordinator must never hardcode fx/fy/cx/cy.
- ros_topic_name: "/pod_camera/image_raw"
  gz_topic_name: "/pod_camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/pod_camera/camera_info"
  gz_topic_name: "/pod_camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS

# ToF cone. Collapsed to sensor_msgs/Range by a second laserscan_to_range
# instance, the same way the ultrasonics are.
- ros_topic_name: "/tof/pod/scan"
  gz_topic_name: "/tof/pod/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

# ARM JOINT COMMANDS.
#
# The six gz-sim-joint-position-controller-system plugins in gazebo.xacro
# subscribe to these as gz transport topics. Until these entries existed the
# arm held its stow pose and NOTHING in ROS could move it - silently, with no
# error on either side, which is this project's recurring failure mode.
#
# std_msgs/Float64 <-> gz.msgs.Double is the supported pair. Float32 is not.
- ros_topic_name: "/arm/shoulder_pan/position"
  gz_topic_name: "/arm/shoulder_pan/position"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/arm/shoulder_lift/position"
  gz_topic_name: "/arm/shoulder_lift/position"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/arm/elbow_flex/position"
  gz_topic_name: "/arm/elbow_flex/position"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/arm/wrist_flex/position"
  gz_topic_name: "/arm/wrist_flex/position"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/arm/wrist_roll/position"
  gz_topic_name: "/arm/wrist_roll/position"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/arm/gripper/position"
  gz_topic_name: "/arm/gripper/position"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
```

There is no `std_msgs/msg/Double` — that name is a natural typo here because
the *gz* side is called `gz.msgs.Double`, and a wrong type produces a topic
that carries nothing with no error on either side. Step 2 guards against it.

- [ ] **Step 2: Verify every declared ROS type exists**

```bash
cd "$(git rev-parse --show-toplevel)"
python3 - <<'PY'
import re, pathlib
text = pathlib.Path('src/drivebase_sim/config/bridge.yaml').read_text()
types = set(re.findall(r'ros_type_name:\s*"([^"]+)"', text))
import importlib
bad = []
for t in sorted(types):
    pkg, _, name = t.split('/')
    try:
        mod = importlib.import_module(f'{pkg}.msg')
        getattr(mod, name)
    except Exception as e:
        bad.append((t, str(e)))
print('BAD:', bad if bad else 'none')
assert not bad, bad
PY
```

Expected: `BAD: none`. If any type is reported, fix it before going further —
a bridge entry naming a type that does not exist fails silently.

- [ ] **Step 3: Add a `radiation_type` parameter to `laserscan_to_range`**

The node currently hardcodes `Range.ULTRASOUND`. A ToF is infrared. In
`src/drivebase_sim/drivebase_sim/laserscan_to_range.py`, after the
`frame_template` parameter declaration add:

```python
        # ULTRASOUND for the HC-SR04s, INFRARED for the pod ToF. The value
        # matters to consumers that filter by sensor type; Nav2's
        # range_sensor_layer does not, but rviz displays them differently.
        self.declare_parameter("radiation_type", "ultrasound")
```

and after the other parameter reads:

```python
        radiation = str(self.get_parameter("radiation_type").value).lower()
        self.radiation_type = (
            Range.INFRARED if radiation == "infrared" else Range.ULTRASOUND
        )
```

then replace the hardcoded line in `on_scan`:

```python
        message.radiation_type = self.radiation_type
```

- [ ] **Step 4: Launch a second shim instance for the ToF**

In `src/drivebase_sim/launch/sim.launch.py`, after the existing
`laserscan_to_range` node, add:

```python
        # Second instance for the pod ToF. Same collapse logic, different
        # topics and radiation type - a ToF reports one distance along its
        # axis, and with <samples>1</samples> the minimum-of-cone reduces to
        # exactly that.
        Node(
            package="drivebase_sim",
            executable="laserscan_to_range",
            name="tof_to_range",
            parameters=[{
                "use_sim_time": True,
                "sensor_names": ["pod"],
                "scan_topic_template": "/tof/{name}/scan",
                "range_topic_template": "/tof/{name}",
                "frame_template": "pod_tof_link",
                "radiation_type": "infrared",
            }],
            output="screen",
        ),
```

- [ ] **Step 5: Verify the topics exist and the arm actually moves**

```bash
colcon build --symlink-install --packages-select drivebase_sim drivebase_description
source install/setup.bash
ros2 launch drivebase_sim sim.launch.py headless:=true &
sleep 25
ros2 topic list | grep -E "pod_camera|tof/pod|arm/"
ros2 topic echo /pod_camera/camera_info --once
ros2 topic echo /tof/pod --once

# Move the arm and confirm joint_states follows.
ros2 topic echo /joint_states --once | grep -A2 shoulder_pan
ros2 topic pub -1 /arm/shoulder_pan/position std_msgs/msg/Float64 "{data: 0.6}"
sleep 3
ros2 topic echo /joint_states --once
kill %1
```

Expected: `camera_info` has non-zero `k` intrinsics; `/tof/pod` publishes a
`Range` with `radiation_type: 1` (INFRARED); `shoulder_pan` in `/joint_states`
moves toward 0.6 rad. **If the joint does not move, stop — every later phase
depends on this.** Check `ros2 topic info /arm/shoulder_pan/position` for a
type mismatch first; that is this project's known silent failure.

- [ ] **Step 6: Commit**

```bash
git add src/drivebase_sim/config/bridge.yaml \
        src/drivebase_sim/drivebase_sim/laserscan_to_range.py \
        src/drivebase_sim/launch/sim.launch.py
git commit -m "Bridge pod camera, ToF and the six arm joint commands"
```

---

### Task 4: Litter objects in the test world

**Files:**
- Modify: `src/drivebase_sim/worlds/test_field.sdf`

**Interfaces:**
- Consumes: nothing
- Produces: three models named `litter_*`, all non-static, on the ground,
  clear of existing obstacles.

- [ ] **Step 1: Add the litter**

Insert before `</world>`:

```xml
    <!-- Litter. Non-static: a successful grasp must be able to lift them, and
         a static model would let the gripper close on an immovable object and
         report success. Placed in x [1.5, 4.0], y [-1.5, 1.5], clear of the
         barriers at x=3,y=0 and x=5,y=2.

         Saturated colours because sim_litter_detector segments by hue - it
         stands in for YOLO, which cannot run in sim (hardware model path, TCP
         stream). Real litter is not this colour and that is fine: the shim
         exists to exercise the loop, not to prove the detector. -->
    <model name="litter_can_a">
      <pose>1.8 0.5 0.035 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.015</mass>
          <inertia><ixx>7e-6</ixx><iyy>7e-6</iyy><izz>4e-6</izz>
                   <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><cylinder><radius>0.033</radius><length>0.07</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.033</radius><length>0.07</length></cylinder></geometry>
          <material><ambient>0.8 0.05 0.05 1</ambient><diffuse>1.0 0.05 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="litter_can_b">
      <pose>3.6 -1.1 0.035 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.015</mass>
          <inertia><ixx>7e-6</ixx><iyy>7e-6</iyy><izz>4e-6</izz>
                   <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><cylinder><radius>0.033</radius><length>0.07</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.033</radius><length>0.07</length></cylinder></geometry>
          <material><ambient>0.8 0.05 0.05 1</ambient><diffuse>1.0 0.05 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="litter_wrapper">
      <pose>2.4 1.3 0.012 0 0 0.4</pose>
      <link name="link">
        <inertial>
          <mass>0.005</mass>
          <inertia><ixx>2e-6</ixx><iyy>2e-6</iyy><izz>2e-6</izz>
                   <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.08 0.05 0.024</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.08 0.05 0.024</size></box></geometry>
          <material><ambient>0.8 0.05 0.05 1</ambient><diffuse>1.0 0.05 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>
```

- [ ] **Step 2: Verify the SDF parses and the models spawn**

```bash
cd "$(git rev-parse --show-toplevel)"
python3 -c "import xml.etree.ElementTree as ET; ET.parse('src/drivebase_sim/worlds/test_field.sdf'); print('PARSES')"
colcon build --symlink-install --packages-select drivebase_sim && source install/setup.bash
ros2 launch drivebase_sim sim.launch.py headless:=true &
sleep 25
gz model --list | grep litter
kill %1
```

Expected: `PARSES`, then all three `litter_*` models listed.

- [ ] **Step 3: Commit**

```bash
git add src/drivebase_sim/worlds/test_field.sdf
git commit -m "Add litter objects to the test field"
```

---

### Task 5: Measure the arm workspace

This produces the figures §12 of the spec deliberately left open. **Every
number written here must come from a command that was actually run.** Do not
estimate.

**Files:**
- Create: `docs/arm_workspace.md`

**Interfaces:**
- Consumes: Tasks 1-3
- Produces: measured values for `grasp_range`, `grasp_min`, `grasp_max`,
  the search pose, and the `wrist_roll` software limit. Task 15 writes these
  into `config/coordinator.yaml`.

- [ ] **Step 1: Write the sweep helper**

Create `/tmp/sweep_arm.py` (a throwaway, not committed):

```python
"""Command an arm pose, wait for it to settle, report the gripper in base_link."""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tf2_ros

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]


class Sweep(Node):
    def __init__(self, targets):
        super().__init__("sweep_arm")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        pubs = {j: self.create_publisher(Float64, f"/arm/{j}/position", 10)
                for j in JOINTS}
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        rclpy.spin_once(self, timeout_sec=2.0)
        for j, v in zip(JOINTS, targets):
            m = Float64()
            m.data = float(v)
            pubs[j].publish(m)
        end = self.get_clock().now().nanoseconds + 6_000_000_000
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        t = self.buffer.lookup_transform(
            "base_link", "arm_gripper_frame_link", rclpy.time.Time())
        p = t.transform.translation
        print(f"targets={targets} gripper_in_base_link "
              f"x={p.x:.4f} y={p.y:.4f} z={p.z:.4f} "
              f"height_above_ground={p.z + 0.060:.4f}")


def main():
    rclpy.init()
    Sweep([float(a) for a in sys.argv[1:7]])
    rclpy.shutdown()


main()
```

- [ ] **Step 2: Run the sweep**

```bash
cd "$(git rev-parse --show-toplevel)" && source install/setup.bash
ros2 launch drivebase_sim sim.launch.py headless:=true &
sleep 25

# Stow, for reference.
python3 /tmp/sweep_arm.py 0 -1.20 1.55 1.10 0 0
# Extended forward and down - candidate grasp poses.
python3 /tmp/sweep_arm.py 0 -0.30 1.00 1.00 0 0
python3 /tmp/sweep_arm.py 0  0.00 0.80 0.90 0 0
python3 /tmp/sweep_arm.py 0  0.30 0.60 0.80 0 0
python3 /tmp/sweep_arm.py 0  0.60 0.40 0.70 0 0
# Candidate search poses - want the camera looking forward and down.
python3 /tmp/sweep_arm.py 0 -0.60 0.80 0.40 0 0
python3 /tmp/sweep_arm.py 0 -0.40 0.60 0.30 0 0
kill %1
```

- [ ] **Step 3: Find the search pose that sees the ground ahead**

For each search-pose candidate, hold the pose and check what the camera sees:

```bash
ros2 launch drivebase_sim sim.launch.py headless:=true &
sleep 25
python3 /tmp/sweep_arm.py 0 -0.60 0.80 0.40 0 0 &
sleep 8
ros2 run tf2_ros tf2_echo base_link pod_camera_optical_frame
# Range straight ahead along the camera axis:
ros2 topic echo /tof/pod --once
kill %1
```

Choose the pose whose ToF reads ground at roughly 0.8-1.5 m — near enough to
grasp after a short approach, far enough to be worth detecting.

> **The ToF can see the arm's own gripper.** Measured during Task 2: at the
> stow pose the ToF reads **0.062 m**, which is not ground — the pod points
> along the wrist axis and the jaws are in front of it. Any candidate search
> pose returning a range under ~0.3 m is looking at the robot, not the world.
> Reject those rather than recording them, and check the camera image agrees
> before trusting a range. If every candidate is occluded, the pod's mounting
> `rpy` needs a pitch offset — raise that rather than working around it, since
> a ToF that sees the gripper during CONFIRMING would confirm a grasp on the
> robot's own hand.

- [ ] **Step 4: Write down what was measured**

Create `docs/arm_workspace.md`. Fill every value from the runs above; do not
carry a number across that was not printed:

```markdown
# Arm workspace, measured

Measured in Gazebo Harmonic via tf `base_link` -> `arm_gripper_frame_link`,
using `/tmp/sweep_arm.py` from Task 5 of the behaviour coordinator plan. Ground
is `base_link` z + 0.060.

## Gripper position by joint pose

| shoulder_lift | elbow_flex | wrist_flex | x (m) | z above ground (m) |
|---|---|---|---|---|
| ... | ... | ... | ... | ... |

## Derived figures

| Parameter | Value | How it was obtained |
|---|---|---|
| `grasp_range` | | x of the pose that puts the gripper lowest |
| `grasp_min` | | nearest x with the gripper still under 0.06 m |
| `grasp_max` | | furthest x with the gripper still under 0.06 m |
| Search pose | | pose whose ToF reads ground at 0.8-1.5 m |
| `wrist_roll` limit | ±1.0 rad (provisional) | harness routing, not yet built |

## Limitations

Simulation only. Contact between a box-collision gripper and a small object is
unreliable in Gazebo, so these bound the *reach*, not the grasp success rate.
```

- [ ] **Step 5: Commit**

```bash
git add docs/arm_workspace.md
git commit -m "Measure arm workspace and search pose in simulation"
```

---

### Task 5b: Re-aim the sensor pod, and re-measure — **DONE, and it reversed**

> **OUTCOME (commit `b9820a5`): the rotation prescribed below is WRONG. The pod
> stays at `rpy 0 0 0`.**
>
> Aiming the pod along the gripper axis makes the ToF read a constant
> **0.058 m** at every pose — it is looking lengthwise down the gripper body.
> `arm_gripper_link`'s origin is 0.0619 m from the pod along −Y, which is that
> reading almost exactly. The decisive check was the sky test inverted: aimed
> at open sky it returned **0.057 m instead of 4.000 m**. The cause is
> positional, not angular — the pod sits at the *root* of a 160 mm gripper, so
> any aim at the grasp point looks down its length. Moving the mount outboard
> was tried and failed too.
>
> **Task 5's "no arm pose sees the grasp zone" was also wrong.** It drove 210
> poses but never paired strongly negative `wrist_flex` with positive
> `shoulder_lift`. That corner works:
>
> | Pose | pan 0, lift 0.9, elbow 0.46, wrist_flex −1.40 |
> |---|---|
> | ToF measured | **0.2716 m** (predicted from tf alone: 0.2715, +0.02%) |
> | Beam lands at | **x = 0.392 m** — mid grasp window (0.287–0.481) |
>
> Search pose unchanged: lift −1.4, elbow 0.0, wrist_flex 0.4.
>
> **Known limitation:** the confirm pose does not hold the gripper over the
> target (gripper x = 0.572, beam x = 0.392). The 87° offset makes seeing the
> grasp zone and holding the jaws above it mutually exclusive on this arm.
> `CONFIRMING` ranges and images the target; the descent stays open-loop, which
> is what spec §6 already required.
>
> The steps below are kept as the record of what was tried and why it failed.
> Do not re-apply Step 1.

Task 5 found that the pod's optical/ranging axis is **87.2°** away from the
gripper's approach direction. `arm_wrist_link`'s +X does not point at the
gripper — the SO-101 puts the gripper along −Y, at
`(0.00790, −0.15923, 0.01832)`, unit `(0.0492, −0.9922, 0.1142)`. Task 1's
`rpy="0 0 0"` was wrong.

The consequence is bigger than a bad reading. The grasp window is
**0.287–0.481 m** from `base_link`, but the search pose's beam cannot return
less than **1.06 m** on flat ground, and every pose that lowers the gripper
swings the pod backward over the robot. **No arm pose sees the grasp zone**, so
`CONFIRMING` — the design's load-bearing gate — has nothing valid to read.

**Files:**
- Modify: `src/drivebase_description/urdf/drivebase.urdf.xacro` (pod joint `rpy`)
- Modify: `docs/arm_workspace.md` (re-measured tables)

**Interfaces:**
- Consumes: Tasks 1-5
- Produces: corrected `pod_camera_joint` / `pod_tof_joint` orientation; a
  re-measured search pose; a **confirm pose** that views the grasp zone.

- [ ] **Step 1: Apply the measured rotation**

Replace the placeholder `pod_camera_rpy` and add a matching ToF rotation:

```xml
  <!-- Aims the pod along the gripper's approach direction.
       NOT zero, and this is worth explaining: arm_wrist_link's +X does not
       point at the gripper. The SO-101 puts arm_gripper_frame_link at
       (0.00790, -0.15923, 0.01832) in this frame - essentially along -Y, 87.2
       degrees off +X. Mounted at rpy 0 0 0 the pod looks sideways past
       everything the arm is about to grab, and at any pose low enough to
       reach the ground it points backward over the robot and ranges the
       chassis at 0.05-0.24 m.

       Solving Rz(yaw)*Ry(pitch)*x_hat = (0.0492, -0.9922, 0.1142):
         pitch = -asin(0.1142)  = -0.1145
         yaw   = atan2(-0.9922, 0.0492) = -1.5213
       Measured in docs/arm_workspace.md 3. -->
  <xacro:property name="pod_aim_rpy" value="0 -0.1145 -1.5213"/>
```

Use `${pod_aim_rpy}` for the `rpy` of **both** `pod_camera_joint` and
`pod_tof_joint`. They must stay boresighted — Task 5 measured them agreeing to
4 decimal places, and the localisation maths assumes it.

- [ ] **Step 2: Verify the pod now points at the gripper**

```bash
./scripts/dev.sh bash -c '
xacro src/drivebase_description/urdf/drivebase.urdf.xacro use_sim:=true \
  use_arm:=true wheel_mu1:=1.0 wheel_mu2:=0.6 > /tmp/u.urdf 2>/dev/null
python3 - <<PY
import math, numpy as np
from urdf_parser_py.urdf import URDF
r = URDF.from_xml_file("/tmp/u.urdf"); J={j.name:j for j in r.joints}
def rpy(rr,pp,yy):
    cr,sr=math.cos(rr),math.sin(rr); cp,sp=math.cos(pp),math.sin(pp)
    cy,sy=math.cos(yy),math.sin(yy)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]])
def T(j):
    m=np.eye(4); m[:3,:3]=rpy(*j.origin.rpy); m[:3,3]=j.origin.xyz; return m
g=(T(J["arm_wrist_roll"])@T(J["arm_gripper_frame_joint"]))[:3,3]
u=g/np.linalg.norm(g)
for name in ("pod_tof_joint","pod_camera_joint"):
    axis=rpy(*J[name].origin.rpy)@np.array([1.0,0,0])
    print(name, "angle to gripper: %.2f deg" % math.degrees(
        math.acos(float(np.clip(np.dot(axis,u),-1,1)))))
PY'
```

Expected: **both under 1°**. Anything near 87° means the rotation did not take.

- [ ] **Step 3: Re-measure — the §1 sweep is invalidated**

Every pose-to-ToF mapping in `docs/arm_workspace.md` §1 and §3 was taken with
the old orientation and is now wrong. The gripper-position table (tf only) is
still valid — rotating a sensor does not move the arm — but every **ToF**
column must be re-taken.

Re-run the Task 5 sweep and produce **two** poses:

1. **Search pose** — pod on open ground at 0.8–1.5 m, gripper tucked (keep
   x ≤ ~0.31 m so the Nav2 footprint does not widen).
2. **Confirm pose** — pod viewing the **0.287–0.481 m grasp window**. This is
   the pose `CONFIRMING` holds while `shoulder_pan` centres the target. It did
   not exist before because no pose could see the grasp zone.

For each, prove the beam terminates on ground and not on the robot, using the
same three checks Task 5 used and which are the reason its numbers are
trustworthy:
- measured ToF within a few % of `pod_height / −ray_z` computed from tf alone;
- rays with `ray_z > 0` returning exactly 4.000 m (sky);
- the saved camera frame showing ground rather than castings.

Reject any candidate reading under ~0.3 m without corroboration — that is the
robot.

- [ ] **Step 4: Update `docs/arm_workspace.md`**

Replace the invalidated ToF tables. Keep §5 (arm droop), §6 (STATUS.md
agreement) and the camera↔ToF offset finding — none depend on pod orientation.
State plainly which tables were re-measured and which carried over.

- [ ] **Step 5: Commit**

```bash
git add src/drivebase_description/urdf/drivebase.urdf.xacro docs/arm_workspace.md
git commit -m "Aim the sensor pod along the gripper approach axis"
```

---

# Phase 2 — Pure logic

Everything here is testable with `pytest` alone. No simulator, no `rclpy`.

---

### Task 6: `drivebase_msgs` package

**Files:**
- Create: `src/drivebase_msgs/package.xml`
- Create: `src/drivebase_msgs/CMakeLists.txt`
- Create: `src/drivebase_msgs/msg/LitterDetection.msg`

**Interfaces:**
- Produces: `drivebase_msgs/msg/LitterDetection`

- [ ] **Step 1: Create the package files**

`src/drivebase_msgs/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>drivebase_msgs</name>
  <version>0.1.0</version>
  <description>Typed interfaces for the litter-collection robot.</description>
  <maintainer email="bryanhsu0123@gmail.com">Robot Curiosity</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/drivebase_msgs/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(drivebase_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LitterDetection.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

`src/drivebase_msgs/msg/LitterDetection.msg`:

```
# Litter detection from a single camera frame.
#
# PROPOSED replacement for the JSON-in-std_msgs/String payload currently on
# /vision/target. See docs/superpowers/specs/2026-07-30-behaviour-coordinator-design.md §9.
#
# Two things this fixes over the String form:
#   * A header. std_msgs/String carries no timestamp, so a consumer must stamp
#     on arrival, which misattributes latency and lets it act on a stale frame.
#     Stamp this at CAPTURE, not at publish.
#   * horizontal_error is primary. LEFT/RIGHT/CENTER was the right shape for
#     steering a base; an eye-in-hand arm uses the continuous value directly
#     and bucketing into thirds discards precision.

std_msgs/Header header

bool    detected
uint32  frame_number
float32 confidence

# Pixels, image coordinates.
float32 bbox_x1
float32 bbox_y1
float32 bbox_x2
float32 bbox_y2

# Normalised 0..1 across the image.
float32 center_x
float32 center_y
float32 floor_x
float32 floor_y

# PRIMARY. Continuous, -0.5 (hard left) .. +0.5 (hard right).
float32 horizontal_error

# Derived from horizontal_error. Retained for existing consumers.
uint8 DIRECTION_NO_TARGET = 0
uint8 DIRECTION_LEFT      = 1
uint8 DIRECTION_CENTER    = 2
uint8 DIRECTION_RIGHT     = 3
uint8 direction
```

- [ ] **Step 2: Build and verify the message generates**

```bash
cd "$(git rev-parse --show-toplevel)"
colcon build --symlink-install --packages-select drivebase_msgs
source install/setup.bash
ros2 interface show drivebase_msgs/msg/LitterDetection
```

Expected: the definition prints, including the four `DIRECTION_*` constants.

- [ ] **Step 3: Commit**

```bash
git add src/drivebase_msgs
git commit -m "Add drivebase_msgs with the proposed LitterDetection message"
```

---

### Task 7: `drivebase_behaviour` package skeleton and the Detection adapter

**Files:**
- Create: `src/drivebase_behaviour/package.xml`, `setup.py`,
  `resource/drivebase_behaviour`, `drivebase_behaviour/__init__.py`
- Create: `src/drivebase_behaviour/drivebase_behaviour/detection.py`
- Test: `src/drivebase_behaviour/test/test_detection.py`

**Interfaces:**
- Produces:
  - `Detection` frozen dataclass with fields `stamp_seconds: float`,
    `detected: bool`, `frame_number: int`, `confidence: float`,
    `center_x: float`, `center_y: float`, `floor_x: float`, `floor_y: float`,
    `horizontal_error: float`, `bbox: tuple[float, float, float, float]`,
    `range_is_fallback: bool = False`
  - `parse_target_json(payload: str, stamp_seconds: float) -> Detection | None`
  - `direction_from_error(horizontal_error: float, deadband: float) -> str`

- [ ] **Step 1: Create the package skeleton**

`src/drivebase_behaviour/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>drivebase_behaviour</name>
  <version>0.1.0</version>
  <description>Mission coordinator: patrol, detect, approach, grasp.</description>
  <maintainer email="bryanhsu0123@gmail.com">Robot Curiosity</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf_transformations</exec_depend>
  <exec_depend>drivebase_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>urdfdom_py</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`src/drivebase_behaviour/setup.py`:

```python
from glob import glob

from setuptools import find_packages, setup

package_name = 'drivebase_behaviour'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Curiosity',
    maintainer_email='bryanhsu0123@gmail.com',
    description='Mission coordinator: patrol, detect, approach, grasp.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coordinator = drivebase_behaviour.coordinator_node:main',
        ],
    },
)
```

Then:

```bash
mkdir -p src/drivebase_behaviour/{drivebase_behaviour,test,config,launch,resource}
touch src/drivebase_behaviour/resource/drivebase_behaviour
touch src/drivebase_behaviour/drivebase_behaviour/__init__.py
```

- [ ] **Step 2: Write the failing test**

`src/drivebase_behaviour/test/test_detection.py`:

```python
import json

import pytest

from drivebase_behaviour.detection import (
    Detection,
    direction_from_error,
    parse_target_json,
)

VALID = json.dumps({
    "detected": True,
    "frame_number": 42,
    "confidence": 0.87,
    "bbox_pixels": {"x1": 100.0, "y1": 200.0, "x2": 160.0, "y2": 260.0},
    "center_normalized": {"x": 0.52, "y": 0.60},
    "floor_point_normalized": {"x": 0.52, "y": 0.68},
    "horizontal_error": 0.02,
    "direction": "CENTER",
})


def test_parses_a_valid_payload():
    d = parse_target_json(VALID, stamp_seconds=12.5)
    assert d is not None
    assert d.detected is True
    assert d.frame_number == 42
    assert d.confidence == pytest.approx(0.87)
    assert d.bbox == (100.0, 200.0, 160.0, 260.0)
    assert d.center_x == pytest.approx(0.52)
    assert d.floor_y == pytest.approx(0.68)
    assert d.horizontal_error == pytest.approx(0.02)
    assert d.stamp_seconds == pytest.approx(12.5)
    assert d.range_is_fallback is False


def test_parses_the_no_target_payload():
    # publish_no_target() emits only these two keys - the parser must not
    # require the geometry fields that are absent.
    payload = json.dumps({"detected": False, "frame_number": 7})
    d = parse_target_json(payload, stamp_seconds=1.0)
    assert d is not None
    assert d.detected is False
    assert d.frame_number == 7


@pytest.mark.parametrize("payload", [
    "",
    "not json at all",
    "{",
    json.dumps([1, 2, 3]),
    json.dumps({"detected": True}),          # truncated: no geometry
    json.dumps({"detected": "yes"}),          # wrong type
])
def test_malformed_payloads_return_none_rather_than_raising(payload):
    # A detector crash must not take the coordinator down with it.
    assert parse_target_json(payload, stamp_seconds=1.0) is None


def test_detection_is_immutable():
    d = parse_target_json(VALID, stamp_seconds=1.0)
    with pytest.raises(Exception):
        d.confidence = 0.1


@pytest.mark.parametrize("error,expected", [
    (-0.4, "LEFT"),
    (-0.13, "LEFT"),
    (-0.11, "CENTER"),
    (0.0, "CENTER"),
    (0.11, "CENTER"),
    (0.13, "RIGHT"),
    (0.4, "RIGHT"),
])
def test_direction_matches_the_detectors_thresholds(error, expected):
    # Same 0.12 deadband trash_vision uses, so the derived value agrees with
    # what the detector itself publishes.
    assert direction_from_error(error, deadband=0.12) == expected
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_detection.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'drivebase_behaviour.detection'`.

- [ ] **Step 4: Implement**

`src/drivebase_behaviour/drivebase_behaviour/detection.py`:

```python
"""One detection, and the parsing of today's JSON payload into it.

PURE. No rclpy. The coordinator never sees JSON - it sees Detection, whichever
source produced it. That is what lets the typed drivebase_msgs/LitterDetection
replace the std_msgs/String interface without the state machine noticing.
"""

import json
from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    # Seconds. From the message header when the typed interface is in use;
    # from arrival time when it is not, because std_msgs/String carries no
    # stamp. The difference matters under load - see the design doc §8.
    stamp_seconds: float
    detected: bool
    frame_number: int
    confidence: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    floor_x: float = 0.0
    floor_y: float = 0.0
    horizontal_error: float = 0.0
    bbox: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    # Set when the point came from the bbox-size estimate rather than the ToF.
    # The confirmation gate demands more consecutive frames when it is set.
    range_is_fallback: bool = False


def direction_from_error(horizontal_error: float, deadband: float) -> str:
    if horizontal_error < -deadband:
        return "LEFT"
    if horizontal_error > deadband:
        return "RIGHT"
    return "CENTER"


def parse_target_json(payload: str, stamp_seconds: float) -> Detection | None:
    """Returns None for anything malformed.

    Deliberately total: a detector that crashes, restarts mid-message or
    changes its schema must degrade the coordinator to "no target", never take
    it down. Every failure here is one dropped frame.
    """
    try:
        raw = json.loads(payload)
    except (ValueError, TypeError):
        return None

    if not isinstance(raw, dict):
        return None

    detected = raw.get("detected")
    if not isinstance(detected, bool):
        return None

    frame_number = raw.get("frame_number", 0)
    if not isinstance(frame_number, int):
        return None

    if not detected:
        return Detection(
            stamp_seconds=stamp_seconds,
            detected=False,
            frame_number=frame_number,
        )

    try:
        bbox_raw = raw["bbox_pixels"]
        center = raw["center_normalized"]
        floor = raw["floor_point_normalized"]
        return Detection(
            stamp_seconds=stamp_seconds,
            detected=True,
            frame_number=frame_number,
            confidence=float(raw["confidence"]),
            center_x=float(center["x"]),
            center_y=float(center["y"]),
            floor_x=float(floor["x"]),
            floor_y=float(floor["y"]),
            horizontal_error=float(raw["horizontal_error"]),
            bbox=(
                float(bbox_raw["x1"]),
                float(bbox_raw["y1"]),
                float(bbox_raw["x2"]),
                float(bbox_raw["y2"]),
            ),
        )
    except (KeyError, TypeError, ValueError):
        return None
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_detection.py -v
```

Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add src/drivebase_behaviour
git commit -m "Add drivebase_behaviour skeleton and the Detection adapter"
```

---

### Task 8: Target localisation — pixel to 3D point

**Files:**
- Create: `src/drivebase_behaviour/drivebase_behaviour/localisation.py`
- Test: `src/drivebase_behaviour/test/test_localisation.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `pixel_to_unit_ray(u, v, fx, fy, cx, cy) -> tuple[float, float, float]`
  - `point_from_range(ray, range_m) -> tuple[float, float, float]`
  - `range_is_plausible(range_m, min_range, max_range) -> bool`
  - `fallback_range_from_bbox(bbox_height_px, image_height_px, reference_height_px, reference_range_m) -> float`

- [ ] **Step 1: Write the failing test**

`src/drivebase_behaviour/test/test_localisation.py`:

```python
import math

import pytest

from drivebase_behaviour.localisation import (
    fallback_range_from_bbox,
    pixel_to_unit_ray,
    point_from_range,
    range_is_plausible,
)

# A 640x480 camera with a 1.15 rad horizontal FOV, matching gazebo.xacro.
# fx = (width/2) / tan(hfov/2)
FX = 320.0 / math.tan(1.15 / 2.0)
FY = FX
CX, CY = 320.0, 240.0


def test_centre_pixel_gives_the_optical_axis():
    ray = pixel_to_unit_ray(CX, CY, FX, FY, CX, CY)
    assert ray == pytest.approx((0.0, 0.0, 1.0))


def test_ray_is_always_unit_length():
    for u, v in [(0, 0), (639, 479), (320, 0), (100, 400)]:
        ray = pixel_to_unit_ray(u, v, FX, FY, CX, CY)
        assert math.sqrt(sum(c * c for c in ray)) == pytest.approx(1.0)


def test_right_of_centre_gives_positive_x():
    # Optical frame convention: x right, y down, z forward.
    ray = pixel_to_unit_ray(CX + 100, CY, FX, FY, CX, CY)
    assert ray[0] > 0.0
    assert ray[1] == pytest.approx(0.0)


def test_below_centre_gives_positive_y():
    ray = pixel_to_unit_ray(CX, CY + 100, FX, FY, CX, CY)
    assert ray[1] > 0.0


def test_edge_pixel_matches_half_the_field_of_view():
    # The right edge must sit at exactly hfov/2 off axis. This is the check
    # that catches an fx/fy mix-up, which is otherwise invisible on a square
    # test image.
    ray = pixel_to_unit_ray(640.0, CY, FX, FY, CX, CY)
    angle = math.atan2(ray[0], ray[2])
    assert angle == pytest.approx(1.15 / 2.0, abs=1e-6)


def test_point_scales_along_the_ray():
    ray = pixel_to_unit_ray(CX, CY, FX, FY, CX, CY)
    assert point_from_range(ray, 0.5) == pytest.approx((0.0, 0.0, 0.5))


def test_point_preserves_distance():
    ray = pixel_to_unit_ray(500.0, 400.0, FX, FY, CX, CY)
    p = point_from_range(ray, 0.42)
    assert math.sqrt(sum(c * c for c in p)) == pytest.approx(0.42)


@pytest.mark.parametrize("value,expected", [
    (0.30, True),
    (0.04, True),
    (4.00, True),
    (0.02, False),      # inside the VL53 minimum
    (5.00, False),      # beyond maximum
    (float("nan"), False),
    (float("inf"), False),
    (-1.0, False),
])
def test_plausibility_rejects_what_the_sensor_cannot_mean(value, expected):
    assert range_is_plausible(value, 0.04, 4.0) is expected


def test_fallback_range_is_inverse_in_apparent_size():
    # Twice as tall in frame means half as far away.
    assert fallback_range_from_bbox(
        bbox_height_px=120.0, image_height_px=480.0,
        reference_height_px=60.0, reference_range_m=1.0,
    ) == pytest.approx(0.5)


def test_fallback_range_rejects_a_zero_height_box():
    assert math.isnan(fallback_range_from_bbox(
        bbox_height_px=0.0, image_height_px=480.0,
        reference_height_px=60.0, reference_range_m=1.0,
    ))
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_localisation.py -v
```

Expected: FAIL — no module named `localisation`.

- [ ] **Step 3: Implement**

`src/drivebase_behaviour/drivebase_behaviour/localisation.py`:

```python
"""Turn a detected pixel plus a ToF range into a point in the camera frame.

PURE. No rclpy.

This replaced a flat-ground homography. The homography needed level pavement
and would have degraded on grass or any slope; a bearing from the camera times
a scalar distance from the ToF is a full 3D fix with no ground assumption at
all. See the design doc §5.

All outputs are in the OPTICAL frame convention: z forward, x right, y down.
"""

import math


def pixel_to_unit_ray(
    u: float, v: float, fx: float, fy: float, cx: float, cy: float,
) -> tuple[float, float, float]:
    """Pinhole back-projection. Intrinsics come from camera_info, never
    hardcoded - the simulated and real cameras differ."""
    x = (u - cx) / fx
    y = (v - cy) / fy
    norm = math.sqrt(x * x + y * y + 1.0)
    return (x / norm, y / norm, 1.0 / norm)


def point_from_range(
    ray: tuple[float, float, float], range_m: float,
) -> tuple[float, float, float]:
    return (ray[0] * range_m, ray[1] * range_m, ray[2] * range_m)


def range_is_plausible(
    range_m: float, min_range: float, max_range: float,
) -> bool:
    """ToF parts drop out on clear plastic, shiny film and dark matte surfaces
    - exactly what litter is made of. A dropout reads as nan, inf, zero or a
    number past the far limit, and acting on any of them drives the arm at
    nothing."""
    if math.isnan(range_m) or math.isinf(range_m):
        return False
    return min_range <= range_m <= max_range


def fallback_range_from_bbox(
    bbox_height_px: float,
    image_height_px: float,
    reference_height_px: float,
    reference_range_m: float,
) -> float:
    """Apparent size gives distance, crudely, but is indifferent to surface
    finish - which is the whole point, since that is what defeats the ToF.

    Calibrated by one reference observation: an object of known size at a known
    range. Assumes litter is roughly uniform in size, which is wrong in general
    and adequate for deciding whether to keep approaching.

    Returns nan when the box has no height to measure.
    """
    if bbox_height_px <= 0.0 or image_height_px <= 0.0:
        return float("nan")
    return reference_range_m * (reference_height_px / bbox_height_px)
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_localisation.py -v
```

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/drivebase_behaviour/drivebase_behaviour/localisation.py \
        src/drivebase_behaviour/test/test_localisation.py
git commit -m "Add pixel-to-point localisation from camera ray and ToF range"
```

---

### Task 9: Arm kinematics derived from the URDF

The spec forbids hardcoding link lengths. The SO-101's joint origins carry
CAD-export rpy values with small perpendicular offsets, so hand-deriving them
is error-prone. This task derives them by evaluating forward kinematics.

**Files:**
- Create: `src/drivebase_behaviour/drivebase_behaviour/kinematics.py`
- Test: `src/drivebase_behaviour/test/test_kinematics.py`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `load_chain(urdf_xml: str, base_link: str, tip_link: str) -> Chain`
  - `Chain.joint_names: list[str]` (movable joints, base to tip)
  - `forward_kinematics(chain, joint_values: dict[str, float]) -> numpy.ndarray` (4x4)
  - `PlanarArm` dataclass: `l1, l2, l3: float`, `a1, a2, a3: float`,
    `origin_r, origin_z: float`
  - `extract_planar_arm(chain) -> PlanarArm`

- [ ] **Step 1: Write the failing test**

`src/drivebase_behaviour/test/test_kinematics.py`:

```python
import math
import pathlib
import subprocess

import numpy as np
import pytest

from drivebase_behaviour.kinematics import (
    extract_planar_arm,
    forward_kinematics,
    load_chain,
)

PITCH_JOINTS = ["arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex"]


@pytest.fixture(scope="module")
def urdf_xml():
    repo = pathlib.Path(__file__).resolve().parents[3]
    xacro = repo / "src/drivebase_description/urdf/drivebase.urdf.xacro"
    return subprocess.run(
        ["xacro", str(xacro), "use_sim:=true", "use_arm:=true",
         "wheel_mu1:=1.0", "wheel_mu2:=0.6"],
        capture_output=True, text=True, check=True,
    ).stdout


@pytest.fixture(scope="module")
def chain(urdf_xml):
    return load_chain(urdf_xml, "arm_base_link", "arm_gripper_frame_link")


def test_chain_finds_the_five_movable_joints_in_order(chain):
    assert chain.joint_names == [
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll",
    ]


def test_fk_at_zero_is_a_valid_transform(chain):
    t = forward_kinematics(chain, {})
    assert t.shape == (4, 4)
    assert t[3, :] == pytest.approx([0, 0, 0, 1])
    # Rotation block must be orthonormal.
    r = t[:3, :3]
    assert (r @ r.T) == pytest.approx(np.eye(3), abs=1e-9)


def test_pan_rotates_the_tip_about_the_base_z_axis(chain):
    at_zero = forward_kinematics(chain, {})[:3, 3]
    quarter = forward_kinematics(
        chain, {"arm_shoulder_pan": math.pi / 2})[:3, 3]
    # Radius from the pan axis is preserved; height is preserved.
    assert math.hypot(*quarter[:2]) == pytest.approx(
        math.hypot(*at_zero[:2]), abs=1e-9)
    assert quarter[2] == pytest.approx(at_zero[2], abs=1e-9)


def test_planar_link_lengths_are_physically_sensible(chain):
    arm = extract_planar_arm(chain)
    # The SO-101 is a desktop arm: every segment is between 3 and 25 cm.
    for length in (arm.l1, arm.l2, arm.l3):
        assert 0.03 < length < 0.25
    # Total reach must be enough to get from a 260 mm mount to the ground,
    # which STATUS.md measured as a 219 mm drop.
    assert arm.l1 + arm.l2 + arm.l3 > 0.219


def test_planar_model_reproduces_full_fk(chain):
    """The planar model is only useful if it agrees with real FK.

    For any pitch-joint triple, the planar (r, z) prediction must match the
    radius and height the full 4x4 chain produces. This is the test that
    catches a wrong angle offset, which no amount of eyeballing the URDF will.
    """
    arm = extract_planar_arm(chain)
    for t1, t2, t3 in [
        (0.0, 0.0, 0.0),
        (0.3, -0.4, 0.2),
        (-0.5, 0.6, -0.3),
        (1.0, -1.0, 0.5),
    ]:
        values = dict(zip(PITCH_JOINTS, (t1, t2, t3)))
        tip = forward_kinematics(chain, values)[:3, 3]
        r_actual = math.hypot(tip[0], tip[1])
        z_actual = tip[2]

        r_model = arm.origin_r + (
            arm.l1 * math.cos(arm.a1 + t1)
            + arm.l2 * math.cos(arm.a2 + t1 + t2)
            + arm.l3 * math.cos(arm.a3 + t1 + t2 + t3)
        )
        z_model = arm.origin_z + (
            arm.l1 * math.sin(arm.a1 + t1)
            + arm.l2 * math.sin(arm.a2 + t1 + t2)
            + arm.l3 * math.sin(arm.a3 + t1 + t2 + t3)
        )
        assert r_model == pytest.approx(r_actual, abs=1e-6)
        assert z_model == pytest.approx(z_actual, abs=1e-6)
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_kinematics.py -v
```

Expected: FAIL — no module named `kinematics`.

- [ ] **Step 3: Implement**

`src/drivebase_behaviour/drivebase_behaviour/kinematics.py`:

```python
"""Forward kinematics for the SO-101, read from the URDF.

PURE except for numpy and urdf_parser_py. No rclpy.

WHY THIS DERIVES RATHER THAN HARDCODES. The SO-101's joint origins come from a
CAD export: link offsets are not aligned to any axis (the elbow sits at
-0.11257, -0.028, 0) and the rpy values carry 1e-16 noise. Reading three link
lengths off that by hand is a transcription error waiting to happen, and a
wrong length fails silently as an arm that reaches slightly past the litter.

So: parse the chain, compose the transforms, and let extract_planar_arm derive
the planar parameters by evaluating the result. The IK then solves against
numbers that provably match the model it will command.
"""

import math
from dataclasses import dataclass

import numpy as np
from urdf_parser_py.urdf import URDF


@dataclass(frozen=True)
class Joint:
    name: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    movable: bool


@dataclass(frozen=True)
class Chain:
    joints: list[Joint]

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.joints if j.movable]


@dataclass(frozen=True)
class PlanarArm:
    """The three pitch joints reduced to a planar 3R arm.

    Tip position in the pan plane, for pitch values (t1, t2, t3):
        r = origin_r + l1*cos(a1+t1) + l2*cos(a2+t1+t2) + l3*cos(a3+t1+t2+t3)
        z = origin_z + l1*sin(a1+t1) + l2*sin(a2+t1+t2) + l3*sin(a3+t1+t2+t3)

    The a_i are the zero-pose angles of each link, which absorb the CAD frame
    rotations so the IK never has to know about them.
    """

    l1: float
    l2: float
    l3: float
    a1: float
    a2: float
    a3: float
    origin_r: float
    origin_z: float


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle_to_matrix(axis: tuple[float, float, float],
                          angle: float) -> np.ndarray:
    a = np.array(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm == 0.0:
        return np.eye(3)
    a = a / norm
    k = np.array([[0.0, -a[2], a[1]],
                  [a[2], 0.0, -a[0]],
                  [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _transform(rotation: np.ndarray,
               translation: tuple[float, float, float]) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def load_chain(urdf_xml: str, base_link: str, tip_link: str) -> Chain:
    robot = URDF.from_xml_string(urdf_xml)

    child_to_joint = {j.child: j for j in robot.joints}

    # Walk up from the tip: the URDF is a tree, so each link has exactly one
    # parent joint, which makes this unambiguous. Walking down would require
    # searching every branch.
    reversed_joints: list[Joint] = []
    link = tip_link
    while link != base_link:
        if link not in child_to_joint:
            raise ValueError(f"{tip_link} does not descend from {base_link}")
        j = child_to_joint[link]
        origin_xyz = tuple(j.origin.xyz) if j.origin else (0.0, 0.0, 0.0)
        origin_rpy = tuple(j.origin.rpy) if j.origin else (0.0, 0.0, 0.0)
        reversed_joints.append(Joint(
            name=j.name,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=tuple(j.axis) if j.axis else (0.0, 0.0, 1.0),
            movable=j.type in ("revolute", "continuous", "prismatic"),
        ))
        link = j.parent

    reversed_joints.reverse()
    return Chain(joints=reversed_joints)


def forward_kinematics(
    chain: Chain, joint_values: dict[str, float],
) -> np.ndarray:
    """4x4 transform from the chain's base link to its tip. Joints absent from
    joint_values are treated as zero."""
    result = np.eye(4)
    for j in chain.joints:
        result = result @ _transform(
            _rpy_to_matrix(*j.origin_rpy), j.origin_xyz)
        if j.movable:
            angle = float(joint_values.get(j.name, 0.0))
            result = result @ _transform(
                _axis_angle_to_matrix(j.axis, angle), (0.0, 0.0, 0.0))
    return result


def extract_planar_arm(chain: Chain) -> PlanarArm:
    """Derive the planar 3R parameters by evaluating FK at the zero pose.

    The three pitch joints have parallel axes, so the tip traces a plane that
    rotates rigidly with shoulder_pan. Measuring each link as the vector
    between consecutive joint origins - in (radius, height) coordinates -
    yields lengths and zero-pose angles that reproduce full FK exactly. The
    test asserts precisely that.
    """
    names = chain.joint_names
    pan, lift, elbow, flex = names[0], names[1], names[2], names[3]

    def origin_of(joint_name: str) -> np.ndarray:
        """Position of a joint's frame origin, in the base link."""
        result = np.eye(4)
        for j in chain.joints:
            result = result @ _transform(
                _rpy_to_matrix(*j.origin_rpy), j.origin_xyz)
            if j.name == joint_name:
                return result[:3, 3]
            if j.movable:
                result = result @ _transform(np.eye(3), (0.0, 0.0, 0.0))
        raise ValueError(f"joint {joint_name} not in chain")

    def tip() -> np.ndarray:
        return forward_kinematics(chain, {})[:3, 3]

    def planar(p: np.ndarray) -> tuple[float, float]:
        return (math.hypot(p[0], p[1]), p[2])

    _ = pan  # the pan joint defines the plane; it contributes no link length

    p0 = planar(origin_of(lift))
    p1 = planar(origin_of(elbow))
    p2 = planar(origin_of(flex))
    p3 = planar(tip())

    def link(a: tuple[float, float],
             b: tuple[float, float]) -> tuple[float, float]:
        dr, dz = b[0] - a[0], b[1] - a[1]
        return (math.hypot(dr, dz), math.atan2(dz, dr))

    l1, a1 = link(p0, p1)
    l2, a2 = link(p1, p2)
    l3, a3 = link(p2, p3)

    return PlanarArm(
        l1=l1, l2=l2, l3=l3, a1=a1, a2=a2, a3=a3,
        origin_r=p0[0], origin_z=p0[1],
    )
```

- [ ] **Step 4: Run the tests**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_kinematics.py -v
```

Expected: all PASS. **`test_planar_model_reproduces_full_fk` is the one that
matters** — if it fails, the planar reduction is wrong and the IK built on it
will aim at the wrong place. Do not proceed past a failure here by loosening
the tolerance.

- [ ] **Step 5: Commit**

```bash
git add src/drivebase_behaviour/drivebase_behaviour/kinematics.py \
        src/drivebase_behaviour/test/test_kinematics.py
git commit -m "Derive SO-101 planar kinematics from the URDF"
```

---

### Task 10: Closed-form inverse kinematics

**Files:**
- Create: `src/drivebase_behaviour/drivebase_behaviour/ik.py`
- Test: `src/drivebase_behaviour/test/test_ik.py`

**Interfaces:**
- Consumes: `PlanarArm` from Task 9
- Produces:
  - `IkSolution` dataclass: `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex: float`
  - `solve(arm: PlanarArm, target_xyz, approach_angle: float, limits: dict[str, tuple[float, float]]) -> IkSolution | None`
  - `JOINT_LIMITS: dict[str, tuple[float, float]]` (from the URDF values in
    the constants table at the top of this plan)

- [x] **Step 1: Write the failing test**

`src/drivebase_behaviour/test/test_ik.py`:

```python
import math
import pathlib
import subprocess

import pytest

from drivebase_behaviour.ik import JOINT_LIMITS, solve
from drivebase_behaviour.kinematics import (
    extract_planar_arm,
    forward_kinematics,
    load_chain,
)


@pytest.fixture(scope="module")
def chain():
    repo = pathlib.Path(__file__).resolve().parents[3]
    xacro = repo / "src/drivebase_description/urdf/drivebase.urdf.xacro"
    xml = subprocess.run(
        ["xacro", str(xacro), "use_sim:=true", "use_arm:=true",
         "wheel_mu1:=1.0", "wheel_mu2:=0.6"],
        capture_output=True, text=True, check=True,
    ).stdout
    return load_chain(xml, "arm_base_link", "arm_gripper_frame_link")


@pytest.fixture(scope="module")
def arm(chain):
    return extract_planar_arm(chain)


def reachable_targets(arm):
    """Points generated by FK itself, so they are reachable by construction."""
    out = []
    for t1, t2, t3 in [(0.2, -0.3, 0.1), (-0.3, 0.5, -0.2), (0.6, -0.8, 0.4)]:
        r = arm.origin_r + (
            arm.l1 * math.cos(arm.a1 + t1)
            + arm.l2 * math.cos(arm.a2 + t1 + t2)
            + arm.l3 * math.cos(arm.a3 + t1 + t2 + t3))
        z = arm.origin_z + (
            arm.l1 * math.sin(arm.a1 + t1)
            + arm.l2 * math.sin(arm.a2 + t1 + t2)
            + arm.l3 * math.sin(arm.a3 + t1 + t2 + t3))
        approach = arm.a3 + t1 + t2 + t3
        out.append((r, z, approach))
    return out


def test_solution_round_trips_through_forward_kinematics(chain, arm):
    """The only IK test that proves anything: solve, then FK the answer back
    and assert it lands on the commanded point."""
    for r, z, approach in reachable_targets(arm):
        for pan in (0.0, 0.4, -0.7):
            target = (r * math.cos(pan), r * math.sin(pan), z)
            s = solve(arm, target, approach, JOINT_LIMITS)
            assert s is not None, (target, approach)

            tip = forward_kinematics(chain, {
                "arm_shoulder_pan": s.shoulder_pan,
                "arm_shoulder_lift": s.shoulder_lift,
                "arm_elbow_flex": s.elbow_flex,
                "arm_wrist_flex": s.wrist_flex,
            })[:3, 3]
            assert tip[0] == pytest.approx(target[0], abs=1e-6)
            assert tip[1] == pytest.approx(target[1], abs=1e-6)
            assert tip[2] == pytest.approx(target[2], abs=1e-6)


def test_returns_none_when_the_point_is_out_of_reach(arm):
    far = (arm.l1 + arm.l2 + arm.l3 + 0.5, 0.0, 0.0)
    assert solve(arm, far, -math.pi / 2, JOINT_LIMITS) is None


def test_returns_none_when_the_point_is_inside_the_dead_zone(arm):
    # Directly on the shoulder axis at shoulder height: no elbow angle reaches
    # it, and a naive 2R solve produces a domain error rather than a rejection.
    at_origin = (arm.origin_r, 0.0, arm.origin_z)
    result = solve(arm, at_origin, 0.0, JOINT_LIMITS)
    assert result is None or all(
        JOINT_LIMITS[n][0] <= v <= JOINT_LIMITS[n][1]
        for n, v in [("shoulder_lift", result.shoulder_lift),
                     ("elbow_flex", result.elbow_flex),
                     ("wrist_flex", result.wrist_flex)])


def test_every_returned_joint_is_inside_its_limit(arm):
    for r, z, approach in reachable_targets(arm):
        s = solve(arm, (r, 0.0, z), approach, JOINT_LIMITS)
        if s is None:
            continue
        for name, value in [
            ("shoulder_pan", s.shoulder_pan),
            ("shoulder_lift", s.shoulder_lift),
            ("elbow_flex", s.elbow_flex),
            ("wrist_flex", s.wrist_flex),
        ]:
            lo, hi = JOINT_LIMITS[name]
            assert lo <= value <= hi, (name, value)


def test_returns_none_when_the_solution_violates_a_limit(arm):
    # Squeeze shoulder_pan to a hair either side of zero: a target off to the
    # side then has no legal pan, so the solver must refuse rather than
    # silently clamp and aim somewhere else.
    tight = dict(JOINT_LIMITS)
    tight["shoulder_pan"] = (-0.01, 0.01)
    r, z, approach = reachable_targets(arm)[0]
    target = (r * math.cos(1.2), r * math.sin(1.2), z)
    assert solve(arm, target, approach, tight) is None


def test_pan_points_at_the_target_bearing(arm):
    r, z, approach = reachable_targets(arm)[0]
    s = solve(arm, (r * math.cos(0.5), r * math.sin(0.5), z),
              approach, JOINT_LIMITS)
    assert s is not None
    assert s.shoulder_pan == pytest.approx(0.5, abs=1e-9)
```

- [x] **Step 2: Run the test to verify it fails**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_ik.py -v
```

Expected: FAIL — no module named `ik`.

- [x] **Step 3: Implement**

`src/drivebase_behaviour/drivebase_behaviour/ik.py`:

```python
"""Closed-form inverse kinematics for the SO-101.

PURE. No rclpy, no numpy.

WHY CLOSED FORM. shoulder_lift, elbow_flex and wrist_flex have parallel axes,
so once shoulder_pan aims the plane the problem is a planar 3R arm. Fixing the
approach direction removes the one redundant degree of freedom and what remains
is the textbook 2R solve. Tens of microseconds, no solver, no MoveIt on a Pi
that is already sharing cores with YOLO.

The approach angle is fixed by the caller rather than chosen here: coming down
vertically onto litter is what the gripper geometry wants, and making it a
parameter keeps that a decision of the grasp sequence rather than of the maths.
"""

import math
from dataclasses import dataclass

from drivebase_behaviour.kinematics import PlanarArm

# From so101_macro.xacro. wrist_roll and gripper are not solved for - roll is
# held near zero to protect the sensor pod harness, and the gripper is a grasp
# command, not a pose.
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


@dataclass(frozen=True)
class IkSolution:
    shoulder_pan: float
    shoulder_lift: float
    elbow_flex: float
    wrist_flex: float


def _within(name: str, value: float,
            limits: dict[str, tuple[float, float]]) -> bool:
    low, high = limits[name]
    return low <= value <= high


def solve(
    arm: PlanarArm,
    target_xyz: tuple[float, float, float],
    approach_angle: float,
    limits: dict[str, tuple[float, float]],
) -> IkSolution | None:
    """Returns None for anything unreachable or outside a joint limit.

    Never clamps. A clamped solution aims the arm somewhere other than the
    commanded point while reporting success, which on this robot means closing
    the gripper on empty ground and reporting a pickup.
    """
    x, y, z = target_xyz

    pan = math.atan2(y, x)
    if not _within("shoulder_pan", pan, limits):
        return None

    # Into the pan plane. r is signed positive because pan already points at
    # the target.
    r = math.hypot(x, y)

    # Back off the last link along the approach direction to find where the
    # wrist_flex joint must sit. That reduces 3R to 2R.
    wrist_r = r - arm.origin_r - arm.l3 * math.cos(approach_angle)
    wrist_z = z - arm.origin_z - arm.l3 * math.sin(approach_angle)

    distance = math.hypot(wrist_r, wrist_z)
    if distance > arm.l1 + arm.l2 or distance < abs(arm.l1 - arm.l2):
        return None
    if distance == 0.0:
        return None

    # Law of cosines. The clamp guards floating-point drift at full extension
    # only - the reachability test above has already rejected genuine misses,
    # so this cannot mask an out-of-range target.
    cos_elbow = (
        distance * distance - arm.l1 * arm.l1 - arm.l2 * arm.l2
    ) / (2.0 * arm.l1 * arm.l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))

    # Elbow-down. The alternative (+) branch folds the arm back over itself,
    # which on a front-mounted arm means driving the elbow into the chassis.
    elbow_interior = -math.acos(cos_elbow)

    beta = math.atan2(
        arm.l2 * math.sin(elbow_interior),
        arm.l1 + arm.l2 * math.cos(elbow_interior),
    )
    link1_angle = math.atan2(wrist_z, wrist_r) - beta

    # Undo the zero-pose offsets absorbed into a1/a2/a3 by extract_planar_arm,
    # converting link angles back into joint angles.
    lift = link1_angle - arm.a1
    elbow = (link1_angle + elbow_interior) - arm.a2 - lift
    flex = approach_angle - arm.a3 - lift - elbow

    for name, value in [("shoulder_lift", lift),
                        ("elbow_flex", elbow),
                        ("wrist_flex", flex)]:
        if not _within(name, value, limits):
            return None

    return IkSolution(
        shoulder_pan=pan,
        shoulder_lift=lift,
        elbow_flex=elbow,
        wrist_flex=flex,
    )
```

- [x] **Step 4: Run the tests**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_ik.py -v
```

Expected: all PASS. If `test_solution_round_trips_through_forward_kinematics`
fails, the angle-offset arithmetic in the last block is wrong — compare against
`test_planar_model_reproduces_full_fk` from Task 9, which defines the
convention.

- [x] **Step 5: Commit**

```bash
git add src/drivebase_behaviour/drivebase_behaviour/ik.py \
        src/drivebase_behaviour/test/test_ik.py
git commit -m "Add closed-form planar IK for the SO-101"
```

---

# Phase 3 — Detection shim

---

### Task 11: `sim_litter_detector`

The real detector cannot run in simulation: it loads weights from
`/home/christopherg/best_ncnn_model` and reads a TCP stream. This shim
publishes the identical interface so hardware swaps the real one in with no
coordinator change.

**Files:**
- Create: `src/drivebase_sim/drivebase_sim/sim_litter_detector.py`
- Modify: `src/drivebase_sim/setup.py` (entry point)
- Modify: `src/drivebase_sim/package.xml` (add `cv_bridge`, `python3-opencv`)
- Modify: `src/drivebase_sim/launch/sim.launch.py`

**Interfaces:**
- Consumes: `/pod_camera/image_raw`
- Produces: `/vision/target` (`std_msgs/String`, JSON identical in shape to
  `trash_vision`'s) and `/vision/direction` (`std_msgs/String`)

- [ ] **Step 1: Add the dependencies and entry point**

In `src/drivebase_sim/package.xml`, alongside the other `exec_depend` entries:

```xml
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

In `src/drivebase_sim/setup.py`, add to `console_scripts`:

```python
            'sim_litter_detector = drivebase_sim.sim_litter_detector:main',
```

- [ ] **Step 2: Implement the shim**

`src/drivebase_sim/drivebase_sim/sim_litter_detector.py`:

```python
"""Stand-in for trash_vision, for simulation only.

trash_vision cannot run here: it loads NCNN weights from a hardcoded home
directory and opens a TCP camera stream. This node segments the saturated red
litter models in test_field.sdf by hue instead.

THE INTERFACE IS THE POINT. Topic names, message types and the exact JSON key
layout match trash_vision's publish_target/publish_no_target byte for byte, so
the coordinator cannot tell which one it is talking to and hardware swaps the
real detector in by simply not launching this.

It is not a perception result. It proves the loop runs, not that detection
works.
"""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SimLitterDetector(Node):
    def __init__(self) -> None:
        super().__init__("sim_litter_detector")

        self.declare_parameter("image_topic", "/pod_camera/image_raw")
        self.declare_parameter("deadband", 0.12)
        # Smallest blob worth reporting, as a fraction of frame area. Below
        # this the centroid is dominated by noise and the bbox height - which
        # the ToF fallback depends on - is meaningless.
        self.declare_parameter("min_area_fraction", 0.0004)
        self.declare_parameter("hue_low", 0)
        self.declare_parameter("hue_high", 10)
        self.declare_parameter("saturation_min", 120)
        self.declare_parameter("value_min", 60)

        self.deadband = float(self.get_parameter("deadband").value)
        self.min_area_fraction = float(
            self.get_parameter("min_area_fraction").value)
        self.hue_low = int(self.get_parameter("hue_low").value)
        self.hue_high = int(self.get_parameter("hue_high").value)
        self.saturation_min = int(self.get_parameter("saturation_min").value)
        self.value_min = int(self.get_parameter("value_min").value)

        self.bridge = CvBridge()
        self.frame_number = 0

        self.target_publisher = self.create_publisher(String, "/vision/target", 10)
        self.direction_publisher = self.create_publisher(
            String, "/vision/direction", 10)

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Simulated litter detector started")

    def on_image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.frame_number += 1

        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([self.hue_low, self.saturation_min, self.value_min]),
            np.array([self.hue_high, 255, 255]),
        )

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.publish_no_target()
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area_fraction * width * height:
            self.publish_no_target()
            return

        x, y, w, h = cv2.boundingRect(largest)
        x1, y1, x2, y2 = float(x), float(y), float(x + w), float(y + h)

        center_x = ((x1 + x2) / 2.0) / width
        center_y = ((y1 + y2) / 2.0) / height
        horizontal_error = center_x - 0.5

        if horizontal_error < -self.deadband:
            direction = "LEFT"
        elif horizontal_error > self.deadband:
            direction = "RIGHT"
        else:
            direction = "CENTER"

        # Key layout copied from trash_vision's publish_target. Do not
        # "improve" it - the coordinator's JSON adapter parses both.
        target = {
            "detected": True,
            "frame_number": self.frame_number,
            "confidence": 1.0,
            "bbox_pixels": {
                "x1": round(x1, 1), "y1": round(y1, 1),
                "x2": round(x2, 1), "y2": round(y2, 1),
            },
            "center_normalized": {
                "x": round(center_x, 4), "y": round(center_y, 4),
            },
            "floor_point_normalized": {
                "x": round(center_x, 4), "y": round(y2 / height, 4),
            },
            "horizontal_error": round(horizontal_error, 4),
            "direction": direction,
        }

        target_message = String()
        target_message.data = json.dumps(target)
        self.target_publisher.publish(target_message)

        direction_message = String()
        direction_message.data = direction
        self.direction_publisher.publish(direction_message)

    def publish_no_target(self) -> None:
        target_message = String()
        target_message.data = json.dumps({
            "detected": False,
            "frame_number": self.frame_number,
        })
        self.target_publisher.publish(target_message)

        direction_message = String()
        direction_message.data = "NO_TARGET"
        self.direction_publisher.publish(direction_message)


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    node = SimLitterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Add it to the sim launch, behind a flag**

In `src/drivebase_sim/launch/sim.launch.py`, add a launch argument alongside
the others:

```python
        DeclareLaunchArgument(
            "litter_detector", default_value="true",
            description="Run the simulated litter detector. Turn off on "
                        "hardware, where trash_vision publishes the same "
                        "topics for real.",
        ),
```

and a node (with `with_litter_detector = LaunchConfiguration("litter_detector")`
declared next to the other configurations):

```python
        Node(
            package="drivebase_sim",
            executable="sim_litter_detector",
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(with_litter_detector),
            output="screen",
        ),
```

- [ ] **Step 4: Verify it detects the litter**

```bash
cd "$(git rev-parse --show-toplevel)"
colcon build --symlink-install --packages-select drivebase_sim && source install/setup.bash
ros2 launch drivebase_sim sim.launch.py headless:=true &
sleep 25

# Nothing in view from the spawn pose: expect detected=false.
ros2 topic echo /vision/target --once

# Point the arm forward and down, then drive toward the can at (1.8, 0.5).
ros2 topic pub -1 /arm/shoulder_lift/position std_msgs/msg/Float64 "{data: -0.6}"
ros2 topic pub -1 /arm/elbow_flex/position   std_msgs/msg/Float64 "{data: 0.8}"
ros2 topic pub -1 /arm/wrist_flex/position   std_msgs/msg/Float64 "{data: 0.4}"
sleep 5
timeout 8 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.12}}"
ros2 topic echo /vision/target --once
ros2 topic echo /vision/direction --once
kill %1
```

Expected: `detected: true` with a plausible bbox once the red can is in frame,
and a direction that agrees with which side of the image it is on.

- [ ] **Step 5: Commit**

```bash
git add src/drivebase_sim/drivebase_sim/sim_litter_detector.py \
        src/drivebase_sim/setup.py src/drivebase_sim/package.xml \
        src/drivebase_sim/launch/sim.launch.py
git commit -m "Add simulated litter detector matching the trash_vision interface"
```

---

# Phase 4 — The coordinator

---

### Task 12: The state machine

**Files:**
- Create: `src/drivebase_behaviour/drivebase_behaviour/state_machine.py`
- Test: `src/drivebase_behaviour/test/test_state_machine.py`

**Interfaces:**
- Consumes: `Detection` from Task 7
- Produces:
  - `State` enum: `IDLE`, `NAVIGATING`, `APPROACHING`, `CONFIRMING`,
    `PICKING`, `STOWING`
  - `Config` dataclass: `deadband`, `confirm_frames`,
    `confirm_frames_fallback`, `grasp_min`, `grasp_max`, `lost_timeout`,
    `approach_timeout`, `confirm_timeout`, `detection_stale_after`,
    `min_confidence`
  - `Inputs` dataclass: `now`, `detection`, `range_m`, `range_valid`,
    `in_workspace`, `nav_goal_active`, `nav_goal_succeeded`,
    `nav_goal_aborted`, `arm_sequence_done`
  - `Outputs` dataclass: `state`, `cancel_nav_goal`, `send_next_waypoint`,
    `store_grasp_point`, `start_grasp`, `start_stow`
  - `StateMachine.tick(inputs: Inputs) -> Outputs`

- [ ] **Step 1: Write the failing test**

`src/drivebase_behaviour/test/test_state_machine.py`:

```python
import pytest

from drivebase_behaviour.detection import Detection
from drivebase_behaviour.state_machine import (
    Config,
    Inputs,
    State,
    StateMachine,
)

CONFIG = Config(
    deadband=0.12,
    confirm_frames=5,
    confirm_frames_fallback=10,
    grasp_min=0.15,
    grasp_max=0.35,
    lost_timeout=2.0,
    approach_timeout=30.0,
    confirm_timeout=8.0,
    detection_stale_after=0.5,
    min_confidence=0.4,
)


def seen(now, error=0.0, confidence=0.9, fallback=False):
    return Detection(
        stamp_seconds=now, detected=True, frame_number=1,
        confidence=confidence, horizontal_error=error,
        range_is_fallback=fallback,
    )


def unseen(now):
    return Detection(stamp_seconds=now, detected=False, frame_number=1)


def blank(now, **kw):
    base = dict(
        now=now, detection=None, range_m=float("nan"), range_valid=False,
        in_workspace=False, nav_goal_active=True, nav_goal_succeeded=False,
        nav_goal_aborted=False, arm_sequence_done=False,
    )
    base.update(kw)
    return Inputs(**base)


def test_starts_idle_and_leaves_on_start():
    m = StateMachine(CONFIG)
    assert m.state is State.IDLE
    m.start()
    out = m.tick(blank(0.0, nav_goal_active=False))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def test_detection_interrupts_navigation_and_cancels_the_goal():
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(1.0, detection=seen(1.0)))
    assert out.state is State.APPROACHING
    assert out.cancel_nav_goal is True


def test_low_confidence_detection_does_not_interrupt():
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(1.0, detection=seen(1.0, confidence=0.2)))
    assert out.state is State.NAVIGATING


def test_stale_detection_does_not_interrupt():
    # std_msgs/String has no header, so a stamp can lag badly under load.
    # Acting on one is how the robot chases a target that is no longer there.
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(5.0, detection=seen(1.0)))
    assert out.state is State.NAVIGATING


def test_reaching_a_waypoint_requests_the_next_one():
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(1.0, nav_goal_succeeded=True, nav_goal_active=False))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def test_aborted_goal_moves_on_rather_than_wedging():
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(1.0, nav_goal_aborted=True, nav_goal_active=False))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def approach(m, t=1.0):
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    m.tick(blank(t, detection=seen(t)))
    return m


def test_entering_the_workspace_moves_to_confirming():
    m = approach(StateMachine(CONFIG))
    out = m.tick(blank(2.0, detection=seen(2.0), in_workspace=True))
    assert out.state is State.CONFIRMING


def test_losing_the_target_during_approach_resumes_the_patrol():
    m = approach(StateMachine(CONFIG))
    out = m.tick(blank(2.0, detection=unseen(2.0)))
    assert out.state is State.APPROACHING       # inside lost_timeout
    out = m.tick(blank(5.0, detection=unseen(5.0)))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def test_approach_timeout_resumes_the_patrol():
    m = approach(StateMachine(CONFIG))
    out = m.tick(blank(100.0, detection=seen(100.0)))
    assert out.state is State.NAVIGATING


def confirming(m):
    approach(m)
    m.tick(blank(2.0, detection=seen(2.0), in_workspace=True))
    return m


def test_confirmation_needs_consecutive_good_frames():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames - 1):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t), range_m=0.25,
                           range_valid=True, in_workspace=True))
        assert out.state is State.CONFIRMING
    t += 0.1
    out = m.tick(blank(t, detection=seen(t), range_m=0.25,
                       range_valid=True, in_workspace=True))
    assert out.state is State.PICKING
    assert out.store_grasp_point is True
    assert out.start_grasp is True


def test_an_off_centre_frame_resets_the_confirmation_count():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames - 1):
        t += 0.1
        m.tick(blank(t, detection=seen(t), range_m=0.25,
                     range_valid=True, in_workspace=True))
    t += 0.1
    out = m.tick(blank(t, detection=seen(t, error=0.4), range_m=0.25,
                       range_valid=True, in_workspace=True))
    assert out.state is State.CONFIRMING
    t += 0.1
    out = m.tick(blank(t, detection=seen(t), range_m=0.25,
                       range_valid=True, in_workspace=True))
    assert out.state is State.CONFIRMING


def test_an_invalid_range_never_confirms():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames + 3):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t), range_valid=False,
                           in_workspace=True))
        assert out.state is State.CONFIRMING


def test_a_range_outside_the_grasp_window_never_confirms():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames + 3):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t), range_m=0.80,
                           range_valid=True, in_workspace=True))
        assert out.state is State.CONFIRMING


def test_a_fallback_range_demands_more_frames():
    # Bbox-size distance is crude, so committing the arm on it needs more
    # evidence than a real ToF read does.
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t, fallback=True), range_m=0.25,
                           range_valid=True, in_workspace=True))
        assert out.state is State.CONFIRMING
    for _ in range(CONFIG.confirm_frames_fallback - CONFIG.confirm_frames):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t, fallback=True), range_m=0.25,
                           range_valid=True, in_workspace=True))
    assert out.state is State.PICKING


def test_confirm_timeout_falls_back_to_approaching_once_then_gives_up():
    m = confirming(StateMachine(CONFIG))
    out = m.tick(blank(20.0, detection=seen(20.0), in_workspace=True))
    assert out.state is State.APPROACHING
    out = m.tick(blank(21.0, detection=seen(21.0), in_workspace=True))
    assert out.state is State.CONFIRMING
    out = m.tick(blank(40.0, detection=seen(40.0), in_workspace=True))
    assert out.state is State.NAVIGATING


def test_grasp_completion_stows_then_resumes_the_patrol():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames):
        t += 0.1
        m.tick(blank(t, detection=seen(t), range_m=0.25,
                     range_valid=True, in_workspace=True))
    assert m.state is State.PICKING
    out = m.tick(blank(t + 1.0, arm_sequence_done=True))
    assert out.state is State.STOWING
    assert out.start_stow is True
    out = m.tick(blank(t + 2.0, arm_sequence_done=True))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def test_a_failed_pickup_never_wedges_the_mission():
    """Every terminal path must return to NAVIGATING. This is the invariant
    that keeps one bad piece of litter from ending the run."""
    for build in (
        lambda: (approach(StateMachine(CONFIG)),
                 blank(100.0, detection=unseen(100.0))),
        lambda: (confirming(StateMachine(CONFIG)),
                 blank(100.0, detection=seen(100.0), in_workspace=True)),
    ):
        m, late = build()
        for _ in range(5):
            out = m.tick(late)
        assert out.state in (State.NAVIGATING, State.APPROACHING,
                             State.CONFIRMING)
        for _ in range(20):
            out = m.tick(Inputs(**{**late.__dict__, "now": late.now + 200.0}))
        assert out.state is State.NAVIGATING
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_state_machine.py -v
```

Expected: FAIL — no module named `state_machine`.

- [ ] **Step 3: Implement**

`src/drivebase_behaviour/drivebase_behaviour/state_machine.py`:

```python
"""The mission state machine.

PURE. No rclpy, no clock, no topics - `now` arrives as a float and every
decision is a function of the Inputs it is handed. That is what lets the whole
mission be tested in milliseconds with no simulator, and it is why this file
must stay free of ROS imports.

THE INVARIANT: every state has a timeout whose fallback is resuming the patrol.
A failed pickup loses one piece of litter. It must never wedge the mission.
"""

from dataclasses import dataclass, field
from enum import Enum

from drivebase_behaviour.detection import Detection


class State(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    APPROACHING = "approaching"
    CONFIRMING = "confirming"
    PICKING = "picking"
    STOWING = "stowing"


@dataclass(frozen=True)
class Config:
    deadband: float
    confirm_frames: int
    confirm_frames_fallback: int
    grasp_min: float
    grasp_max: float
    lost_timeout: float
    approach_timeout: float
    confirm_timeout: float
    detection_stale_after: float
    min_confidence: float


@dataclass(frozen=True)
class Inputs:
    now: float
    detection: Detection | None
    range_m: float
    range_valid: bool
    in_workspace: bool
    nav_goal_active: bool
    nav_goal_succeeded: bool
    nav_goal_aborted: bool
    arm_sequence_done: bool


@dataclass(frozen=True)
class Outputs:
    state: State
    cancel_nav_goal: bool = False
    send_next_waypoint: bool = False
    store_grasp_point: bool = False
    start_grasp: bool = False
    start_stow: bool = False


@dataclass
class StateMachine:
    config: Config
    state: State = State.IDLE
    _entered_at: float = 0.0
    _last_seen_at: float = 0.0
    _confirm_count: int = 0
    _confirm_attempts: int = 0
    _started: bool = field(default=False, repr=False)

    def start(self) -> None:
        self._started = True

    def _enter(self, state: State, now: float) -> None:
        self.state = state
        self._entered_at = now
        if state is not State.CONFIRMING:
            self._confirm_count = 0
        if state is State.APPROACHING:
            self._last_seen_at = now
        if state is State.NAVIGATING:
            self._confirm_attempts = 0

    def _usable(self, inputs: Inputs) -> bool:
        """A detection worth acting on: present, confident, and recent.

        The recency test exists because std_msgs/String carries no stamp, so
        the adapter stamps on arrival and a backed-up queue looks exactly like
        a target that is still there.
        """
        d = inputs.detection
        if d is None or not d.detected:
            return False
        if d.confidence < self.config.min_confidence:
            return False
        age = inputs.now - d.stamp_seconds
        return age <= self.config.detection_stale_after

    def tick(self, inputs: Inputs) -> Outputs:
        if self.state is State.IDLE:
            if self._started:
                self._enter(State.NAVIGATING, inputs.now)
                return Outputs(state=self.state, send_next_waypoint=True)
            return Outputs(state=self.state)

        if self.state is State.NAVIGATING:
            return self._navigating(inputs)
        if self.state is State.APPROACHING:
            return self._approaching(inputs)
        if self.state is State.CONFIRMING:
            return self._confirming(inputs)
        if self.state is State.PICKING:
            return self._picking(inputs)
        return self._stowing(inputs)

    def _navigating(self, inputs: Inputs) -> Outputs:
        if self._usable(inputs):
            self._enter(State.APPROACHING, inputs.now)
            # Cancel first: controller_server must stop publishing before the
            # coordinator writes /cmd_vel_nav, or two publishers fight over it.
            return Outputs(state=self.state, cancel_nav_goal=True)

        if inputs.nav_goal_succeeded or inputs.nav_goal_aborted:
            self._enter(State.NAVIGATING, inputs.now)
            return Outputs(state=self.state, send_next_waypoint=True)

        if not inputs.nav_goal_active:
            return Outputs(state=self.state, send_next_waypoint=True)

        return Outputs(state=self.state)

    def _approaching(self, inputs: Inputs) -> Outputs:
        if inputs.now - self._entered_at > self.config.approach_timeout:
            self._enter(State.NAVIGATING, inputs.now)
            return Outputs(state=self.state, send_next_waypoint=True)

        if self._usable(inputs):
            self._last_seen_at = inputs.now
            if inputs.in_workspace:
                self._enter(State.CONFIRMING, inputs.now)
                return Outputs(state=self.state)
            return Outputs(state=self.state)

        if inputs.now - self._last_seen_at > self.config.lost_timeout:
            self._enter(State.NAVIGATING, inputs.now)
            return Outputs(state=self.state, send_next_waypoint=True)

        return Outputs(state=self.state)

    def _confirming(self, inputs: Inputs) -> Outputs:
        if inputs.now - self._entered_at > self.config.confirm_timeout:
            self._confirm_attempts += 1
            # One retry from a fresh approach; a second timeout means the
            # target is not graspable from here and the patrol matters more.
            if self._confirm_attempts >= 2:
                self._enter(State.NAVIGATING, inputs.now)
                return Outputs(state=self.state, send_next_waypoint=True)
            attempts = self._confirm_attempts
            self._enter(State.APPROACHING, inputs.now)
            self._confirm_attempts = attempts
            return Outputs(state=self.state)

        good = (
            self._usable(inputs)
            and abs(inputs.detection.horizontal_error) <= self.config.deadband
            and inputs.range_valid
            and self.config.grasp_min <= inputs.range_m <= self.config.grasp_max
        )

        if not good:
            self._confirm_count = 0
            return Outputs(state=self.state)

        self._confirm_count += 1
        needed = (
            self.config.confirm_frames_fallback
            if inputs.detection.range_is_fallback
            else self.config.confirm_frames
        )
        if self._confirm_count < needed:
            return Outputs(state=self.state)

        self._enter(State.PICKING, inputs.now)
        return Outputs(
            state=self.state, store_grasp_point=True, start_grasp=True)

    def _picking(self, inputs: Inputs) -> Outputs:
        if inputs.arm_sequence_done:
            self._enter(State.STOWING, inputs.now)
            return Outputs(state=self.state, start_stow=True)
        return Outputs(state=self.state)

    def _stowing(self, inputs: Inputs) -> Outputs:
        if inputs.arm_sequence_done:
            self._enter(State.NAVIGATING, inputs.now)
            return Outputs(state=self.state, send_next_waypoint=True)
        return Outputs(state=self.state)
```

- [ ] **Step 4: Run the tests**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -m pytest test/test_state_machine.py -v
```

Expected: all PASS.

- [ ] **Step 5: Run every unit test together**

```bash
python3 -m pytest test/ -v
```

Expected: all PASS, and the whole suite in under ~5 seconds.

- [ ] **Step 6: Commit**

```bash
git add src/drivebase_behaviour/drivebase_behaviour/state_machine.py \
        src/drivebase_behaviour/test/test_state_machine.py
git commit -m "Add the mission state machine"
```

---

### Task 13: Arm driver

**Files:**
- Create: `src/drivebase_behaviour/drivebase_behaviour/arm_driver.py`

**Interfaces:**
- Consumes: `IkSolution` from Task 10
- Produces:
  - `ArmDriver(node, wrist_roll_limit: float)`
  - `ArmDriver.go_to(pose: dict[str, float]) -> None`
  - `ArmDriver.start_sequence(steps: list[tuple[dict[str, float], float]]) -> None`
  - `ArmDriver.update(now: float) -> None`
  - `ArmDriver.sequence_done: bool`
  - `grasp_sequence(solution, search_pose, gripper_open, gripper_closed, lift_height_offset, hold_seconds) -> list[tuple[dict, float]]`

- [ ] **Step 1: Implement**

`src/drivebase_behaviour/drivebase_behaviour/arm_driver.py`:

```python
"""Publishes the six arm joint positions, and sequences timed poses.

Thin on purpose: the six gz JointPositionController plugins (and the Feetech
bus on hardware) accept a position and hold it, so there is nothing to close a
loop around here. Timing is open-loop because the descent is open-loop - by the
time the gripper is at grab range the target is occluded and inside the ToF's
minimum range. See the design doc §6.
"""

from rclpy.node import Node
from std_msgs.msg import Float64

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class ArmDriver:
    def __init__(self, node: Node, wrist_roll_limit: float) -> None:
        self.node = node
        # Software limit, tighter than the URDF's +-2.84 rad. A harness
        # carrying the pod's camera and ToF cable through wrist_roll will not
        # survive its full travel; the URDF limit describes the servo, not the
        # cable.
        self.wrist_roll_limit = abs(wrist_roll_limit)
        self.publishers = {
            j: node.create_publisher(Float64, f"/arm/{j}/position", 10)
            for j in JOINTS
        }
        self._steps: list[tuple[dict[str, float], float]] = []
        self._step_index = 0
        self._step_deadline = 0.0
        self.sequence_done = True

    def go_to(self, pose: dict[str, float]) -> None:
        for joint, value in pose.items():
            if joint not in self.publishers:
                continue
            if joint == "wrist_roll":
                value = max(-self.wrist_roll_limit,
                            min(self.wrist_roll_limit, value))
            message = Float64()
            message.data = float(value)
            self.publishers[joint].publish(message)

    def start_sequence(
        self, steps: list[tuple[dict[str, float], float]],
    ) -> None:
        self._steps = list(steps)
        self._step_index = -1
        self._step_deadline = 0.0
        self.sequence_done = not self._steps

    def update(self, now: float) -> None:
        if self.sequence_done:
            return
        if self._step_index >= 0 and now < self._step_deadline:
            return

        self._step_index += 1
        if self._step_index >= len(self._steps):
            self.sequence_done = True
            return

        pose, hold = self._steps[self._step_index]
        self.go_to(pose)
        self._step_deadline = now + hold


def grasp_sequence(
    solution,
    search_pose: dict[str, float],
    gripper_open: float,
    gripper_closed: float,
    lift_height_offset: float,
    hold_seconds: float,
) -> list[tuple[dict[str, float], float]]:
    """Pre-grasp above the target, descend, close, lift, return to search.

    Pre-grasp comes first with the gripper already open: opening it after
    arriving would sweep the jaws through the litter and push it away.
    """
    reach = {
        "shoulder_pan": solution.shoulder_pan,
        "shoulder_lift": solution.shoulder_lift,
        "elbow_flex": solution.elbow_flex,
        "wrist_flex": solution.wrist_flex,
        "wrist_roll": 0.0,
    }
    pre_grasp = dict(reach, gripper=gripper_open)
    # Approach from above by tipping shoulder_lift back; the wrist keeps its
    # solved angle so the jaws stay vertical through the descent.
    pre_grasp["shoulder_lift"] = solution.shoulder_lift - lift_height_offset

    return [
        (pre_grasp, hold_seconds),
        (dict(reach, gripper=gripper_open), hold_seconds),
        (dict(reach, gripper=gripper_closed), hold_seconds),
        (dict(pre_grasp, gripper=gripper_closed), hold_seconds),
        (dict(search_pose, gripper=gripper_closed), hold_seconds),
    ]
```

- [ ] **Step 2: Verify it imports cleanly**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -c "
from drivebase_behaviour.arm_driver import JOINTS, grasp_sequence
print(JOINTS)
class S: shoulder_pan=0.1; shoulder_lift=0.2; elbow_flex=0.3; wrist_flex=0.4
steps = grasp_sequence(S(), {'shoulder_lift': -0.6}, 0.9, 0.0, 0.3, 1.5)
assert len(steps) == 5, len(steps)
assert steps[0][0]['gripper'] == 0.9
assert steps[2][0]['gripper'] == 0.0
print('OK')
"
```

Expected: the joint tuple, then `OK`.

- [ ] **Step 3: Commit**

```bash
git add src/drivebase_behaviour/drivebase_behaviour/arm_driver.py
git commit -m "Add arm driver and the grasp sequence"
```

---

### Task 14: Detection sources

**Files:**
- Create: `src/drivebase_behaviour/drivebase_behaviour/detection_source.py`

**Interfaces:**
- Consumes: `Detection`, `parse_target_json` from Task 7
- Produces:
  - `StringJsonDetectionSource(node, topic)` and
    `TypedDetectionSource(node, topic)`, both exposing `latest: Detection | None`
  - `make_detection_source(node, interface: str, topic: str)`

- [ ] **Step 1: Implement**

`src/drivebase_behaviour/drivebase_behaviour/detection_source.py`:

```python
"""Adapters that turn whatever the detector publishes into a Detection.

The coordinator never sees JSON and never imports a message type. Swapping
trash_vision's std_msgs/String for drivebase_msgs/LitterDetection is a
parameter change here and nothing at all anywhere else.
"""

from rclpy.node import Node
from std_msgs.msg import String

from drivebase_behaviour.detection import Detection, parse_target_json


class StringJsonDetectionSource:
    """Today's interface: JSON in a std_msgs/String.

    Stamps on ARRIVAL, because std_msgs/String carries no header. Under load
    this misattributes latency - the state machine's staleness check is what
    stops that turning into servoing at a target that has already moved.
    """

    def __init__(self, node: Node, topic: str) -> None:
        self.node = node
        self.latest: Detection | None = None
        node.create_subscription(String, topic, self._on_message, 10)

    def _on_message(self, message: String) -> None:
        now = self.node.get_clock().now().nanoseconds * 1e-9
        parsed = parse_target_json(message.data, now)
        if parsed is not None:
            self.latest = parsed


class TypedDetectionSource:
    """The proposed interface. Uses the header stamp, so latency is real."""

    def __init__(self, node: Node, topic: str) -> None:
        from drivebase_msgs.msg import LitterDetection

        self.node = node
        self.latest: Detection | None = None
        node.create_subscription(LitterDetection, topic, self._on_message, 10)

    def _on_message(self, message) -> None:
        stamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1e-9
        )
        self.latest = Detection(
            stamp_seconds=stamp,
            detected=message.detected,
            frame_number=message.frame_number,
            confidence=message.confidence,
            center_x=message.center_x,
            center_y=message.center_y,
            floor_x=message.floor_x,
            floor_y=message.floor_y,
            horizontal_error=message.horizontal_error,
            bbox=(message.bbox_x1, message.bbox_y1,
                  message.bbox_x2, message.bbox_y2),
        )


def make_detection_source(node: Node, interface: str, topic: str):
    if interface == "typed":
        return TypedDetectionSource(node, topic)
    if interface == "json_string":
        return StringJsonDetectionSource(node, topic)
    raise ValueError(
        f"unknown detection_interface '{interface}'; "
        "expected 'json_string' or 'typed'")
```

- [ ] **Step 2: Verify the selector rejects a bad value**

```bash
cd "$(git rev-parse --show-toplevel)/src/drivebase_behaviour"
python3 -c "
from drivebase_behaviour.detection_source import make_detection_source
try:
    make_detection_source(None, 'nonsense', '/x')
except ValueError as e:
    print('rejected:', e)
else:
    raise SystemExit('should have rejected')
"
```

Expected: `rejected: unknown detection_interface 'nonsense'; ...`

- [ ] **Step 3: Commit**

```bash
git add src/drivebase_behaviour/drivebase_behaviour/detection_source.py
git commit -m "Add detection source adapters"
```

---

### Task 15: The coordinator node, config and launch

**Files:**
- Create: `src/drivebase_behaviour/drivebase_behaviour/coordinator_node.py`
- Create: `src/drivebase_behaviour/config/coordinator.yaml`
- Create: `src/drivebase_behaviour/launch/coordinator.launch.py`

**Interfaces:**
- Consumes: every module from Tasks 7-14
- Produces: the `coordinator` executable

- [ ] **Step 1: Write the config**

`src/drivebase_behaviour/config/coordinator.yaml`. **Replace every value
marked `# MEASURED` with the figure from `docs/arm_workspace.md` (Task 5).**

```yaml
/**:
  ros__parameters:
    # Which detector interface to consume. Change to "typed" once
    # trash_vision publishes drivebase_msgs/LitterDetection.
    detection_interface: "json_string"
    detection_topic: "/vision/target"

    camera_info_topic: "/pod_camera/camera_info"
    tof_topic: "/tof/pod"
    camera_optical_frame: "pod_camera_optical_frame"
    static_frame: "odom"
    arm_base_frame: "arm_base_link"

    tick_hz: 10.0

    # Patrol. Flattened [x, y, x, y, ...] in the static frame - ROS 2 params
    # have no nested-array type.
    waypoints: [2.0, 0.0, 4.0, 1.5, 2.0, 3.0, 0.0, 1.5]
    loop_patrol: true

    # Detection gating.
    min_confidence: 0.4
    deadband: 0.12
    detection_stale_after: 0.5

    # Confirmation gate. Fallback ranges are crude, so they need more frames.
    confirm_frames: 5
    confirm_frames_fallback: 10

    # ToF limits, matching the gpu_lidar in gazebo.xacro.
    tof_min_range: 0.04
    tof_max_range: 4.0

    # Fallback range calibration. Both figures come from one observation of a
    # litter model at a known distance.
    fallback_reference_height_px: 60.0   # MEASURED
    fallback_reference_range_m: 1.0      # MEASURED

    # Grasp window.
    grasp_min: 0.15                      # MEASURED
    grasp_max: 0.35                      # MEASURED
    workspace_radius: 0.45               # MEASURED

    # Timeouts. Every one of these falls back to resuming the patrol.
    lost_timeout: 2.0
    approach_timeout: 30.0
    confirm_timeout: 8.0

    # Approach servo. linear_speed is never zero while correcting heading:
    # arcs, not point turns. RPP and the power budget both prefer arcs, and the
    # base cannot reliably point-turn more than once per run (STATUS.md #1).
    approach_linear_speed: 0.22
    approach_min_linear_speed: 0.06
    approach_angular_gain: 1.2
    approach_max_angular_speed: 0.8

    # Arm poses.
    search_pose: [0.0, -0.60, 0.80, 0.40, 0.0, 0.0]   # MEASURED
    stow_pose: [0.0, -1.20, 1.55, 1.10, 0.0, 0.0]
    gripper_open: 1.2
    gripper_closed: 0.0
    grasp_lift_offset: 0.35
    grasp_hold_seconds: 1.5
    # Tighter than the URDF's +-2.84: protects the pod harness.
    wrist_roll_limit: 1.0                # MEASURED
    # Straight down onto the litter.
    approach_angle: -1.5708
```

- [ ] **Step 2: Implement the node**

`src/drivebase_behaviour/drivebase_behaviour/coordinator_node.py`:

```python
"""The coordinator: ticks the state machine and wires it to ROS.

Deliberately thin. Every decision lives in state_machine.py, every piece of
geometry in localisation.py and ik.py, all of which are pure and tested without
a simulator. This file only gathers inputs, applies outputs, and owns the one
thing that cannot be pure: talking to Nav2, tf and the wheels.

VELOCITY OWNERSHIP. During APPROACHING this node publishes to /cmd_vel_nav -
the same topic controller_server uses - and it cancels the Nav2 goal BEFORE
transitioning. controller_server then stops publishing, so there is exactly one
publisher at any instant, and the approach still inherits velocity_smoother and
collision_monitor. Writing /cmd_vel directly would skip both, losing ultrasonic
collision safety exactly when the robot is closest to something.
"""

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Range

from drivebase_behaviour.arm_driver import ArmDriver, grasp_sequence
from drivebase_behaviour.detection_source import make_detection_source
from drivebase_behaviour.ik import JOINT_LIMITS, solve
from drivebase_behaviour.kinematics import extract_planar_arm, load_chain
from drivebase_behaviour.localisation import (
    fallback_range_from_bbox,
    pixel_to_unit_ray,
    point_from_range,
    range_is_plausible,
)
from drivebase_behaviour.state_machine import (
    Config,
    Inputs,
    State,
    StateMachine,
)

ARM_JOINT_ORDER = (
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
)


class Coordinator(Node):
    def __init__(self) -> None:
        super().__init__("behaviour_coordinator")

        self._declare_parameters()
        self.config = Config(
            deadband=self.p("deadband"),
            confirm_frames=int(self.p("confirm_frames")),
            confirm_frames_fallback=int(self.p("confirm_frames_fallback")),
            grasp_min=self.p("grasp_min"),
            grasp_max=self.p("grasp_max"),
            lost_timeout=self.p("lost_timeout"),
            approach_timeout=self.p("approach_timeout"),
            confirm_timeout=self.p("confirm_timeout"),
            detection_stale_after=self.p("detection_stale_after"),
            min_confidence=self.p("min_confidence"),
        )
        self.machine = StateMachine(self.config)

        flat = list(self.get_parameter("waypoints").value)
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        if not self.waypoints:
            raise RuntimeError("no waypoints configured")
        self.waypoint_index = -1

        self.detections = make_detection_source(
            self,
            str(self.get_parameter("detection_interface").value),
            str(self.get_parameter("detection_topic").value),
        )

        self.camera_info: CameraInfo | None = None
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info, qos_profile_sensor_data)

        self.tof: Range | None = None
        self.create_subscription(
            Range, str(self.get_parameter("tof_topic").value),
            self._on_tof, qos_profile_sensor_data)

        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.nav_goal_handle = None
        self.nav_succeeded = False
        self.nav_aborted = False

        self.arm = ArmDriver(self, self.p("wrist_roll_limit"))
        self.planar_arm = self._load_arm_model()

        self.stored_grasp_point: tuple[float, float, float] | None = None

        self.arm.go_to(self._pose("search_pose"))
        self.machine.start()

        period = 1.0 / self.p("tick_hz")
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Coordinator started with {len(self.waypoints)} waypoints")

    # ---------- parameters -------------------------------------------------

    def _declare_parameters(self) -> None:
        defaults = {
            "detection_interface": "json_string",
            "detection_topic": "/vision/target",
            "camera_info_topic": "/pod_camera/camera_info",
            "tof_topic": "/tof/pod",
            "camera_optical_frame": "pod_camera_optical_frame",
            "static_frame": "odom",
            "arm_base_frame": "arm_base_link",
            "tick_hz": 10.0,
            "waypoints": [2.0, 0.0, 4.0, 1.5, 2.0, 3.0, 0.0, 1.5],
            "loop_patrol": True,
            "min_confidence": 0.4,
            "deadband": 0.12,
            "detection_stale_after": 0.5,
            "confirm_frames": 5,
            "confirm_frames_fallback": 10,
            "tof_min_range": 0.04,
            "tof_max_range": 4.0,
            "fallback_reference_height_px": 60.0,
            "fallback_reference_range_m": 1.0,
            "grasp_min": 0.15,
            "grasp_max": 0.35,
            "workspace_radius": 0.45,
            "lost_timeout": 2.0,
            "approach_timeout": 30.0,
            "confirm_timeout": 8.0,
            "approach_linear_speed": 0.22,
            "approach_min_linear_speed": 0.06,
            "approach_angular_gain": 1.2,
            "approach_max_angular_speed": 0.8,
            "search_pose": [0.0, -0.60, 0.80, 0.40, 0.0, 0.0],
            "stow_pose": [0.0, -1.20, 1.55, 1.10, 0.0, 0.0],
            "gripper_open": 1.2,
            "gripper_closed": 0.0,
            "grasp_lift_offset": 0.35,
            "grasp_hold_seconds": 1.5,
            "wrist_roll_limit": 1.0,
            "approach_angle": -1.5708,
            "robot_description": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _pose(self, name: str) -> dict[str, float]:
        values = list(self.get_parameter(name).value)
        return dict(zip(ARM_JOINT_ORDER, values))

    # ---------- setup ------------------------------------------------------

    def _load_arm_model(self):
        """Reads the arm's geometry from /robot_description.

        Blocking on purpose: without the URDF there is no IK, and starting
        anyway would mean discovering that at the moment the arm is asked to
        grasp something.
        """
        urdf = str(self.get_parameter("robot_description").value)
        if not urdf:
            raise RuntimeError(
                "robot_description parameter is empty; the coordinator cannot "
                "derive arm kinematics. Pass it from the launch file.")
        chain = load_chain(
            urdf,
            str(self.get_parameter("arm_base_frame").value),
            "arm_gripper_frame_link",
        )
        return extract_planar_arm(chain)

    # ---------- subscriptions ---------------------------------------------

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _on_tof(self, message: Range) -> None:
        self.tof = message

    # ---------- the tick ---------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_point(self):
        """Camera ray times ToF range, in the camera optical frame.

        Returns (point, used_fallback) or (None, False).
        """
        detection = self.detections.latest
        if detection is None or not detection.detected:
            return None, False
        if self.camera_info is None:
            return None, False

        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx == 0.0 or fy == 0.0:
            return None, False

        u = detection.center_x * self.camera_info.width
        v = detection.center_y * self.camera_info.height
        ray = pixel_to_unit_ray(u, v, fx, fy, cx, cy)

        used_fallback = False
        range_m = self.tof.range if self.tof is not None else float("nan")
        if not range_is_plausible(
            range_m, self.p("tof_min_range"), self.p("tof_max_range")
        ):
            bbox_height = detection.bbox[3] - detection.bbox[1]
            range_m = fallback_range_from_bbox(
                bbox_height,
                float(self.camera_info.height),
                self.p("fallback_reference_height_px"),
                self.p("fallback_reference_range_m"),
            )
            used_fallback = True
            if math.isnan(range_m):
                return None, False

        return point_from_range(ray, range_m), used_fallback

    def _tick(self) -> None:
        now = self._now()
        self.arm.update(now)

        point, used_fallback = self._target_point()
        range_m = (
            math.sqrt(sum(c * c for c in point)) if point else float("nan")
        )
        range_valid = point is not None

        detection = self.detections.latest
        if detection is not None and used_fallback:
            # Carry the fallback flag into the gate, which demands more frames
            # for it.
            from dataclasses import replace
            detection = replace(detection, range_is_fallback=True)

        in_workspace = (
            range_valid and range_m <= self.p("workspace_radius")
        )

        outputs = self.machine.tick(Inputs(
            now=now,
            detection=detection,
            range_m=range_m,
            range_valid=range_valid,
            in_workspace=in_workspace,
            nav_goal_active=self.nav_goal_handle is not None,
            nav_goal_succeeded=self.nav_succeeded,
            nav_goal_aborted=self.nav_aborted,
            arm_sequence_done=self.arm.sequence_done,
        ))
        self.nav_succeeded = False
        self.nav_aborted = False

        if outputs.cancel_nav_goal:
            self._cancel_nav_goal()
        if outputs.send_next_waypoint:
            self._send_next_waypoint()
        if outputs.store_grasp_point and point is not None:
            self._store_and_grasp(point)
        if outputs.start_stow:
            self.arm.start_sequence(
                [(self._pose("search_pose"), self.p("grasp_hold_seconds"))])

        if outputs.state is State.APPROACHING:
            self._servo(detection)
        elif outputs.state in (State.CONFIRMING, State.PICKING, State.STOWING):
            self.cmd_publisher.publish(Twist())
            if outputs.state is State.CONFIRMING:
                self._aim_arm(detection)

    # ---------- actions ----------------------------------------------------

    def _servo(self, detection) -> None:
        """Arcs, never point turns.

        linear.x never drops to zero while correcting heading. The base cannot
        reliably rotate in place more than once per run (STATUS.md #1), and RPP
        and the power budget both prefer arcs anyway.
        """
        command = Twist()
        if detection is None or not detection.detected:
            command.linear.x = self.p("approach_min_linear_speed")
            self.cmd_publisher.publish(command)
            return

        error = detection.horizontal_error
        angular = -self.p("approach_angular_gain") * error
        limit = self.p("approach_max_angular_speed")
        command.angular.z = max(-limit, min(limit, angular))

        # Slow down when badly off heading, but never stop.
        straightness = max(0.0, 1.0 - abs(error) / 0.5)
        fast, slow = (self.p("approach_linear_speed"),
                      self.p("approach_min_linear_speed"))
        command.linear.x = slow + (fast - slow) * straightness

        self.cmd_publisher.publish(command)

    def _aim_arm(self, detection) -> None:
        """Bearing first. A single-point ToF only means anything once it is
        pointed at the target - reading range before centring measures the
        ground."""
        if detection is None or not detection.detected:
            return
        pose = self._pose("search_pose")
        pose["shoulder_pan"] = (
            pose.get("shoulder_pan", 0.0)
            - detection.horizontal_error * 1.0
        )
        low, high = JOINT_LIMITS["shoulder_pan"]
        pose["shoulder_pan"] = max(low, min(high, pose["shoulder_pan"]))
        self.arm.go_to(pose)

    def _store_and_grasp(self, point) -> None:
        """Freeze the target in a static frame, then grasp open-loop.

        Everything from here is blind: the jaws occlude the target and it is
        inside the ToF's minimum range. See the design doc §6.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("arm_base_frame").value),
                str(self.get_parameter("camera_optical_frame").value),
                rclpy.time.Time(),
            )
        except tf2_ros.TransformException as error:
            self.get_logger().warning(f"tf lookup failed, skipping grasp: {error}")
            self.arm.start_sequence([])
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        target = _apply_transform(point, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

        solution = solve(
            self.planar_arm, target, self.p("approach_angle"), JOINT_LIMITS)
        if solution is None:
            self.get_logger().warning(
                f"target {target} unreachable, abandoning this piece")
            self.arm.start_sequence([])
            return

        self.stored_grasp_point = target
        self.arm.start_sequence(grasp_sequence(
            solution,
            self._pose("search_pose"),
            self.p("gripper_open"),
            self.p("gripper_closed"),
            self.p("grasp_lift_offset"),
            self.p("grasp_hold_seconds"),
        ))

    def _send_next_waypoint(self) -> None:
        self.waypoint_index += 1
        if self.waypoint_index >= len(self.waypoints):
            if not bool(self.get_parameter("loop_patrol").value):
                self.get_logger().info("Patrol complete")
                return
            self.waypoint_index = 0

        x, y = self.waypoints[self.waypoint_index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(
            self.get_parameter("static_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning("Nav2 action server unavailable")
            return

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(f"Waypoint {self.waypoint_index}: ({x}, {y})")

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.nav_aborted = True
            return
        self.nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        status = future.result().status
        self.nav_goal_handle = None
        # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus.
        if status == 4:
            self.nav_succeeded = True
        else:
            self.nav_aborted = True

    def _cancel_nav_goal(self) -> None:
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None


def _apply_transform(point, translation, quaternion):
    x, y, z, w = quaternion
    px, py, pz = point
    # q * p * q^-1, expanded.
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    rx = px + w * tx + (y * tz - z * ty)
    ry = py + w * ty + (z * tx - x * tz)
    rz = pz + w * tz + (x * ty - y * tx)
    return (rx + translation[0], ry + translation[1], rz + translation[2])


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    node = Coordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Write the launch file**

`src/drivebase_behaviour/launch/coordinator.launch.py`:

```python
"""Bring up the behaviour coordinator.

    ros2 launch drivebase_behaviour coordinator.launch.py use_sim_time:=true

Expects drivebase_sim (or the real robot) and drivebase_navigation to already
be running. This launch file publishes no transforms and starts no Nav2 nodes.

robot_description is passed as a parameter rather than read from a topic: the
coordinator needs the arm's geometry before it can accept a grasp, and a node
that starts without it would fail at the worst possible moment.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    params = PathJoinSubstitution([
        FindPackageShare("drivebase_behaviour"), "config", "coordinator.yaml",
    ])
    xacro_file = PathJoinSubstitution([
        FindPackageShare("drivebase_description"),
        "urdf", "drivebase.urdf.xacro",
    ])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_sim:=true", " use_arm:=true"]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Set true when running against the simulator.",
        ),
        Node(
            package="drivebase_behaviour",
            executable="coordinator",
            name="behaviour_coordinator",
            output="screen",
            parameters=[
                params,
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description,
                },
            ],
        ),
    ])
```

- [ ] **Step 4: Build and verify the node starts**

```bash
cd "$(git rev-parse --show-toplevel)"
colcon build --symlink-install --packages-select drivebase_msgs drivebase_behaviour
source install/setup.bash
ros2 launch drivebase_behaviour coordinator.launch.py use_sim_time:=true &
sleep 8
ros2 node list | grep behaviour_coordinator
ros2 param get /behaviour_coordinator detection_interface
kill %1
```

Expected: node listed, parameter reads `json_string`. It will warn that the
Nav2 server is unavailable — correct, nothing else is running yet.

- [ ] **Step 5: Commit**

```bash
git add src/drivebase_behaviour/drivebase_behaviour/coordinator_node.py \
        src/drivebase_behaviour/config/coordinator.yaml \
        src/drivebase_behaviour/launch/coordinator.launch.py
git commit -m "Add the coordinator node, config and launch"
```

---

### Task 16: End-to-end run in simulation

**Files:** none created; this validates Tasks 1-15.

- [ ] **Step 1: Bring the whole stack up**

```bash
cd "$(git rev-parse --show-toplevel)"
colcon build && source install/setup.bash

ros2 launch drivebase_sim sim.launch.py rviz:=true &
sleep 25
ros2 launch drivebase_navigation navigation.launch.py use_sim_time:=true &
sleep 15
ros2 launch drivebase_behaviour coordinator.launch.py use_sim_time:=true &
```

- [ ] **Step 2: Check the interface wiring before judging behaviour**

```bash
# This project's recurring silent failure is a topic carrying two types.
ros2 topic info /cmd_vel
ros2 topic info /cmd_vel_nav
ros2 topic info /vision/target
ros2 topic hz /pod_camera/image_raw --window 20
ros2 topic hz /tof/pod --window 20
```

Expected: one type per topic; camera near 15 Hz; ToF near 20 Hz. **A topic with
two types means stop and fix that first** — everything downstream will look
broken for the wrong reason.

- [ ] **Step 3: Watch a full cycle**

```bash
ros2 topic echo /vision/direction &
ros2 run tf2_ros tf2_echo odom base_footprint &
```

In the coordinator's log, expect the sequence: `Waypoint 0` → detection →
approach (base slows and arcs) → confirming (base stops, `shoulder_pan` moves)
→ grasp sequence → back to a waypoint.

- [ ] **Step 4: Record what actually happened**

Note, for `docs/STATUS.md` in Task 17:

- Did the base reach a waypoint? Did detection interrupt it?
- Did the approach stay in arcs, or did it attempt a point turn?
- Did confirmation ever pass? How many attempts?
- Did IK return a solution, or log "unreachable"?
- Did the gripper close on the litter, and did the litter move?
- Control-loop rate: `ros2 topic hz /cmd_vel_nav`.

**Report failures honestly.** Simulated contact between a box-collision gripper
and a small object is unreliable; a failed grasp with a successful approach is
a real and useful result, not something to tune away.

- [ ] **Step 5: Shut down**

```bash
kill %1 %2 %3 2>/dev/null
```

- [ ] **Step 6: Commit any fixes found**

```bash
git add -A && git commit -m "Fix issues found in end-to-end simulation run"
```

---

# Phase 5 — Documentation

---

### Task 17: Update `STATUS.md` and `README.md`

**Files:**
- Modify: `docs/STATUS.md`
- Modify: `README.md`

- [ ] **Step 1: Update the state table**

In `docs/STATUS.md`, change these rows and add the new ones:

```markdown
| Behaviour coordinator | Done — patrol, detect, approach, confirm, grasp |
| Eye-in-hand camera + ToF | Done in sim; hardware pod not built |
| Arm command interface | Done — six joints bridged, IK from URDF |
| `trash_vision` restructuring | Not started (partner's package; proposal in the coordinator design doc) |
```

- [ ] **Step 2: Add the new measured figures**

Add to the "Measured figures" table, using values from `docs/arm_workspace.md`
and the Task 16 run. **Only include figures that were actually measured.**

```markdown
| Arm forward reach | | `arm_workspace.md` |
| Grasp window | | `arm_workspace.md` |
| Camera→ToF agreement | | Task 16 run |
```

- [ ] **Step 3: Amend the rotation-bug entry**

Known issue #1 says the bug blocks rotation-heavy behaviour. That is now
narrower. Append to it:

```markdown
**Update.** This no longer blocks pickup. With the camera eye-in-hand,
`shoulder_pan` aims and the base only has to park the litter inside the arm's
workspace — a ~200 mm target rather than ~40 mm, which it can hit despite the
0.35 m turn drift. The approach servo also steers in arcs and never commands a
point turn. The bug still blocks multi-turn drift measurement and Nav2 spin
recovery.
```

- [ ] **Step 4: Amend the "one thing to know" in `README.md`**

That section says the arm must be aimed by the camera. That is now implemented,
not aspirational. Update it to say so, and add `drivebase_behaviour` and
`drivebase_msgs` to the package table:

```markdown
| `drivebase_msgs` | Typed interfaces, incl. the proposed `LitterDetection` |
| `drivebase_behaviour` | Mission coordinator: patrol, detect, approach, grasp |
```

Add to the run instructions:

```bash
ros2 launch drivebase_behaviour coordinator.launch.py use_sim_time:=true  # 3rd terminal
```

- [ ] **Step 5: Update the next-steps list**

Step 6 (behaviour coordinator) is done. Step 5 (`trash_vision`) is unchanged
and still belongs to its author. Renumber, and note that the coordinator
consumes the current String interface through an adapter, so the typed message
can land whenever its author is ready.

- [ ] **Step 6: Commit**

```bash
git add docs/STATUS.md README.md
git commit -m "Update status and README for the behaviour coordinator"
```

---

## Self-review notes

Checked against the spec:

| Spec section | Implemented by |
|---|---|
| §1 no camera in sim | Tasks 2, 3 |
| §1 arm not commandable | Task 3 |
| §2 hand-rolled FSM | Task 12 |
| §2 waypoint patrol | Task 15 |
| §2 `/cmd_vel_nav`, cancel first | Task 12 `_navigating`, Task 15 `_servo` |
| §2 arcs, never point turns | Task 15 `_servo`, config `approach_min_linear_speed` |
| §2 eye-in-hand + search pose | Tasks 1, 5, 15 |
| §2 pod before `wrist_roll` | Task 1 |
| §2 closed-form IK | Task 10 |
| §4 co-location, extrinsics | Task 1 |
| §4 `wrist_roll` software limit | Task 13 `ArmDriver` |
| §5 ray × range | Task 8 |
| §5 bearing-first | Task 15 `_aim_arm` |
| §5 ToF fallback | Tasks 8, 12, 15 |
| §6 open-loop grasp from stored point | Task 15 `_store_and_grasp` |
| §7 all six states + timeouts | Task 12 |
| §8 adapter, both sources | Tasks 7, 14 |
| §9 typed message proposal | Task 6 |
| §10 unit + integration tests | Tasks 7-12, 16 |
| §11 phasing | Phase structure |
| §12 measure, don't assume | Task 5, config `# MEASURED` markers |
| §13 risks | Task 16 Step 4, Task 17 |
