# so101_description

SO-101 arm description, vendored from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(`Simulation/SO101/so101_new_calib.urdf`), wrapped as a mountable xacro macro.

## Use

```xml
<xacro:include filename="$(find so101_description)/urdf/so101_macro.xacro"/>

<xacro:so101 prefix="arm_" parent="arm_mount_link">
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:so101>
```

Every link and joint takes the `prefix`, because upstream's root link is called
`base_link` — the same name the drivebase uses.

## Files

| Path | |
|---|---|
| `urdf/so101_upstream.urdf` | untouched upstream copy, for diffing against new releases |
| `urdf/so101_macro.xacro` | **generated** — the file to actually include |
| `urdf/so101.ros2_control.xacro` | `ros2_control` block; mock or `gz_ros2_control` backend |
| `urdf/so101.urdf.xacro` | standalone top-level, arm on a `world` link |
| `config/so101_controllers.yaml` | `joint_state_broadcaster` + `arm_controller` |
| `launch/` | `display`, `bringup` (mock), `gazebo` |
| `scripts/vendor_so101.py` | regenerates the macro from upstream |
| `meshes/*.stl` | 13 visual meshes, ~15 MB |

Do not hand-edit `so101_macro.xacro`. To pick up an upstream change, replace
`so101_upstream.urdf` and re-run `python3 scripts/vendor_so101.py`.

## Control

The arm on the robot is built by `drivebase_description`, which calls the macro
directly. `urdf/so101.urdf.xacro` exists to bring the arm and its controllers up
*without* the drivebase, for debugging.

```bash
# mock hardware — no physics, commanded positions mirrored back
ros2 launch so101_description bringup.launch.py

# Gazebo Sim; controller_manager runs inside the Gazebo process
ros2 launch so101_description gazebo.launch.py gui:=false
```

Do not combine the two launch files — each starts its own
`robot_state_publisher`, and `gazebo.launch.py` gets its `controller_manager`
from the `gz_ros2_control` plugin rather than a standalone node.

**The joint names in `config/so101_controllers.yaml` are prefixed `arm_`, and
that is load-bearing.** The macro namespaces every joint, and the drivebase
instantiates it as `prefix="arm_"`. A config with bare joint names works
standalone and then binds to nothing once the arm is on the chassis — the
controller spawns, reports `active`, and no joint moves. There is deliberately
one config file, so `so101.urdf.xacro` defaults to `prefix:=arm_` too. Change
the prefix and you must change it in three places: the `so101` macro call, the
`so101_ros2_control` macro call, and the joint list in the YAML.

Verified on mock hardware and in Gazebo:

```bash
ros2 control list_controllers          # both active
ros2 control list_hardware_interfaces  # 6 command interfaces [available] [claimed]
```

`/joint_states` orders joints **alphabetically**, not as listed in the YAML:
`arm_elbow_flex, arm_gripper, arm_shoulder_lift, arm_shoulder_pan,
arm_wrist_flex, arm_wrist_roll`.

Effort and velocity limits are upstream placeholders, not STS3215 datasheet
values, and there is no real-hardware interface yet — a Feetech STS3215
`<hardware>` block is the third backend to add.

## Deviations from upstream

**Collision geometry is boxes, not meshes.** The visual meshes are 0.5–2.6 MB
each; using them for collision makes Gazebo compute mesh-mesh contacts every
physics step, which costs more real-time factor than the fidelity is worth for
drivebase navigation work. The generator fits an axis-aligned box to each mesh's
bounding volume. Boxes never under-approximate, so avoidance stays conservative.

Revisit this if the arm team needs accurate self-collision or grasp contact —
that is the point at which mesh collision starts earning its cost.

## Notes

Total mass is **0.632 kg** across 8 links — considerably lighter than a typical
guess for an arm this size, which matters for the drivebase tipping calculation
in `docs/power_budget.md`.

Joint limits come from upstream and are in radians:

| Joint | Lower | Upper |
|---|---|---|
| `shoulder_pan` | −1.920 | 1.920 |
| `shoulder_lift` | −1.745 | 1.745 |
| `elbow_flex` | −1.690 | 1.690 |
| `wrist_flex` | −1.658 | 1.658 |
| `wrist_roll` | −2.744 | 2.841 |
| `gripper` | −0.175 | 1.745 |
