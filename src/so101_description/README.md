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
| `scripts/vendor_so101.py` | regenerates the macro from upstream |
| `meshes/*.stl` | 13 visual meshes, ~15 MB |

Do not hand-edit `so101_macro.xacro`. To pick up an upstream change, replace
`so101_upstream.urdf` and re-run `python3 scripts/vendor_so101.py`.

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
