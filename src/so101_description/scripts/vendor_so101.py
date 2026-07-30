#!/usr/bin/env python3
"""Convert the upstream SO-101 URDF into a mountable xacro macro.

Upstream ships a standalone robot whose root link is called `base_link` — the
same name the drivebase uses. Including it directly would collide, so every link
and joint gets a `${prefix}`.

Two other transformations happen here:

  * Mesh paths are rewritten to package:// URIs.
  * Collision geometry is replaced with an axis-aligned box fitted to each STL's
    bounding volume. The visual meshes are 0.5-2.6 MB each; using them for
    collision as well makes Gazebo compute mesh-mesh contacts every step, which
    costs far more real-time factor than the accuracy is worth for a drivebase
    navigation simulation. Boxes are conservative (they never under-approximate),
    so obstacle avoidance stays safe.

Re-run after pulling a newer upstream URDF:

    python3 scripts/vendor_so101.py

Writes urdf/so101_macro.xacro, which is the file the rest of the project uses.
"""

import math
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent.parent
UPSTREAM = HERE / "urdf" / "so101_upstream.urdf"
MESHES = HERE / "meshes"
OUT = HERE / "urdf" / "so101_macro.xacro"

UPSTREAM_URL = (
    "https://github.com/TheRobotStudio/SO-ARM100/blob/main/"
    "Simulation/SO101/so101_new_calib.urdf"
)


def stl_bounds(path):
    """Return ((min_x,min_y,min_z),(max_x,max_y,max_z)) for a binary or ASCII STL."""
    raw = path.read_bytes()
    lo = [math.inf] * 3
    hi = [-math.inf] * 3

    def note(x, y, z):
        for i, v in enumerate((x, y, z)):
            lo[i] = min(lo[i], v)
            hi[i] = max(hi[i], v)

    is_ascii = raw[:5].lower() == b"solid" and b"facet" in raw[:2048]
    if is_ascii:
        for line in raw.decode("utf-8", "replace").splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                note(*(float(p) for p in parts[1:]))
    else:
        count = struct.unpack("<I", raw[80:84])[0]
        offset = 84
        for _ in range(count):
            # 12 floats per triangle: normal then 3 vertices, + 2 pad bytes
            vals = struct.unpack_from("<12f", raw, offset)
            for v in range(3):
                note(*vals[3 + v * 3: 6 + v * 3])
            offset += 50
    return tuple(lo), tuple(hi)


def rotate(rpy, vec):
    """Apply URDF rpy (Rz*Ry*Rx) to a vector."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    m = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]
    return [sum(m[i][j] * vec[j] for j in range(3)) for i in range(3)]


def floats(text, default=(0.0, 0.0, 0.0)):
    if not text:
        return list(default)
    return [float(v) for v in text.split()]


def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)


def main():
    if not UPSTREAM.exists():
        sys.exit(f"missing {UPSTREAM} — see {UPSTREAM_URL}")

    tree = ET.parse(UPSTREAM)
    root = tree.getroot()

    for link in root.findall("link"):
        link.set("name", "${prefix}" + link.get("name"))

        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            stl = MESHES / pathlib.Path(mesh.get("filename")).name
            if not stl.exists():
                sys.exit(f"missing mesh {stl}")

            scale = floats(mesh.get("scale"), (1.0, 1.0, 1.0))
            lo, hi = stl_bounds(stl)
            size = [(hi[i] - lo[i]) * abs(scale[i]) for i in range(3)]
            centre = [(hi[i] + lo[i]) / 2.0 * scale[i] for i in range(3)]

            origin = collision.find("origin")
            if origin is None:
                origin = ET.SubElement(collision, "origin")
            xyz = floats(origin.get("xyz"))
            rpy = floats(origin.get("rpy"))

            # Box sits at the mesh's bbox centre expressed in the collision frame.
            shifted = rotate(rpy, centre)
            origin.set("xyz", fmt([xyz[i] + shifted[i] for i in range(3)]))
            origin.set("rpy", fmt(rpy))

            geometry = collision.find("geometry")
            geometry.remove(mesh)
            ET.SubElement(geometry, "box", {"size": fmt(size)})

        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                name = pathlib.Path(mesh.get("filename")).name
                mesh.set("filename", f"package://so101_description/meshes/{name}")

    for joint in root.findall("joint"):
        joint.set("name", "${prefix}" + joint.get("name"))
        for tag in ("parent", "child"):
            node = joint.find(tag)
            if node is not None:
                node.set("link", "${prefix}" + node.get("link"))

    body = "".join(ET.tostring(child, encoding="unicode") for child in root)

    OUT.write_text(f'''<?xml version="1.0"?>
<!--
  GENERATED by scripts/vendor_so101.py — do not edit by hand.
  Source: {UPSTREAM_URL}

  Collision geometry is boxes fitted to each mesh's bounding volume, not the
  meshes themselves; see the generator for why.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Attaches the arm to `parent`. Pass the mount transform as a nested
       <origin/> block:

         <xacro:so101 prefix="arm_" parent="arm_mount_link">
           <origin xyz="0 0 0" rpy="0 0 0"/>
         </xacro:so101>
  -->
  <xacro:macro name="so101" params="prefix parent *origin">

    <!-- `base_attach_joint`, not `mount_joint`: the drivebase already owns a
         joint called arm_mount_joint for the mounting plate itself, and URDF
         joint names must be globally unique. -->
    <joint name="${{prefix}}base_attach_joint" type="fixed">
      <parent link="${{parent}}"/>
      <child link="${{prefix}}base_link"/>
      <xacro:insert_block name="origin"/>
    </joint>
{body}
  </xacro:macro>
</robot>
''')
    print(f"wrote {OUT.relative_to(HERE)}")


if __name__ == "__main__":
    main()
