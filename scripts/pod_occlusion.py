#!/usr/bin/env python3
"""Ray-cast the wrist and gripper visual meshes to find where the pod can see.

    python3 scripts/pod_occlusion.py

WHY. Task 5b aimed the pod along the gripper approach axis and the ToF then
read a constant 0.058 m at every arm pose, INCLUDING poses whose ray points at
open sky - so the beam never leaves the robot. The sim proves the blockage
exists but cannot say which casting causes it or how far the pod must move to
clear it, because mount geometry is fixed at launch and each candidate costs a
whole Gazebo session.

This does that part offline. Everything is exact triangle geometry from the
same STLs the gpu_lidar renders (it renders visuals, not the collision boxes),
so a "clear" verdict here is a prediction to be confirmed in the sim, not a
substitute for it.

All frames are arm_wrist_link, at wrist_roll = 0. The gripper opening is swept
because the moving jaw is the one obstruction that is not rigid.
"""

from __future__ import annotations

import math
import struct
import sys

import numpy as np

MESH_DIR = "src/so101_description/meshes"


def rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def transform(xyz, r, p, y) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rpy(r, p, y)
    m[:3, 3] = xyz
    return m


def load_stl(name: str) -> np.ndarray:
    """Binary STL -> (n, 3, 3) triangle vertex array."""
    with open(f"{MESH_DIR}/{name}", "rb") as fh:
        fh.read(80)
        count = struct.unpack("<I", fh.read(4))[0]
        raw = np.frombuffer(fh.read(count * 50), dtype=np.uint8)
    raw = raw.reshape(count, 50)
    floats = raw[:, :48].copy().view(np.float32).reshape(count, 4, 3)
    return floats[:, 1:, :].astype(np.float64)  # drop the normal


# Visual geometry downstream of (and including) arm_wrist_link, as
# (mesh, origin xyz, origin rpy, parent chain transform).
WRIST_ROLL = transform((0.0, -0.0611, 0.0181), 1.5708, 0.0486795, 3.14159)
GRIPPER_JOINT = transform((0.0202, 0.0188, -0.0234), 1.5708, 0.0, 0.0)

PARTS = [
    ("wrist", "sts3215_03a_no_horn_v1.stl",
     (0.0, -0.0424, 0.0306), (1.5708, 1.5708, 0.0), np.eye(4)),
    ("wrist", "wrist_roll_pitch_so101_v2.stl",
     (0.0, -0.028, 0.0181), (-1.5708, -1.5708, 0.0), np.eye(4)),
    ("gripper", "sts3215_03a_v1.stl",
     (0.0077, 0.0001, -0.0234), (-1.5708, 0.0, 0.0), WRIST_ROLL),
    ("gripper", "wrist_roll_follower_so101_v1.stl",
     (0.0, -0.000218214, 0.000949706), (-3.14159, 0.0, 0.0), WRIST_ROLL),
]

JAW = ("jaw", "moving_jaw_so101_v1.stl", (0.0, 0.0, 0.0189), (0.0, 0.0, 0.0))


def scene(grip: float) -> list[tuple[str, np.ndarray]]:
    """Triangles in arm_wrist_link, for one gripper opening."""
    out = []
    for label, mesh, xyz, rot, chain in PARTS:
        m = chain @ transform(xyz, *rot)
        tri = load_stl(mesh)
        out.append((label, tri @ m[:3, :3].T + m[:3, 3]))
    label, mesh, xyz, rot = JAW
    m = (WRIST_ROLL @ GRIPPER_JOINT
         @ transform((0, 0, 0), 0, 0, grip) @ transform(xyz, *rot))
    out.append((label, load_stl(mesh) @ m[:3, :3].T + m[:3, 3]))
    return out


def first_hit(tris: np.ndarray, origin: np.ndarray,
              direction: np.ndarray) -> float:
    """Moller-Trumbore over every triangle at once. inf if the ray is clear."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    h = np.cross(direction, e2)
    a = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(a) > 1e-12
    f = np.zeros_like(a)
    f[ok] = 1.0 / a[ok]
    s = origin - v0
    u = f * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, e1)
    v = f * (q @ direction)
    t = f * np.einsum("ij,ij->i", e2, q)
    hit = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-4)
    return float(t[hit].min()) if hit.any() else math.inf


def clearance(origin, direction, grips=(0.0, 1.5)) -> tuple[float, str]:
    """Worst-case (nearest) obstruction over the swept gripper openings."""
    worst, who = math.inf, "clear"
    for grip in grips:
        for label, tris in scene(grip):
            d = first_hit(tris, np.asarray(origin, float),
                          np.asarray(direction, float))
            if d < worst:
                worst, who = d, f"{label}@grip{grip}"
    return worst, who


# The gripper's approach direction in arm_wrist_link, measured in
# docs/arm_workspace.md §3 and reconfirmed by the Step 2 check.
APPROACH = np.array([0.04924, -0.99228, 0.11384])
APPROACH /= np.linalg.norm(APPROACH)
TCP = np.array([0.00790, -0.15923, 0.01832])


def main() -> int:
    print("Current mount, aimed along the approach axis:")
    for name, pos in (("pod_tof_link", (0.010, 0.0, 0.015)),
                      ("pod_camera", (0.010, 0.0, 0.030))):
        d, who = clearance(pos, APPROACH)
        print(f"  {name:14s} at {pos}  first hit {d:.4f} m  ({who})")

    # Aiming AT the TCP is hopeless by construction - the gripper body runs from
    # the wrist to the jaws, so that line of sight is straight down it. The
    # useful question is whether the beam can pass BESIDE the jaws and converge
    # just beyond them, where a grasped object actually sits.
    print("\nMount offset vs converge distance along the approach axis.")
    print("Clear means the line of sight reaches the target with nothing in it.")
    print(f"{'dx':>6} {'dz':>6} {'D':>6} {'range':>7} {'clear':>8}  {'off-axis':>8}  what")
    best = []
    for dx in (0.010, 0.030, 0.050, 0.070):
        for dz in (0.015, 0.040, 0.070):
            for target_d in (0.18, 0.22, 0.26, 0.32, 0.40):
                pos = np.array([dx, 0.0, dz])
                target = APPROACH * target_d
                aim = target - pos
                reach = float(np.linalg.norm(aim))
                aim /= reach
                d, who = clearance(pos, aim)
                # How far the beam lands from the gripper's own axis. Big
                # numbers mean it is measuring somewhere the jaws are not.
                miss = float(np.linalg.norm(
                    np.cross(APPROACH, pos + aim * reach)))
                mark = ""
                if d > reach:
                    best.append((dx, dz, target_d, reach, miss))
                    mark = "  <- CLEAR"
                print(f"{dx:6.3f} {dz:6.3f} {target_d:6.3f} {reach:7.3f} "
                      f"{d:8.4f}  {miss:8.4f}  {who}{mark}")

    print(f"\n{len(best)} clear configurations")
    for dx, dz, target_d, reach, miss in best:
        print(f"  mount ({dx}, 0, {dz})  converge {target_d} m  "
              f"beam length {reach:.3f} m  lands {miss * 1000:.1f} mm off the "
              f"gripper axis")
    return 0


if __name__ == "__main__":
    sys.exit(main())
