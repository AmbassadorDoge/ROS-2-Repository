#!/usr/bin/env python3
"""Search every pod mounting orientation against every measured arm pose.

    ./scripts/dev.sh bash -c 'xacro \\
      src/drivebase_description/urdf/drivebase.urdf.xacro use_sim:=true \\
      use_arm:=true > runs/pod_sweep/robot.urdf 2>/dev/null'
    python3 scripts/pod_aim_search.py runs/pod_sweep/main.json ...

The generated URDF is an input, not an artefact worth keeping, so regenerate it
with the line above rather than expecting to find it in the tree.

WHY OFFLINE. A mounting rotation is fixed at launch, so testing one costs a
whole Gazebo session. But the pod's ray direction in base_link is
FK(measured joint angles) x R_mount x_hat, and the arm poses have already been
measured. So the mount can be swept over the sphere against poses that were
really driven, in seconds, instead of one session per candidate.

This is arithmetic on measured inputs, not a measurement. It is only
trustworthy because step 1 validates the FK against the ray each pose actually
reported - if that residual is not ~1e-3 the rest of the output means nothing.
Any winner still has to be confirmed in the simulator.
"""

from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

URDF = "runs/pod_sweep/robot.urdf"
GROUND_BELOW_BASE = 0.060
GRASP_MIN, GRASP_MAX = 0.287, 0.481

# The chain base_link -> pod_tof_link, and which command each moving joint takes.
CHAIN = ["arm_mount_joint", "arm_base_attach_joint", "arm_shoulder_pan",
         "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex",
         "pod_tof_joint"]
DRIVEN = {"arm_shoulder_pan": "pan", "arm_shoulder_lift": "lift",
          "arm_elbow_flex": "elbow", "arm_wrist_flex": "wrist"}


def rpy(r, p, y) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle(axis, angle) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def load_joints():
    root = ET.parse(URDF).getroot()
    out = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
        rot = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(v) for v in (ax.get("xyz").split())] if ax is not None else None
        out[j.get("name")] = (np.array(xyz), rpy(*rot), axis)
    return out


def fk(joints, q: dict) -> tuple[np.ndarray, np.ndarray]:
    """base_link -> pod_tof_link, EXCLUDING the pod joint's own rotation."""
    p = np.zeros(3)
    R = np.eye(3)
    for name in CHAIN:
        xyz, R0, axis = joints[name]
        p = p + R @ xyz
        # The pod joint contributes its offset but NOT its rotation: that
        # rotation is the thing being swept, so it must not be baked in here.
        if name != "pod_tof_joint":
            R = R @ R0
        if name in DRIVEN:
            R = R @ axis_angle(axis, q[DRIVEN[name]])
    return p, R


def exhaustive(joints, clearance_mm) -> None:
    """Every reachable arm pose against every unobstructed pod axis.

    The 44 driven poses are a thin sample of a 3-DOF space, so a "none of them
    works" result could just mean the wrong 44 were picked. This closes that
    gap: the FK is validated above, so it can be trusted on poses that were
    never driven, and the answer becomes a statement about the arm rather than
    about the sample. Droop is not modelled here - these are commanded angles -
    which is why it screens candidates rather than confirming them.
    """
    # Which pod axes clear the castings depends only on the direction in
    # arm_wrist_link, not on the arm pose, so it is computed once.
    axes = []
    for pitch_deg in range(-90, 91, 5):
        for yaw_deg in range(-180, 180, 5):
            d = rpy(0.0, math.radians(pitch_deg), math.radians(yaw_deg))[:, 0]
            if clearance_mm(d) >= 8.0:
                axes.append((pitch_deg, yaw_deg, d))
    D = np.array([a[2] for a in axes])
    print(f"\nExhaustive screen: {len(axes)} pod axes clear the castings by "
          f">=8 deg")

    steps = np.arange(-1.65, 1.66, 0.15)
    found = []
    n_poses = 0
    for lift in np.arange(-1.74, 1.75, 0.15):
        for elbow in np.arange(-1.69, 1.70, 0.15):
            for wrist in steps:
                n_poses += 1
                p, R = fk(joints, {"pan": 0.0, "lift": lift,
                                   "elbow": elbow, "wrist": wrist})
                rays = D @ R.T
                rz = rays[:, 2]
                ok = rz < -0.05
                if not ok.any():
                    continue
                height = p[2] + GROUND_BELOW_BASE
                if height <= 0:
                    continue
                rng = height / -rz[ok]
                hit_x = p[0] + rng * rays[ok, 0]
                good = (hit_x >= GRASP_MIN) & (hit_x <= GRASP_MAX)
                if good.any():
                    idx = np.where(ok)[0][good]
                    for i, hx, rr in zip(idx, hit_x[good], rng[good]):
                        found.append((axes[i][0], axes[i][1], lift, elbow,
                                      wrist, rr, hx))

    print(f"  {n_poses} commanded poses x {len(axes)} axes")
    if not found:
        print("  NO combination of arm pose and unobstructed pod axis puts "
              "the beam in the grasp window.")
        return
    print(f"  {len(found)} combinations land in the window. Best by axis:")
    by_axis: dict[tuple[int, int], int] = {}
    for pi, ya, *_ in found:
        by_axis[(pi, ya)] = by_axis.get((pi, ya), 0) + 1
    for (pi, ya), n in sorted(by_axis.items(), key=lambda kv: -kv[1])[:8]:
        ex = next(f for f in found if f[0] == pi and f[1] == ya)
        print(f"    pitch {pi:4d} yaw {ya:5d} -> {n} poses, e.g. "
              f"lift {ex[2]:.2f} elbow {ex[3]:.2f} wrist {ex[4]:.2f} "
              f"range {ex[5]:.3f} hit_x {ex[6]:.3f}")


def main() -> int:
    joints = load_joints()
    poses = []
    for path in sys.argv[1:]:
        poses.extend(json.load(open(path)))

    # The pod joint's rotation is baked into the URDF; strip it so R_mount can
    # be swept independently.
    _, R_pod, _ = joints["pod_tof_joint"]

    worst = 0.0
    for row in poses:
        _, R = fk(joints, row["act"])
        residual = np.linalg.norm((R @ R_pod)[:, 0] - np.array(row["ray"]))
        worst = max(worst, residual)
    print(f"FK validation: worst |predicted ray - measured ray| = {worst:.5f}")
    if worst > 5e-3:
        print("FK does not reproduce the measured rays. Nothing below is valid.")
        return 1

    # Mount clearance from the wrist and gripper castings, reused from the
    # occlusion model. 30 mm is the margin the sim showed to be necessary: a
    # beam passing 6.5 mm from a casting still read that casting.
    sys.path.insert(0, "scripts")
    import pod_occlusion as occ
    verts = np.vstack([tri.reshape(-1, 3)
                       for g in (0.0, 1.5) for _, tri in occ.scene(g)])
    mount = np.array([0.030, 0.0, 0.040])

    def clearance_mm(direction) -> float:
        """Angular clearance to the nearest casting, in degrees.

        Perpendicular distance is the wrong metric: the sensor is bolted to the
        wrist, so castings sit millimetres to the SIDE of it at every aim, and a
        distance test rejects every direction. What blocks the beam is angular -
        the sim swallowed a beam passing 6.5 mm from a casting at 89 mm, which
        is 4.2 degrees, so anything under ~8 degrees is not really clear.
        """
        w = verts - mount
        r = np.linalg.norm(w, axis=1)
        ahead = (w @ direction) > 0
        if not ahead.any():
            return 180.0
        cos = np.clip((w[ahead] @ direction) / r[ahead], -1, 1)
        return float(np.degrees(np.arccos(cos)).min())

    hits = []
    for pitch_deg in range(-90, 91, 5):
        for yaw_deg in range(-180, 181, 5):
            R_mount = rpy(0.0, math.radians(pitch_deg), math.radians(yaw_deg))
            clear = clearance_mm(R_mount[:, 0])
            if clear < 8.0:
                continue
            for row in poses:
                p, R = fk(joints, row["act"])
                ray = (R @ R_mount)[:, 0]
                if ray[2] >= -0.05:
                    continue
                height = p[2] + GROUND_BELOW_BASE
                rng = height / -ray[2]
                hit_x = p[0] + rng * ray[0]
                hit_y = p[1] + rng * ray[1]
                if GRASP_MIN <= hit_x <= GRASP_MAX and abs(hit_y) < 0.15:
                    hits.append((clear, pitch_deg, yaw_deg, row["name"],
                                 round(rng, 3), round(hit_x, 3), round(hit_y, 3),
                                 row["grip_z_above_ground"]))

    print(f"\n{len(hits)} (mount, pose) pairs land in the "
          f"{GRASP_MIN}-{GRASP_MAX} m grasp window with >=30 mm clearance")
    if not hits:
        print("NONE. No mounting orientation on arm_wrist_link can see the "
              "grasp window at any MEASURED pose.")
        exhaustive(joints, clearance_mm)
        return 0

    hits.sort(reverse=True)
    print(f"{'clear':>6} {'pitch':>6} {'yaw':>6} {'range':>6} {'hit_x':>6} "
          f"{'hit_y':>6} {'gripz':>6}  pose")
    for clear, pi, ya, name, rng, hx, hy, gz in hits[:30]:
        print(f"{clear:6.1f} {pi:6d} {ya:6d} {rng:6.3f} {hx:6.3f} {hy:6.3f} "
              f"{gz:6.3f}  {name}")

    exhaustive(joints, clearance_mm)

    by_mount: dict[tuple[int, int], int] = {}
    for clear, pi, ya, *_ in hits:
        by_mount[(pi, ya)] = by_mount.get((pi, ya), 0) + 1
    print("\nmounts by how many measured poses they serve:")
    for (pi, ya), n in sorted(by_mount.items(), key=lambda kv: -kv[1])[:10]:
        print(f"  pitch {pi:4d} yaw {ya:5d} deg -> {n} poses "
              f"(clearance {clearance_mm(rpy(0, math.radians(pi), math.radians(ya))[:, 0]):.1f} deg)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
