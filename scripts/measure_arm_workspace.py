#!/usr/bin/env python3
"""Every arm figure the coordinator is calibrated on, for a given mount height.

    bash scripts/dev-native.sh python scripts/measure_arm_workspace.py
    bash scripts/dev-native.sh python scripts/measure_arm_workspace.py --mount 0.224

Pure geometry off the URDF - no simulator. Both mount heights come out of this
one script so the two columns cannot drift apart, which is the whole point:
docs/arm_workspace.md section 5 quotes it for 0.260 and 0.224 side by side.

Everything is reported in base_footprint, i.e. metres above the ground the
robot is standing on, because that is the frame the bracket decision is argued
in. The chains are built from base_footprint for the same reason - the mount
height then enters the maths exactly once, through the URDF, rather than being
added back by hand at each call site.

WHY GEOMETRY IS ENOUGH HERE. The one figure that was checked both ways agreed:
PROGRESS.md records confirm_pose's ToF reading 0.2716 m against a tf prediction
of 0.2715 m. The sensor readings are not independent measurements of the model.
"""

import argparse
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src" / "drivebase_behaviour"))

from drivebase_behaviour.ik import JOINT_LIMITS, solve  # noqa: E402
from drivebase_behaviour.kinematics import (  # noqa: E402
    extract_planar_arm, forward_kinematics, load_chain,
)

SEARCH = {"arm_shoulder_lift": -1.4, "arm_elbow_flex": 0.0, "arm_wrist_flex": 0.4}
CONFIRM = {"arm_shoulder_lift": 0.9, "arm_elbow_flex": 0.46, "arm_wrist_flex": -1.40}
STRAIGHT_DOWN = -math.pi / 2
US_BAND_Z = 0.200          # ultrasonic band height, from the URDF
LIFT_LIMIT = JOINT_LIMITS["shoulder_lift"][1]


def build(mount: float):
    urdf = subprocess.run(
        ["xacro", str(REPO / "src/drivebase_description/urdf/drivebase.urdf.xacro"),
         "use_arm:=true", f"arm_mount_height:={mount}"],
        capture_output=True, text=True, check=True).stdout
    return {
        "arm": extract_planar_arm(
            load_chain(urdf, "arm_base_link", "arm_gripper_frame_link")),
        "tip": load_chain(urdf, "base_footprint", "arm_gripper_frame_link"),
        "tof": load_chain(urdf, "base_footprint", "pod_tof_link"),
        "base": load_chain(urdf, "base_footprint", "arm_base_link"),
        "pod": load_chain(urdf, "base_footprint", "pod_camera_optical_frame"),
    }


def ray_to_ground(chain, joints):
    """Where the ToF boresight (+x of pod_tof_link) meets z = 0, and how far."""
    frame = forward_kinematics(chain, joints)
    origin, direction = frame[:3, 3], frame[:3, 0]
    if abs(direction[2]) < 1e-9 or (origin[2] / -direction[2]) <= 0:
        return None, None
    distance = origin[2] / -direction[2]
    return origin + distance * direction, distance


def band(arm, depth: float):
    """Reachable radial band for a target `depth` below the arm base."""
    found = [r / 2000 for r in range(0, 800)
             if solve(arm, arm.from_planar(r / 2000, -depth, 0.0),
                      STRAIGHT_DOWN, JOINT_LIMITS)]
    return (min(found), max(found)) if found else None


def report(mount: float) -> None:
    model = build(mount)
    arm = model["arm"]

    base_z = forward_kinematics(model["base"], {})[2, 3]
    floor = min(arm.tip(l / 100, e / 100, f / 100)[1]
                for l in range(-174, 175, 6)
                for e in range(-169, 170, 6)
                for f in range(-165, 166, 12))

    print(f"\n{'=' * 66}\nARM MOUNT AT {mount * 1000:.0f} mm ABOVE GROUND\n{'=' * 66}")
    print(f"arm_base_link height          {base_z * 1000:8.1f} mm")
    print(f"reach floor, below the mount  {-floor * 1000:8.1f} mm")
    print(f"lowest the gripper can go     {(base_z + floor) * 1000:8.1f} mm above ground")
    print(f"clearance over the {US_BAND_Z * 1000:.0f} mm US band "
          f"{(base_z - US_BAND_Z) * 1000:6.1f} mm")

    print("\n-- grasp band vs grasp height (straight-down approach) --")
    print(f"{'height':>9s} {'radial band':>17s} {'width':>7s} "
          f"{'lift @ centre':>13s} {'margin':>8s}")
    for mm in (0, 12, 20, 35.4, 50, 70):
        height = mm / 1000
        if height < base_z + floor:
            print(f"{mm:8.1f}mm {'below the reach floor':>17s}")
            continue
        found = band(arm, base_z - height)
        if found is None:
            print(f"{mm:8.1f}mm {'unreachable':>17s}")
            continue
        low, high = found
        centre = solve(arm, arm.from_planar((low + high) / 2, height - base_z, 0.0),
                       STRAIGHT_DOWN, JOINT_LIMITS)
        print(f"{mm:8.1f}mm {low:7.3f}-{high:<9.3f} {(high - low) * 1000:6.0f}mm "
              f"{centre.shoulder_lift:+13.3f} {LIFT_LIMIT - centre.shoulder_lift:+8.3f}")

    print("\n-- the two measured poses --")
    for name, joints in (("search", SEARCH), ("confirm", CONFIRM)):
        hit, distance = ray_to_ground(model["tof"], joints)
        tip = forward_kinematics(model["tip"], joints)[:3, 3]
        if hit is None:
            print(f"{name:8s} ToF ray never meets the ground")
        else:
            print(f"{name:8s} ToF lands x = {hit[0]:.4f} m, range {distance:.4f} m "
                  f"| gripper x = {tip[0]:.4f} z = {tip[2] * 1000:.1f} mm")

    # What grasp_min / grasp_max in the coordinator config have to bracket: the
    # pod sits at confirm_pose while CONFIRMING, so the range it reports to a
    # target anywhere in the band is what the gate sees.
    print("\n-- camera range to the grasp plane, over the band --")
    pod = forward_kinematics(model["pod"], CONFIRM)[:3, 3]
    pan_axis_x = 0.215 + arm.axis_xy[0]
    for mm in (12, 50):
        height = mm / 1000
        found = band(arm, base_z - height) if height >= base_z + floor else None
        if found is None:
            continue
        ranges = [float(np.linalg.norm(np.array([pan_axis_x + r, 0.0, height]) - pod))
                  for r in found]
        print(f"grasp at {mm:5.1f} mm: camera range "
              f"{min(ranges):.3f}-{max(ranges):.3f} m")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mount", type=float, default=None,
                        help="mount height above ground, metres")
    arguments = parser.parse_args()
    for mount in ([arguments.mount] if arguments.mount else [0.260, 0.224]):
        report(mount)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
