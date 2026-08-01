#!/usr/bin/env python3
"""Pins the pod camera's image axes against the arm's joints.

Pure geometry, off the URDF - no simulator, no ROS. Run it after touching
anything in the pod mount, arm_wrist_link, or the optical-frame convention:

    bash scripts/dev-native.sh python scripts/verify_pod_orientation.py

WHAT IT GUARDS. Three control loops in drivebase_behaviour each pair one image
axis with one actuator:

    horizontal error -> base yaw (APPROACHING) and shoulder_pan (CONFIRMING)
    vertical error   -> the pitch chain, search_pose -> confirm_pose

Those pairings are only valid if the image is upright in the robot's sense -
image up = forward, image right = the robot's right. arm_wrist_link's own frame
is rolled 90 degrees, so mounting the camera square to it produces a sideways
image in which horizontal error means RANGE. That failure is invisible: the
detector reports plausible numbers, the arm tracks its commands faithfully, and
the loops simply never converge. It cost one debugging session already.
"""

import math
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src" / "drivebase_behaviour"))

from drivebase_behaviour.kinematics import forward_kinematics, load_chain  # noqa: E402

# Both measured; see docs/arm_workspace.md section 3.
POSES = {
    "search": {"arm_shoulder_lift": -1.4, "arm_elbow_flex": 0.0,
               "arm_wrist_flex": 0.4},
    "confirm": {"arm_shoulder_lift": 0.9, "arm_elbow_flex": 0.46,
                "arm_wrist_flex": -1.40},
}
# Where each pose's beam lands, in arm_base_link. The confirm figure is the
# 0.392 m base_footprint reading minus the 0.215 m mount offset.
TARGETS = {"search": (0.95, 0.0, -0.26), "confirm": (0.177, 0.0, -0.2246)}

FX = FY = 493.7925  # from camera_info, = 320 / tan(1.15 / 2)
WIDTH, HEIGHT = 640.0, 480.0


def project(chain, pose, pan, point):
    """Normalised image error of a base-link point, or None if behind."""
    transform = forward_kinematics(chain, dict(pose, arm_shoulder_pan=pan))
    p = np.linalg.inv(transform) @ np.array([*point, 1.0])
    if p[2] <= 0.0:
        return None
    return ((FX * p[0] / p[2] + WIDTH / 2) / WIDTH - 0.5,
            (FY * p[1] / p[2] + HEIGHT / 2) / HEIGHT - 0.5)


def main() -> int:
    urdf = subprocess.run(
        ["xacro", str(REPO / "src/drivebase_description/urdf/drivebase.urdf.xacro"),
         "use_arm:=true"],
        capture_output=True, text=True, check=True).stdout
    chain = load_chain(urdf, "arm_base_link", "pod_camera_optical_frame")

    failures = []
    for name, pose in POSES.items():
        target = TARGETS[name]
        rotation = forward_kinematics(
            chain, dict(pose, arm_shoulder_pan=0.0))[:3, :3]
        right, down = rotation[:, 0], rotation[:, 1]

        # Image right must be the robot's right (-y); image down must be the
        # robot's rear (-x). Both to within the camera's own downward tilt.
        right_ok = right[1] < -0.9
        down_ok = down[0] < -0.4

        offset = list(target)
        offset[1] += 0.05
        base = project(chain, pose, 0.0, target)
        moved = project(chain, pose, 0.0, tuple(offset))
        du_lateral = (moved[0] - base[0]) / 0.05

        plus = project(chain, pose, 0.10, target)
        minus = project(chain, pose, -0.10, target)
        du_dpan = (plus[0] - minus[0]) / 0.20

        # The aim loop integrates pan on horizontal error. It converges only if
        # pan has real, negative authority over that error.
        pan_ok = du_dpan < -0.2
        # Deliberately loose. This derivative scales with 1/range, so it is
        # -3.124 at the confirm target 0.18 m out and -0.697 at the search
        # target 0.95 m out; a tight bound would encode the target distance
        # rather than the mount. What is being caught is the sideways case,
        # where it is exactly zero.
        lateral_ok = du_lateral < -0.3

        print(f"{name}_pose:")
        print(f"  image right in base   {np.round(right, 3)}  "
              f"{'ok' if right_ok else 'WRONG - not the robot right'}")
        print(f"  image down  in base   {np.round(down, 3)}  "
              f"{'ok' if down_ok else 'WRONG - not the robot rear'}")
        print(f"  d(h_error)/d(lateral) {du_lateral:+.3f} /m    "
              f"{'ok' if lateral_ok else 'WRONG - bearing is not in this axis'}")
        print(f"  d(h_error)/d(pan)     {du_dpan:+.3f} /rad  "
              f"{'ok' if pan_ok else 'WRONG - aim loop cannot converge'}")
        print(f"  aim_gain for deadbeat {1.0 / abs(du_dpan):.2f}")

        for ok, what in ((right_ok, "image right"), (down_ok, "image down"),
                         (lateral_ok, "lateral authority"),
                         (pan_ok, "pan authority")):
            if not ok:
                failures.append(f"{name}_pose: {what}")

    if failures:
        print("\nFAILED: " + "; ".join(failures))
        return 1
    print("\nAll checks passed - the pod images the world upright.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
