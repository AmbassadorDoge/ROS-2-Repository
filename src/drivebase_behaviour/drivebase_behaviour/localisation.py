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
