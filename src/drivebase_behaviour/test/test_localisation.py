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
