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
