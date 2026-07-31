"""One detection, and the parsing of today's JSON payload into it.

PURE. No rclpy. The coordinator never sees JSON - it sees Detection, whichever
source produced it. That is what lets the typed drivebase_msgs/LitterDetection
replace the std_msgs/String interface without the state machine noticing.
"""

import json
from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    # Seconds. From the message header when the typed interface is in use;
    # from arrival time when it is not, because std_msgs/String carries no
    # stamp. The difference matters under load - see the design doc §8.
    stamp_seconds: float
    detected: bool
    frame_number: int
    confidence: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    floor_x: float = 0.0
    floor_y: float = 0.0
    horizontal_error: float = 0.0
    bbox: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    # Set when the point came from the bbox-size estimate rather than the ToF.
    # The confirmation gate demands more consecutive frames when it is set.
    range_is_fallback: bool = False


def direction_from_error(horizontal_error: float, deadband: float) -> str:
    if horizontal_error < -deadband:
        return "LEFT"
    if horizontal_error > deadband:
        return "RIGHT"
    return "CENTER"


def parse_target_json(payload: str, stamp_seconds: float) -> Detection | None:
    """Returns None for anything malformed.

    Deliberately total: a detector that crashes, restarts mid-message or
    changes its schema must degrade the coordinator to "no target", never take
    it down. Every failure here is one dropped frame.
    """
    try:
        raw = json.loads(payload)
    except (ValueError, TypeError):
        return None

    if not isinstance(raw, dict):
        return None

    detected = raw.get("detected")
    if not isinstance(detected, bool):
        return None

    frame_number = raw.get("frame_number", 0)
    if not isinstance(frame_number, int):
        return None

    if not detected:
        return Detection(
            stamp_seconds=stamp_seconds,
            detected=False,
            frame_number=frame_number,
        )

    try:
        bbox_raw = raw["bbox_pixels"]
        center = raw["center_normalized"]
        floor = raw["floor_point_normalized"]
        return Detection(
            stamp_seconds=stamp_seconds,
            detected=True,
            frame_number=frame_number,
            confidence=float(raw["confidence"]),
            center_x=float(center["x"]),
            center_y=float(center["y"]),
            floor_x=float(floor["x"]),
            floor_y=float(floor["y"]),
            horizontal_error=float(raw["horizontal_error"]),
            bbox=(
                float(bbox_raw["x1"]),
                float(bbox_raw["y1"]),
                float(bbox_raw["x2"]),
                float(bbox_raw["y2"]),
            ),
        )
    except (KeyError, TypeError, ValueError):
        return None
