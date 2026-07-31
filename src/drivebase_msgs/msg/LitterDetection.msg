# Litter detection from a single camera frame.
#
# PROPOSED replacement for the JSON-in-std_msgs/String payload currently on
# /vision/target. See docs/superpowers/specs/2026-07-30-behaviour-coordinator-design.md §9.
#
# Two things this fixes over the String form:
#   * A header. std_msgs/String carries no timestamp, so a consumer must stamp
#     on arrival, which misattributes latency and lets it act on a stale frame.
#     Stamp this at CAPTURE, not at publish.
#   * horizontal_error is primary. LEFT/RIGHT/CENTER was the right shape for
#     steering a base; an eye-in-hand arm uses the continuous value directly
#     and bucketing into thirds discards precision.

std_msgs/Header header

bool    detected
uint32  frame_number
float32 confidence

# Pixels, image coordinates.
float32 bbox_x1
float32 bbox_y1
float32 bbox_x2
float32 bbox_y2

# Normalised 0..1 across the image.
float32 center_x
float32 center_y
float32 floor_x
float32 floor_y

# PRIMARY. Continuous, -0.5 (hard left) .. +0.5 (hard right).
float32 horizontal_error

# Derived from horizontal_error. Retained for existing consumers.
uint8 DIRECTION_NO_TARGET = 0
uint8 DIRECTION_LEFT      = 1
uint8 DIRECTION_CENTER    = 2
uint8 DIRECTION_RIGHT     = 3
uint8 direction
