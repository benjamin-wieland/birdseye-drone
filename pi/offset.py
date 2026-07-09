"""Convert a detected target's pixel position into a body-frame velocity
command.

Camera convention assumed throughout this module: the camera points
straight down (nadir), mounted so that:
  - image +x (rightward in the frame) corresponds to the drone's +right
  - image +y (downward in the frame) corresponds to the drone's +forward

If your camera is mounted rotated relative to the drone's forward axis,
adjust `pixel_offset_to_body_frame` accordingly (a 90 degree mount would
just be a coordinate swap; anything else needs an actual rotation).

No compass/yaw math happens here on purpose - by working in the drone's
own body frame, we don't need to know its heading at all. The flight
controller is told "go this far forward, this far right", not "go north".
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class VelocityCommand:
    forward_mps: float
    right_mps: float


def meters_per_pixel(altitude_m: float, fov_deg: float, frame_dimension_px: int) -> float:
    """Ground distance represented by one pixel, given altitude and FOV.

    Simple pinhole approximation: the camera sees a ground swath of width
    `2 * altitude * tan(fov/2)` across `frame_dimension_px` pixels.
    """
    fov_rad = math.radians(fov_deg)
    ground_swath_m = 2.0 * altitude_m * math.tan(fov_rad / 2.0)
    return ground_swath_m / frame_dimension_px


def pixel_offset_to_world_offset(
    center_x: float,
    center_y: float,
    frame_width: int,
    frame_height: int,
    altitude_m: float,
    hfov_deg: float,
    vfov_deg: float,
) -> tuple[float, float]:
    """Convert a detection's pixel center into a (right_m, forward_m) offset
    from the drone's current position, in the drone's body frame.
    """
    offset_px_x = center_x - frame_width / 2.0
    # Image y grows downward; "forward" in the frame is up, hence the negation.
    offset_px_y = frame_height / 2.0 - center_y

    mpp_x = meters_per_pixel(altitude_m, hfov_deg, frame_width)
    mpp_y = meters_per_pixel(altitude_m, vfov_deg, frame_height)

    right_m = offset_px_x * mpp_x
    forward_m = offset_px_y * mpp_y
    return right_m, forward_m


def world_offset_to_velocity(
    right_m: float,
    forward_m: float,
    kp: float,
    max_speed_mps: float,
) -> VelocityCommand:
    """Simple proportional controller: velocity is proportional to how far
    off-center the target is, clamped to a max speed.

    This is intentionally simple - no integral/derivative terms, no
    smoothing. It's a starting point to get the pipeline flying, not a
    tuned controller. Expect to add at least a low-pass filter on the
    detection center (targets jitter frame to frame) before this feels
    smooth in the air.
    """
    def clamp(v: float) -> float:
        return max(-max_speed_mps, min(max_speed_mps, v))

    return VelocityCommand(
        forward_mps=clamp(kp * forward_m),
        right_mps=clamp(kp * right_m),
    )


def distance_to_target_m(right_m: float, forward_m: float) -> float:
    return math.hypot(right_m, forward_m)
