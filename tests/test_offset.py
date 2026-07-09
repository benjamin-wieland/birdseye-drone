import math

from pi.offset import (
    meters_per_pixel,
    pixel_offset_to_world_offset,
    world_offset_to_velocity,
    distance_to_target_m,
)


def test_target_at_center_gives_zero_offset():
    right_m, forward_m = pixel_offset_to_world_offset(
        center_x=320, center_y=240,
        frame_width=640, frame_height=480,
        altitude_m=3.0, hfov_deg=62.2, vfov_deg=48.8,
    )
    assert math.isclose(right_m, 0.0, abs_tol=1e-6)
    assert math.isclose(forward_m, 0.0, abs_tol=1e-6)


def test_target_right_of_center_gives_positive_right_offset():
    right_m, _ = pixel_offset_to_world_offset(
        center_x=640, center_y=240,  # far right edge
        frame_width=640, frame_height=480,
        altitude_m=3.0, hfov_deg=62.2, vfov_deg=48.8,
    )
    assert right_m > 0


def test_target_above_center_gives_positive_forward_offset():
    # "above" in image = smaller y = target is further from the camera
    # in the forward direction (nadir camera, forward = up in frame).
    _, forward_m = pixel_offset_to_world_offset(
        center_x=320, center_y=0,  # top edge
        frame_width=640, frame_height=480,
        altitude_m=3.0, hfov_deg=62.2, vfov_deg=48.8,
    )
    assert forward_m > 0


def test_higher_altitude_means_larger_ground_swath_per_pixel():
    low = meters_per_pixel(altitude_m=1.0, fov_deg=60, frame_dimension_px=640)
    high = meters_per_pixel(altitude_m=10.0, fov_deg=60, frame_dimension_px=640)
    assert high > low


def test_velocity_clamped_to_max_speed():
    cmd = world_offset_to_velocity(right_m=100, forward_m=100, kp=1.0, max_speed_mps=1.0)
    assert cmd.right_mps == 1.0
    assert cmd.forward_mps == 1.0


def test_velocity_proportional_below_clamp():
    cmd = world_offset_to_velocity(right_m=1.0, forward_m=0.0, kp=0.5, max_speed_mps=5.0)
    assert math.isclose(cmd.right_mps, 0.5)
    assert math.isclose(cmd.forward_mps, 0.0)


def test_distance_to_target():
    assert math.isclose(distance_to_target_m(3.0, 4.0), 5.0)
