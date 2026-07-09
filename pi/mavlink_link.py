"""MAVLink connection to the flight controller, and velocity setpoint
sending.

Uses pymavlink directly rather than a higher-level framework (like
MAVSDK or DroneKit) to keep the dependency footprint small and the
control loop transparent - you can see exactly what bytes go out.

Sends SET_POSITION_TARGET_LOCAL_NED with only the velocity bits of the
type_mask enabled, in MAV_FRAME_BODY_OFFSET_NED - meaning vx/vy are
forward/right relative to the drone's current heading, not compass
directions. This is what lets offset.py stay yaw-agnostic.

IMPORTANT: this module sends commands, it does not arm the vehicle or
change flight modes. Your flight controller needs to already be in a
mode that accepts offboard/guided velocity setpoints (e.g. GUIDED on
ArduPilot, OFFBOARD on PX4) before these commands will do anything -
and you should set that up deliberately, with a manual mode switch,
not automatically from this code.
"""

from __future__ import annotations

import time


class MavlinkLink:
    def __init__(self, connection_string: str, baud: int, dry_run: bool = False):
        self.dry_run = dry_run
        self.master = None

        if not dry_run:
            from pymavlink import mavutil

            self.master = mavutil.mavlink_connection(connection_string, baud=baud)
            self.master.wait_heartbeat()
            print(
                f"Heartbeat received from system {self.master.target_system}, "
                f"component {self.master.target_component}"
            )

    def send_body_velocity(self, forward_mps: float, right_mps: float, down_mps: float = 0.0) -> None:
        """Send a single velocity setpoint in the drone's body frame."""
        if self.dry_run:
            print(
                f"[dry-run] would send velocity: forward={forward_mps:+.2f} m/s  "
                f"right={right_mps:+.2f} m/s  down={down_mps:+.2f} m/s"
            )
            return

        from pymavlink import mavutil

        # type_mask bits: only enable vx, vy, vz - position, accel, and
        # yaw fields are ignored. See MAVLink SET_POSITION_TARGET_LOCAL_NED
        # docs for the full bit layout.
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            0, 0, 0,  # x, y, z position (ignored)
            forward_mps, right_mps, down_mps,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz (ignored)
            0, 0,  # yaw, yaw_rate (ignored)
        )

    def send_hold(self) -> None:
        """Zero velocity - hover in place."""
        self.send_body_velocity(0.0, 0.0, 0.0)

    def close(self) -> None:
        if self.master is not None:
            self.master.close()
