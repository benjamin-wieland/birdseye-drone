#!/usr/bin/env python3
"""
MAVLink bridge between the object-detection pipeline and an ArduPilot
flight controller (Pixhawk), via pymavlink.

SAFETY FIRST:
- Test against ArduPilot SITL (software simulation) before ever connecting
  to a real flight controller. See README section 6 for SITL setup.
- This module defaults to DRY-RUN mode (prints commands instead of sending
  them). You must explicitly set dry_run=False once you've verified
  behavior in simulation and understand what each command does.
- Always test with props OFF first when moving to real hardware.

Install:
    pip install pymavlink --break-system-packages
"""

import time
from pymavlink import mavutil


class MavlinkBridge:
    def __init__(self, connection_string="/dev/serial0", baud=921600, dry_run=True):
        """
        connection_string examples:
            - "/dev/serial0"          Pi GPIO UART -> Pixhawk TELEM2
            - "/dev/ttyUSB0"          USB-connected telemetry radio or FTDI
            - "udp:127.0.0.1:14550"   SITL simulator (for testing without hardware)
            - "tcp:127.0.0.1:5760"    Alternative SITL connection
        """
        self.connection_string = connection_string
        self.baud = baud
        self.dry_run = dry_run
        self.master = None

    def connect(self, timeout=30):
        print(f"Connecting to {self.connection_string} (dry_run={self.dry_run})...")
        if self.connection_string.startswith(("udp:", "tcp:")):
            self.master = mavutil.mavlink_connection(self.connection_string)
        else:
            self.master = mavutil.mavlink_connection(
                self.connection_string, baud=self.baud
            )

        print("Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=timeout)
        print(
            f"Heartbeat received (system {self.master.target_system}, "
            f"component {self.master.target_component})"
        )
        return True

    def get_mode(self):
        msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            return mode
        return None

    def send_velocity_command(self, vx, vy, vz, yaw_rate=0.0):
        """
        Send a body-frame velocity command (GUIDED mode required on the
        flight controller for this to take effect).

        vx: forward/back (m/s, +forward)
        vy: right/left    (m/s, +right)
        vz: down/up       (m/s, +down, so negative = climb)
        yaw_rate: rad/s
        """
        if self.dry_run:
            print(f"[DRY RUN] Would send velocity: vx={vx:.2f} vy={vy:.2f} "
                  f"vz={vz:.2f} yaw_rate={yaw_rate:.2f}")
            return

        type_mask = 0b0000111111000111  # velocity + yaw_rate only
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            0, 0, 0,        # position (ignored per type_mask)
            vx, vy, vz,     # velocity
            0, 0, 0,        # acceleration (ignored)
            0, yaw_rate     # yaw, yaw_rate (yaw ignored per type_mask)
        )

    def close(self):
        if self.master:
            self.master.close()


def centroid_to_velocity(cx, cy, frame_w, frame_h, gain=0.5, max_speed=1.0):
    """
    Convert a detection's pixel centroid into a simple proportional velocity
    command that nudges the drone to center the object in frame.

    This is intentionally basic (a proportional controller, no PID tuning,
    no distance/altitude estimation) — a starting point to build on, not a
    tuned control law. Treat max_speed conservatively until tested in SITL.
    """
    norm_x = (cx - frame_w / 2) / (frame_w / 2)   # -1 (left) to 1 (right)
    norm_y = (cy - frame_h / 2) / (frame_h / 2)   # -1 (top) to 1 (bottom)

    vy = max(-max_speed, min(max_speed, norm_x * gain))   # left/right
    vz = max(-max_speed, min(max_speed, norm_y * gain))   # up/down
    vx = 0.0  # no forward/back logic yet — add distance estimation for this

    return vx, vy, vz


if __name__ == "__main__":
    # Minimal standalone test: connect and print incoming heartbeats/mode.
    # Point this at SITL first: connection_string="udp:127.0.0.1:14550"
    bridge = MavlinkBridge(connection_string="udp:127.0.0.1:14550", dry_run=True)
    bridge.connect()
    print("Current mode:", bridge.get_mode())

    # Example dry-run command
    bridge.send_velocity_command(vx=0, vy=0.3, vz=-0.1)
    bridge.close()
