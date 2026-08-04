#!/usr/bin/env python3
"""
Runs on the Raspberry Pi. Captures camera frames and streams them to a
laptop over the network for detection (offloading the heavy inference off
the Pi's CPU). Also listens for detection results coming back from the
laptop and relays them to the flight controller via mavlink_bridge.py.

Two connections, both initiated FROM the laptop TO the Pi:
    - Port 5001: Pi sends JPEG frames to the laptop (video out)
    - Port 5002: Pi receives JSON detection results from the laptop (data in)

Run:
    python3 pi_stream_server.py
    python3 pi_stream_server.py --mavlink --mavlink-connection udp:127.0.0.1:14550
"""

import argparse
import json
import socket
import struct
import threading
import time

import cv2
from picamera2 import Picamera2

from mavlink_bridge import MavlinkBridge, centroid_to_velocity

FRAME_PORT = 5001
COMMAND_PORT = 5002
CAPTURE_RES = (640, 480)
JPEG_QUALITY = 80


def send_frames(conn, picam2):
    """Continuously JPEG-encode frames and send them length-prefixed."""
    try:
        while True:
            frame = picam2.capture_array()
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if not ok:
                continue
            data = encoded.tobytes()
            # 4-byte big-endian length prefix, then the JPEG bytes
            conn.sendall(struct.pack(">I", len(data)) + data)
    except (BrokenPipeError, ConnectionResetError):
        print("Frame client disconnected.")


def recv_exact(conn, n):
    """Read exactly n bytes from a socket, or return None if closed."""
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def receive_commands(conn, bridge, track_label, frame_w, frame_h):
    """Receive JSON detection lists from the laptop and act on them."""
    try:
        while True:
            header = recv_exact(conn, 4)
            if header is None:
                break
            length = struct.unpack(">I", header)[0]
            payload = recv_exact(conn, length)
            if payload is None:
                break

            detections = json.loads(payload.decode("utf-8"))
            for d in detections:
                print(f"[{time.strftime('%H:%M:%S')}] {d['label']} "
                      f"({d['confidence']:.2f}) centroid={d['centroid']}")

            if bridge:
                target = next((d for d in detections if d["label"] == track_label), None)
                if target:
                    cx, cy = target["centroid"]
                    vx, vy, vz = centroid_to_velocity(cx, cy, frame_w, frame_h)
                    bridge.send_velocity_command(vx, vy, vz)
    except (ConnectionResetError, json.JSONDecodeError) as e:
        print(f"Command connection ended: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", action="store_true",
                         help="Enable MAVLink integration (defaults to dry-run).")
    parser.add_argument("--mavlink-connection", default="udp:127.0.0.1:14550")
    parser.add_argument("--mavlink-live", action="store_true",
                         help="DANGER: actually send commands, not just print them.")
    parser.add_argument("--track-label", default="person")
    args = parser.parse_args()

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": CAPTURE_RES, "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)

    bridge = None
    if args.mavlink:
        bridge = MavlinkBridge(
            connection_string=args.mavlink_connection,
            dry_run=not args.mavlink_live,
        )
        bridge.connect()

    frame_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    frame_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    frame_server.bind(("0.0.0.0", FRAME_PORT))
    frame_server.listen(1)

    command_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    command_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    command_server.bind(("0.0.0.0", COMMAND_PORT))
    command_server.listen(1)

    print(f"Waiting for laptop to connect on ports {FRAME_PORT} (video) "
          f"and {COMMAND_PORT} (commands)...")
    print(f"Find this Pi's IP with: hostname -I")

    frame_conn, frame_addr = frame_server.accept()
    print(f"Video client connected from {frame_addr}")

    command_conn, command_addr = command_server.accept()
    print(f"Command client connected from {command_addr}")

    frame_thread = threading.Thread(
        target=send_frames, args=(frame_conn, picam2), daemon=True
    )
    command_thread = threading.Thread(
        target=receive_commands,
        args=(command_conn, bridge, args.track_label, *CAPTURE_RES),
        daemon=True,
    )
    frame_thread.start()
    command_thread.start()

    try:
        while frame_thread.is_alive() or command_thread.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        picam2.stop()
        if bridge:
            bridge.close()
        frame_conn.close()
        command_conn.close()


if __name__ == "__main__":
    main()
