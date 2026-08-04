#!/usr/bin/env python3
"""
Runs on your LAPTOP (not the Pi). Connects to pi_stream_server.py running
on the Raspberry Pi, receives the live camera feed, runs object detection
locally using YOLOv8 (more accurate than the SSD MobileNet model used in
the original Pi-only version), displays the annotated video, and sends
detection results back to the Pi so it can still act on them via MAVLink.

Install (on your laptop):
    pip install opencv-python numpy ultralytics

First run will auto-download the YOLOv8 weights file (~6MB for the
smallest "nano" model) — needs internet access once, then it's cached
locally.

Run:
    python3 laptop_client.py --pi-ip 192.168.1.42
    python3 laptop_client.py --pi-ip 192.168.1.42 --model yolov8s.pt --conf 0.4
"""

import argparse
import json
import socket
import struct

import cv2
import numpy as np
from ultralytics import YOLO

FRAME_PORT = 5001
COMMAND_PORT = 5002
DEFAULT_MODEL = "yolov8n.pt"   # nano: fastest, least accurate
DEFAULT_CONF = 0.5


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def run_inference(model, frame, conf_threshold):
    results = model(frame, verbose=False, conf=conf_threshold)[0]
    detections = []

    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        conf = float(box.conf[0])
        class_id = int(box.cls[0])
        label = model.names[class_id]
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        detections.append({
            "label": label, "confidence": conf, "centroid": [cx, cy]
        })

        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {conf:.2f}", (int(x1), max(int(y1) - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

    return frame, detections


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pi-ip", required=True, help="IP address of the Raspberry Pi")
    parser.add_argument("--model", default=DEFAULT_MODEL,
                         help="YOLOv8 weights: yolov8n.pt (fastest) / yolov8s.pt / "
                              "yolov8m.pt (more accurate, slower). Auto-downloads on first use.")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF,
                         help="Confidence threshold (0-1). Lower = more detections, more false positives.")
    args = parser.parse_args()

    print(f"Loading {args.model} (first run downloads weights automatically)...")
    model = YOLO(args.model)

    print(f"Connecting to {args.pi_ip}:{FRAME_PORT} (video)...")
    frame_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    frame_sock.connect((args.pi_ip, FRAME_PORT))

    print(f"Connecting to {args.pi_ip}:{COMMAND_PORT} (commands)...")
    command_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    command_sock.connect((args.pi_ip, COMMAND_PORT))

    print("Connected. Press 'q' in the video window to quit.")

    try:
        while True:
            header = recv_exact(frame_sock, 4)
            if header is None:
                print("Pi disconnected.")
                break
            length = struct.unpack(">I", header)[0]
            jpeg_bytes = recv_exact(frame_sock, length)
            if jpeg_bytes is None:
                break

            frame = cv2.imdecode(
                np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
            )

            frame, detections = run_inference(model, frame, args.conf)

            payload = json.dumps(detections).encode("utf-8")
            command_sock.sendall(struct.pack(">I", len(payload)) + payload)

            cv2.imshow("Live Detection (via Pi stream)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        frame_sock.close()
        command_sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
