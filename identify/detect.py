#!/usr/bin/env python3
"""
Live object detection on Raspberry Pi 4B with the Camera Module (CSI).
Uses picamera2 for capture and TFLite (SSD MobileNet V2, COCO classes) for inference.

Run:
    python3 detect.py

Press 'q' in the preview window to quit (if display attached),
or Ctrl+C in the terminal if running headless.
"""

import time
import argparse
import numpy as np
import cv2

from picamera2 import Picamera2
from tflite_runtime.interpreter import Interpreter

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
MODEL_PATH = "models/ssd_mobilenet_v2_coco_quant.tflite"
LABELS_PATH = "models/coco_labels.txt"
CONF_THRESHOLD = 0.5
CAPTURE_RES = (640, 480)      # camera capture resolution
INPUT_SIZE = (300, 300)       # model expects 300x300 for this SSD model


def load_labels(path):
    with open(path, "r") as f:
        return [line.strip() for line in f.readlines()]


def setup_camera():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": CAPTURE_RES, "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)  # let auto-exposure settle
    return picam2


def setup_interpreter(model_path):
    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter


def run_inference(interpreter, frame):
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    resized = cv2.resize(frame, INPUT_SIZE)
    input_tensor = np.expand_dims(resized, axis=0)

    # Quantized model expects uint8; adjust here if you use a float model instead.
    if input_details[0]["dtype"] == np.uint8:
        input_tensor = input_tensor.astype(np.uint8)
    else:
        input_tensor = (input_tensor.astype(np.float32) - 127.5) / 127.5

    interpreter.set_tensor(input_details[0]["index"], input_tensor)
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[0]["index"])[0]
    classes = interpreter.get_tensor(output_details[1]["index"])[0]
    scores = interpreter.get_tensor(output_details[2]["index"])[0]
    count = int(interpreter.get_tensor(output_details[3]["index"])[0])

    return boxes, classes, scores, count


def draw_and_report(frame, boxes, classes, scores, count, labels, threshold):
    h, w, _ = frame.shape
    detections = []

    for i in range(count):
        if scores[i] < threshold:
            continue

        ymin, xmin, ymax, xmax = boxes[i]
        x1, y1, x2, y2 = int(xmin * w), int(ymin * h), int(xmax * w), int(ymax * h)
        class_id = int(classes[i])
        label = labels[class_id] if class_id < len(labels) else f"id_{class_id}"
        conf = float(scores[i])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        detections.append({
            "label": label,
            "confidence": conf,
            "box": (x1, y1, x2, y2),
            "centroid": (cx, cy),
        })

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            frame, f"{label} {conf:.2f}", (x1, max(y1 - 8, 0)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
        )
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

    return frame, detections


def main(show_preview=True, headless=False):
    labels = load_labels(LABELS_PATH)
    interpreter = setup_interpreter(MODEL_PATH)
    picam2 = setup_camera()

    print("Starting detection loop. Ctrl+C to stop.")
    try:
        while True:
            frame = picam2.capture_array()
            boxes, classes, scores, count = run_inference(interpreter, frame)
            frame, detections = draw_and_report(
                frame, boxes, classes, scores, count, labels, CONF_THRESHOLD
            )

            for d in detections:
                print(f"[{time.strftime('%H:%M:%S')}] {d['label']} "
                      f"({d['confidence']:.2f}) centroid={d['centroid']}")

            if show_preview and not headless:
                cv2.imshow("Detections", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        picam2.stop()
        if show_preview and not headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without a display window (e.g. over SSH with no X forwarding)."
    )
    args = parser.parse_args()
    main(headless=args.headless)
