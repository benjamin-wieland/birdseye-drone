"""Object detection wrapper.

Uses Ultralytics YOLO. Start with a stock pretrained model (yolov8n.pt is
small enough to run at a few FPS on a Pi 4B) and swap in a fine-tuned
model later once you know exactly what you're detecting from altitude -
stock COCO classes (person, car, etc.) look very different from a bird's
eye view than they do in the training data, so don't be surprised if
accuracy is mediocre until you fine-tune on your own top-down footage.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Detection:
    class_name: str
    confidence: float
    # Bounding box center, in pixel coordinates of the source frame.
    center_x: float
    center_y: float
    width: float
    height: float


class Detector:
    def __init__(self, model_path: str, target_classes: list[str], confidence_threshold: float):
        from ultralytics import YOLO  # local import: heavy, only needed here

        self.model = YOLO(model_path)
        self.target_classes = set(target_classes)
        self.confidence_threshold = confidence_threshold

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Run detection on a single frame, returning only detections that
        match the configured target classes above the confidence threshold.
        """
        results = self.model(frame, verbose=False)[0]
        detections: list[Detection] = []

        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < self.confidence_threshold:
                continue

            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            if class_name not in self.target_classes:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append(
                Detection(
                    class_name=class_name,
                    confidence=conf,
                    center_x=(x1 + x2) / 2.0,
                    center_y=(y1 + y2) / 2.0,
                    width=x2 - x1,
                    height=y2 - y1,
                )
            )

        return detections

    @staticmethod
    def best(detections: list[Detection]) -> Detection | None:
        """Pick the highest-confidence detection, or None if the list is empty.

        This is a simple starting policy. Once you have a real mission,
        you'll likely want smarter target selection - e.g. prefer the
        detection closest to the last known target position, to avoid
        the drone darting between multiple matches.
        """
        if not detections:
            return None
        return max(detections, key=lambda d: d.confidence)
