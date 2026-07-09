"""Frame sources for the vision loop.

Two backends:
  - "picamera": the Raspberry Pi camera module, via picamera2. Use this
    on the actual drone.
  - "webcam": any OpenCV-compatible camera. Use this on a laptop while
    developing the detection/offset/controller logic before you have
    the drone hardware in front of you.

Both expose the same interface: a `read()` method returning a BGR numpy
frame, and `close()`.
"""

from __future__ import annotations

import numpy as np


class WebcamSource:
    """Fallback frame source for development on a laptop."""

    def __init__(self, width: int, height: int, device_index: int = 0):
        import cv2  # local import so this module doesn't hard-require cv2

        self._cv2 = cv2
        self.cap = cv2.VideoCapture(device_index)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open webcam at index {device_index}. "
                "Check the camera is connected and not in use elsewhere."
            )
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self) -> np.ndarray:
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("Failed to read frame from webcam.")
        return frame

    def close(self) -> None:
        self.cap.release()


class PiCameraSource:
    """Raspberry Pi camera module via picamera2. Use on the drone itself."""

    def __init__(self, width: int, height: int):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "picamera2 is not installed. It ships with Raspberry Pi OS "
                "(Bullseye or later) - install with "
                "`sudo apt install python3-picamera2` on the Pi itself, "
                "or use --source webcam for development off the Pi."
            ) from exc

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def read(self) -> np.ndarray:
        # picamera2 delivers RGB; downstream (OpenCV-based detector) expects
        # BGR, so flip channel order here in one place.
        frame_rgb = self.picam2.capture_array()
        return frame_rgb[:, :, ::-1]

    def close(self) -> None:
        self.picam2.stop()


def make_source(name: str, width: int, height: int):
    """Factory: build a frame source by name from config/CLI."""
    if name == "webcam":
        return WebcamSource(width, height)
    if name == "picamera":
        return PiCameraSource(width, height)
    raise ValueError(f"Unknown frame source: {name!r} (expected 'webcam' or 'picamera')")
