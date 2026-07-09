"""Entry point: capture -> detect -> compute offset -> send velocity, repeat.

Usage:
    python3 -m pi.main --config config/default.yaml --source webcam --dry-run
    python3 -m pi.main --config config/default.yaml --source picamera
"""

from __future__ import annotations

import argparse
import time

import yaml

from pi.capture import make_source
from pi.detector import Detector
from pi.mavlink_link import MavlinkLink
from pi.offset import (
    distance_to_target_m,
    pixel_offset_to_world_offset,
    world_offset_to_velocity,
)


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def get_altitude_m(cfg: dict) -> float:
    """Look up current altitude per the configured source.

    Only "fixed" is implemented so far - it's a placeholder good enough
    for bench-testing the vision/math pipeline. Wire up "rangefinder" or
    "baro" here once you've picked and mounted a real altitude sensor;
    see docs/HARDWARE_TODO.md.
    """
    source = cfg["altitude"]["source"]
    if source == "fixed":
        return cfg["altitude"]["fixed_altitude_m"]
    raise NotImplementedError(
        f"Altitude source '{source}' is not implemented yet. "
        "Only 'fixed' works today - see docs/HARDWARE_TODO.md."
    )


def run(cfg: dict, source_name: str, dry_run: bool) -> None:
    cam_cfg = cfg["camera"]
    frame_source = make_source(source_name, cam_cfg["frame_width"], cam_cfg["frame_height"])

    detector = Detector(
        model_path=cfg["detection"]["model_path"],
        target_classes=cfg["detection"]["target_classes"],
        confidence_threshold=cfg["detection"]["confidence_threshold"],
    )

    link = MavlinkLink(
        connection_string=cfg["mavlink"]["connection"],
        baud=cfg["mavlink"]["baud"],
        dry_run=dry_run,
    )

    period_s = 1.0 / cam_cfg["loop_rate_hz"]

    print("Starting loop. Ctrl+C to stop.")
    try:
        while True:
            loop_start = time.time()

            frame = frame_source.read()
            detections = detector.detect(frame)
            target = Detector.best(detections)

            if target is None:
                # Nothing found this frame - hold position rather than
                # guessing. A real mission would add a search pattern here.
                link.send_hold()
            else:
                altitude_m = get_altitude_m(cfg)
                right_m, forward_m = pixel_offset_to_world_offset(
                    center_x=target.center_x,
                    center_y=target.center_y,
                    frame_width=cam_cfg["frame_width"],
                    frame_height=cam_cfg["frame_height"],
                    altitude_m=altitude_m,
                    hfov_deg=cam_cfg["horizontal_fov_deg"],
                    vfov_deg=cam_cfg["vertical_fov_deg"],
                )

                distance_m = distance_to_target_m(right_m, forward_m)
                arrival_radius_m = cfg["controller"]["arrival_radius_m"]

                if distance_m <= arrival_radius_m:
                    print(f"Arrived - {target.class_name} within {arrival_radius_m} m. Holding.")
                    link.send_hold()
                else:
                    cmd = world_offset_to_velocity(
                        right_m=right_m,
                        forward_m=forward_m,
                        kp=cfg["controller"]["kp"],
                        max_speed_mps=cfg["controller"]["max_speed_mps"],
                    )
                    print(
                        f"{target.class_name} ({target.confidence:.2f}) "
                        f"offset=({right_m:+.2f}m right, {forward_m:+.2f}m fwd) "
                        f"dist={distance_m:.2f}m -> "
                        f"cmd=(fwd={cmd.forward_mps:+.2f}, right={cmd.right_mps:+.2f})"
                    )
                    link.send_body_velocity(cmd.forward_mps, cmd.right_mps)

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, period_s - elapsed))

    except KeyboardInterrupt:
        print("\nStopping - sending hold command before exit.")
        link.send_hold()
    finally:
        frame_source.close()
        link.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="Path to a YAML config file")
    parser.add_argument(
        "--source", choices=["webcam", "picamera"], required=True,
        help="Frame source: 'webcam' for development, 'picamera' on the drone",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print velocity commands instead of sending them over MAVLink",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    run(cfg, source_name=args.source, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
