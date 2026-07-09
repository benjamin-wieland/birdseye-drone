# birdseye-drone

Autonomous target-approach for a custom-built drone: a downward-facing (nadir)
camera spots an object from altitude, and the drone flies itself toward it.

## What this does (and doesn't) do

This is **not** end-to-end learned flight. Low-level stabilization (staying
level, holding altitude) stays on the flight controller, where that problem
is already solved. This stack sits above the flight controller and handles
one job: *look down, find the target, tell the flight controller which way
to fly to get there.*

The loop, once running:

```
capture frame -> detect target -> pixel offset -> body-frame velocity -> send to FC
                       ^                                                     |
                       +---------------------- repeat every frame -----------+
```

## Architecture

- **Companion computer (Raspberry Pi 4B)** — runs everything in `pi/`.
  Captures video, runs the detector, computes the velocity command, and
  sends it to the flight controller over MAVLink (serial link).
- **Flight controller** — handles attitude stabilization and motor output.
  Receives velocity setpoints from the Pi; does not run any vision itself.
- **Ground station (optional)** — not required for this task. Useful later
  if you add more object classes, mission logic, or want to watch a live
  feed while testing.

See `docs/ARCHITECTURE.md` for the fuller picture.

## Assumptions baked in right now (change these before flying)

These came from analyzing your STL export — confirm/correct them in
`config/default.yaml` before doing anything with a real motor spinning:

| Assumption | Current value | Source |
|---|---|---|
| Frame footprint | ~289 x 260 x 93 mm | Measured from `Assembly_1.stl` bounding box, **assuming the export units were meters** — please confirm |
| Camera orientation | Fixed nadir (straight down) | Stated goal, not yet in your CAD |
| Altitude source | Placeholder / not yet chosen | See `docs/HARDWARE_TODO.md` |
| Flight controller | Undecided (ESP32 route is experimental in ArduPilot — see prior discussion) | Open |
| Target class | Single configurable class (e.g. "person") | Placeholder in config |

## Repo layout

```
pi/                  companion computer software (runs on the Pi 4B)
  capture.py          camera frame source (Pi camera or webcam fallback for dev)
  detector.py          object detection wrapper (Ultralytics YOLO)
  offset.py            pixel offset -> body-frame velocity math
  mavlink_link.py       MAVLink connection + velocity setpoint sender
  main.py               ties the loop together
config/
  default.yaml          all tunable parameters live here
tests/
  test_offset.py         unit tests for the offset/velocity math (no hardware needed)
docs/
  ARCHITECTURE.md        system architecture writeup
  HARDWARE_TODO.md        open hardware questions to resolve
```

## Getting started (development, no hardware required)

You can develop and test the vision + math pipeline on a laptop before any
of it touches the drone:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Run the offset-math unit tests (no camera/FC needed)
python3 -m pytest tests/

# Run the loop against a laptop webcam, printing commands instead of sending them
python3 -m pi.main --config config/default.yaml --dry-run --source webcam
```

## Running on the Pi (once hardware is ready)

```bash
python3 -m pi.main --config config/default.yaml --source picamera
```

`--dry-run` prints the velocity commands it would send instead of sending
them over MAVLink — use this for your first real flight-adjacent test with
props off.

## Safety

- First runs: props off, watch the printed velocity commands (`--dry-run`),
  confirm they point the right direction before trusting them near motors.
- Keep a manual RC override / kill switch active at all times during testing.
  This stack should never be the only thing standing between the drone and
  a crash while you're developing it.
- `max_speed_mps` in the config is a hard clamp on commanded velocity —
  keep it small until you trust the pipeline.
