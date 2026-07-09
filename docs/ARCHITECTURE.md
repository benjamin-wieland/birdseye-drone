# Architecture

## Split of responsibility

| Layer | Runs on | Job | Speed requirement |
|---|---|---|---|
| Flight controller | ESP32 (or a proven FC board, TBD) | Attitude stabilization, motor mixing, holds altitude/level | Hundreds of Hz, hard real-time |
| Vision + targeting | Raspberry Pi 4B | Capture, detect target, compute velocity command | A few Hz is fine (5-10) |
| Ground station | Laptop/PC (optional) | Heavier models, mission planning, live monitoring | Not latency-critical for this task |

For the bird's-eye target-approach task specifically, the ground station
isn't required - a single-class detector at a few FPS runs fine on the
Pi 4B alone. It becomes more useful once you're running multiple object
classes, larger models, or adding a planning layer on top (e.g. search
patterns, multi-target prioritization).

## Why body-frame velocity, not GPS waypoints

The offset math in `pi/offset.py` produces a velocity command in the
drone's own body frame (forward/right relative to wherever it's currently
pointed), not a compass-referenced direction. This means:

- No magnetometer/yaw estimate needed on the Pi side.
- No GPS needed for this specific task - meaning it also works indoors
  or anywhere GPS is unreliable, as long as you have *some* altitude
  reference.
- The flight controller does need to accept offboard velocity setpoints
  in a mode like ArduPilot's GUIDED or PX4's OFFBOARD - that's a mode
  switch you'll trigger deliberately, not something this code does
  automatically.

## The altitude dependency

The whole pixel-to-meters conversion depends on knowing altitude
accurately (`pi/offset.py::meters_per_pixel`). Bad altitude data means
bad distance estimates, which means the drone either undershoots or
overshoots how far it thinks it needs to travel. This is the single
biggest accuracy lever in the whole pipeline - see
`docs/HARDWARE_TODO.md` for sensor options.

## Where this goes next

Once the basic approach-and-hover loop works reliably:

1. **Smoothing** - the current controller reacts to every frame's raw
   detection, which will be jittery. A simple moving average or Kalman
   filter on the detected position is the first upgrade.
2. **Search behavior** - right now, "no detection" just holds position.
   A real mission needs a search pattern (e.g. slow spiral or lawnmower)
   when the target isn't yet visible.
3. **Multi-target logic** - `Detector.best()` currently just picks the
   highest-confidence detection. Worth revisiting once you have more
   than one class or need to track a specific target across frames
   rather than re-acquiring the "best" one each frame.
4. **Obstacle awareness** - this stack assumes clear airspace between
   the drone and the target. Not needed for a first working version,
   but worth flagging before flying anywhere with obstacles.
