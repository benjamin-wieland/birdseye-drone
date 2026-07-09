# Hardware open questions

Things worth nailing down soon, roughly in priority order:

## 1. Confirm the frame dimensions
The bounding box pulled from `Assembly_1.stl` (289 x 260 x 93 mm) assumes
the STL was exported in meters. If that's wrong, everything downstream
(payload budget, prop clearance, camera FOV coverage at a given altitude)
is off by a scale factor. Double check the export units in Onshape.

## 2. Flight controller decision
ESP32 support in ArduPilot is explicitly labeled experimental (no OEM
autopilot ships on it; you'd be wiring your own sensors to a dev board
or custom PCB). Two honest paths:
- Buy a proven FC (any ArduPilot/PX4-compatible board) and treat the
  ESP32 as a separate project or future upgrade.
- Commit to the ESP32 route knowing you're doing FC bring-up and vision
  integration as two hard projects at once.

Either way, `pi/mavlink_link.py` doesn't care which you choose - it just
needs a MAVLink-speaking flight controller on the other end of a serial
connection.

## 3. Altitude sensor
`get_altitude_m()` in `pi/main.py` only implements a fixed placeholder
value right now. Real options, roughly cheapest/simplest to most robust:
- Barometer already on the flight controller, read over MAVLink
  (noisy, drifts, but zero extra hardware).
- Downward-facing single-point rangefinder (e.g. TF-Mini, TF-Luna) -
  cheap, accurate at the low altitudes this task likely flies at.
- Downward optical flow + rangefinder combo module (e.g. PX4Flow) -
  overkill for just altitude, useful if you later want better hover
  stability too.

## 4. Camera selection and mount
Needs to be:
- Fixed nadir (or you'll need to add a gimbal + attitude compensation
  to the offset math, which is a real complexity jump).
- FOV known and entered into `config/default.yaml` accurately - this
  directly scales the meters-per-pixel math.
- Physically clear of props/landing gear in its downward view.

Any camera that works with `picamera2` (Pi Camera Module 3, or the
older HQ camera) is the path of least resistance for `pi/capture.py`.

## 5. Payload budget
Once motors/battery are chosen, revisit whether the ~150-250 g
companion-computer payload (Pi 4B + camera + wiring) fits your thrust
budget alongside the frame's own weight - see the estimate in the main
README.
