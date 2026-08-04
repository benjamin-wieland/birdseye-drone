# RPi 4B Object Detection for Drone Project

Live object detection using the Camera Module (CSI) + TensorFlow Lite, tuned to
actually run at usable speed on a Pi 4B's CPU.

## 1. One-time setup on the Pi

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv unzip wget
pip install tflite-runtime --break-system-packages

cd rpi_object_detection
chmod +x setup_model.sh
./setup_model.sh
```

This downloads a pretrained **SSD MobileNet V2** model trained on the COCO
dataset (80 everyday object classes: person, car, dog, backpack, bottle,
etc.) into `models/`.

If `setup_model.sh` fails to fetch the file (network changes on Google's
end happen occasionally), grab any COCO SSD MobileNet `.tflite` +
`labelmap.txt` pair from the [TFLite examples
repo](https://github.com/tensorflow/examples/tree/master/lite/examples/object_detection)
and drop them in `models/` with the filenames the script expects.

## 2. Run it

```bash
python3 detect.py
```

Add `--headless` if you're SSH'd in without a display:

```bash
python3 detect.py --headless
```

You'll see console output like:

```
[14:02:11] person (0.87) centroid=(320, 210)
[14:02:11] backpack (0.61) centroid=(410, 300)
```

## 3. Performance expectations

- ~5-10 FPS on a Pi 4B at 640x480 capture / 300x300 inference, CPU only.
- If that's too slow for your drone's reaction time, next steps in order of effort:
  1. Drop capture resolution further (e.g. 416x416).
  2. Add a **Coral USB Accelerator** ($25-60) — biggest single speed win, minimal code change (swap to the Edge TPU delegate).
  3. Move to a smaller/faster model (EfficientDet-Lite0, or YOLOv8n exported to NCNN).

## 4. Hooking into the drone logic

Right now `run_inference` / `draw_and_report` return a `detections` list of
dicts with `label`, `confidence`, `box`, and `centroid`. That's the natural
place to plug in flight behavior — e.g.:

```python
for d in detections:
    if d["label"] == "person" and d["confidence"] > 0.6:
        # send a MAVLink command, adjust heading toward d["centroid"], etc.
        ...
```

If your flight controller uses MAVLink (Pixhawk/ArduPilot/PX4), `pymavlink`
or `dronekit` are the usual libraries to send commands from this same
script or a companion process. Happy to help wire that up once detection
is running reliably — that's a good next step once you've confirmed FPS
and accuracy are acceptable for your use case.

## 5. MAVLink integration (Pixhawk + ArduPilot)

This lets the drone react to what it sees — the vision pipeline sends
velocity commands to the flight controller via `pymavlink`.

### Install

```bash
pip install pymavlink --break-system-packages
```

### Test in simulation first (SITL) — do this before touching real hardware

1. On a laptop/desktop (or the Pi, if it has the headroom), install ArduPilot SITL:
   ```bash
   git clone https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   Tools/environment_install/install-prereq-ubuntu.sh -y
   ./waf configure --board sitl
   ./waf copter
   sim_vehicle.py -v ArduCopter --console --map
   ```
2. This starts a simulated copter broadcasting MAVLink on `udp:127.0.0.1:14550`.
3. Run the detection script pointed at it:
   ```bash
   python3 detect.py --mavlink --mavlink-connection udp:127.0.0.1:14550
   ```
   By default this is **dry-run** — commands print to console instead of
   being sent. Confirm the printed velocity commands make sense (they
   should nudge toward centering whatever `--track-label` you set, default
   `person`) before ever adding `--mavlink-live`.

### Wiring to real hardware (once SITL behavior looks correct)

Two common options:

**Option A — Pi GPIO UART to Pixhawk TELEM2** (no extra hardware, but uses GPIO pins):
- Wire Pi GPIO pins 8 (TXD) → Pixhawk TELEM2 RX, Pi pin 10 (RXD) → TELEM2 TX, plus a shared GND.
- On the Pi: disable the serial console but keep the UART enabled (`sudo raspi-config` → Interface Options → Serial Port → login shell: No, hardware enabled: Yes).
- On ArduPilot (via Mission Planner/QGroundControl params): set `SERIAL2_PROTOCOL = 2` (MAVLink2) and `SERIAL2_BAUD` to match (commonly 921600 or 57600 — start at 57600 if you get dropped connections).
- Connection string: `/dev/serial0` at the matching baud rate.

**Option B — USB telemetry radio / USB-to-Pixhawk cable** (simpler, recommended if you have the hardware):
- Connection string: `/dev/ttyUSB0` (or `/dev/ttyACM0` depending on the adapter).

### Going live — safety checklist

- [ ] Verified expected behavior in SITL first
- [ ] Props removed for first hardware test
- [ ] Flight controller in **GUIDED mode** (required for velocity commands to take effect — otherwise they're ignored, which is a reasonable safety default)
- [ ] Someone on a separate RC transmitter ready to override/switch modes
- [ ] Start with `--mavlink-live` only after all of the above

The `centroid_to_velocity()` function in `mavlink_bridge.py` is a basic
proportional controller — it nudges toward centering the tracked object in
frame but has no distance/altitude estimation and no PID tuning. Treat it
as a starting point, not a finished control law.

## 7. Offloading detection to your laptop (Pi → laptop streaming)

If the Pi's CPU is too slow for smooth detection, you can move the heavy
inference to your laptop instead: the Pi just captures and streams frames,
your laptop does the detection, and sends results back so the drone can
still react via MAVLink.

### Setup

**On the Pi**, files needed: `pi_stream_server.py`, `mavlink_bridge.py`.
No model files needed here anymore — the Pi doesn't run inference.

**On your laptop**, copy over:
- `laptop_client.py`

No model files to copy anymore — YOLOv8 weights auto-download on first run.

Install on the laptop:
```bash
pip install opencv-python numpy ultralytics
```

### Find the Pi's IP address

On the Pi:
```bash
hostname -I
```
Use the first address shown (usually starts with `192.168.` on a home network).

### Run it

**On the Pi**, start the server first (it waits for your laptop to connect):
```bash
python3 pi_stream_server.py
```
Add `--mavlink` to enable the flight-command relay (same dry-run-by-default
safety behavior as before — see section 5 before adding `--mavlink-live`).

**On your laptop**, connect to it:
```bash
python3 laptop_client.py --pi-ip 192.168.1.42
```
(replace with the IP `hostname -I` gave you)

You should see a live annotated video window on your laptop, and the Pi's
terminal will start printing detections it received back.

### What to expect

- This moves the bottleneck from Pi-CPU-lag to network-lag. On a normal
  home WiFi network with both devices close to the router, this is
  typically much smoother than the pure on-Pi version, but isn't
  zero-latency — expect some delay between what the camera sees and what
  shows on your laptop.
- Both devices need to be reachable over the same network. If you're
  routing through a VPN, guest network, or separate subnets, they may not
  be able to reach each other directly.
- If frames get further behind under a slow network, that shows up as
  increasing lag in the video window rather than dropped frames — this
  simple version doesn't include a frame-skipping/catch-up mechanism yet.

### Model accuracy vs. speed (laptop side)

The laptop client uses YOLOv8, which is meaningfully more accurate than
the SSD MobileNet model from the Pi-only setup — worth it now that
inference isn't CPU-constrained to the Pi. Three weight sizes to choose
from, biggest tradeoff is speed vs. accuracy:

| Model | Relative speed | Relative accuracy |
|---|---|---|
| `yolov8n.pt` (default) | Fastest | Good |
| `yolov8s.pt` | Slower | Better |
| `yolov8m.pt` | Slowest | Best |

Try `yolov8n.pt` first — if it's still fast enough on your laptop and you
want better accuracy, bump up:
```bash
python3 laptop_client.py --pi-ip 192.168.1.42 --model yolov8s.pt
```

You can also tune `--conf` (confidence threshold, default 0.5). Lowering
it (e.g. `--conf 0.3`) surfaces more detections but increases false
positives; raising it does the opposite.

## 8. Training on a custom object instead

If down the line you want to detect something specific (e.g. a landing pad
or a particular target) rather than general COCO classes, that's a
different path — collecting your own images and fine-tuning or training a
small custom model (transfer learning off MobileNet is realistic on a
laptop/cloud GPU, not on the Pi itself). Let me know if you want to go that
route and I can lay out that workflow separately.
