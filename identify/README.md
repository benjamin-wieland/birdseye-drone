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

## 5. Training on a custom object instead

If down the line you want to detect something specific (e.g. a landing pad
or a particular target) rather than general COCO classes, that's a
different path — collecting your own images and fine-tuning or training a
small custom model (transfer learning off MobileNet is realistic on a
laptop/cloud GPU, not on the Pi itself). Let me know if you want to go that
route and I can lay out that workflow separately.
