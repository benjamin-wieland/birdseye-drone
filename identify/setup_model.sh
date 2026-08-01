#!/usr/bin/env bash
# Run this once on the Raspberry Pi to fetch the pretrained model + labels.
set -e

mkdir -p models
cd models

echo "Downloading SSD MobileNet V2 (COCO, quantized TFLite)..."
wget -O ssd_mobilenet_v2_coco_quant.tflite \
  "https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip" \
  -q --show-progress || true

# The above is a zip; unzip and rename the pieces we need.
if [ -f ssd_mobilenet_v2_coco_quant.tflite ]; then
    mv ssd_mobilenet_v2_coco_quant.tflite model.zip
    unzip -o model.zip
    mv detect.tflite ssd_mobilenet_v2_coco_quant.tflite 2>/dev/null || true
    mv labelmap.txt coco_labels.txt 2>/dev/null || true
    rm -f model.zip
fi

echo "Done. Files in ./models:"
ls -la
