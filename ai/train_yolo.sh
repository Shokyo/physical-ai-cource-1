#!/usr/bin/env bash
# Fine-tune a small YOLO on the auto-labelled Gazebo dataset.
# Usage: ai/train_yolo.sh [dataset_dir] [epochs]
set -e
AI_DIR=$(cd "$(dirname "$0")" && pwd)
DATA=${1:-$AI_DIR/dataset}
EPOCHS=${2:-40}
cd "$AI_DIR/models"
"$AI_DIR/.venv/bin/yolo" detect train model=yolo11n.pt data="$DATA/data.yaml" epochs="$EPOCHS" imgsz=800 batch=16 \
  project="$AI_DIR/runs" name=gp8_sort exist_ok=True plots=True
cp "$AI_DIR/runs/gp8_sort/weights/best.pt" "$AI_DIR/models/gp8_sort_yolo.pt"
echo "saved $AI_DIR/models/gp8_sort_yolo.pt"
