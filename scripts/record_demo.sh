#!/usr/bin/env bash
# Record the demo off-screen: start the topic recorder, run N randomised sorting
# runs, stop the recorder and compose the final video.
# Usage: scripts/record_demo.sh [runs=3] [seed0=1] [out=media/PAI_final_demo.mp4]
set -e
WS=$(cd "$(dirname "$0")/.." && pwd)
RUNS=${1:-3}; SEED=${2:-1}; OUT=${3:-$WS/media/PAI_final_demo.mp4}
REC=$WS/media/rec
source "$WS/install/setup.bash"
rm -rf "$REC"
"$WS/ai/.venv/bin/python" -m gp8_sort_ai.record_topics --out "$REC" --fps 20 /scene_camera/image /detections/image &
RP=$!
sleep 4
T0=$(date +%s)
"$WS/scripts/run_demo.sh" "$RUNS" "$SEED"
sleep 2
DUR=$(( $(date +%s) - T0 + 4 ))
kill -TERM $RP; wait $RP || true
"$WS/scripts/compose_video.sh" "$REC" "$OUT" "$DUR"
