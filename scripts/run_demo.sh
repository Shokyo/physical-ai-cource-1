#!/usr/bin/env bash
# Run the sorting task N times with a freshly randomised scene each time.
# Usage: scripts/run_demo.sh [runs=3] [seed0=1]
set -e
WS=$(cd "$(dirname "$0")/.." && pwd)
RUNS=${1:-3}; SEED=${2:-1}
source "$WS/install/setup.bash"
PY="$WS/ai/.venv/bin/python"
for i in $(seq 1 "$RUNS"); do
  echo "=== run $i / $RUNS (seed $((SEED+i-1)))"
  "$PY" -m gp8_sort_ai.scene_tool random --seed $((SEED+i-1))
  sleep 3
  ros2 run gp8_sort_ai pick_place --ros-args -p use_sim_time:=true
  sleep 2
done
echo "=== demo finished"
