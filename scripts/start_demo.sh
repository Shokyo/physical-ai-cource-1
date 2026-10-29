#!/usr/bin/env bash
# Start simulation + detector + image viewer in the background and arrange the windows for recording.
# Usage: scripts/start_demo.sh [logdir]
set -e
WS=$(cd "$(dirname "$0")/.." && pwd)
LOG=${1:-/tmp/gp8_sort_logs}; mkdir -p "$LOG"
source "$WS/install/setup.bash"
export DISPLAY=${DISPLAY:-:1}
setsid nohup ros2 launch gp8_sort_gazebo sim.launch.py > "$LOG/sim.log" 2>&1 &
echo "waiting for controllers ..."
for i in $(seq 1 40); do
  if timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "gp8_arm_controller.*active"; then break; fi
  sleep 5
done
setsid nohup "$WS/ai/.venv/bin/python" -m gp8_sort_ai.yolo_detector --ros-args -p use_sim_time:=true > "$LOG/detector.log" 2>&1 &
setsid nohup "$WS/ai/.venv/bin/python" -m gp8_sort_ai.image_view /detections/image "Overhead camera - YOLO" > "$LOG/viewer.log" 2>&1 &
sleep 20
wmctrl -r "Gazebo Sim" -b remove,maximized_vert,maximized_horz
wmctrl -r "Gazebo Sim" -e 0,0,0,1180,1150
wmctrl -r " - RViz" -b remove,maximized_vert,maximized_horz
wmctrl -r " - RViz" -e 0,1185,560,730,590
wmctrl -r "Overhead camera - YOLO" -e 0,1185,0,730,550
for w in "Gazebo Sim" " - RViz" "Overhead camera - YOLO"; do wmctrl -a "$w"; done
echo "demo stack running (logs in $LOG)"
