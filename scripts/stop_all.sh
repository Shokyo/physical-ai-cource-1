#!/usr/bin/env bash
# Stop every process of the demo (simulation, MoveIt, detector, viewers).
# never kill ourselves or the shell that called us (its command line may contain these words)
me="$$ $PPID $(ps -o ppid= -p $PPID | tr -d ' ')"
kill_pat() {
  for pid in $(pgrep -f -- "$1"); do
    case " $me " in *" $pid "*) ;; *) kill ${2:-} "$pid" 2>/dev/null ;; esac
  done
}
for pat in "ros2 launch gp8_sort_gazebo" "gz sim" "move_group" "rviz2" "parameter_bridge" "robot_state_publisher" "ros2_control_node" "gp8_sort_ai.yolo_detector" "rqt_image_view" "rqt_gui" "gp8_sort_ai.image_view" "pick_place"; do
  kill_pat "$pat"
done
sleep 2
for pat in "gz sim" "move_group" "rviz2" "rqt_image_view"; do kill_pat "$pat" -9; done
echo "stopped"
