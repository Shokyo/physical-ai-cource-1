"""Start the pick-and-place node (and optionally the detector in the YOLO venv).

  ros2 launch gp8_sort_ai sort.launch.py                 # pick_place only
  ros2 launch gp8_sort_ai sort.launch.py detector:=true  # also start yolo_detector with venv_python
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    venv_python = LaunchConfiguration("venv_python")
    weights_dir = LaunchConfiguration("weights_dir")
    detector = ExecuteProcess(
        cmd=[venv_python, "-m", "gp8_sort_ai.yolo_detector", "--ros-args",
             "-p", ["weights_dir:=", weights_dir], "-p", "use_sim_time:=true"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("detector")),
    )
    pick_place = Node(
        package="gp8_sort_ai",
        executable="pick_place",
        output="screen",
        parameters=[{"use_sim_time": True, "dry_run": LaunchConfiguration("dry_run")}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("detector", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("venv_python", default_value=os.path.expanduser("~/pai_final_ws/ai/.venv/bin/python")),
        DeclareLaunchArgument("weights_dir", default_value=os.path.expanduser("~/pai_final_ws/ai/models")),
        detector,
        pick_place,
    ])
