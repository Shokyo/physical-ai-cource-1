"""move_group (+ optional RViz) for the GP8 sorting cell.

Arguments:
  use_sim_time  true when running with Gazebo (default true)
  sim           gazebo | mock   (passed to the xacro, default gazebo)
  rviz          start RViz with the MoveIt motion planning plugin (default true)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    sim = LaunchConfiguration("sim").perform(context)

    moveit_config = (
        MoveItConfigsBuilder("gp8_sort", package_name="gp8_sort_moveit_config")
        .robot_description(file_path="config/gp8_sort.urdf.xacro", mappings={"sim": sim})
        .robot_description_semantic(file_path="config/gp8_sort.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("gp8_sort_moveit_config"), "rviz", "moveit.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("sim", default_value="gazebo"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
