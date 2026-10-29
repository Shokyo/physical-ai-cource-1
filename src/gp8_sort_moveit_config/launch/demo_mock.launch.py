"""Fallback demo without Gazebo: mock hardware + ros2_control + move_group + RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = get_package_share_directory("gp8_sort_description")
    moveit_pkg = get_package_share_directory("gp8_sort_moveit_config")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare("gp8_sort_description"), "urdf", "gp8_sort.urdf.xacro"]),
            " sim:=mock",
        ]),
        value_type=str,
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(desc_pkg, "config", "ros2_controllers.yaml")],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    arm = Node(package="controller_manager", executable="spawner", arguments=["gp8_arm_controller"])

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_pkg, "launch", "move_group.launch.py")),
        launch_arguments={"use_sim_time": "false", "sim": "mock", "rviz": "true"}.items(),
    )

    return LaunchDescription([
        rsp,
        control_node,
        jsb,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        move_group,
    ])
