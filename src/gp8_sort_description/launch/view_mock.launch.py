"""Fallback / debugging launch: GP8 with ros2_control mock hardware + RViz (no Gazebo)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("gp8_sort_description")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")

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
        parameters=[os.path.join(pkg, "config", "ros2_controllers.yaml")],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    arm = Node(package="controller_manager", executable="spawner", arguments=["gp8_arm_controller"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg, "rviz", "view.rviz")],
        condition=None,
    )

    return LaunchDescription([
        rviz_arg,
        rsp,
        control_node,
        jsb,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        rviz,
    ])
