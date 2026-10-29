"""Gazebo (gz-sim) simulation of the GP8 sorting cell.

Starts: Gazebo + world, robot_state_publisher, robot spawn, ros_gz bridge,
ros2_control controllers, move_group and RViz.

Arguments:
  gui    start the Gazebo GUI (default true)
  rviz   start RViz (default true)
  world  world file (default: sort_table.sdf of this package)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PICK_OBJECTS = ["red_cube", "green_cube", "blue_cylinder"]


def generate_launch_description():
    pkg = get_package_share_directory("gp8_sort_gazebo")
    desc_pkg = get_package_share_directory("gp8_sort_description")
    moveit_pkg = get_package_share_directory("gp8_sort_moveit_config")
    gp8_pkg = get_package_share_directory("motoman_gp8_support")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")

    # let Gazebo resolve package:// mesh URIs
    resource_path = os.pathsep.join([
        os.path.dirname(gp8_pkg),
        os.path.dirname(desc_pkg),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ])
    set_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                "'-r ' + ('--gui-config ", os.path.join(pkg, "config", "gui.config"),
                " ' if '", gui, "' == 'true' else '-s ') + '", world, "'"
            ]),
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare("gp8_sort_description"), "urdf", "gp8_sort.urdf.xacro"]),
            " sim:=gazebo",
            " controllers_file:=", os.path.join(desc_pkg, "config", "ros2_controllers.yaml"),
            " pick_objects:='", " ".join(PICK_OBJECTS), "'",
        ]),
        value_type=str,
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "gp8", "-x", "0", "-y", "0", "-z", "0"],
    )

    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/scene_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
    ]
    for name in PICK_OBJECTS:
        bridge_args.append(f"/gp8/attach/{name}@std_msgs/msg/Empty]gz.msgs.Empty")
        bridge_args.append(f"/gp8/detach/{name}@std_msgs/msg/Empty]gz.msgs.Empty")
        bridge_args.append(f"/gp8/attached/{name}@std_msgs/msg/String[gz.msgs.StringMsg")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=bridge_args,
        parameters=[{"use_sim_time": True}],
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "120"],
        output="screen",
    )
    arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gp8_arm_controller", "--controller-manager-timeout", "120"],
        output="screen",
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_pkg, "launch", "move_group.launch.py")),
        launch_arguments={"use_sim_time": "true", "sim": "gazebo", "rviz": rviz}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("world", default_value=os.path.join(pkg, "worlds", "sort_table.sdf")),
        set_resource_path,
        gz_sim,
        rsp,
        spawn,
        bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        move_group,
    ])
