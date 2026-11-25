#!/usr/bin/env python3
"""Nav2 导航 + 在线 SLAM（仿真模式）。

与 nav.launch.py 的唯一区别：不启动 map_server / AMCL，改由 slam_toolbox
独占 map -> odom，并持续更新 /map。global_costmap 的 static layer 直接
订阅 slam_toolbox 发布的 /map，因此地图变化会进入规划。

用法:
    ros2 launch diffbot_nav nav_slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory("diffbot_nav")
    bringup_dir = get_package_share_directory("diffbot_bringup")

    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")
    slam_params = LaunchConfiguration("slam_params")
    autostart = LaunchConfiguration("autostart")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup.launch.py")),
        launch_arguments={
            "use_sim": use_sim,
            "use_sim_time": use_sim_time,
            "gui": gui,
            "world": world,
        }.items(),
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings + [("cmd_vel", "cmd_vel_nav"),
                                 ("cmd_vel_smoothed", "cmd_vel")],
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "autostart": autostart,
                     "bond_timeout": 0.0,
                     "node_names": ["controller_server",
                                    "smoother_server",
                                    "planner_server",
                                    "behavior_server",
                                    "bt_navigator",
                                    "velocity_smoother"]}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(nav_dir, "worlds", "mapping_quality.world")),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav_dir, "config", "nav2_params.yaml")),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(bringup_dir, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("autostart", default_value="true"),

        bringup,
        slam_toolbox,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager_navigation,
    ])
