#!/usr/bin/env python3
"""建图入口：bringup（含 EKF）+ slam_toolbox（online async）。

用法（headless）:
    ros2 launch diffbot_bringup mapping.launch.py \
        use_sim:=true gui:=false world:=/path/to/your.world

建图期间 map -> odom 由 slam_toolbox 发布；不要同时启动 AMCL。
保存地图:
    ros2 run nav2_map_server map_saver_cli -f <前缀> --ros-args -p use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("diffbot_bringup")

    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    slam_params = LaunchConfiguration("slam_params")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup.launch.py")),
        launch_arguments={
            "use_sim": use_sim,
            "use_sim_time": use_sim_time,
            "gui": gui,
            "world": world,
            "use_ros2_control": use_ros2_control,
        }.items(),
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim", default_value="true",
            description="true=Gazebo 仿真，false=实车"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="使用仿真时间（实车请设 false）"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="是否启动 Gazebo 客户端"),
        DeclareLaunchArgument(
            "world", default_value="",
            description="Gazebo world 路径"),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(bringup_dir, "config", "slam_toolbox.yaml"),
            description="slam_toolbox 参数文件"),
        DeclareLaunchArgument(
            "use_ros2_control", default_value="true",
            description="true=ros2_control 驱动分支（默认）；false=旧 gazebo_ros_diff_drive 分支"),

        bringup,
        slam_toolbox,
    ])
