#!/usr/bin/env python3
"""Nav2 导航（仿真 + Gazebo GUI + RViz2 交互点目标）。

用法:
    DISPLAY=:0 ros2 launch diffbot_nav nav_gui.launch.py

RViz 里用工具栏的 "Nav2 Goal" 按钮在地图上点目标点即可。
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

    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    rviz_config = LaunchConfiguration("rviz_config")

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, "launch", "nav.launch.py")),
        launch_arguments={
            "use_sim": "true",
            "use_sim_time": "true",
            "gui": "true",
            "world": world,
            "map": map_yaml,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(nav_dir, "worlds", "mapping_quality.world")),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(nav_dir, "maps", "room_map.yaml")),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(nav_dir, "rviz", "nav2.rviz")),
        nav,
        rviz,
    ])
