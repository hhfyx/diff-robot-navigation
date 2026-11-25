#!/usr/bin/env python3
"""定位入口：bringup（含 EKF）+ map_server + AMCL。

用法（headless）:
    ros2 launch diffbot_bringup localization.launch.py map:=/path/to/map.yaml \
        use_sim:=true gui:=false world:=/path/to/your.world

启动后需要给 AMCL 一个初始位姿（二选一）:
    RViz2 的 "2D Pose Estimate"
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{...}"

定位期间 map -> odom 由 AMCL 发布；不要同时启动 slam_toolbox。
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
    map_yaml = LaunchConfiguration("map")
    amcl_params = LaunchConfiguration("amcl_params")
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

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_yaml,
            "use_sim_time": use_sim_time,
        }],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params, {"use_sim_time": use_sim_time}],
    )

    # map_server 与 amcl 都是生命周期节点，必须由 manager 统一 activate
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
            "bond_timeout": 0.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value="",
            description="先验地图 yaml 路径（必填）"),
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
            "amcl_params",
            default_value=os.path.join(bringup_dir, "config", "amcl.yaml"),
            description="AMCL 参数文件"),
        DeclareLaunchArgument(
            "use_ros2_control", default_value="true",
            description="true=ros2_control 驱动分支（默认）；false=旧 gazebo_ros_diff_drive 分支"),

        bringup,
        map_server,
        amcl,
        lifecycle_manager,
    ])
