#!/usr/bin/env python3
"""Nav2 导航（仿真模式）：复用 diffbot_bringup 底座（Gazebo + 底盘 + EKF），
再启动 map_server / AMCL / planner / controller / smoother / behavior /
bt_navigator / velocity_smoother / lifecycle_manager。

headless 用法:
    ros2 launch diffbot_nav nav.launch.py

带 Gazebo 界面:
    ros2 launch diffbot_nav nav.launch.py gui:=true

话题链:
    controller_server / behavior_server --cmd_vel_nav--> velocity_smoother
    velocity_smoother --/cmd_vel--> 底盘（gazebo_ros_diff_drive）
    EKF --/odometry/filtered--> AMCL / velocity_smoother / bt_navigator
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
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    # 底座：Gazebo（可关界面）+ 机器人描述 + 差速驱动 + IMU + EKF
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

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"yaml_filename": map_yaml,
                                  "use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

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

    # 生命周期：先定位（map_server/amcl），再导航栈
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "autostart": autostart,
                     "bond_timeout": 0.0,
                     "node_names": ["map_server", "amcl"]}],
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
            "map",
            default_value=os.path.join(nav_dir, "maps", "room_map.yaml")),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav_dir, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "use_ros2_control", default_value="true",
            description="true=ros2_control 驱动分支（默认）；false=旧 gazebo_ros_diff_drive 分支"),

        bringup,
        map_server,
        amcl,
        lifecycle_manager_localization,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager_navigation,
    ])
