#!/usr/bin/env python3
"""在 RViz2 中显示机器人模型，用于核对 URDF 与实车姿态是否一致。

用法:
    ros2 launch diff_chassis_description display.launch.py
    ros2 launch diff_chassis_description display.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("diff_chassis_description")

    xacro_file = os.path.join(pkg, "urdf", "diff_chassis.urdf.xacro")

    use_gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_gazebo:=false"]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="是否启动 joint_state_publisher_gui（拖动滑块看关节）"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="是否使用仿真时间"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),

        # GUI 模式：手动拖动关节滑块
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(use_gui),
        ),

        # 无 GUI：发布全零关节状态
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(pkg, "config", "display.rviz")],
        ),
    ])
