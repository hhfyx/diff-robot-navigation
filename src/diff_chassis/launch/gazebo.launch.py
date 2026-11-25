#!/usr/bin/env python3
"""在 Gazebo 中启动机器人（含激光雷达与 IMU），不含导航。

用法:
    ros2 launch diff_chassis_description gazebo.launch.py
    ros2 launch diff_chassis_description gazebo.launch.py world:=/path/to/my.world

启动后可用键盘遥控:
    另开终端: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("diff_chassis_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    xacro_file = os.path.join(pkg, "urdf", "diff_chassis.urdf.xacro")

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    # 让 Gazebo 能找到 package:// 里的网格
    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.join(pkg, ".."),
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_gazebo:=true",
                 " use_ros2_control:=", use_ros2_control]),
        value_type=str,
    )

    # Gazebo 服务端
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={
            "world": world,
            "verbose": verbose,
        }.items(),
    )

    # Gazebo 客户端（可关掉跑无界面）
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")),
        condition=IfCondition(gui),
    )

    # 把机器人 spawn 到 Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "diff_chassis",
            # 稍微抬高一点，避免初始就穿透地面
            "-z", "0.05",
        ],
        output="screen",
    )

    # ros2_control 分支：spawn 完成后依次激活 joint_state_broadcaster 与 diff_drive_controller。
    # 与 gazebo_ros_diff_drive 分支互斥（use_ros2_control:=false 时不执行）。
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "diff_drive_controller"],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="",
            description="Gazebo world 文件路径，留空则用空世界"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="使用仿真时间"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="是否启动 Gazebo 客户端界面"),
        DeclareLaunchArgument(
            "verbose", default_value="false",
            description="Gazebo 是否输出详细日志"),
        DeclareLaunchArgument(
            "use_ros2_control", default_value="true",
            description="true=controller_manager/diff_drive_controller；"
                        "false=旧 gazebo_ros_diff_drive 分支（回退/对照用）"),

        set_model_path,
        gzserver,
        gzclient,

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

        spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_controller],
            )
        ),
    ])
