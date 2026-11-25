#!/usr/bin/env python3
"""基础 bringup：机器人描述 + 底盘（仿真或实车）+ robot_localization EKF。

这是 mapping / localization 的公共底座，也可单独启动用于遥控测试。

仿真（headless）:
    ros2 launch diffbot_bringup bringup.launch.py \
        use_sim:=true gui:=false world:=/path/to/your.world

仿真（带界面）:
    ros2 launch diffbot_bringup bringup.launch.py use_sim:=true

实车:
    ros2 launch diffbot_bringup bringup.launch.py use_sim:=false

EKF 约定（规格书 4.4 节）：
    输入  /wheel/odom + /imu/data
    输出  /odometry/filtered，且独占发布 odom -> base_footprint。
    因此 diff_chassis.urdf.xacro 中差速插件的 publish_odom_tf 必须为 false。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("diffbot_bringup")
    chassis_dir = get_package_share_directory("diff_chassis_description")

    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    world = LaunchConfiguration("world")
    ekf_params = LaunchConfiguration("ekf_params")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    wheel_odom_topic = LaunchConfiguration("wheel_odom_topic")

    xacro_file = os.path.join(chassis_dir, "urdf", "diff_chassis.urdf.xacro")

    # ---------- 仿真底盘：复用 diff_chassis_description 的 gazebo.launch.py ----------
    # 注意：该 launch 已包含 robot_state_publisher，不要重复启动，否则 /tf 冲突。
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(chassis_dir, "launch", "gazebo.launch.py")),
        condition=IfCondition(use_sim),
        launch_arguments={
            "world": world,
            "gui": gui,
            "verbose": verbose,
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
        }.items(),
    )

    # ---------- 实车：只起 robot_state_publisher ----------
    # 关节状态将由后续的 diffbot_control 硬件接口提供。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=UnlessCondition(use_sim),
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", xacro_file, " use_gazebo:=false"]),
                value_type=str),
            "use_sim_time": use_sim_time,
        }],
    )

    # ---------- EKF：仿真与实车共用同一份配置 ----------
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params,
                    {"use_sim_time": use_sim_time,
                     "odom0": wheel_odom_topic}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim", default_value="true",
            description="true=启动 Gazebo 仿真，false=接实车底盘"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="使用仿真时间（实车请设 false）"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="是否启动 Gazebo 客户端"),
        DeclareLaunchArgument(
            "verbose", default_value="false",
            description="Gazebo 是否输出详细日志"),
        DeclareLaunchArgument(
            "world", default_value="",
            description="Gazebo world 路径；留空会用到 gazebo 默认空世界"),
        DeclareLaunchArgument(
            "ekf_params", default_value=os.path.join(bringup_dir, "config", "ekf.yaml"),
            description="robot_localization 参数文件"),
        DeclareLaunchArgument(
            "use_ros2_control", default_value="true",
            description="仿真底盘驱动分支：true=ros2_control，false=旧 gazebo_ros_diff_drive"),
        DeclareLaunchArgument(
            "wheel_odom_topic", default_value="/wheel/odom",
            description="EKF 的轮式里程计输入话题；两分支都归一化为 /wheel/odom，"
                        "若 diff_drive_controller 的实际话题名不同可在此覆盖"),

        sim_bringup,
        robot_state_publisher,
        ekf_node,
    ])
