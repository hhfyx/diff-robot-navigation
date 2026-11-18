#!/usr/bin/env bash
# ============================================================================
# 一键安装 ROS 2 Humble 导航相关依赖
#
# 适用：Ubuntu 22.04 + 已按官方文档装好 ROS 2 Humble
# 用法：bash scripts/install_deps.sh
# ============================================================================
set -euo pipefail

echo "==> 检查 ROS 2 Humble 是否已安装"
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "错误：未找到 /opt/ros/humble/setup.bash"
    echo "请先按官方文档安装 ROS 2 Humble："
    echo "  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    exit 1
fi

echo "==> 安装导航与仿真依赖（需要 sudo 密码）"
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf2-tools \
    ros-humble-rqt-graph \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-matplotlib

echo ""
echo "==> 完成。下一步："
echo ""
echo "  # 1. 确认环境"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 --version"
echo ""
echo "  # 2. 先跑官方示例验证整条链路（关键一步）"
echo "  export TURTLEBOT3_MODEL=burger"
echo "  ros2 launch nav2_bringup tb3_simulation_launch.py"
echo ""
echo "  # 3. 编译本仓库"
echo "  cd ~/robot_ws"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "  source install/setup.bash"
echo ""
echo "  # 4. 查看本机器人模型"
echo "  ros2 launch diff_chassis_description display.launch.py"
