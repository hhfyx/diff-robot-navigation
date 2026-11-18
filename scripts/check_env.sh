#!/usr/bin/env bash
# ============================================================================
# 环境自检：确认 ROS 2 / Gazebo / Nav2 三件套可用
# 用法：bash scripts/check_env.sh
# ============================================================================
set -uo pipefail

RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; NC=$'\033[0m'
ok=0; fail=0

chk() {
    local name="$1" cmd="$2"
    if eval "$cmd" >/dev/null 2>&1; then
        printf "  ${GREEN}[OK]${NC}   %s\n" "$name"
        ok=$((ok+1))
    else
        printf "  ${RED}[FAIL]${NC} %s\n" "$name"
        fail=$((fail+1))
    fi
}

echo "=============================================="
echo " 环境自检"
echo "=============================================="

echo ""
echo "[1] 操作系统"
. /etc/os-release 2>/dev/null || true
echo "  $PRETTY_NAME"
chk "Ubuntu 22.04" "grep -q '22.04' /etc/os-release"

echo ""
echo "[2] ROS 2"
chk "ROS 2 humble 已安装" "[ -f /opt/ros/humble/setup.bash ]"
chk "ros2 命令可用" "source /opt/ros/humble/setup.bash && ros2 --version"

echo ""
echo "[3] 关键软件包"
for p in navigation2 nav2_bringup slam_toolbox robot_localization \
         ros2_control ros2_controllers gazebo_ros xacro; do
    chk "$p" "source /opt/ros/humble/setup.bash && ros2 pkg prefix $p"
done

echo ""
echo "[4] 仿真工具"
chk "Gazebo 可执行" "which gazebo"
chk "gzserver 可执行" "which gzserver"

echo ""
echo "[5] 构建工具"
chk "colcon" "which colcon"
chk "xacro" "source /opt/ros/humble/setup.bash && which xacro"

echo ""
echo "=============================================="
if [ $fail -eq 0 ]; then
    printf "${GREEN}全部通过（%d 项）${NC}\n" $ok
    echo ""
    echo "下一步："
    echo "  export TURTLEBOT3_MODEL=burger"
    echo "  ros2 launch nav2_bringup tb3_simulation_launch.py"
    echo "  能在地图上点目标点并导航 = 环境正常"
else
    printf "${RED}%d 项失败${NC}，%d 项通过\n" $fail $ok
    echo ""
    echo "先解决失败项，再往下做。运行："
    echo "  bash scripts/install_deps.sh"
fi
echo "=============================================="
exit $([ $fail -eq 0 ] && echo 0 || echo 1)
