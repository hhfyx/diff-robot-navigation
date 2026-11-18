#!/usr/bin/env bash
# ============================================================================
# 编译工作空间
# 用法：bash scripts/build.sh
# ============================================================================
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> 工作空间: $WS_ROOT"
echo "==> source ROS 2 Humble"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$WS_ROOT"

echo ""
echo "==> 解析依赖（首次运行需要，可能要求 sudo）"
rosdep install --from-paths src --ignore-src -r -y || \
    echo "警告：rosdep 有未解析项，若编译失败请手动检查"

echo ""
echo "==> 开始编译（Release 模式，必须）"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "==> 完成。请执行："
echo "    source $WS_ROOT/install/setup.bash"
