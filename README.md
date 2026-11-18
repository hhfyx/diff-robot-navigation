# 两轮差速移动机器人 定位导航与路径规划系统

> 本科毕业设计项目 · 基于 ROS 2 Humble + Nav2 的差速底盘自主导航系统

## 项目简介

在一台自制的两轮差速移动机器人上，实现完整的**建图、定位、路径规划与避障**功能，并对六种规划算法（全局 3 种、局部 3 种）做同条件横向对比。

**这不是算法研究项目。** 目标是把成熟组件正确集成、参数整到能跑，并用可复现的实验数据说明取舍。

## 目标硬件

| 项目 | 参数 |
|---|---|
| 底盘 | 两轮差速 + 前后万向轮 |
| 整机质量 | 3.53 kg |
| 车体外廓 | 440 × 440 mm |
| 驱动轮 | 直径 126 mm |
| 轮距 | 422.3 mm（模型实测，固件默认 330 需标定） |
| 主控 | STM32F103C8T6 |
| 激光雷达 | RPLIDAR A2，扫描面离地 335 mm |
| IMU | WHEELTEC H30 mini |

## 软件栈

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Nav2（navigation2）
- slam_toolbox（建图）/ AMCL（定位）
- robot_localization（EKF 融合）
- Gazebo（仿真）
- ros2_control（底盘控制）

---

## 快速开始

### 1. 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions
```

### 2. 编译

```bash
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone <本仓库地址> robot_project
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> **必须用 Release**。Debug 会让局部规划周期从 20 ms 恶化到 60 ms 以上，导致控制抖动。

### 3. 验证环境（关键一步）

先跑官方示例，确认整条链路可用：

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup tb3_simulation_launch.py
```

能在地图上点目标点并导航 = 环境正常。**这一步不通，不要往下做。**

### 4. 查看本项目的机器人模型

```bash
ros2 launch diff_chassis_description display.launch.py
```

---

## 目录结构

```
.
├── README.md                       本文件
├── docs/                           文档（规格书、开题报告、计划）
│   ├── 技术规格书.md               ★ 接手必读
│   ├── 开题报告.docx
│   ├── 任务书.docx
│   └── 两个月执行计划.xlsx
├── src/                            ROS 2 工作空间源码
│   └── diff_chassis/               机器人描述包
│       ├── urdf/                   URDF 模型
│       ├── meshes/                 STL 网格
│       ├── launch/                 启动文件（ROS 2 Python 格式）
│       ├── config/                 参数配置
│       ├── CMakeLists.txt          ament_cmake
│       └── package.xml
├── firmware/                       STM32 底盘固件（硬件同学维护）
├── tools/                          分析脚本
└── .gitignore
```

---

## 开发路线（16 步）

| 步骤 | 内容 | 依赖硬件 |
|---|---|---|
| 1 | 环境：Ubuntu + ROS 2 Humble | 否 |
| 2 | 仿真环境：Gazebo + nav2 示例跑通 | 否 |
| 3 | URDF 补全：base_footprint / ament / gazebo 标签 | 否 |
| 4 | 仿真车可控：gazebo 差速插件 + gz_ros2_control | 否 |
| 5 | 仿真建图：Gazebo 场地 + slam_toolbox | 否 |
| 6 | 仿真导航：Nav2 单点导航 | 否 |
| 7 | 串口对接：serial 节点（可先用假数据） | 部分 |
| 8 | ros2_control 硬件接口插件 | 是 |
| 9 | 标定：轮径 / 轮距 / CPR | 是 |
| 10 | EKF 融合：对比开/关效果 | 是 |
| 11 | 实车建图 | 是 |
| 12 | 实车定位：AMCL + 重定位 | 是 |
| 13 | 单点导航：代价地图调参 + 安全层 | 是 |
| 14 | 算法对比：六算法 × 多场景 | 是 |
| 15 | 系统集成：统一 launch + 鲁棒性测试 | 是 |
| 16 | 论文撰写 | 否 |

**第 1–6 步完全不依赖硬件。硬件延期时继续推进，不要停等。**

---

## 六个已确认的坑

详见 [docs/技术规格书.md](docs/技术规格书.md) 第 5 章。摘要：

1. **轮距冲突**：模型 422.3 mm vs 固件 330 mm，转一圈偏 100°。必须实测标定。
2. **编码器分辨率**：固件取 1600，电机 27:1 实际可能 1188，尺度偏 26%。
3. **右轮关节轴**是 (0,−1,0)，左轮是 (0,1,0)，差速控制器需在硬件接口层取反。
4. **激光只到 335 mm**，看不到桌面以上，避障需补偿并在论文说明局限。
5. **EKF 位序**：15 个布尔位，vyaw 是第 12 位（不是第 15 位）。
6. **缺 base_footprint**，包是 catkin 需改 ament。

---

## 软硬件分工

| 角色 | 负责 |
|---|---|
| **软件（本仓库主要工作）** | ROS 2 节点、URDF、参数、标定脚本、实验、论文 |
| **硬件同学** | STM32 固件、电机驱动、接线、机械、电源急停 |

**边界规则**：不改固件逻辑；不用软件掩盖硬件问题；协议变更需双方确认；实车数据优先于仿真。

---

## 串口协议（不可单方面修改）

**指令**（上位机 → 下位机，115200 8N1）

```
V <v> <w>    线速度 m/s + 角速度 rad/s
S            立即停车
Z            编码器清零
P <kp> <ki> <kd>   在线改 PID
R            恢复默认
?            查询参数
```

**遥测帧**（下位机 → 上位机，50 Hz）

```
T,<ms>,<vL>,<vR>,<tgtL>,<tgtR>,<tickL>,<tickR>,<dutyL>,<dutyR>,<lost>
```

**断链保护**：500 ms 未收到 `V` 指令，固件自动停车。软件侧必须 ≥10 Hz 持续下发。

---

## 文档索引

| 文档 | 说明 |
|---|---|
| [docs/技术规格书.md](docs/技术规格书.md) | **接手必读**，含接口定义、六个坑、验收标准 |
| [docs/开题报告.docx](docs/开题报告.docx) | 毕设开题报告 |
| [docs/任务书.docx](docs/任务书.docx) | 学校任务书（已填写） |
| [docs/两个月执行计划.xlsx](docs/两个月执行计划.xlsx) | 60 天逐日逐时计划 |

---

## 常见故障速查

| 现象 | 最可能原因 | 处理 |
|---|---|---|
| 串口乱码 | 波特率或晶振 | 换波特率试，查 8 MHz 晶振 |
| 给占空比就复位 | 电池带不动电机 | 换电池或并电容 |
| 轮子转但编码器为 0 | A/B 相接错 | 对调 A/B，量编码器 VCC |
| 车自己停了 | 断链保护 | 检查是否 10 Hz 下发 `V` |
| 给正速度两轮反向 | 右轮轴方向 | 硬件接口层取反右轮 |
| 建图后墙是弯的 | 轮距不准 | 重标轮距，降速到 0.2 m/s |
| 定位跳变 / 瞬移 | TF 重复发布者 | 检查 `enable_odom_tf` |
| AMCL 定位失败 | 初始位姿错 | RViz 用 2D Pose Estimate 手动给 |
| 窄通道规划无解 | 膨胀半径过大 | 调小 `inflation_radius` |

---

## 许可

本项目为本科毕业设计，仅供学习参考。
