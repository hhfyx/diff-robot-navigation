# 目录说明

## docs/ 文档

| 文件 | 说明 | 读者 |
|---|---|---|
| **技术规格书.md** | 完整技术规格：接口定义、六个坑、验收标准、算法对比方案 | **接手必读**，可直接给 AI 模型 |
| 技术规格书.docx | 同上，Word 版，适合打印 | 导师 / 评审 |
| 开题报告.docx | 毕设开题报告（已定稿） | 提交材料 |
| 任务书.docx | 学校任务书（已填写专业班级姓名） | 提交材料 |
| 工作计划.docx | 学校工作计划（已填时间节点） | 提交材料 |
| 两个月执行计划.xlsx | 60 天逐日逐时执行计划 | 自己用 |

## src/ ROS 2 工作空间

```
src/
└── diff_chassis/                 机器人描述包
    ├── urdf/
    │   ├── diff_chassis.urdf.xacro    ★ 主模型（已补全，用这个）
    │   └── diff_chassis.orig.urdf     SolidWorks 原始导出（仅作参考）
    ├── meshes/                   6 个 STL 网格（11.9 MB）
    ├── launch/
    │   ├── display.launch.py     在 RViz2 中看模型
    │   └── gazebo.launch.py      在 Gazebo 中启动（含雷达/IMU）
    ├── config/
    │   ├── display.rviz          RViz 配置
    │   └── joint_names.yaml      关节名与标定参数
    ├── CMakeLists.txt            ament_cmake
    └── package.xml               format 3
```

### 与原始模型的差异

`diff_chassis.urdf.xacro` 相比 `diff_chassis.orig.urdf` 做了这些改动：

| 改动 | 原因 |
|---|---|
| 新增 `base_footprint` | Nav2 需要地面投影坐标系，原模型缺失 |
| 关节补 `limit` 与 `dynamics` | `ros2_control` 需要限幅；仿真需要阻尼 |
| collision 改为简化几何 | 原模型用 7 万面 STL 做碰撞，仿真极慢 |
| 新增 Gazebo 标签 | 差速插件、雷达与 IMU 仿真、轮地摩擦 |
| 新增 `camera_link` | 原模型无相机，Nav2 与视觉节点需要 |
| 补材质定义 | 原模型全白，RViz 中难辨部件 |
| 参数化（xacro） | 轮径、轮距等可一处改全局生效 |

**保留未改**：右轮关节轴 `(0,-1,0)`。这是导出特性，正确做法是在硬件接口层对右轮取反，不是改模型。

## firmware/ STM32 固件

硬件同学维护。软件侧只需知道：

- `Inc/board.h` —— 关键参数（轮径、轮距、CPR、PID、限幅）
- `Src/protocol.c` —— 串口协议实现（`V`/`S`/`Z`/`P`/`R`/`?`）
- `README.md` —— **接线表、四步上电自检、标定方法、PID 整定**，写得很细

标定时直接照 README 第 4 章做：

| 项目 | 方法 | 判据 |
|---|---|---|
| CPR | 手转 10 圈数脉冲 | `COUNTS_PER_OUT_REV = N/10` |
| 轮径 | 走 2 m 记脉冲差 | 与卷尺实测对比 |
| 轮距 | 原地转整圈计时 | 理论 12.57 s（2π/0.5） |

## scripts/ 脚本

| 文件 | 用途 |
|---|---|
| `check_env.sh` | 环境自检：ROS 2 / Gazebo / Nav2 / 构建工具逐项检查 |
| `install_deps.sh` | 一键装齐导航与仿真依赖 |
| `build.sh` | 编译工作空间（强制 Release） |

**先跑 `check_env.sh`**，全部通过再往下做。

## tools/ 工具

| 文件 | 用途 |
|---|---|
| `analyze_model.py` | 解析 STL 网格，提取包围盒、质量、惯量，用于核对模型与实物 |

```bash
python3 tools/analyze_model.py
```

---

## 工作流建议

```bash
# 1. 环境自检
bash scripts/check_env.sh

# 2. 先跑官方示例（关键）
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup tb3_simulation_launch.py
#   能在地图上点目标点并导航 = 环境正常

# 3. 编译本仓库
bash scripts/build.sh
source install/setup.bash

# 4. 看模型
ros2 launch diff_chassis_description display.launch.py

# 5. 跑仿真
ros2 launch diff_chassis_description gazebo.launch.py
#   另开终端遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 下一步要补的东西

当前仓库是**可编译、可看模型、可跑仿真的起点**，还缺：

| 缺什么 | 哪一步 | 说明 |
|---|---|---|
| `diffbot_control` | 第 7–8 步 | ros2_control 硬件接口插件，串口通信 |
| `diffbot_bringup` | 第 9 步起 | 顶层 launch，聚合所有节点 |
| `diffbot_nav` | 第 13 步 | Nav2 参数、代价地图、行为树 |
| `diffbot_safety` | 第 13 步 | 安全层：限速、前向防护、看门狗 |
| `diffbot_eval` | 第 14 步 | 批量实验、指标计算、图表 |
| Gazebo 场景 | 第 5 步 | 内墙、桌腿、门口，1:1 对照实车 |

按 `docs/技术规格书.md` 第 8 章的 16 步顺序推进即可。
