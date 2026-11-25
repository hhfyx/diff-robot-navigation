#ifndef DIFFBOT_CONTROL__DIFFBOT_SYSTEM_HPP_
#define DIFFBOT_CONTROL__DIFFBOT_SYSTEM_HPP_

#include <string>
#include <vector>

#include "gazebo_ros2_control/gazebo_system_interface.hpp"

namespace diffbot_control
{

/// 仿真用 ros2_control 硬件接口。
///
/// 与 gazebo_ros2_control 自带的 GazeboSystem 相比，唯一区别是支持在硬件接口层
/// 对单个关节做符号取反。原因见 urdf/diff_chassis.urdf.xacro：
///   - 规格书约定右轮关节轴为 (0,-1,0)（与实车一致，不为了仿真去改模型）；
///   - 差速控制器 diff_drive_controller 要求左右轮“正速度 = 前进”；
///   - 于是右轮的位置/速度/命令在此处统一乘 -1，控制器侧看到的是归一化后的关节。
///
/// URDF 里用关节级参数声明取反：
///   <joint name="right_wheel_joint">
///     <param name="invert">true</param>
///     ...
///   </joint>
class DiffBotSystem : public gazebo_ros2_control::GazeboSystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

private:
  struct JointHandle
  {
    std::string name;
    gazebo::physics::JointPtr joint;
    double sign = 1.0;      // +1 正常，-1 表示该关节物理轴与“前进为正”相反
    double position = 0.0;  // 归一化后的状态（控制器侧可见）
    double velocity = 0.0;
    double command = 0.0;   // 归一化后的速度指令
  };

  /// 与 info_.joints 一一对应；export_*_interfaces 之后不得再增删元素，
  /// 因为导出的句柄直接指向这里的成员地址。
  std::vector<JointHandle> joints_;
  gazebo::physics::ModelPtr model_;
  /// ODE 关节速度电机需要同时给 fmax（最大力矩），否则电机没有出力、轮子不转。
  double max_effort_ = 10.0;

  JointHandle * findOrCreateJoint(const std::string & name);
};

}  // namespace diffbot_control

#endif  // DIFFBOT_CONTROL__DIFFBOT_SYSTEM_HPP_
