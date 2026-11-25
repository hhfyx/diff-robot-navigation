#include "diffbot_control/diffbot_system.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace diffbot_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn DiffBotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // 注意调用顺序：gazebo_ros2_control 先调 initSim 再调 on_init（导入组件时）。
  // 所以这里不能清空 joints_，只能补全/更新符号，否则会把已绑定的 Gazebo 关节点丢掉。
  joints_.reserve(info_.joints.size());
  auto effort = info_.hardware_parameters.find("max_effort");
  if (effort != info_.hardware_parameters.end()) {
    max_effort_ = std::stod(effort->second);
  }
  for (const auto & joint_info : info_.joints) {
    JointHandle * handle = findOrCreateJoint(joint_info.name);
    auto it = joint_info.parameters.find("invert");
    if (it != joint_info.parameters.end()) {
      handle->sign = (it->second == "true" || it->second == "1") ? -1.0 : 1.0;
    }
  }
  return CallbackReturn::SUCCESS;
}

DiffBotSystem::JointHandle * DiffBotSystem::findOrCreateJoint(const std::string & name)
{
  for (auto & handle : joints_) {
    if (handle.name == name) {
      return &handle;
    }
  }
  JointHandle handle;
  handle.name = name;
  joints_.push_back(handle);
  return &joints_.back();
}

bool DiffBotSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr /*sdf*/)
{
  nh_ = model_nh;
  model_ = parent_model;

  joints_.reserve(hardware_info.joints.size());
  for (const auto & joint_info : hardware_info.joints) {
    JointHandle * handle = findOrCreateJoint(joint_info.name);
    handle->joint = model_->GetJoint(handle->name);
    if (!handle->joint) {
      RCLCPP_ERROR(
        nh_->get_logger(), "Gazebo 中找不到关节 [%s]，差速底盘无法驱动", handle->name.c_str());
      return false;
    }
    RCLCPP_INFO(nh_->get_logger(), "硬件接口绑定关节 %s", handle->name.c_str());
  }
  return true;
}

std::vector<hardware_interface::StateInterface> DiffBotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 2);
  for (auto & handle : joints_) {
    interfaces.emplace_back(
      handle.name, hardware_interface::HW_IF_POSITION, &handle.position);
    interfaces.emplace_back(
      handle.name, hardware_interface::HW_IF_VELOCITY, &handle.velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size());
  for (auto & handle : joints_) {
    interfaces.emplace_back(
      handle.name, hardware_interface::HW_IF_VELOCITY, &handle.command);
  }
  return interfaces;
}

hardware_interface::return_type DiffBotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & handle : joints_) {
    if (!handle.joint) {
      continue;
    }
    // Gazebo 返回的是“关节自身轴”下的量；乘 sign 后交给控制器的是统一约定。
    handle.position = handle.joint->Position(0) * handle.sign;
    handle.velocity = handle.joint->GetVelocity(0) * handle.sign;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & handle : joints_) {
    if (!handle.joint) {
      continue;
    }
    // 速度电机：先给最大力矩，再给目标速度（与 gazebo_ros_diff_drive 的做法一致）
    handle.joint->SetParam("fmax", 0, max_effort_);
    handle.joint->SetParam("vel", 0, handle.command * handle.sign);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace diffbot_control

PLUGINLIB_EXPORT_CLASS(diffbot_control::DiffBotSystem, gazebo_ros2_control::GazeboSystemInterface)
