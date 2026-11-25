#include "diffbot_control/diffbot_serial_system.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffbot_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn DiffBotSerialSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto port = info_.hardware_parameters.find("serial_port");
  if (port != info_.hardware_parameters.end()) {
    serial_port_ = port->second;
  }
  auto baud = info_.hardware_parameters.find("baudrate");
  if (baud != info_.hardware_parameters.end()) {
    baudrate_ = std::stoi(baud->second);
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());
  for (const auto & joint_info : info_.joints) {
    JointHandle handle;
    handle.name = joint_info.name;
    auto it = joint_info.parameters.find("invert");
    if (it != joint_info.parameters.end()) {
      handle.sign = (it->second == "true" || it->second == "1") ? -1.0 : 1.0;
    }
    joints_.push_back(handle);
  }

  RCLCPP_WARN(
    rclcpp::get_logger("diffbot_control"),
    "DiffBotSerialSystem 只是接口骨架：串口 %s @ %d 未打开，协议尚未实现。",
    serial_port_.c_str(), baudrate_);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSerialSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 2);
  for (auto & handle : joints_) {
    interfaces.emplace_back(handle.name, hardware_interface::HW_IF_POSITION, &handle.position);
    interfaces.emplace_back(handle.name, hardware_interface::HW_IF_VELOCITY, &handle.velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSerialSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size());
  for (auto & handle : joints_) {
    interfaces.emplace_back(handle.name, hardware_interface::HW_IF_VELOCITY, &handle.command);
  }
  return interfaces;
}

CallbackReturn DiffBotSerialSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(实车): 打开 serial_port_，配置 baudrate_，握手并读取固件版本。
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSerialSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(实车): 从串口读一帧，解析左右轮编码器 -> position/velocity（乘 sign 归一化）。
  if (warn_once_) {
    warn_once_ = false;
    RCLCPP_ERROR(
      rclcpp::get_logger("diffbot_control"),
      "DiffBotSerialSystem::read 未实现，控制器将无法获得真实关节状态。");
  }
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type DiffBotSerialSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(实车): 把 command * sign 转成差速指令帧写串口。
  return hardware_interface::return_type::ERROR;
}

}  // namespace diffbot_control

PLUGINLIB_EXPORT_CLASS(
  diffbot_control::DiffBotSerialSystem, hardware_interface::SystemInterface)
