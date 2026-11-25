#ifndef DIFFBOT_CONTROL__DIFFBOT_SERIAL_SYSTEM_HPP_
#define DIFFBOT_CONTROL__DIFFBOT_SERIAL_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace diffbot_control
{

/// 实车串口底盘硬件接口骨架（未实现）。
///
/// 实车链路应为：
///   controller_manager --(速度指令)--> DiffBotSerialSystem --(串口帧)--> 下位机固件
///   下位机固件 --(编码器/IMU 帧)--> DiffBotSerialSystem --(关节状态)--> joint_state_broadcaster
///                                                                     -> diff_drive_controller(/wheel/odom)
///
/// 接入前还缺（详见 2026-11-23/ros2-control/work/ 报告“实车接入还差什么”一节）：
///   1. 串口协议文档 / 帧格式（速度指令、左右轮编码器、时间戳、校验）；
///   2. 串口设备名与权限（udev 规则，见流程说明“串口权限”）；
///   3. 固件把右轮电机方向放在哪一层取反（本项目约定同仿真：轴 (0,-1,0)，
///      编码器与速度指令都由硬件接口层归一化为“前进为正”）；
///   4. IMU 传输方式（若走同一串口，需要在此解析后发布 /imu/data）。
class DiffBotSerialSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointHandle
  {
    std::string name;
    double sign = 1.0;
    double position = 0.0;
    double velocity = 0.0;
    double command = 0.0;
  };

  std::vector<JointHandle> joints_;
  std::string serial_port_ = "/dev/ttyUSB0";
  int baudrate_ = 115200;
  bool warn_once_ = true;
};

}  // namespace diffbot_control

#endif  // DIFFBOT_CONTROL__DIFFBOT_SERIAL_SYSTEM_HPP_
