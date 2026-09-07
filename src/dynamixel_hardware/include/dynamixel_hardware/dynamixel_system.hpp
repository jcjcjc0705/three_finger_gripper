#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_SYSTEM_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_SYSTEM_HPP_

#include <string>
#include <vector>
#include <memory>

#include "dynamixel_hardware/dynamixel_bus.hpp"
#include "dynamixel_msgs/srv/read_register.hpp"
#include "dynamixel_msgs/srv/reboot.hpp"
#include "dynamixel_msgs/srv/set_torque.hpp"
#include "dynamixel_msgs/srv/set_zero.hpp"
#include "dynamixel_msgs/srv/write_register.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dynamixel_hardware
{

class DynamixelSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool pull_states();
  void create_services();
  std::vector<uint8_t> resolve_ids(uint8_t id) const;

  std::unique_ptr<DynamixelBus> bus_;

  struct InterfaceKeys
  {
    std::string name;
    std::string position_command;
    std::string effort_command;   ///< Goal Current ceiling, empty if not exported
    std::string position_state;
    std::string velocity_state;
    std::string effort_state;
    std::string current_state;
  };
  std::vector<InterfaceKeys> keys_;

  std::vector<JointSpec> pending_specs_;

  std::vector<JointFeedback> feedback_;
  std::vector<double> commands_;
  std::vector<double> current_commands_;
  std::vector<double> written_currents_;   ///< last value actually sent

  rclcpp::Service<dynamixel_msgs::srv::SetTorque>::SharedPtr srv_set_torque_;
  rclcpp::Service<dynamixel_msgs::srv::Reboot>::SharedPtr srv_reboot_;
  rclcpp::Service<dynamixel_msgs::srv::SetZero>::SharedPtr srv_set_zero_;
  rclcpp::Service<dynamixel_msgs::srv::ReadRegister>::SharedPtr srv_read_register_;
  rclcpp::Service<dynamixel_msgs::srv::WriteRegister>::SharedPtr srv_write_register_;
};

}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_SYSTEM_HPP_
