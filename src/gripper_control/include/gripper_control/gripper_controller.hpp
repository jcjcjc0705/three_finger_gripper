// ros2_control shell around gripper_servo.
//
// Phase 1 scaffolding: it carries the servo core while the control loop still
// runs on the PC. When the loop moves onto the NT98532 this controller is
// replaced by a plain forwarding controller and the same gripper_servo runs in
// the firmware -- so nothing that matters to the robot lives in this file.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "gripper_msgs/msg/gripper_goal.hpp"
#include "gripper_msgs/msg/gripper_state.hpp"
#include "gripper_servo/servo.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace gripper_control
{

class GripperController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool bind_interfaces();

  gripper_servo::Servo servo_;
  gripper_servo::Config config_{};

  // Ordered by gripper_servo::kJointNames, not by whatever order the resource
  // manager hands the interfaces over in.
  std::vector<std::size_t> position_command_;
  std::vector<std::size_t> effort_command_;
  std::vector<std::size_t> position_state_;
  std::vector<std::size_t> velocity_state_;
  std::vector<std::size_t> current_state_;

  /// The sequence number lets update() notice a new goal without needing a
  /// clock that agrees with the subscription thread's.
  struct StampedGoal
  {
    gripper_msgs::msg::GripperGoal goal;
    uint64_t sequence{0};
  };
  realtime_tools::RealtimeBuffer<StampedGoal> goal_buffer_;
  uint64_t received_{0};
  uint64_t applied_{0};
  rclcpp::Subscription<gripper_msgs::msg::GripperGoal>::SharedPtr goal_subscription_;
  std::shared_ptr<realtime_tools::RealtimePublisher<gripper_msgs::msg::GripperState>> state_publisher_;
  rclcpp::Publisher<gripper_msgs::msg::GripperState>::SharedPtr state_publisher_base_;


};

}  // namespace gripper_control
