#include "gripper_control/gripper_controller.hpp"

#include <algorithm>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace gripper_control
{
namespace
{

using gripper_servo::kJoints;
using gripper_servo::kJointNames;

std::string key(int joint, const char * interface)
{
  return std::string(kJointNames[joint]) + "/" + interface;
}

}  // namespace

controller_interface::InterfaceConfiguration
GripperController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 0; i < kJoints; ++i) {
    cfg.names.push_back(key(i, "position"));
    cfg.names.push_back(key(i, "effort"));
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
GripperController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 0; i < kJoints; ++i) {
    cfg.names.push_back(key(i, "position"));
    cfg.names.push_back(key(i, "velocity"));
    cfg.names.push_back(key(i, "current"));
  }
  return cfg;
}

controller_interface::CallbackReturn GripperController::on_init()
{
  auto node = get_node();
  node->declare_parameter("max_velocity", config_.max_velocity);
  node->declare_parameter("max_acceleration", config_.max_acceleration);
  node->declare_parameter("watchdog_timeout", config_.watchdog_timeout);
  node->declare_parameter("hold_current", config_.hold_current);
  node->declare_parameter("stall_current", config_.stall_current);
  node->declare_parameter("stall_release", config_.stall_release);
  node->declare_parameter("stall_bias", config_.stall_bias);
  node->declare_parameter("stall_speed", config_.stall_speed);
  node->declare_parameter("stall_time", config_.stall_time);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  config_.max_velocity = node->get_parameter("max_velocity").as_double();
  config_.max_acceleration = node->get_parameter("max_acceleration").as_double();
  config_.watchdog_timeout = node->get_parameter("watchdog_timeout").as_double();
  config_.hold_current = node->get_parameter("hold_current").as_double();
  config_.stall_current = node->get_parameter("stall_current").as_double();
  config_.stall_release = node->get_parameter("stall_release").as_double();
  config_.stall_bias = node->get_parameter("stall_bias").as_double();
  config_.stall_speed = node->get_parameter("stall_speed").as_double();
  config_.stall_time = node->get_parameter("stall_time").as_double();
  servo_.configure(config_);

  goal_subscription_ = node->create_subscription<gripper_msgs::msg::GripperGoal>(
    "~/goal", rclcpp::SystemDefaultsQoS(),
    [this](const gripper_msgs::msg::GripperGoal::SharedPtr msg) {
      StampedGoal stamped;
      stamped.goal = *msg;
      stamped.sequence = ++received_;
      goal_buffer_.writeFromNonRT(stamped);
    });

  state_publisher_base_ = node->create_publisher<gripper_msgs::msg::GripperState>(
    "~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<gripper_msgs::msg::GripperState>>(
      state_publisher_base_);

  RCLCPP_INFO(node->get_logger(),
              "servo: v<=%.2f rad/s, a<=%.2f rad/s^2, watchdog %.2f s, hold current %.0f",
              config_.max_velocity, config_.max_acceleration,
              config_.watchdog_timeout, config_.hold_current);
  return controller_interface::CallbackReturn::SUCCESS;
}

bool GripperController::bind_interfaces()
{
  auto find = [this](const std::vector<std::string> & wanted,
                     auto & pool, std::vector<std::size_t> & out) {
    out.clear();
    for (const auto & name : wanted) {
      const auto it = std::find_if(
        pool.begin(), pool.end(),
        [&](const auto & handle) { return handle.get_name() == name; });
      if (it == pool.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "missing interface %s", name.c_str());
        return false;
      }
      out.push_back(static_cast<std::size_t>(std::distance(pool.begin(), it)));
    }
    return true;
  };

  std::vector<std::string> pos_cmd, eff_cmd, pos_st, vel_st, cur_st;
  for (int i = 0; i < kJoints; ++i) {
    pos_cmd.push_back(key(i, "position"));
    eff_cmd.push_back(key(i, "effort"));
    pos_st.push_back(key(i, "position"));
    vel_st.push_back(key(i, "velocity"));
    cur_st.push_back(key(i, "current"));
  }

  return find(pos_cmd, command_interfaces_, position_command_) &&
         find(eff_cmd, command_interfaces_, effort_command_) &&
         find(pos_st, state_interfaces_, position_state_) &&
         find(vel_st, state_interfaces_, velocity_state_) &&
         find(cur_st, state_interfaces_, current_state_);
}

controller_interface::CallbackReturn GripperController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bind_interfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Start from where the hardware is, so activating never commands a jump.
  gripper_servo::Feedback feedback;
  for (int i = 0; i < kJoints; ++i) {
    feedback.position[i] = static_cast<float>(
      state_interfaces_[position_state_[i]].get_optional().value_or(0.0));
  }
  servo_.reset(feedback);
  applied_ = received_;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  gripper_servo::Feedback feedback;
  for (int i = 0; i < kJoints; ++i) {
    feedback.position[i] = static_cast<float>(
      state_interfaces_[position_state_[i]].get_optional().value_or(0.0));
    feedback.velocity[i] = static_cast<float>(
      state_interfaces_[velocity_state_[i]].get_optional().value_or(0.0));
    feedback.current[i] = static_cast<float>(
      state_interfaces_[current_state_[i]].get_optional().value_or(0.0));
  }

  const double now = time.seconds();
  const StampedGoal & latest = *goal_buffer_.readFromRT();
  if (latest.sequence != applied_) {
    applied_ = latest.sequence;
    gripper_servo::Goal goal;
    for (int i = 0; i < kJoints; ++i) {
      goal.position[i] = latest.goal.position[i];
      goal.current_limit[i] = latest.goal.current_limit[i];
    }
    // Zero means "leave the configured limit alone" so a caller that only
    // cares about where the fingers go does not have to fill these in.
    auto cfg = config_;
    if (latest.goal.max_velocity > 0.0f) { cfg.max_velocity = latest.goal.max_velocity; }
    if (latest.goal.max_acceleration > 0.0f) {
      cfg.max_acceleration = latest.goal.max_acceleration;
    }
    servo_.configure(cfg);
    servo_.set_goal(goal, now);
  }

  const auto command = servo_.step(feedback, static_cast<float>(period.seconds()), now);
  for (int i = 0; i < kJoints; ++i) {
    command_interfaces_[position_command_[i]].set_value(
      static_cast<double>(command.position[i]));
    command_interfaces_[effort_command_[i]].set_value(
      static_cast<double>(command.current_limit[i]));
  }

  if (state_publisher_ && state_publisher_->trylock()) {
    auto & msg = state_publisher_->msg_;
    msg.header.stamp = time;
    for (int i = 0; i < kJoints; ++i) {
      msg.position[i] = feedback.position[i];
      msg.velocity[i] = feedback.velocity[i];
      msg.current[i] = feedback.current[i];
      msg.stalled[i] = servo_.stalled(i);
    }
    msg.status = static_cast<uint8_t>(servo_.status());
    msg.goal_stale = servo_.goal_stale();
    msg.reach = servo_.reach();
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace gripper_control

PLUGINLIB_EXPORT_CLASS(
  gripper_control::GripperController, controller_interface::ControllerInterface)
