#include "dynamixel_hardware/dynamixel_system.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
rclcpp::Logger hw_logger() { return rclcpp::get_logger("DynamixelSystem"); }

std::string param_or(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  return (it != params.end()) ? it->second : fallback;
}
}  // namespace

namespace dynamixel_hardware
{

hardware_interface::CallbackReturn DynamixelSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = get_hardware_info();

  const std::string port_name =
    param_or(info.hardware_parameters, "port_name", "/dev/ttyUSB0");
  const int baud_rate =
    std::stoi(param_or(info.hardware_parameters, "baud_rate", "1000000"));

  std::vector<JointSpec> specs;
  keys_.clear();

  for (const auto & joint : info.joints)
  {
    JointSpec spec;
    spec.name = joint.name;
    spec.operating_mode = param_or(joint.parameters, "operating_mode", "position");
    spec.direction = (param_or(joint.parameters, "direction", "1") == "-1") ? -1 : 1;
    spec.parameters = joint.parameters;

    const auto id_it = joint.parameters.find("device_id");
    if (id_it == joint.parameters.end())
    {
      RCLCPP_FATAL(hw_logger(), "Joint '%s' is missing device_id parameter.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    spec.id = static_cast<uint8_t>(std::stoi(id_it->second));

    bool wants_current = false;
    for (const auto & ci : joint.command_interfaces)
    {
      if (ci.name == hardware_interface::HW_IF_POSITION) { spec.commandable = true; }
      if (ci.name == hardware_interface::HW_IF_EFFORT) { wants_current = true; }
    }

    InterfaceKeys k;
    k.position_command = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    // Interpreted as the Goal Current ceiling in the motor's register units,
    // which is what bounds the push in current_based_position mode.
    if (wants_current)
    {
      k.effort_command = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
    }
    k.position_state = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    k.velocity_state = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    k.effort_state = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
    k.current_state = joint.name + "/current";
    k.name = joint.name;

    specs.push_back(spec);
    keys_.push_back(k);
  }

  bus_ = std::make_unique<DynamixelBus>(port_name, baud_rate);
  feedback_.assign(specs.size(), JointFeedback{});
  commands_.assign(specs.size(), std::numeric_limits<double>::quiet_NaN());
  current_commands_.assign(specs.size(), std::numeric_limits<double>::quiet_NaN());
  // NaN so the first cycle always sends whatever the controller asks for.
  written_currents_.assign(specs.size(), std::numeric_limits<double>::quiet_NaN());

  pending_specs_ = std::move(specs);

  RCLCPP_INFO(
    hw_logger(), "initialized: %zu joints, port=%s, baud=%d",
    pending_specs_.size(), port_name.c_str(), baud_rate);
  for (const auto & s : pending_specs_)
  {
    RCLCPP_INFO(
      hw_logger(), "  %-16s id=%-3d dir=%+d mode=%-22s %s",
      s.name.c_str(), s.id, s.direction, s.operating_mode.c_str(),
      s.commandable ? "position" : "(readonly)");
  }

  create_services();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bus_->open())
  {
    RCLCPP_FATAL(hw_logger(), "%s", bus_->last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!bus_->configure(pending_specs_))
  {
    RCLCPP_FATAL(hw_logger(), "%s", bus_->last_error().c_str());
    bus_->close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & j : bus_->joints())
  {
    RCLCPP_INFO(
      hw_logger(), "  %-16s id=%-3d model_number=%s",
      j.name.c_str(), j.id,
      param_or(j.parameters, "_detected_model_number", "?").c_str());
  }

  for (const auto & k : keys_)
  {
    set_state(k.position_state, 0.0);
    set_state(k.velocity_state, 0.0);
    set_state(k.effort_state, 0.0);
  }

  RCLCPP_INFO(hw_logger(), "configure completed: %zu joints online, torque disabled.", keys_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!pull_states())
  {
    RCLCPP_FATAL(hw_logger(), "Failed to read states before activation: %s", bus_->last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joints = bus_->joints();
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    if (!joints[i].commandable) { continue; }
    const double now = get_state<double>(keys_[i].position_state);
    set_command(keys_[i].position_command, now);
    RCLCPP_INFO(hw_logger(), "  %-16s initial position %.4f rad", joints[i].name.c_str(), now);
  }

  for (const auto & j : joints)
  {
    if (!j.commandable) { continue; }
    if (!bus_->set_torque(j.id, true))
    {
      RCLCPP_FATAL(hw_logger(), "%s", bus_->last_error().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(hw_logger(), "activate completed, torque enabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & j : bus_->joints())
  {
    bus_->set_torque(j.id, false);
  }
  RCLCPP_INFO(hw_logger(), "deactivate completed, torque disabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bus_->close();
  RCLCPP_INFO(hw_logger(), "Serial port closed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool DynamixelSystem::pull_states()
{
  if (!bus_->read_all(feedback_)) { return false; }

  for (std::size_t i = 0; i < keys_.size(); ++i)
  {
    if (!feedback_[i].valid) { continue; }
    set_state(keys_[i].position_state, feedback_[i].position);
    set_state(keys_[i].velocity_state, feedback_[i].velocity);
    set_state(keys_[i].effort_state, feedback_[i].effort);
    set_state(keys_[i].current_state, feedback_[i].current);
    set_state(keys_[i].name + "/temperature", feedback_[i].temperature);
    set_state(keys_[i].name + "/voltage", feedback_[i].voltage);
    set_state(keys_[i].name + "/pwm", feedback_[i].pwm);
    set_state(keys_[i].name + "/moving", feedback_[i].moving);
    set_state(keys_[i].name + "/error", feedback_[i].error);
    set_state(keys_[i].name + "/homing_offset", feedback_[i].homing_offset);
  }
  return true;
}

hardware_interface::return_type DynamixelSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!pull_states())
  {
    static rclcpp::Clock clk(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      hw_logger(), clk, 1000, "%s (Remaining in last known state. )", bus_->last_error().c_str());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool current_changed = false;
  for (std::size_t i = 0; i < keys_.size(); ++i)
  {
    const bool commandable = bus_->joints()[i].commandable;
    commands_[i] = commandable
                     ? get_command<double>(keys_[i].position_command)
                     : std::numeric_limits<double>::quiet_NaN();

    current_commands_[i] = std::numeric_limits<double>::quiet_NaN();
    if (commandable && !keys_[i].effort_command.empty())
    {
      const double want = get_command<double>(keys_[i].effort_command);
      // Goal Current is a ceiling that changes between grasp phases, not every
      // cycle. Sending it unconditionally would double the bus transactions,
      // and over USB CDC each one costs about a millisecond.
      if (!std::isnan(want) && want != written_currents_[i])
      {
        current_commands_[i] = want;
        written_currents_[i] = want;
        current_changed = true;
      }
    }
  }

  static rclcpp::Clock clk(RCL_STEADY_TIME);
  if (current_changed && !bus_->write_currents(current_commands_))
  {
    RCLCPP_WARN_THROTTLE(hw_logger(), clk, 1000, "%s", bus_->last_error().c_str());
  }
  if (!bus_->write_positions(commands_))
  {
    RCLCPP_WARN_THROTTLE(hw_logger(), clk, 1000, "%s", bus_->last_error().c_str());
  }
  return hardware_interface::return_type::OK;
}

std::vector<uint8_t> DynamixelSystem::resolve_ids(uint8_t id) const
{
  std::vector<uint8_t> ids;
  if (id == 0)
  {
    for (const auto & j : bus_->joints()) { ids.push_back(j.id); }
  }
  else
  {
    ids.push_back(id);
  }
  return ids;
}

void DynamixelSystem::create_services()
{
  auto node = get_node();
  if (!node)
  {
    RCLCPP_WARN(hw_logger(), "Cannot get node, skipping service creation.");
    return;
  }

  srv_set_torque_ = node->create_service<dynamixel_msgs::srv::SetTorque>(
    "~/set_torque",
    [this](
      const std::shared_ptr<dynamixel_msgs::srv::SetTorque::Request> req,
      std::shared_ptr<dynamixel_msgs::srv::SetTorque::Response> res)
    {
      res->success = true;
      for (const auto id : resolve_ids(req->id))
      {
        if (!bus_->set_torque(id, req->enable))
        {
          res->success = false;
          res->message = bus_->last_error();
          return;
        }
      }
      res->message = std::string("torque ") + (req->enable ? "enabled" : "disabled");
    });

  srv_reboot_ = node->create_service<dynamixel_msgs::srv::Reboot>(
    "~/reboot",
    [this](
      const std::shared_ptr<dynamixel_msgs::srv::Reboot::Request> req,
      std::shared_ptr<dynamixel_msgs::srv::Reboot::Response> res)
    {
      res->success = true;
      for (const auto id : resolve_ids(req->id))
      {
        if (!bus_->reboot(id))
        {
          res->success = false;
          res->message = bus_->last_error();
          return;
        }
      }
      res->message = "rebooted (torque is now off)";
    });

  srv_set_zero_ = node->create_service<dynamixel_msgs::srv::SetZero>(
    "~/set_zero",
    [this](
      const std::shared_ptr<dynamixel_msgs::srv::SetZero::Request> req,
      std::shared_ptr<dynamixel_msgs::srv::SetZero::Response> res)
    {
      res->success = true;
      for (const auto id : resolve_ids(req->id))
      {
        if (!bus_->set_zero(id))
        {
          res->success = false;
          res->message = bus_->last_error();
          return;
        }
      }
      res->message = "homing offset updated";
    });

  srv_read_register_ = node->create_service<dynamixel_msgs::srv::ReadRegister>(
    "~/read_register",
    [this](
      const std::shared_ptr<dynamixel_msgs::srv::ReadRegister::Request> req,
      std::shared_ptr<dynamixel_msgs::srv::ReadRegister::Response> res)
    {
      int64_t value = 0;
      res->success = bus_->read_register(req->id, req->address, req->size, value);
      res->value = value;
      if (!res->success) { res->message = bus_->last_error(); }
    });

  srv_write_register_ = node->create_service<dynamixel_msgs::srv::WriteRegister>(
    "~/write_register",
    [this](
      const std::shared_ptr<dynamixel_msgs::srv::WriteRegister::Request> req,
      std::shared_ptr<dynamixel_msgs::srv::WriteRegister::Response> res)
    {
      res->success = true;
      for (const auto id : resolve_ids(req->id))
      {
        if (!bus_->write_register(id, req->address, req->size, req->value))
        {
          res->success = false;
          res->message = bus_->last_error();
          return;
        }
      }
    });

  RCLCPP_INFO(
    hw_logger(), "Created 5 services: set_torque / reboot / set_zero / "
                 "read_register / write_register");
}

}  // namespace dynamixel_hardware

PLUGINLIB_EXPORT_CLASS(
  dynamixel_hardware::DynamixelSystem, hardware_interface::SystemInterface)
