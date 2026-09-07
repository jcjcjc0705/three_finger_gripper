#include "dynamixel_hardware/dynamixel_bus.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <cmath>
#include <utility>

namespace dynamixel_hardware
{
namespace ct = control_table;

DynamixelBus::DynamixelBus(std::string port_name, int baud_rate)
: port_name_(std::move(port_name)), baud_rate_(baud_rate)
{
}

DynamixelBus::~DynamixelBus() { close(); }

double DynamixelBus::tick_to_rad(int32_t tick, int direction)
{
  return direction * (static_cast<double>(tick) - ct::CENTER_TICK) *
         (2.0 * M_PI / ct::TICKS_PER_REV);
}

int32_t DynamixelBus::rad_to_tick(double rad, int direction)
{
  return static_cast<int32_t>(
    std::lround(direction * rad * (ct::TICKS_PER_REV / (2.0 * M_PI)) + ct::CENTER_TICK));
}

double DynamixelBus::raw_to_rad_per_s(int32_t raw)
{
  return static_cast<double>(raw) * ct::VELOCITY_UNIT_RPM * 2.0 * M_PI / 60.0;
}

bool DynamixelBus::open()
{
  std::lock_guard<std::mutex> lock(port_mutex_);

  lock_fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (lock_fd_ < 0)
  {
    last_error_ = "Cannot open port: " + port_name_;
    return false;
  }
  if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0)
  {
    last_error_ = port_name_ + " is already locked by another process "
                               "(another ros2_control stack or dxl_offline?)";
    ::close(lock_fd_);
    lock_fd_ = -1;
    return false;
  }

  port_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
  packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (!port_->openPort())
  {
    last_error_ = "Cannot open port: " + port_name_;
    port_ = nullptr;
    release_lock();
    return false;
  }
  if (!port_->setBaudRate(baud_rate_))
  {
    last_error_ = "Cannot set baud rate: " + std::to_string(baud_rate_);
    port_->closePort();
    port_ = nullptr;
    release_lock();
    return false;
  }
  return true;
}

void DynamixelBus::close()
{
  std::lock_guard<std::mutex> lock(port_mutex_);

  sync_read_.reset();
  sync_write_.reset();
  sync_write_current_.reset();
  if (port_ != nullptr)
  {
    port_->closePort();
    port_ = nullptr;
  }
  release_lock();
}

void DynamixelBus::release_lock()
{
  if (lock_fd_ >= 0)
  {
    ::flock(lock_fd_, LOCK_UN);
    ::close(lock_fd_);
    lock_fd_ = -1;
  }
}

bool DynamixelBus::configure(const std::vector<JointSpec> & joints)
{
  std::lock_guard<std::mutex> lock(port_mutex_);

  if (port_ == nullptr)
  {
    last_error_ = "Serial port not opened";
    return false;
  }
  joints_ = joints;

  for (auto & j : joints_)
  {
    uint16_t model = 0;
    if (!ping_unlocked(j.id, model)) { return false; }
    j.parameters["_detected_model_number"] = std::to_string(model);
  }

  for (const auto & j : joints_)
  {
    if (!set_torque_unlocked(j.id, false)) { return false; }
  }

  for (const auto & j : joints_)
  {
    if (!set_operating_mode_unlocked(j.id, j.operating_mode)) { return false; }
    if (!apply_parameters(j)) { return false; }
  }

  homing_offset_.assign(joints_.size(), 0.0);
  error_.assign(joints_.size(), 0.0);
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    int64_t raw = 0;
    if (read_register_unlocked(joints_[i].id, ct::HOMING_OFFSET, 4, raw))
    {
      homing_offset_[i] = joints_[i].direction * static_cast<double>(raw) *
                          (2.0 * M_PI / ct::TICKS_PER_REV);
    }
  }
  error_cursor_ = 0;

  return setup_sync();
}

bool DynamixelBus::apply_parameters(const JointSpec & joint)
{
  auto parameters = joint.parameters;

  // Position limits are motor ticks. On a reversed joint the URDF's lower
  // bound is the motor's upper one, mirrored about the centre -- so the
  // description can state limits in joint sense and still protect the right end.
  if (joint.direction < 0)
  {
    const auto mirror = [](const std::string & text) {
      return std::to_string(2 * static_cast<int>(ct::CENTER_TICK) - std::stoi(text));
    };
    const auto lo = joint.parameters.find("min_position_limit");
    const auto hi = joint.parameters.find("max_position_limit");
    try
    {
      if (hi != joint.parameters.end()) { parameters["min_position_limit"] = mirror(hi->second); }
      if (lo != joint.parameters.end()) { parameters["max_position_limit"] = mirror(lo->second); }
    }
    catch (const std::exception &)
    {
      last_error_ = "Position limits are not integers on reversed joint " + joint.name;
      return false;
    }
  }

  for (const auto & spec : ct::WRITABLE_REGISTERS)
  {
    const auto it = parameters.find(spec.param_name);
    if (it == parameters.end()) { continue; }

    int64_t value = 0;
    try
    {
      value = std::stoll(it->second);
    }
    catch (const std::exception &)
    {
      last_error_ = std::string("Parameter ") + spec.param_name + " is not an integer: " + it->second;
      return false;
    }

    if (!write_register_unlocked(joint.id, spec.address, spec.size, value))
    {
      last_error_ = std::string("Failed to write ") + spec.param_name + " (" + joint.name +
                    "): " + last_error_;
      return false;
    }
  }
  return true;
}

bool DynamixelBus::setup_sync()
{
  sync_read_ = std::make_unique<dynamixel::GroupSyncRead>(
    port_, packet_, ct::SYNC_READ_START, ct::SYNC_READ_LENGTH);
  sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(
    port_, packet_, ct::GOAL_POSITION, ct::GOAL_POSITION_LENGTH);
  sync_write_current_ = std::make_unique<dynamixel::GroupSyncWrite>(
    port_, packet_, ct::GOAL_CURRENT, ct::GOAL_CURRENT_LENGTH);

  for (const auto & j : joints_)
  {
    if (!sync_read_->addParam(j.id))
    {
      last_error_ = "GroupSyncRead addParam failed: " + j.name;
      return false;
    }
  }
  return true;
}

bool DynamixelBus::read_all(std::vector<JointFeedback> & out)
{
  std::lock_guard<std::mutex> lock(port_mutex_);

  out.assign(joints_.size(), JointFeedback{});

  const int rc = sync_read_->txRxPacket();
  if (rc != COMM_SUCCESS)
  {
    last_error_ = std::string("SyncRead failed: ") + packet_->getTxRxResult(rc);
    return false;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    const uint8_t id = joints_[i].id;
    if (!sync_read_->isAvailable(id, ct::PRESENT_POSITION, 4)) { continue; }

    const int dir = joints_[i].direction;

    out[i].position = tick_to_rad(
      static_cast<int32_t>(sync_read_->getData(id, ct::PRESENT_POSITION, 4)), dir);

    if (sync_read_->isAvailable(id, ct::PRESENT_VELOCITY, 4))
    {
      out[i].velocity = dir * raw_to_rad_per_s(
        static_cast<int32_t>(sync_read_->getData(id, ct::PRESENT_VELOCITY, 4)));
    }

    if (sync_read_->isAvailable(id, ct::PRESENT_PWM, 2))
    {
      out[i].pwm = static_cast<int16_t>(sync_read_->getData(id, ct::PRESENT_PWM, 2));
    }
    if (sync_read_->isAvailable(id, ct::MOVING, 1))
    {
      out[i].moving = static_cast<double>(sync_read_->getData(id, ct::MOVING, 1));
    }
    if (sync_read_->isAvailable(id, ct::PRESENT_INPUT_VOLTAGE, 2))
    {
      out[i].voltage =
        static_cast<double>(sync_read_->getData(id, ct::PRESENT_INPUT_VOLTAGE, 2)) * 0.1;
    }
    if (sync_read_->isAvailable(id, ct::PRESENT_TEMPERATURE, 1))
    {
      out[i].temperature =
        static_cast<double>(sync_read_->getData(id, ct::PRESENT_TEMPERATURE, 1));
    }

    if (sync_read_->isAvailable(id, ct::PRESENT_CURRENT, 2))
    {
      // Signed: the sign is the direction of push. Left in register units --
      // converting to N·m needs a torque constant per model, and address 126
      // is not even the same quantity across the X range.
      const auto raw = static_cast<int16_t>(
        sync_read_->getData(id, ct::PRESENT_CURRENT, 2));
      out[i].current = dir * static_cast<double>(raw);
    }

    out[i].effort = 0.0;

    out[i].homing_offset = homing_offset_[i];
    out[i].error = error_[i];
    out[i].valid = true;
  }

  if (!joints_.empty())
  {
    const std::size_t i = error_cursor_ % joints_.size();
    int64_t raw = 0;
    if (read_register_unlocked(joints_[i].id, ct::HARDWARE_ERROR_STATUS, 1, raw))
    {
      error_[i] = static_cast<double>(raw);
    }
    error_cursor_ = (error_cursor_ + 1) % joints_.size();
  }

  return true;
}

bool DynamixelBus::write_positions(const std::vector<double> & rads)
{
  std::lock_guard<std::mutex> lock(port_mutex_);

  sync_write_->clearParam();

  for (std::size_t i = 0; i < joints_.size() && i < rads.size(); ++i)
  {
    if (!joints_[i].commandable) { continue; }
    if (std::isnan(rads[i])) { continue; }

    const int32_t tick = rad_to_tick(rads[i], joints_[i].direction);
    uint8_t buf[4] = {
      DXL_LOBYTE(DXL_LOWORD(tick)), DXL_HIBYTE(DXL_LOWORD(tick)),
      DXL_LOBYTE(DXL_HIWORD(tick)), DXL_HIBYTE(DXL_HIWORD(tick))};
    sync_write_->addParam(joints_[i].id, buf);
  }

  const int rc = sync_write_->txPacket();
  if (rc != COMM_SUCCESS)
  {
    last_error_ = std::string("SyncWrite failed: ") + packet_->getTxRxResult(rc);
    return false;
  }
  return true;
}

bool DynamixelBus::write_currents(const std::vector<double> & units)
{
  std::lock_guard<std::mutex> lock(port_mutex_);

  sync_write_current_->clearParam();

  for (std::size_t i = 0; i < joints_.size() && i < units.size(); ++i)
  {
    if (!joints_[i].commandable) { continue; }
    if (std::isnan(units[i])) { continue; }

    const int32_t raw = static_cast<int32_t>(units[i]);
    uint8_t buf[2] = {DXL_LOBYTE(raw), DXL_HIBYTE(raw)};
    sync_write_current_->addParam(joints_[i].id, buf);
  }

  const int rc = sync_write_current_->txPacket();
  if (rc != COMM_SUCCESS)
  {
    last_error_ = std::string("SyncWrite goal current failed: ") + packet_->getTxRxResult(rc);
    return false;
  }
  return true;
}

bool DynamixelBus::ping(uint8_t id, uint16_t & model_number)
{
  std::lock_guard<std::mutex> lock(port_mutex_);
  return ping_unlocked(id, model_number);
}

bool DynamixelBus::set_torque(uint8_t id, bool on)
{
  std::lock_guard<std::mutex> lock(port_mutex_);
  return set_torque_unlocked(id, on);
}

bool DynamixelBus::set_operating_mode(uint8_t id, const std::string & mode)
{
  std::lock_guard<std::mutex> lock(port_mutex_);
  return set_operating_mode_unlocked(id, mode);
}

bool DynamixelBus::set_led(uint8_t id, bool on)
{
  std::lock_guard<std::mutex> lock(port_mutex_);
  return write_register_unlocked(id, ct::LED, 1, on ? 1 : 0);
}

bool DynamixelBus::set_profile(uint8_t id, int32_t velocity, int32_t acceleration)
{
  std::lock_guard<std::mutex> lock(port_mutex_);
  if (acceleration >= 0 && !write_register_unlocked(id, 108, 4, acceleration)) { return false; }
  if (velocity >= 0 && !write_register_unlocked(id, 112, 4, velocity)) { return false; }
  return true;
}

bool DynamixelBus::set_zero(uint8_t id)
{
  std::lock_guard<std::mutex> lock(port_mutex_);

  if (!write_register_unlocked(id, ct::HOMING_OFFSET, 4, 0)) { return false; }

  int64_t present = 0;
  if (!read_register_unlocked(id, ct::PRESENT_POSITION, 4, present)) { return false; }

  if (!write_register_unlocked(id, ct::HOMING_OFFSET, 4, -present)) { return false; }

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    if (joints_[i].id == id)
    {
      homing_offset_[i] = static_cast<double>(-present) * (2.0 * M_PI / ct::TICKS_PER_REV);
    }
  }
  return true;
}

bool DynamixelBus::reboot(uint8_t id)
{
  std::lock_guard<std::mutex> lock(port_mutex_);

  uint8_t err = 0;
  const int rc = packet_->reboot(port_, id, &err);
  if (rc != COMM_SUCCESS)
  {
    last_error_ = "REBOOT failed (id=" + std::to_string(id) + "): " + packet_->getTxRxResult(rc);
    return false;
  }
  return true;
}

bool DynamixelBus::read_register(uint8_t id, uint16_t address, uint8_t size, int64_t & value)
{
  std::lock_guard<std::mutex> lock(port_mutex_);
  return read_register_unlocked(id, address, size, value);
}

bool DynamixelBus::write_register(uint8_t id, uint16_t address, uint8_t size, int64_t value)
{
  std::lock_guard<std::mutex> lock(port_mutex_);
  return write_register_unlocked(id, address, size, value);
}

bool DynamixelBus::ping_unlocked(uint8_t id, uint16_t & model_number)
{
  uint8_t err = 0;
  const int rc = packet_->ping(port_, id, &model_number, &err);
  if (rc != COMM_SUCCESS)
  {
    last_error_ = "PING failed (id=" + std::to_string(id) + "): " + packet_->getTxRxResult(rc);
    return false;
  }
  if (err != 0)
  {
    last_error_ = "id=" + std::to_string(id) + " reported hardware error: " + packet_->getRxPacketError(err);
  }
  return true;
}

bool DynamixelBus::set_torque_unlocked(uint8_t id, bool on)
{
  return write_register_unlocked(id, ct::TORQUE_ENABLE, 1, on ? 1 : 0);
}

bool DynamixelBus::set_operating_mode_unlocked(uint8_t id, const std::string & mode)
{
  return write_register_unlocked(id, ct::OPERATING_MODE, 1, ct::operating_mode_value(mode));
}

bool DynamixelBus::read_register_unlocked(
  uint8_t id, uint16_t address, uint8_t size, int64_t & value)
{
  uint8_t err = 0;
  int rc = COMM_TX_FAIL;

  switch (size)
  {
    case 1:
    {
      uint8_t v = 0;
      rc = packet_->read1ByteTxRx(port_, id, address, &v, &err);
      value = static_cast<int8_t>(v);
      break;
    }
    case 2:
    {
      uint16_t v = 0;
      rc = packet_->read2ByteTxRx(port_, id, address, &v, &err);
      value = static_cast<int16_t>(v);
      break;
    }
    case 4:
    {
      uint32_t v = 0;
      rc = packet_->read4ByteTxRx(port_, id, address, &v, &err);
      value = static_cast<int32_t>(v);
      break;
    }
    default:
      last_error_ = "Unsupported register size: " + std::to_string(size);
      return false;
  }

  if (rc != COMM_SUCCESS)
  {
    last_error_ = "Failed to read register " + std::to_string(address) + " (id=" + std::to_string(id) +
                  "): " + packet_->getTxRxResult(rc);
    return false;
  }
  return true;
}

bool DynamixelBus::write_register_unlocked(
  uint8_t id, uint16_t address, uint8_t size, int64_t value)
{
  uint8_t err = 0;
  int rc = COMM_TX_FAIL;

  switch (size)
  {
    case 1:
      rc = packet_->write1ByteTxRx(port_, id, address, static_cast<uint8_t>(value), &err);
      break;
    case 2:
      rc = packet_->write2ByteTxRx(port_, id, address, static_cast<uint16_t>(value), &err);
      break;
    case 4:
      rc = packet_->write4ByteTxRx(port_, id, address, static_cast<uint32_t>(value), &err);
      break;
    default:
      last_error_ = "Unsupported register size: " + std::to_string(size);
      return false;
  }

  if (rc != COMM_SUCCESS)
  {
    last_error_ = "Failed to write register " + std::to_string(address) + " (id=" + std::to_string(id) +
                  "): " + packet_->getTxRxResult(rc);
    return false;
  }
  if (err != 0)
  {
    last_error_ = "id=" + std::to_string(id) + " reported hardware error: " + packet_->getRxPacketError(err);
  }
  return true;
}

}  // namespace dynamixel_hardware
