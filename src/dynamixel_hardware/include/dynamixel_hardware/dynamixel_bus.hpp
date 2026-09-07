#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_BUS_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_BUS_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamixel_hardware/control_table.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel_hardware
{

struct JointSpec
{
  std::string name;
  uint8_t id{0};

  /// +1, or -1 when the motor is mounted so that its positive rotation is the
  /// URDF joint's negative one. Everything the joint exposes -- position,
  /// velocity, current, homing offset, commands and the firmware position
  /// limits -- is expressed in URDF sense; this is the only place the wiring
  /// is reconciled. The description stays faithful to the CAD.
  int direction{1};
  std::string operating_mode{"position"};
  bool commandable{false};
  std::unordered_map<std::string, std::string> parameters;
};

struct JointFeedback
{
  double position{0.0};  ///< rad
  double velocity{0.0};  ///< rad/s
  double effort{0.0};    ///< N·m, unpopulated: needs a per-model torque constant
  double current{0.0};   ///< Present Current, raw register units
  double temperature{0.0};    ///< deg C
  double voltage{0.0};        ///< V
  double pwm{0.0};            ///< raw
  double moving{0.0};         ///< 0/1
  double error{0.0};          ///< Hardware Error Status bitfield
  double homing_offset{0.0};  ///< rad
  bool valid{false};     ///< no response
};

class DynamixelBus
{
public:
  DynamixelBus(std::string port_name, int baud_rate);
  ~DynamixelBus();

  DynamixelBus(const DynamixelBus &) = delete;
  DynamixelBus & operator=(const DynamixelBus &) = delete;

  bool open();
  void close();
  bool is_open() const { return port_ != nullptr; }

  bool configure(const std::vector<JointSpec> & joints);

  bool read_all(std::vector<JointFeedback> & out);

  bool write_positions(const std::vector<double> & rads);

  /// Goal Current, in the motor's own register units. Only worth sending when
  /// the ceiling actually changes -- it is a second bus transaction, and over
  /// USB CDC that costs a millisecond whatever the baud rate.
  bool write_currents(const std::vector<double> & units);

  bool ping(uint8_t id, uint16_t & model_number);
  bool set_torque(uint8_t id, bool on);
  bool set_led(uint8_t id, bool on);
  bool reboot(uint8_t id);
  bool set_operating_mode(uint8_t id, const std::string & mode);
  bool set_profile(uint8_t id, int32_t velocity, int32_t acceleration);
  bool set_zero(uint8_t id);

  bool read_register(uint8_t id, uint16_t address, uint8_t size, int64_t & value);
  bool write_register(uint8_t id, uint16_t address, uint8_t size, int64_t value);

  static double tick_to_rad(int32_t tick, int direction);
  static int32_t rad_to_tick(double rad, int direction);
  static double raw_to_rad_per_s(int32_t raw);

  const std::vector<JointSpec> & joints() const { return joints_; }
  const std::string & last_error() const { return last_error_; }

private:
  bool ping_unlocked(uint8_t id, uint16_t & model_number);
  bool set_torque_unlocked(uint8_t id, bool on);
  bool set_operating_mode_unlocked(uint8_t id, const std::string & mode);
  bool read_register_unlocked(uint8_t id, uint16_t address, uint8_t size, int64_t & value);
  bool write_register_unlocked(uint8_t id, uint16_t address, uint8_t size, int64_t value);
  void release_lock();
  bool apply_parameters(const JointSpec & joint);
  bool setup_sync();

  mutable std::mutex port_mutex_;

  std::string port_name_;
  int baud_rate_;
  std::string last_error_;

  int lock_fd_{-1};

  dynamixel::PortHandler * port_{nullptr};
  dynamixel::PacketHandler * packet_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead> sync_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_current_;

  std::vector<JointSpec> joints_;

  std::vector<double> homing_offset_;
  std::vector<double> error_;
  std::size_t error_cursor_{0};
};

}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_BUS_HPP_
