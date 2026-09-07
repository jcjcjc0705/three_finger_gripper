#ifndef DYNAMIXEL_HARDWARE__CONTROL_TABLE_HPP_
#define DYNAMIXEL_HARDWARE__CONTROL_TABLE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace dynamixel_hardware::control_table
{

constexpr uint16_t DRIVE_MODE = 10;
constexpr uint16_t OPERATING_MODE = 11;
constexpr uint16_t HOMING_OFFSET = 20;
constexpr uint16_t TORQUE_ENABLE = 64;
constexpr uint16_t LED = 65;
constexpr uint16_t GOAL_CURRENT = 102;
constexpr uint16_t GOAL_POSITION = 116;
constexpr uint16_t HARDWARE_ERROR_STATUS = 70;
constexpr uint16_t MOVING = 122;
constexpr uint16_t PRESENT_PWM = 124;
constexpr uint16_t PRESENT_CURRENT = 126;
constexpr uint16_t PRESENT_VELOCITY = 128;
constexpr uint16_t PRESENT_POSITION = 132;
constexpr uint16_t PRESENT_INPUT_VOLTAGE = 144;
constexpr uint16_t PRESENT_TEMPERATURE = 146;

constexpr uint16_t SYNC_READ_START = MOVING;
constexpr uint16_t SYNC_READ_LENGTH = 25;
constexpr uint16_t GOAL_POSITION_LENGTH = 4;
constexpr uint16_t GOAL_CURRENT_LENGTH = 2;

constexpr double TICKS_PER_REV = 4096.0;
constexpr double CENTER_TICK = 2048.0;
constexpr double VELOCITY_UNIT_RPM = 0.229;

struct RegisterSpec
{
  const char * param_name;
  uint16_t address;
  uint8_t size;  // bytes
  bool eeprom;
};

inline constexpr std::array<RegisterSpec, 19> WRITABLE_REGISTERS = {{
  // EEPROM
  {"drive_mode", 10, 1, true},
  {"homing_offset", 20, 4, true},
  {"temperature_limit", 31, 1, true},
  {"max_voltage_limit", 32, 2, true},
  {"min_voltage_limit", 34, 2, true},
  {"pwm_limit", 36, 2, true},
  {"current_limit", 38, 2, true},
  {"velocity_limit", 44, 4, true},
  {"max_position_limit", 48, 4, true},
  {"min_position_limit", 52, 4, true},
  {"shutdown", 63, 1, true},
  // RAM
  {"velocity_i_gain", 76, 2, false},
  {"velocity_p_gain", 78, 2, false},
  {"position_d_gain", 80, 2, false},
  {"position_i_gain", 82, 2, false},
  {"position_p_gain", 84, 2, false},
  {"goal_current", 102, 2, false},
  {"profile_acceleration", 108, 4, false},
  {"profile_velocity", 112, 4, false},
}};

struct ReadableSpec
{
  const char * name;
  uint16_t address;
  uint8_t size;
  bool is_signed;
  const char * note;
};

inline constexpr std::array<ReadableSpec, 38> READABLE_REGISTERS = {{
  {"Return Delay Time", 9, 1, false, "x2 usec"},
  {"Drive Mode", 10, 1, false, "bit2: time-based profile"},
  {"Operating Mode", 11, 1, false, ""},
  {"Homing Offset", 20, 4, true, "ticks"},
  {"Temperature Limit", 31, 1, false, "deg C"},
  {"Max Voltage Limit", 32, 2, false, "x0.1 V"},
  {"Min Voltage Limit", 34, 2, false, "x0.1 V"},
  {"PWM Limit", 36, 2, false, ""},
  {"Current Limit", 38, 2, false, "model ticks"},
  {"Velocity Limit", 44, 4, false, ""},
  {"Max Position Limit", 48, 4, false, "ticks"},
  {"Min Position Limit", 52, 4, false, "ticks"},
  {"Shutdown", 63, 1, false, "error-bit mask"},
  {"Torque Enable", 64, 1, false, "0/1"},
  {"LED", 65, 1, false, "0/1"},
  {"Velocity I Gain", 76, 2, false, ""},
  {"Velocity P Gain", 78, 2, false, ""},
  {"Position D Gain", 80, 2, false, ""},
  {"Position I Gain", 82, 2, false, ""},
  {"Position P Gain", 84, 2, false, ""},
  {"Feedforward 2nd Gain", 88, 2, false, "acceleration"},
  {"Feedforward 1st Gain", 90, 2, false, "velocity"},
  {"Bus Watchdog", 98, 1, false, ""},
  {"Goal PWM", 100, 2, true, ""},
  {"Goal Current", 102, 2, true, ""},
  {"Goal Velocity", 104, 4, true, ""},
  {"Profile Acceleration", 108, 4, false, "ms or deg/s^2"},
  {"Profile Velocity", 112, 4, false, "ms or deg/s"},
  {"Goal Position", 116, 4, true, "ticks"},
  {"Moving", 122, 1, false, "0/1"},
  {"Moving Status", 123, 1, false, "bitfield"},
  {"Present PWM", 124, 2, true, ""},
  {"Present Current/Load", 126, 2, true, "Load on XL430, Current on XL330"},
  {"Present Velocity", 128, 4, true, ""},
  {"Present Position", 132, 4, true, "ticks"},
  {"Velocity Trajectory", 136, 4, true, ""},
  {"Position Trajectory", 140, 4, true, "ticks"},
  {"Present Input Voltage", 144, 2, false, "x0.1 V"},
}};

inline uint8_t operating_mode_value(const std::string & mode)
{
  if (mode == "current") { return 0; }
  if (mode == "velocity") { return 1; }
  if (mode == "position") { return 3; }
  if (mode == "extended_position") { return 4; }
  if (mode == "current_based_position") { return 5; }
  if (mode == "pwm") { return 16; }
  return 3;
}

}  // namespace dynamixel_hardware::control_table

#endif  // DYNAMIXEL_HARDWARE__CONTROL_TABLE_HPP_
