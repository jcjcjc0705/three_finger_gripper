// The eight motors. The angles behind the position limits are also in the URDF's
// <limit> tags: if those change, change this too. Nothing checks that they agree.
//
// Position limits are motor ticks:
//     int(2048 + radians(deg) * 4096 / (2*pi))
// They are the firmware's own end stops, the last protection below anything
// the servo layer does.
#pragma once

#include <string>
#include <vector>

#include "dynamixel/dynamixel_bus.hpp"
#include "servo/gripper_model.hpp"

namespace gripper_joints
{

/// Ceiling on what any joint may draw, written to the motor's Current Limit.
constexpr int kCurrentLimit = 800;
/// Goal Current the flexion joints start at -- how hard a finger may curl.
constexpr int kFlexCurrent = 350;
/// Goal Current for the two spread joints, which only have to swing a finger
/// sideways and can meet another finger doing it.
constexpr int kSpreadCurrent = 250;

struct JointDef
{
  const char * name;
  uint8_t id;
  int direction;      ///< -1 where the motor turns opposite to the URDF joint
  int min_position;   ///< ticks
  int max_position;   ///< ticks
  int goal_current;
};

/// Order matters: it is gripper_servo::kJointNames, and Servo indexes its
/// fixed arrays by it. Motor ID happens to be index + 1.
inline constexpr JointDef kJointDefs[gripper_servo::kJoints] = {
  {"C_2_Joint", 1,  1, 1903, 2869, kFlexCurrent},    // thumb proximal   -12.7° ~  72.2°
  {"C_3_Joint", 2,  1,  682, 2467, kFlexCurrent},    // thumb distal    -120.0° ~  36.9°
  {"A_2_Joint", 3,  1, 1722, 2854, kFlexCurrent},    // index proximal   -28.6° ~  70.9°
  {"A_3_Joint", 4,  1,  682, 2467, kFlexCurrent},    // index distal    -120.0° ~  36.9°
  {"B_2_Joint", 5,  1, 1722, 2854, kFlexCurrent},    // middle proximal  -28.6° ~  70.9°
  {"B_3_Joint", 6,  1,  682, 2467, kFlexCurrent},    // middle distal   -120.0° ~  36.9°
  {"A_1_Joint", 7, -1, 1024, 3072, kSpreadCurrent},  // index spread       ±90°, mounted reversed
  {"B_1_Joint", 8, -1, 1024, 3072, kSpreadCurrent},  // middle spread      ±90°, mounted reversed
};

/// Build the JointSpec list DynamixelBus::configure() takes. The parameter
/// names are the ones control_table::WRITABLE_REGISTERS knows; anything not
/// listed there is silently ignored, so a typo here goes unnoticed.
inline std::vector<dxl::JointSpec> make_joint_specs()
{
  std::vector<dxl::JointSpec> specs;
  specs.reserve(gripper_servo::kJoints);

  for (const auto & def : kJointDefs)
  {
    dxl::JointSpec spec;
    spec.name = def.name;
    spec.id = def.id;
    spec.direction = def.direction;
    spec.operating_mode = "current_based_position";
    spec.commandable = true;

    spec.parameters["current_limit"] = std::to_string(kCurrentLimit);
    spec.parameters["goal_current"] = std::to_string(def.goal_current);
    spec.parameters["min_position_limit"] = std::to_string(def.min_position);
    spec.parameters["max_position_limit"] = std::to_string(def.max_position);
    // Time-based profile, so profile_velocity/acceleration are milliseconds.
    spec.parameters["drive_mode"] = "4";
    spec.parameters["profile_velocity"] = "50";
    spec.parameters["profile_acceleration"] = "25";
    spec.parameters["position_p_gain"] = "1000";
    spec.parameters["position_i_gain"] = "0";
    spec.parameters["position_d_gain"] = "1000";
    // Overload (32), electrical shock (16) and overheating (4), the factory
    // default. Not input voltage (1): the rail sags under a grip and a reboot
    // mid-grasp drops the payload.
    spec.parameters["shutdown"] = "52";

    specs.push_back(std::move(spec));
  }
  return specs;
}

}  // namespace gripper_joints
