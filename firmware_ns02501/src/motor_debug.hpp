// Motor debugging while uros_node runs: what dynamixel_tools asks of the board.
//
// Requests arrive as text on /gripper/motor_cmd, through dynamixel_tools/board_bridge:
//   <seq> read <id> <address> <size>
//   <seq> write <id> <address> <size> <value>
//   <seq> torque <id> <0|1>
//   <seq> reboot <id>
//   <seq> zero <id>
//   <seq> jog <eight joint positions>     no reply
// and are answered on /gripper/motor_reply:
//   <seq> ok <value> <text>   or   <seq> err 0 <text>
// Id 0 means every motor, except for read.
//
// No hardware and no ROS, so test/motor_debug_check.cpp can drive it on a PC. It
// runs on the control thread, between bus transactions.
#ifndef MOTOR_DEBUG_HPP_
#define MOTOR_DEBUG_HPP_

#include <cstddef>
#include <cstdint>

#include "servo/gripper_model.hpp"

namespace motor_debug
{

constexpr int kJoints = gripper_servo::kJoints;

enum class Op { kRead, kWrite, kTorque, kReboot, kZero };

struct Request
{
  uint32_t seq{0};
  Op op{Op::kRead};
  uint8_t id{0};
  uint16_t address{0};
  uint8_t size{0};
  int64_t value{0};
};

struct Reply
{
  uint32_t seq{0};
  bool ok{false};
  int64_t value{0};
  char text[80]{};
};

enum class Parsed { kRequest, kJog, kBad };

/// `jog` receives kJoints positions when the result is kJog. `request.seq` is
/// filled whenever the text starts with one, so kBad can still be answered.
Parsed parse(const char * text, Request & request, float * jog);

void format_reply(const Reply & reply, char * out, std::size_t size);

/// The bus and the servo underneath.
class Hardware
{
public:
  virtual ~Hardware() = default;
  virtual bool ping(uint8_t id) = 0;
  virtual bool reboot(uint8_t id) = 0;
  virtual bool read(uint8_t id, uint16_t address, uint8_t size, int64_t & value) = 0;
  virtual bool write(uint8_t id, uint16_t address, uint8_t size, int64_t value) = 0;
  virtual bool zero(uint8_t id) = 0;
  /// Back to the configured RAM registers, and the next cycle writes Goal Current.
  virtual bool restore_ram(int joint) = 0;
  /// Setpoint and Goal Position on the present position.
  virtual bool hold_here(int joint) = 0;
  virtual const char * last_error() = 0;
};

class MotorDebug
{
public:
  /// `ids` holds kJoints motor ids in joint order. `dry_run` refuses torque.
  MotorDebug(Hardware & hardware, const uint8_t * ids, bool dry_run);

  /// Runs a request. False means it is still going: keep calling poll() until it
  /// answers, and start nothing else while busy(). `read_ok` says this cycle's
  /// bus read succeeded, and with it that the position hold_here() uses is fresh.
  bool start(const Request & request, bool read_ok, double now, Reply & reply);

  /// Waits on a reboot. True when `reply` is ready.
  bool poll(double now, Reply & reply);
  bool busy() const { return rebooting_; }

  /// This cycle's Hardware Error Status. A motor can shut itself down without
  /// asking anyone, and its setpoint has to stop being trusted when it does.
  void observe_error(int joint, int error);

  /// Whether this cycle's measured pose is one Servo cannot move away from. If it
  /// stays that way every motor loses torque, and torque is refused until the pose
  /// has been cleared by hand. It does not come back on its own: there is a hand on
  /// the fingers at that moment.
  void observe_pose(bool trapped, double now);
  bool self_collision() const { return self_collision_; }
  /// The pose is clear again, and torque is still off because of it.
  bool resume_needed() const { return resume_needed_ && !self_collision_; }

  bool torque_on(int joint) const { return torque_on_[joint]; }
  bool all_on() const { return count_on() == kJoints; }
  bool any_on() const { return count_on() > 0; }
  int count_on() const;
  int faults() const;

private:
  int resolve(uint8_t id, int * joints) const;
  bool fail(Reply & reply, const char * text);
  bool enable(int joint, bool read_ok, Reply & reply);
  bool disable(int joint, Reply & reply);
  void run_torque(const int * joints, int count, bool on, bool read_ok, Reply & reply);
  void run_write(const Request & request, const int * joints, int count, bool read_ok,
                 Reply & reply);
  void run_zero(const int * joints, int count, Reply & reply);
  bool run_reboot(const int * joints, int count, double now, Reply & reply);

  Hardware & hardware_;
  uint8_t ids_[kJoints];
  bool dry_run_;
  bool torque_on_[kJoints];
  int last_error_[kJoints]{};
  double trapped_since_{-1.0};
  bool self_collision_{false};
  bool resume_needed_{false};
  bool restoring_[kJoints]{};
  bool rebooting_{false};
  uint32_t reboot_seq_{0};
  double reboot_from_{0.0};
  double reboot_until_{0.0};
  char reboot_failure_[80]{};
};

}  // namespace motor_debug

#endif  // MOTOR_DEBUG_HPP_
