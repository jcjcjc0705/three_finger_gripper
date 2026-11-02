#include "motor_debug.hpp"

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace motor_debug
{
namespace
{

constexpr uint16_t kIdRegister = 7;
constexpr uint16_t kBaudRegister = 8;
constexpr uint16_t kTorqueEnable = 64;
constexpr uint16_t kHardwareError = 70;
constexpr uint16_t kControlLoopFirst = 100;   // Goal PWM
constexpr uint16_t kControlLoopEnd = 120;     // past Goal Position

/// A motor still answering the instant after a reboot has not rebooted yet.
constexpr double kRebootSettleS = 0.3;
constexpr double kRebootTimeoutS = 3.0;

/// Long enough that a pose grazed in passing does not drop what is in the hand.
constexpr double kTrappedHoldS = 0.3;

bool overlaps(uint16_t address, uint8_t size, uint16_t first, uint16_t end)
{
  return address < end && address + size > first;
}

int64_t masked(int64_t value, uint8_t size)
{
  return size >= 8 ? value : value & ((int64_t{1} << (size * 8)) - 1);
}

}  // namespace

Parsed parse(const char * text, Request & request, float * jog)
{
  request = Request{};
  unsigned seq = 0;
  char verb[8] = "";
  int used = 0;
  if (std::sscanf(text, "%u %7s %n", &seq, verb, &used) < 2) { return Parsed::kBad; }
  request.seq = seq;
  const char * args = text + used;

  unsigned id = 0, address = 0, size = 0;
  long long value = 0;
  if (std::strcmp(verb, "jog") == 0) {
    int consumed = 0;
    for (int i = 0; i < kJoints; ++i) {
      int n = 0;
      if (std::sscanf(args + consumed, "%f%n", &jog[i], &n) != 1) { return Parsed::kBad; }
      consumed += n;
    }
    return Parsed::kJog;
  } else if (std::strcmp(verb, "read") == 0) {
    request.op = Op::kRead;
    if (std::sscanf(args, "%u %u %u", &id, &address, &size) != 3) { return Parsed::kBad; }
  } else if (std::strcmp(verb, "write") == 0) {
    request.op = Op::kWrite;
    if (std::sscanf(args, "%u %u %u %lld", &id, &address, &size, &value) != 4) {
      return Parsed::kBad;
    }
  } else if (std::strcmp(verb, "torque") == 0) {
    request.op = Op::kTorque;
    if (std::sscanf(args, "%u %lld", &id, &value) != 2) { return Parsed::kBad; }
  } else if (std::strcmp(verb, "reboot") == 0) {
    request.op = Op::kReboot;
    if (std::sscanf(args, "%u", &id) != 1) { return Parsed::kBad; }
  } else if (std::strcmp(verb, "zero") == 0) {
    request.op = Op::kZero;
    if (std::sscanf(args, "%u", &id) != 1) { return Parsed::kBad; }
  } else {
    return Parsed::kBad;
  }
  if (id > 255 || address > 0xFFFF || size > 4) { return Parsed::kBad; }
  request.id = static_cast<uint8_t>(id);
  request.address = static_cast<uint16_t>(address);
  request.size = static_cast<uint8_t>(size);
  request.value = value;
  return Parsed::kRequest;
}

void format_reply(const Reply & reply, char * out, std::size_t size)
{
  std::snprintf(out, size, "%" PRIu32 " %s %" PRId64 " %s", reply.seq,
                reply.ok ? "ok" : "err", reply.value, reply.text);
}

MotorDebug::MotorDebug(Hardware & hardware, const uint8_t * ids, bool dry_run)
: hardware_(hardware), dry_run_(dry_run)
{
  for (int i = 0; i < kJoints; ++i) {
    ids_[i] = ids[i];
    torque_on_[i] = !dry_run;
  }
}

int MotorDebug::count_on() const
{
  int count = 0;
  for (bool on : torque_on_) { count += on ? 1 : 0; }
  return count;
}

int MotorDebug::faults() const
{
  int count = 0;
  for (int error : last_error_) { count += error != 0 ? 1 : 0; }
  return count;
}

int MotorDebug::resolve(uint8_t id, int * joints) const
{
  int count = 0;
  for (int i = 0; i < kJoints; ++i) {
    if (id == 0 || ids_[i] == id) { joints[count++] = i; }
  }
  return count;
}

bool MotorDebug::fail(Reply & reply, const char * text)
{
  reply.ok = false;
  reply.value = 0;
  std::snprintf(reply.text, sizeof(reply.text), "%s", text);
  return false;
}

bool MotorDebug::enable(int joint, bool read_ok, Reply & reply)
{
  // Already on means it may be gripping, and holding here would let go.
  if (torque_on_[joint]) { return true; }
  if (dry_run_) { return fail(reply, "dry run: torque stays off"); }
  if (!read_ok) { return fail(reply, "bus read failed, no position to hold"); }
  if (trapped_since_ >= 0.0) {
    return fail(reply, "self-collision pose, move the fingers apart by hand first");
  }

  const uint8_t id = ids_[joint];
  int64_t state = 0;
  if (!hardware_.hold_here(joint) || !hardware_.write(id, kTorqueEnable, 1, 1) ||
      !hardware_.read(id, kTorqueEnable, 1, state))
  {
    return fail(reply, hardware_.last_error());
  }
  // The bus reports a write the motor refused as a success, so ask the motor.
  if (state != 1) {
    int64_t error = 0;
    hardware_.read(id, kHardwareError, 1, error);
    char text[sizeof(reply.text)];
    std::snprintf(text, sizeof(text), "id=%d refused torque, hardware error 0x%02x",
                  id, static_cast<unsigned>(masked(error, 1)));
    return fail(reply, text);
  }
  torque_on_[joint] = true;
  return true;
}

bool MotorDebug::disable(int joint, Reply & reply)
{
  const uint8_t id = ids_[joint];
  int64_t state = 1;
  if (!hardware_.write(id, kTorqueEnable, 1, 0) ||
      !hardware_.read(id, kTorqueEnable, 1, state))
  {
    return fail(reply, hardware_.last_error());
  }
  torque_on_[joint] = false;
  if (state != 0) { return fail(reply, "motor did not drop torque"); }
  return true;
}

void MotorDebug::run_torque(const int * joints, int count, bool on, bool read_ok, Reply & reply)
{
  for (int i = 0; i < count; ++i) {
    if (!(on ? enable(joints[i], read_ok, reply) : disable(joints[i], reply))) { return; }
  }
  reply.ok = true;
  std::snprintf(reply.text, sizeof(reply.text), "torque %s", on ? "enabled" : "disabled");
}

void MotorDebug::run_write(const Request & request, const int * joints, int count, bool read_ok,
                           Reply & reply)
{
  const uint16_t address = request.address;
  const uint8_t size = request.size;
  if (address == kTorqueEnable && size == 1) {
    // Torque written behind the servo's back would drive to a stale setpoint.
    run_torque(joints, count, request.value != 0, read_ok, reply);
    return;
  }
  if (overlaps(address, size, kTorqueEnable, kTorqueEnable + 1)) {
    fail(reply, "write Torque Enable on its own");
    return;
  }
  if (overlaps(address, size, kIdRegister, kBaudRegister + 1)) {
    fail(reply, "id and baud cannot change here, stop uros_node and use dxl_offline");
    return;
  }
  // Goal Current is only written when its value changes, so a value written here
  // would stay. A changed profile would stay too, and would lag every later move.
  if (overlaps(address, size, kControlLoopFirst, kControlLoopEnd)) {
    fail(reply, "goal and profile registers belong to the control loop");
    return;
  }
  if (address < kTorqueEnable) {
    for (int i = 0; i < count; ++i) {
      if (torque_on_[joints[i]]) {
        fail(reply, "EEPROM register, turn torque off first");
        return;
      }
    }
  }
  for (int i = 0; i < count; ++i) {
    const uint8_t id = ids_[joints[i]];
    int64_t stored = 0;
    if (!hardware_.write(id, address, size, request.value) ||
        !hardware_.read(id, address, size, stored))
    {
      fail(reply, hardware_.last_error());
      return;
    }
    if (masked(stored, size) != masked(request.value, size)) {
      char text[sizeof(reply.text)];
      std::snprintf(text, sizeof(text), "id=%d refused the write, register holds %" PRId64,
                    id, stored);
      fail(reply, text);
      return;
    }
  }
  reply.ok = true;
  std::snprintf(reply.text, sizeof(reply.text), "written");
}

void MotorDebug::run_zero(const int * joints, int count, Reply & reply)
{
  for (int i = 0; i < count; ++i) {
    if (torque_on_[joints[i]]) {
      fail(reply, "homing offset is in EEPROM, turn torque off first");
      return;
    }
  }
  for (int i = 0; i < count; ++i) {
    if (!hardware_.zero(ids_[joints[i]])) {
      fail(reply, hardware_.last_error());
      return;
    }
  }
  reply.ok = true;
  std::snprintf(reply.text, sizeof(reply.text), "homing offset updated");
}

bool MotorDebug::run_reboot(const int * joints, int count, double now, Reply & reply)
{
  reboot_failure_[0] = '\0';
  for (int i = 0; i < count; ++i) {
    if (!hardware_.reboot(ids_[joints[i]])) {
      std::snprintf(reboot_failure_, sizeof(reboot_failure_), "%s", hardware_.last_error());
      break;
    }
    torque_on_[joints[i]] = false;
    restoring_[joints[i]] = true;
    rebooting_ = true;
  }
  if (!rebooting_) {
    fail(reply, reboot_failure_);
    return true;
  }
  // Whatever did reboot still has to be restored before answering.
  reboot_seq_ = reply.seq;
  reboot_from_ = now;
  reboot_until_ = now + kRebootTimeoutS;
  return false;
}

bool MotorDebug::start(const Request & request, bool read_ok, double now, Reply & reply)
{
  reply = Reply{};
  reply.seq = request.seq;

  int joints[kJoints];
  if (request.op == Op::kRead) {
    const int count = request.id == 0 ? 0 : resolve(request.id, joints);
    if (count == 0) {
      fail(reply, "no such motor");
    } else if (!hardware_.read(request.id, request.address, request.size, reply.value)) {
      fail(reply, hardware_.last_error());
    } else {
      reply.ok = true;
    }
    return true;
  }

  const int count = resolve(request.id, joints);
  if (count == 0) {
    fail(reply, "no such motor");
    return true;
  }
  switch (request.op) {
    case Op::kWrite: run_write(request, joints, count, read_ok, reply); return true;
    case Op::kTorque: run_torque(joints, count, request.value != 0, read_ok, reply); return true;
    case Op::kZero: run_zero(joints, count, reply); return true;
    case Op::kReboot: return run_reboot(joints, count, now, reply);
    default: return true;
  }
}

bool MotorDebug::poll(double now, Reply & reply)
{
  if (!rebooting_) { return false; }
  if (now - reboot_from_ < kRebootSettleS) { return false; }

  reply = Reply{};
  reply.seq = reboot_seq_;

  // One motor a cycle: a ping that goes unanswered takes a while to give up.
  for (int i = 0; i < kJoints; ++i) {
    if (!restoring_[i]) { continue; }
    if (hardware_.ping(ids_[i])) {
      restoring_[i] = false;
      if (!hardware_.restore_ram(i)) {
        rebooting_ = false;
        for (bool & r : restoring_) { r = false; }
        fail(reply, hardware_.last_error());
        return true;
      }
    } else if (now >= reboot_until_) {
      rebooting_ = false;
      for (bool & r : restoring_) { r = false; }
      char text[sizeof(reply.text)];
      std::snprintf(text, sizeof(text), "id=%d did not come back", ids_[i]);
      fail(reply, text);
      return true;
    }
    return false;
  }

  rebooting_ = false;
  if (reboot_failure_[0] != '\0') {
    fail(reply, reboot_failure_);
    return true;
  }
  reply.ok = true;
  std::snprintf(reply.text, sizeof(reply.text), "rebooted (torque is now off)");
  return true;
}

void MotorDebug::observe_pose(bool trapped, double now)
{
  if (resume_needed_ && all_on()) { resume_needed_ = false; }
  if (!trapped) {
    trapped_since_ = -1.0;
    self_collision_ = false;
    return;
  }
  if (trapped_since_ < 0.0) { trapped_since_ = now; }
  if (now - trapped_since_ < kTrappedHoldS) { return; }

  self_collision_ = true;
  if (dry_run_) { return; }
  resume_needed_ = true;
  // Every cycle: a motor that missed the write is still pushing.
  Reply ignored;
  for (int i = 0; i < kJoints; ++i) {
    if (torque_on_[i]) { disable(i, ignored); }
  }
}

void MotorDebug::observe_error(int joint, int error)
{
  // Only on the change: the reading lags a reboot by up to a full round of the
  // motors, and it must not cancel a torque enable that came after the reboot.
  if (error != 0 && last_error_[joint] == 0 && torque_on_[joint]) {
    int64_t state = 0;
    if (!hardware_.read(ids_[joint], kTorqueEnable, 1, state) || state == 0) {
      torque_on_[joint] = false;
    }
  }
  last_error_[joint] = error;
}

}  // namespace motor_debug
