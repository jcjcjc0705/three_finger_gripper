// Behaviour checks for motor debugging that need no hardware.
//
//   ./build.sh check

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "motor_debug.hpp"

using namespace motor_debug;

namespace
{

int failures = 0;

void check(bool ok, const char * what)
{
  std::printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok) { ++failures; }
}

constexpr uint16_t kTorque = 64;
constexpr uint16_t kError = 70;
constexpr uint16_t kGoalCurrent = 102;
constexpr uint16_t kHoming = 20;

const uint8_t kIds[kJoints] = {1, 2, 3, 4, 5, 6, 7, 8};

/// Behaves like the bus: a write the motor refuses still reports success.
struct FakeMotors : Hardware
{
  std::map<int, std::map<int, int64_t>> regs;
  std::map<int, int> asleep;     // pings left before a rebooted motor answers
  std::vector<std::string> log;
  bool bus_dead{false};
  bool reboot_fails{false};
  int restored{-1};
  char error[32] = "bus error";

  FakeMotors()
  {
    for (uint8_t id : kIds) { regs[id][kTorque] = 1; regs[id][kGoalCurrent] = 350; }
  }

  bool ping(uint8_t id) override
  {
    if (asleep[id] > 0) { --asleep[id]; return false; }
    return !bus_dead;
  }
  bool reboot(uint8_t id) override
  {
    if (reboot_fails) { return false; }
    log.push_back("reboot " + std::to_string(id));
    regs[id][kTorque] = 0;
    regs[id][kError] = 0;
    regs[id][kGoalCurrent] = 800;
    asleep[id] = 2;
    return true;
  }
  bool read(uint8_t id, uint16_t address, uint8_t, int64_t & value) override
  {
    if (bus_dead) { return false; }
    value = regs[id][address];
    return true;
  }
  bool write(uint8_t id, uint16_t address, uint8_t, int64_t value) override
  {
    if (bus_dead) { return false; }
    log.push_back("write " + std::to_string(id) + " " + std::to_string(address));
    const bool eeprom_locked = address < kTorque && regs[id][kTorque] != 0;
    const bool enable_refused = address == kTorque && value != 0 && regs[id][kError] != 0;
    if (!eeprom_locked && !enable_refused) { regs[id][address] = value; }
    return true;
  }
  bool zero(uint8_t id) override
  {
    log.push_back("zero " + std::to_string(id));
    regs[id][kHoming] = -1234;
    return true;
  }
  bool restore_ram(int joint) override
  {
    restored = joint;
    regs[kIds[joint]][kGoalCurrent] = 350;
    return true;
  }
  bool hold_here(int joint) override
  {
    log.push_back("hold " + std::to_string(joint));
    return true;
  }
  const char * last_error() override { return error; }

  bool wrote(const std::string & entry) const
  {
    for (const auto & e : log) { if (e == entry) { return true; } }
    return false;
  }
};

Request make(Op op, int id, int address = 0, int size = 0, int64_t value = 0)
{
  Request r;
  r.seq = 7;
  r.op = op;
  r.id = static_cast<uint8_t>(id);
  r.address = static_cast<uint16_t>(address);
  r.size = static_cast<uint8_t>(size);
  r.value = value;
  return r;
}

/// Runs a request that answers straight away.
Reply run(MotorDebug & debug, const Request & request, bool read_ok = true)
{
  Reply reply;
  debug.start(request, read_ok, 0.0, reply);
  return reply;
}

/// Polls until a reboot answers, the way the control loop would.
Reply finish_reboot(MotorDebug & debug, double & now)
{
  Reply reply;
  for (int i = 0; i < 400; ++i) {
    now += 0.04;
    if (debug.poll(now, reply)) { return reply; }
  }
  reply.ok = false;
  std::snprintf(reply.text, sizeof(reply.text), "never answered");
  return reply;
}

}  // namespace

int main()
{
  std::printf("\nparsing\n");
  {
    Request r;
    float jog[kJoints];
    check(parse("12 read 3 146 1", r, jog) == Parsed::kRequest && r.seq == 12 &&
          r.op == Op::kRead && r.id == 3 && r.address == 146 && r.size == 1, "read");
    check(parse("13 write 0 65 1 -1", r, jog) == Parsed::kRequest && r.op == Op::kWrite &&
          r.id == 0 && r.value == -1, "write, id 0 and a negative value");
    check(parse("14 torque 3 1", r, jog) == Parsed::kRequest && r.op == Op::kTorque &&
          r.value == 1, "torque");
    check(parse("15 reboot 0", r, jog) == Parsed::kRequest && r.op == Op::kReboot, "reboot");
    check(parse("16 zero 2", r, jog) == Parsed::kRequest && r.op == Op::kZero, "zero");
    check(parse("0 jog 0 -0.5 1 1.5 2 -2 0.25 -0.25", r, jog) == Parsed::kJog &&
          jog[1] == -0.5f && jog[7] == -0.25f, "jog");
    check(parse("0 jog 1 2 3", r, jog) == Parsed::kBad, "jog with too few joints");
    check(parse("9 read 3 146", r, jog) == Parsed::kBad && r.seq == 9,
          "too few arguments keeps the seq, so it can be answered");
    check(parse("garbage", r, jog) == Parsed::kBad, "no seq");
    check(parse("9 launch 3", r, jog) == Parsed::kBad, "unknown verb");
    check(parse("9 read 300 146 1", r, jog) == Parsed::kBad, "id out of range");
    check(parse("9 read 3 146 8", r, jog) == Parsed::kBad, "size out of range");

    Reply reply;
    reply.seq = 4;
    reply.ok = true;
    reply.value = -6;
    std::snprintf(reply.text, sizeof(reply.text), "written");
    char out[64];
    format_reply(reply, out, sizeof(out));
    check(std::strcmp(out, "4 ok -6 written") == 0, "reply format");
  }

  std::printf("\nreads\n");
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    motors.regs[3][146] = 41;
    const Reply reply = run(debug, make(Op::kRead, 3, 146, 1));
    check(reply.ok && reply.value == 41, "reads the register");
    check(reply.seq == 7, "answers with the request's seq");
    check(!run(debug, make(Op::kRead, 0, 146, 1)).ok, "cannot read every motor at once");
    check(!run(debug, make(Op::kRead, 254, 146, 1)).ok, "broadcast id is refused");
    motors.bus_dead = true;
    check(!run(debug, make(Op::kRead, 3, 146, 1)).ok, "a dead bus is an error");
  }

  std::printf("\nregister protection\n");
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    check(!run(debug, make(Op::kWrite, 3, kGoalCurrent, 2, 800)).ok, "Goal Current refused");
    check(motors.regs[3][kGoalCurrent] == 350, "and not written");
    check(!run(debug, make(Op::kWrite, 3, 116, 4, 2048)).ok, "Goal Position refused");
    check(!run(debug, make(Op::kWrite, 0, 112, 4, 300)).ok, "Profile Velocity refused");
    check(!run(debug, make(Op::kWrite, 0, 108, 4, 150)).ok, "Profile Acceleration refused");
    check(!run(debug, make(Op::kWrite, 3, 7, 1, 9)).ok, "id refused");
    check(!run(debug, make(Op::kWrite, 3, 8, 1, 1)).ok, "baud refused");
    check(!run(debug, make(Op::kWrite, 3, 62, 4, 0)).ok, "a write across Torque Enable refused");
    check(!run(debug, make(Op::kWrite, 3, kHoming, 4, 5)).ok,
          "EEPROM refused with torque on");

    const Reply led = run(debug, make(Op::kWrite, 3, 65, 1, 1));
    check(led.ok && motors.regs[3][65] == 1, "LED is written");
    const Reply gain = run(debug, make(Op::kWrite, 0, 84, 2, 1200));
    check(gain.ok && motors.regs[1][84] == 1200 && motors.regs[8][84] == 1200,
          "a gain on every motor");
  }
  {
    // A register the motor will not hold has to come back as an error.
    struct ReadOnly : FakeMotors
    {
      bool write(uint8_t id, uint16_t address, uint8_t, int64_t) override
      {
        log.push_back("write " + std::to_string(id) + " " + std::to_string(address));
        return true;
      }
    } motors;
    MotorDebug debug(motors, kIds, false);
    check(!run(debug, make(Op::kWrite, 3, 146, 1, 20)).ok,
          "a write the motor ignored is reported");
    motors.regs[3][65] = 100;
    check(!run(debug, make(Op::kWrite, 3, 65, 1, -56)).ok, "the read-back is compared");
    motors.regs[3][65] = -56;
    check(run(debug, make(Op::kWrite, 3, 65, 1, 200)).ok,
          "signedness alone is not a mismatch");
  }

  std::printf("\ntorque\n");
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    check(debug.all_on(), "starts on");
    check(run(debug, make(Op::kTorque, 3, 0, 0, 0)).ok, "off one motor");
    check(!debug.torque_on(2) && debug.torque_on(0) && !debug.all_on() && debug.any_on(),
          "only that one");
    check(motors.regs[3][kTorque] == 0, "the motor agrees");

    motors.log.clear();
    check(!run(debug, make(Op::kTorque, 3, 0, 0, 1), false).ok,
          "no enable on a cycle whose read failed");
    check(!debug.torque_on(2), "still off");
    check(motors.log.empty(), "and nothing was sent");

    motors.log.clear();
    check(run(debug, make(Op::kTorque, 3, 0, 0, 1)).ok, "back on");
    check(motors.log.size() >= 2 && motors.log[0] == "hold 2" && motors.log[1] == "write 3 64",
          "setpoint first, torque after");
    check(debug.torque_on(2), "recorded");

    motors.log.clear();
    check(run(debug, make(Op::kTorque, 0, 0, 0, 1)).ok, "all on, when they already are");
    check(motors.log.empty(), "does not touch a motor that may be gripping");

    check(run(debug, make(Op::kTorque, 0, 0, 0, 0)).ok, "all off");
    check(!debug.any_on(), "every one");

    motors.log.clear();
    check(run(debug, make(Op::kWrite, 4, kTorque, 1, 1)).ok,
          "writing Torque Enable goes the same way");
    check(motors.wrote("hold 3") && debug.torque_on(3), "through the setpoint");
  }
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, true);
    check(!debug.any_on(), "dry run starts off");
    check(!run(debug, make(Op::kTorque, 0, 0, 0, 1)).ok, "and refuses torque");
    check(!run(debug, make(Op::kWrite, 3, kTorque, 1, 1)).ok, "by register too");
    check(motors.regs[3][kTorque] == 1 && !motors.wrote("write 3 64"), "nothing sent");
  }

  std::printf("\nzero\n");
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    check(!run(debug, make(Op::kZero, 3)).ok, "refused with torque on");
    check(!motors.wrote("zero 3"), "nothing sent");
    run(debug, make(Op::kTorque, 3, 0, 0, 0));
    check(run(debug, make(Op::kZero, 3)).ok && motors.wrote("zero 3"), "works with it off");
    check(!run(debug, make(Op::kZero, 0)).ok, "all motors, while others are on, is refused");
    check(!motors.wrote("zero 1"), "before touching any");
  }

  std::printf("\nreboot\n");
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    motors.regs[3][kError] = 0x20;
    double now = 0.0;
    Reply reply;
    check(!debug.start(make(Op::kReboot, 3), true, now, reply), "does not answer straight away");
    check(debug.busy(), "busy until it does");
    check(!debug.torque_on(2) && debug.torque_on(0), "that motor's torque is gone");
    check(!debug.poll(now + 0.1, reply), "waits before trusting a ping");
    check(motors.restored == -1, "and restores nothing yet");
    reply = finish_reboot(debug, now);
    check(reply.ok && reply.seq == 7, "answers once it is back");
    check(motors.restored == 2, "restores that motor's registers");
    check(motors.regs[3][kGoalCurrent] == 350, "Goal Current no longer the default");
    check(!debug.torque_on(2), "and leaves torque off");
    check(!debug.busy(), "no longer busy");
  }
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    double now = 0.0;
    Reply reply;
    debug.start(make(Op::kReboot, 0), true, now, reply);
    check(!debug.any_on(), "reboot all");
    reply = finish_reboot(debug, now);
    check(reply.ok, "answers once every motor is back");
    for (uint8_t id : kIds) { check(motors.asleep[id] == 0, "motor back"); }
  }
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    motors.reboot_fails = true;
    Reply reply;
    check(debug.start(make(Op::kReboot, 3), true, 0.0, reply) && !reply.ok,
          "a reboot that could not be sent answers at once");
    check(debug.torque_on(2) && !debug.busy(), "and changes nothing");
  }
  {
    // Motor 1 reboots, motor 2 cannot be reached. Motor 1 still has to be restored.
    struct SecondFails : FakeMotors
    {
      bool reboot(uint8_t id) override
      {
        return id == 2 ? false : FakeMotors::reboot(id);
      }
    } motors;
    MotorDebug debug(motors, kIds, false);
    double now = 0.0;
    Reply reply;
    check(!debug.start(make(Op::kReboot, 0), true, now, reply), "partial reboot still waits");
    reply = finish_reboot(debug, now);
    check(!reply.ok, "and reports the failure");
    check(motors.restored == 0, "after restoring the motor that did reboot");
    check(!debug.torque_on(0) && debug.torque_on(1), "torque matches what happened");
  }
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    double now = 0.0;
    Reply reply;
    debug.start(make(Op::kReboot, 3), true, now, reply);
    motors.bus_dead = true;
    reply = finish_reboot(debug, now);
    check(!reply.ok && std::strstr(reply.text, "id=3") != nullptr,
          "a motor that never returns is named");
    check(!debug.busy(), "and it stops waiting");
  }

  std::printf("\nhardware errors\n");
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    debug.observe_error(2, 0);
    check(debug.torque_on(2) && debug.faults() == 0, "no error, nothing changes");

    motors.regs[3][kError] = 0x20;
    motors.regs[3][kTorque] = 0;
    debug.observe_error(2, 0x20);
    check(!debug.torque_on(2), "a motor that shut itself down is off");
    check(debug.faults() == 1, "and counted");

    const Reply reply = run(debug, make(Op::kTorque, 3, 0, 0, 1));
    check(!reply.ok && std::strstr(reply.text, "0x20") != nullptr,
          "it refuses torque, and the reply says why");
    check(!debug.torque_on(2), "still off");

    double now = 0.0;
    Reply rebooted;
    debug.start(make(Op::kReboot, 3), true, now, rebooted);
    rebooted = finish_reboot(debug, now);
    check(rebooted.ok, "reboot clears it");
    check(run(debug, make(Op::kTorque, 3, 0, 0, 1)).ok, "torque comes back");
    debug.observe_error(2, 0x20);
    check(debug.torque_on(2), "a stale reading does not cancel it");
    debug.observe_error(2, 0);
    check(debug.faults() == 0, "cleared once it is read again");

    motors.regs[3][kTorque] = 0;
    debug.observe_error(2, 0x04);
    check(!debug.torque_on(2), "a new error after that still counts");
  }
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    motors.regs[3][kTorque] = 1;
    debug.observe_error(2, 0x01);
    check(debug.torque_on(2), "an error that did not shut the motor down leaves it on");
    check(debug.faults() == 1, "but is still counted");
  }

  std::printf("\nself-collision\n");
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    debug.observe_pose(false, 0.0);
    check(debug.all_on() && !debug.self_collision(), "a clear pose changes nothing");

    debug.observe_pose(true, 1.0);
    debug.observe_pose(true, 1.2);
    check(debug.all_on() && !debug.self_collision(), "a pose grazed in passing keeps the torque");
    debug.observe_pose(false, 1.25);
    debug.observe_pose(true, 1.3);
    debug.observe_pose(true, 1.5);
    check(debug.all_on(), "and the wait starts again after it clears");

    debug.observe_pose(true, 1.61);
    check(debug.self_collision(), "a pose that stays is a fault");
    check(!debug.any_on(), "every motor loses torque");
    bool all_agree = true;
    for (uint8_t id : kIds) { all_agree = all_agree && motors.regs[id][kTorque] == 0; }
    check(all_agree, "and the motors agree");

    motors.log.clear();
    Reply reply = run(debug, make(Op::kTorque, 0, 0, 0, 1));
    check(!reply.ok && std::strstr(reply.text, "self-collision") != nullptr,
          "torque is refused, and the reply says why");
    check(motors.log.empty(), "with nothing sent");
    check(!run(debug, make(Op::kWrite, 3, kTorque, 1, 1)).ok, "by register too");

    debug.observe_pose(false, 2.0);
    check(!debug.self_collision() && debug.resume_needed(), "moved clear, the fault ends");
    check(!debug.any_on(), "but torque does not come back on its own");

    check(run(debug, make(Op::kTorque, 0, 0, 0, 1)).ok, "turning it on works now");
    debug.observe_pose(false, 2.1);
    check(debug.all_on() && !debug.resume_needed(), "and the hand is back to normal");

    debug.observe_pose(true, 3.0);
    motors.bus_dead = true;
    debug.observe_pose(true, 3.4);
    check(debug.self_collision() && debug.any_on(), "a motor that missed the write stays on");
    motors.bus_dead = false;
    debug.observe_pose(true, 3.44);
    check(!debug.any_on(), "and is retried");
  }
  {
    // Refused even before it has lasted long enough to count as a fault.
    FakeMotors motors;
    MotorDebug debug(motors, kIds, false);
    run(debug, make(Op::kTorque, 3, 0, 0, 0));
    debug.observe_pose(true, 1.0);
    check(!run(debug, make(Op::kTorque, 3, 0, 0, 1)).ok, "no torque on the first cycle either");
  }
  {
    FakeMotors motors;
    MotorDebug debug(motors, kIds, true);
    motors.log.clear();
    debug.observe_pose(true, 1.0);
    debug.observe_pose(true, 1.5);
    check(debug.self_collision(), "a dry run reports it");
    check(motors.log.empty(), "with nothing written");
  }

  std::printf("\n%s\n\n", failures == 0 ? "all passed" : "FAILED");
  return failures == 0 ? 0 : 1;
}
