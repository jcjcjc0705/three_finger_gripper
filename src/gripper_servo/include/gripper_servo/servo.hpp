// Servo core: goal -> per-cycle joint commands.
//
// Everything that has to behave identically on the PC and on the NT98532 lives
// here: trajectory generation, joint and self-collision limits, current
// ceilings and the watchdog. No ROS, no allocation, no exceptions, no clock of
// its own -- the caller supplies time.
//
// It knows nothing about grasping. A grasp strategy is whatever produces the
// goals, and that stays on the PC.
#pragma once

#include "gripper_servo/gripper_model.hpp"

namespace gripper_servo
{

struct Config
{
  float max_velocity{2.0f};        ///< rad/s
  float max_acceleration{8.0f};    ///< rad/s^2
  float watchdog_timeout{0.5f};    ///< s without a goal before holding

  /// Ceiling used only before any goal has arrived. Once one has, losing the
  /// link keeps that goal's ceiling: lowering it would relax the grip, and a
  /// gripper that quietly weakens its hold when the link drops is a gripper
  /// that drops the payload. The motor's own overheating shutdown is the
  /// backstop against holding hard forever.
  ///
  /// It has to be enough to actually move an unloaded finger. Whatever runs
  /// next inherits this ceiling from the motor register, and one too low to
  /// move anything looks exactly like a dead command path.
  float hold_current{250.0f};
  float stall_current{0.9f};       ///< fraction of the limit that counts as stalled
  float stall_release{0.5f};       ///< fraction below which a stalled joint lets go
  float stall_speed{0.10f};        ///< rad/s below which a stalled joint is stuck
  float stall_time{0.15f};         ///< s of stall before the joint stops advancing

  /// Setpoint kept beyond the contact point while stalled. In
  /// current_based_position the push comes from position error, so parking the
  /// setpoint on the measured position removes the grip, which drops the stall,
  /// which lets the setpoint advance again -- a limit cycle that shows up as the
  /// fingers buzzing against the object. Offsetting from the measured position
  /// keeps the error bounded, so nothing winds up to be dumped on release.
  float stall_bias{0.20f};         ///< rad, cap on the held error
};

struct Goal
{
  float position[kJoints]{};
  float current_limit[kJoints]{};
};

struct Feedback
{
  float position[kJoints]{};
  float velocity[kJoints]{};
  float current[kJoints]{};
};

struct Command
{
  float position[kJoints]{};
  float current_limit[kJoints]{};
};

enum class Status
{
  kIdle,        ///< no goal yet
  kTracking,
  kHolding,     ///< watchdog fired, or the goal is reached
  kBlocked,     ///< the goal is not reachable without self-collision
};

class Servo
{
public:
  void configure(const Config & config) { config_ = config; }

  /// Seed the internal setpoint from where the hardware actually is. Call once
  /// after activation, before the first step.
  void reset(const Feedback & feedback);

  void set_goal(const Goal & goal, double now);

  /// One control cycle. `dt` is the elapsed time in seconds, `now` a monotonic
  /// timestamp used only to age the goal.
  Command step(const Feedback & feedback, float dt, double now);

  Status status() const { return status_; }

  /// True where the joint is pushing at its current ceiling and not moving --
  /// contact, whether with an object, another finger or a hard stop. Deciding
  /// what that means is the caller's business.
  bool stalled(int joint) const { return stalled_[joint]; }

  /// Fraction of the way to the last goal the collision model allowed.
  float reach() const { return reach_; }

  /// True while holding because the goal aged out rather than because it was
  /// reached. Both look like kHolding, and a caller whose link dropped has no
  /// other way to learn that it happened.
  bool goal_stale() const { return goal_stale_; }

private:
  Config config_{};
  Goal goal_{};
  Command command_{};
  float velocity_[kJoints]{};
  float stall_for_[kJoints]{};
  bool stalled_[kJoints]{};
  double goal_stamp_{0.0};
  float reach_{1.0f};
  bool have_goal_{false};
  bool goal_stale_{false};
  Status status_{Status::kIdle};
};

}  // namespace gripper_servo
