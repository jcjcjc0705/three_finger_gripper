#include "gripper_servo/servo.hpp"

#include <cmath>

#include "gripper_servo/collision.hpp"

namespace gripper_servo
{
namespace
{

float clamp(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

/// Below one encoder tick (2*pi/4096), so settling finer than this is not
/// something the motor can act on. Without it sqrt(2*a*err) keeps returning a
/// small non-zero speed and the setpoint dithers around the target.
constexpr float kSettled = 0.0005f;

}  // namespace

void Servo::reset(const Feedback & feedback)
{
  for (int i = 0; i < kJoints; ++i) {
    command_.position[i] = feedback.position[i];
    command_.current_limit[i] = config_.hold_current;
    goal_.position[i] = feedback.position[i];
    goal_.current_limit[i] = config_.hold_current;
    velocity_[i] = 0.0f;
    stall_for_[i] = 0.0f;
    stalled_[i] = false;
  }
  have_goal_ = false;
  goal_stale_ = false;
  reach_ = 1.0f;
  status_ = Status::kIdle;
}

void Servo::set_goal(const Goal & goal, double now)
{
  for (int i = 0; i < kJoints; ++i) {
    goal_.position[i] = clamp(goal.position[i],
                              kJointModel[i].lower, kJointModel[i].upper);
    goal_.current_limit[i] = goal.current_limit[i];
  }
  goal_stamp_ = now;
  have_goal_ = true;
}

Command Servo::step(const Feedback & feedback, float dt, double now)
{
  if (dt <= 0.0f) {
    return command_;
  }

  // Losing the link must not drop whatever is being held: the fingers keep both
  // their position and the ceiling the last goal asked for. Never release
  // torque, and never quietly weaken the grip either.
  const bool stale = !have_goal_ ||
                     (now - goal_stamp_) > static_cast<double>(config_.watchdog_timeout);
  goal_stale_ = stale && have_goal_;

  float target[kJoints];
  if (stale) {
    for (int i = 0; i < kJoints; ++i) {
      target[i] = command_.position[i];
    }
    status_ = have_goal_ ? Status::kHolding : Status::kIdle;
  } else {
    // Walk toward the goal only as far as the capsule model allows, starting
    // from the current setpoint. A goal that is unreachable leaves the fingers
    // as close as they can legally get instead of refusing to move at all.
    reach_ = advance_feasible(command_.position, goal_.position, target);
    status_ = (reach_ < 1.0f) ? Status::kBlocked : Status::kTracking;
  }

  bool moving = false;
  for (int i = 0; i < kJoints; ++i) {
    // Keep the grip the last goal asked for even when it goes stale. Only an
    // idle gripper, which is holding nothing, falls back to hold_current.
    const float limit = have_goal_ ? goal_.current_limit[i] : config_.hold_current;

    // A joint pushing at its ceiling without moving has met something. Freeze
    // its setpoint where it is: continuing to advance only winds up position
    // error that the motor will dump the moment the obstruction clears.
    // Entering and leaving use different fractions: one threshold chatters on
    // the noise either side of it.
    const float fraction = stalled_[i] ? config_.stall_release : config_.stall_current;
    const bool pushing = limit > 0.0f &&
                         std::fabs(feedback.current[i]) >= fraction * limit &&
                         std::fabs(feedback.velocity[i]) < config_.stall_speed;
    stall_for_[i] = pushing ? stall_for_[i] + dt : 0.0f;
    stalled_[i] = pushing && (stalled_[i] || stall_for_[i] >= config_.stall_time);

    // A stall only blocks pushing further into whatever is in the way. A goal
    // that retreats from it has to be followed, or releasing an object would
    // need the object pulled out of the hand first: the grip holds the current
    // up, which holds the latch, which freezes the setpoint.
    if (stalled_[i]) {
      const float held = command_.position[i] - feedback.position[i];
      const float wanted = target[i] - feedback.position[i];
      if (held * wanted <= 0.0f) {
        stalled_[i] = false;
        stall_for_[i] = 0.0f;
      }
    }

    if (stalled_[i]) {
      // Freeze, do not reset. The setpoint already carries the position error
      // that is generating the push, and in current_based_position that error
      // is the grip: recomputing the setpoint from feedback shrinks it and the
      // motor lets go, which drops the stall and starts the cycle again. Only
      // cap it, so nothing winds up to be dumped when the obstruction clears.
      const float error = command_.position[i] - feedback.position[i];
      if (std::fabs(error) > config_.stall_bias) {
        command_.position[i] = clamp(
          feedback.position[i] + std::copysign(config_.stall_bias, error),
          kJointModel[i].lower, kJointModel[i].upper);
      }
      velocity_[i] = 0.0f;
      command_.current_limit[i] = limit;
      continue;
    }

    // Velocity that still allows stopping on target under the acceleration
    // limit, rate limited so the profile stays smooth when the target jumps.
    // Inside kSettled the demand is simply zero: sqrt never quite reaches it
    // and the setpoint would dither.
    // sqrt(2*a*e) is the continuous-time answer and runs late once discretised:
    // the demand is evaluated at the start of a step that then travels. Taking
    // the half step into account keeps the profile on the curve instead of
    // trailing it and having to brake hard at the end.
    const float error = target[i] - command_.position[i];
    const float half = 0.5f * config_.max_acceleration * dt;
    const float stopping = std::sqrt(half * half +
                                     2.0f * config_.max_acceleration * std::fabs(error)) - half;
    const float wanted = (std::fabs(error) < kSettled)
                         ? 0.0f
                         : clamp(std::copysign(stopping, error),
                                 -config_.max_velocity, config_.max_velocity);
    const float step = config_.max_acceleration * dt;
    velocity_[i] = clamp(wanted, velocity_[i] - step, velocity_[i] + step);

    // A discrete integrator lets the profile sit slightly above the ideal
    // deceleration curve, so cap the step at the distance that is left.
    // Without this the setpoint overshoots and has to snap back.
    const float reachable = std::fabs(error) / dt;
    if (std::fabs(velocity_[i]) > reachable) {
      velocity_[i] = std::copysign(reachable, error);
    }

    command_.position[i] = clamp(command_.position[i] + velocity_[i] * dt,
                                 kJointModel[i].lower, kJointModel[i].upper);
    command_.current_limit[i] = limit;

    if (std::fabs(velocity_[i]) > 1e-4f) {
      moving = true;
    }
  }

  if (!stale && !moving && status_ == Status::kTracking) {
    status_ = Status::kHolding;
  }
  return command_;
}

}  // namespace gripper_servo
