// Behaviour checks that need no hardware. The firmware build runs the same
// file, so a port that changes behaviour fails here rather than on the bench.
//
//   ros2 run gripper_servo gripper_servo_check

#include <cmath>
#include <cstdio>

#include "gripper_servo/collision.hpp"
#include "gripper_servo/servo.hpp"

using namespace gripper_servo;

namespace
{

int failures = 0;

void check(bool ok, const char * what)
{
  std::printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok) { ++failures; }
}

Feedback follow(const Command & c)
{
  Feedback f;
  for (int i = 0; i < kJoints; ++i) {
    f.position[i] = c.position[i];
  }
  return f;
}

}  // namespace

int main()
{
  std::printf("\nkinematics\n");
  {
    float zero[kJoints] = {};
    Transform links[kLinks];
    forward_kinematics(zero, links);
    // Palm is the identity, and the index fingertip sits where the Python
    // model puts it at zero: 62 mm out, 177 mm up.
    check(links[0].m[0] == 1.0f && links[0].m[5] == 1.0f, "palm frame is identity");
    const float * tip = links[4].m;   // A_3_Link
    const float r = std::sqrt(tip[3] * tip[3] + tip[7] * tip[7]);
    check(std::fabs(r - 0.0712f) < 0.002f, "index distal origin 71 mm from the palm axis");
    check(std::fabs(tip[11] - 0.127f) < 0.002f, "index distal origin 127 mm up");
  }

  std::printf("\nself-collision\n");
  {
    float open_pose[kJoints] = {};
    check(feasible(open_pose), "fully open pose is feasible");

    // Curling every flexion joint to its limit drives the index and thumb
    // through each other on the way; the model has to notice.
    float closed[kJoints] = {};
    for (int i = 0; i < 6; ++i) { closed[i] = kJointModel[i].lower * 0.55f; }
    float reached[kJoints] = {};
    const float frac = advance_feasible(open_pose, closed, reached);
    check(frac < 1.0f, "proportional 55% closure is blocked");
    check(frac > 0.5f, "but it still gets most of the way there");
    check(feasible(reached), "the point it stops at is itself feasible");

    // Full closure from a tripod spread is feasible at the goal but collides
    // on the way, so checking only the endpoint would let the setpoint through.
    float tripod[kJoints] = {};
    tripod[6] = 0.9076f;
    tripod[7] = -0.9076f;
    float full[kJoints] = {};
    full[6] = tripod[6];
    full[7] = tripod[7];
    for (int i = 0; i < 6; ++i) { full[i] = kJointModel[i].lower; }
    check(feasible(full), "full closure at tripod spread is itself feasible");
    const float through = advance_feasible(tripod, full, reached);
    check(through < 1.0f, "but the path to it is blocked, so it stops short");
  }

  std::printf("\ntrajectory\n");
  {
    Servo servo;
    Config cfg;
    cfg.max_velocity = 1.0f;
    cfg.max_acceleration = 4.0f;
    servo.configure(cfg);

    Feedback f;
    servo.reset(f);

    Goal g;
    g.position[0] = 0.4f;
    for (int i = 0; i < kJoints; ++i) { g.current_limit[i] = 0.35f; }

    const float dt = 0.002f;
    double t = 0.0;
    float previous = 0.0f;
    float peak_speed = 0.0f, peak_accel = 0.0f;
    Command c;
    for (int step = 0; step < 2000; ++step) {
      servo.set_goal(g, t);
      c = servo.step(f, dt, t);
      const float speed = (c.position[0] - previous) / dt;
      peak_accel = std::fmax(peak_accel, std::fabs(speed - peak_speed) / dt);
      peak_speed = speed;
      previous = c.position[0];
      f = follow(c);
      t += dt;
    }
    check(std::fabs(c.position[0] - 0.4f) < 1e-3f, "reaches the goal");
    check(peak_speed <= cfg.max_velocity * 1.02f, "respects the velocity limit");
    check(peak_accel <= cfg.max_acceleration * 1.02f, "respects the acceleration limit");
    check(servo.status() == Status::kHolding, "settles into holding");
  }

  std::printf("\nwatchdog\n");
  {
    Servo servo;
    Config cfg;
    cfg.watchdog_timeout = 0.1f;
    cfg.hold_current = 0.12f;
    servo.configure(cfg);

    Feedback f;
    servo.reset(f);
    Goal g;
    g.position[0] = 0.5f;
    for (int i = 0; i < kJoints; ++i) { g.current_limit[i] = 0.4f; }

    double t = 0.0;
    servo.set_goal(g, t);
    Command c;
    for (int step = 0; step < 25; ++step) {
      c = servo.step(f, 0.002f, t);
      f = follow(c);
      t += 0.002;
    }
    check(c.position[0] > 0.0f, "moves while the goal is fresh");

    // No new goal from here. It decelerates under the acceleration limit first,
    // so let it settle before deciding whether it is holding still.
    for (int step = 0; step < 300; ++step) {
      c = servo.step(f, 0.002f, t);
      f = follow(c);
      t += 0.002;
    }
    check(servo.status() == Status::kHolding, "goes to holding when the goal goes stale");
    check(servo.goal_stale(), "and says so, since holding alone does not tell you");
    const float settled = c.position[0];
    check(settled < 0.5f, "stops short of the goal it can no longer see");

    for (int step = 0; step < 200; ++step) {
      c = servo.step(f, 0.002f, t);
      f = follow(c);
      t += 0.002;
    }
    check(std::fabs(c.position[0] - settled) < 1e-4f, "does not drift once held");
    check(c.current_limit[0] == 0.4f, "keeps the grip the goal asked for");
    check(c.current_limit[0] > cfg.hold_current, "does not weaken it to hold_current");
  }

  std::printf("\nstall\n");
  {
    Servo servo;
    Config cfg;
    cfg.stall_time = 0.02f;
    servo.configure(cfg);

    Feedback f;
    servo.reset(f);
    Goal g;
    g.position[0] = 1.0f;
    for (int i = 0; i < kJoints; ++i) { g.current_limit[i] = 0.4f; }

    // Track freely for a while so the setpoint leads the joint the way it does
    // in flight, then jam it. Teleporting the feedback ahead of the setpoint
    // would test a situation the mechanism never produces.
    double t = 0.0;
    Command c;
    for (int step = 0; step < 100; ++step) {
      servo.set_goal(g, t);
      c = servo.step(f, 0.002f, t);
      f.position[0] = c.position[0] - 0.01f;      // the joint trails its setpoint
      f.velocity[0] = 0.5f;
      f.current[0] = 0.1f;
      t += 0.002;
    }
    const float contact = f.position[0];
    for (int step = 0; step < 200; ++step) {
      servo.set_goal(g, t);
      c = servo.step(f, 0.002f, t);
      // Jammed against the object: at the ceiling, not moving.
      f.position[0] = contact;
      f.velocity[0] = 0.0f;
      f.current[0] = 0.4f;
      t += 0.002;
    }
    check(servo.stalled(0), "detects a joint pushing without moving");
    // Resetting the setpoint to the measured position zeroes the position error,
    // and in current_based_position that error is the grip: the motor lets go,
    // the stall drops, and the finger buzzes against the object at a few hertz.
    // Freezing keeps the push; the cap keeps it from winding up.
    const float held = c.position[0] - contact;
    check(held > 0.0f, "keeps the error that generates the push");
    check(held <= cfg.stall_bias + 1e-3f, "and caps it at stall_bias");

    // Hysteresis: the current sagging below the latch fraction must not let go
    // while it is still above the release fraction.
    f.current[0] = 0.5f * 0.4f + 0.01f;
    c = servo.step(f, 0.002f, t);
    t += 0.002;
    check(servo.stalled(0), "stays latched while the current is above stall_release");

    f.current[0] = 0.1f * 0.4f;
    c = servo.step(f, 0.002f, t);
    check(!servo.stalled(0), "lets go once the current falls below stall_release");

    // Releasing an object must not need the object taken out of the hand: the
    // grip keeps the current at the ceiling, which keeps the latch, so a
    // retreating goal has to break it.
    for (int step = 0; step < 200; ++step) {
      servo.set_goal(g, t);
      c = servo.step(f, 0.002f, t);
      f.position[0] = contact;
      f.velocity[0] = 0.0f;
      f.current[0] = 0.4f;
      t += 0.002;
    }
    check(servo.stalled(0), "latches again when the push resumes");

    Goal back;
    for (int i = 0; i < kJoints; ++i) { back.current_limit[i] = 0.4f; }
    back.position[0] = -1.0f;
    const float frozen = c.position[0];
    for (int step = 0; step < 20; ++step) {
      servo.set_goal(back, t);
      c = servo.step(f, 0.002f, t);
      t += 0.002;
    }
    check(c.position[0] < frozen, "still follows a goal that retreats from the stall");
  }

  std::printf("\n%s\n\n", failures ? "FAILURES" : "all checks passed");
  return failures ? 1 : 0;
}
