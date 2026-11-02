// Behaviour checks for the grasp strategy that need no hardware.
//
//   ./build.sh check

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "grasp_sequencer.hpp"
#include "grasp_store.hpp"
#include "servo/collision.hpp"
#include "servo/servo.hpp"
#include "reach_table.hpp"

using namespace grasping;

namespace
{

int failures = 0;

void check(bool ok, const char * what)
{
  std::printf("  %-58s %s\n", what, ok ? "ok" : "FAILED");
  if (!ok) { ++failures; }
}

constexpr float kDt = 0.04f;

/// Feeds a steady scene for `seconds`, one TOF reading per tick.
struct Sim
{
  GraspStore store;
  GraspSequencer seq;
  double now{0.0};
  Tick scene{};

  explicit Sim(const SequencerConfig & config = SequencerConfig{}) : seq(config, store) {}

  void run(float seconds)
  {
    const int ticks = static_cast<int>(seconds / kDt + 0.5f);
    for (int i = 0; i < ticks; ++i) {
      now += kDt;
      Tick t = scene;
      t.now = now;
      t.dt = kDt;
      seq.update(t);
    }
  }

  void object_at(float metres)
  {
    scene.tof_fresh = true;
    scene.tof_valid = true;
    scene.tof_m = metres;
  }

  void nothing_seen()
  {
    scene.tof_fresh = true;
    scene.tof_valid = false;
    scene.tof_m = 0.0f;
  }
};

}  // namespace

int main()
{
  std::printf("\ngrasp table\n");
  check(std::fabs(closure_limit(kGrasps[0]) - 0.28f) < 1e-4f, "tripod closes to 28%");
  check(std::fabs(closure_limit(kGrasps[1]) - 0.30f) < 1e-4f, "pinch closes to 30%");
  check(std::fabs(closure_limit(kGrasps[2]) - 0.37f) < 1e-4f, "wide closes to 37%");
  for (const Grasp & grasp : kGrasps) {
    float open[gripper_servo::kJoints], closed[gripper_servo::kJoints], out[gripper_servo::kJoints];
    grasp_pose(grasp, 0.0f, open);
    grasp_pose(grasp, 1.0f, closed);
    char what[64];
    std::snprintf(what, sizeof(what), "%s closes without self-collision", grasp.name);
    check(gripper_servo::advance_feasible(open, closed, out) >= 1.0f, what);
  }

  std::printf("\nevery grasp\n");
  for (const Grasp & grasp : kGrasps) {
    float open[gripper_servo::kJoints], closed[gripper_servo::kJoints];
    grasp_pose(grasp, 0.0f, open);
    grasp_pose(grasp, 1.0f, closed);
    bool inside = true;
    for (int i = 0; i < gripper_servo::kJoints; ++i) {
      const auto & model = gripper_servo::kJointModel[i];
      inside = inside && open[i] >= model.lower && open[i] <= model.upper &&
               closed[i] >= model.lower && closed[i] <= model.upper;
    }
    char what[64];
    std::snprintf(what, sizeof(what), "%s stays inside the joint limits", grasp.name);
    check(inside, what);
  }
  for (const Grasp & from : kGrasps) {
    for (const Grasp & to : kGrasps) {
      // The real Servo rather than a straight line: its joints do not arrive
      // together, so the path it takes is not the one advance_feasible is asked about.
      float start[gripper_servo::kJoints], target[gripper_servo::kJoints];
      grasp_pose(from, 0.0f, start);
      grasp_pose(to, 0.0f, target);
      gripper_servo::Servo servo;
      servo.configure(gripper_servo::Config{});
      gripper_servo::Feedback fb{};
      for (int i = 0; i < gripper_servo::kJoints; ++i) { fb.position[i] = start[i]; }
      servo.reset(fb);
      gripper_servo::Goal goal{};
      for (int i = 0; i < gripper_servo::kJoints; ++i) {
        goal.position[i] = target[i];
        goal.current_limit[i] = 350.0f;
      }
      double now = 0.0;
      gripper_servo::Command command{};
      for (int k = 0; k < 150; ++k) {
        now += kDt;
        servo.set_goal(goal, now);
        command = servo.step(fb, kDt, now);
        for (int i = 0; i < gripper_servo::kJoints; ++i) { fb.position[i] = command.position[i]; }
      }
      bool arrived = true;
      for (int i = 0; i < gripper_servo::kJoints; ++i) {
        arrived = arrived && std::fabs(command.position[i] - target[i]) < 0.01f;
      }
      char what[64];
      std::snprintf(what, sizeof(what), "%s preshape reaches %s preshape", from.name, to.name);
      check(arrived, what);
    }
  }
  {
    float pose[gripper_servo::kJoints];
    const Grasp & envelop = kGrasps[find_grasp("envelop")];
    grasp_pose(envelop, 0.0f, pose);
    check(pose[0] > 0.5f && pose[2] > 0.5f, "envelop opens with the proximal joints leaning out");
    grasp_pose(envelop, 1.0f, pose);
    check(pose[1] < -0.5f && pose[0] < 0.0f, "and curls in to close");
    const Grasp & two = kGrasps[find_grasp("two_finger")];
    float open[gripper_servo::kJoints], closed[gripper_servo::kJoints];
    grasp_pose(two, 0.0f, open);
    grasp_pose(two, 1.0f, closed);
    check(open[0] == closed[0] && open[1] == closed[1], "two_finger leaves the thumb alone");
    check(open[0] > 0.4f, "leaning out of the way");
    check(closed[2] == closed[4] && closed[3] == closed[5] && closed[3] < -0.3f,
          "and closes index and middle together");
    float tips[gripper_servo::kJoints];
    grasp_pose(two, 1.0f, tips);
    gripper_servo::Transform links[gripper_servo::kLinks];
    gripper_servo::forward_kinematics(tips, links);
    const auto tip = [&](int capsule, float * out) {
      const auto & model = gripper_servo::kCapsules[capsule];
      const auto & t = links[model.link];
      for (int r = 0; r < 3; ++r) {
        out[r] = t.m[r * 4] * model.a[0] + t.m[r * 4 + 1] * model.a[1] +
                 t.m[r * 4 + 2] * model.a[2] + t.m[r * 4 + 3];
      }
    };
    float index_tip[3], middle_tip[3];
    tip(3, index_tip);
    tip(6, middle_tip);
    const float gap = std::hypot(index_tip[0] - middle_tip[0], index_tip[1] - middle_tip[1],
                                 index_tip[2] - middle_tip[2]);
    check(gap < 0.016f, "closed, their tips meet");
    check(find_grasp("nope") == -1, "an unknown grasp name is not found");
  }

  std::printf("\nobjects\n");
  for (const ObjectGrasp & row : kObjects) {
    char what[64];
    std::snprintf(what, sizeof(what), "%s maps to a grasp that exists", row.object);
    check(find_grasp(row.grasp) >= 0, what);
  }
  {
    Sim s;
    s.seq.set_object("cup");
    s.run(0.04f);
    check(s.seq.phase() == Phase::kPreshape && s.seq.grasp().name == kGrasps[1].name,
          "cup goes to the pinch preshape");
    check(std::strcmp(s.seq.object(), "cup") == 0, "and is remembered");
    s.seq.set_object("bear");
    s.run(0.04f);
    check(s.seq.grasp().name == kGrasps[find_grasp("envelop")].name, "bear switches to envelop");

    s.seq.set_object("nope");
    s.run(0.04f);
    check(s.seq.grasp().name == kGrasps[find_grasp("envelop")].name &&
          std::strcmp(s.seq.object(), "bear") == 0, "an unknown object changes nothing");
    check(std::strstr(s.seq.note(), "unknown object nope") != nullptr, "and says so");

    s.seq.set_object("");
    s.run(0.04f);
    check(std::strcmp(s.seq.object(), "bear") == 0, "neither does an empty name");
  }
  {
    // A recogniser repeats itself every frame. That must not disturb the hand.
    Sim s;
    s.seq.set_object("foam");
    s.run(0.04f);
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.05f);
    for (int i = 0; i < 40; ++i) {
      s.seq.set_object("foam");
      s.run(0.04f);
    }
    check(s.seq.phase() == Phase::kClosing, "repeating the name does not disturb it");

    for (int i = 0; i < 10; ++i) {
      s.seq.set_object("cup");
      s.run(0.04f);
    }
    check(s.seq.phase() == Phase::kClosing && s.seq.grasp().name == kGrasps[0].name,
          "a new object does not disturb a closing hand");
    s.seq.command(Command::kRelease);
    s.run(0.04f);
    check(s.seq.grasp().name == kGrasps[1].name && s.seq.closure() == 0.0f,
          "it applies after the release");
  }
  {
    Sim s;
    s.seq.set_object("cup");
    s.run(0.04f);
    s.seq.command(Command::kSelect, find_grasp("wide"));
    s.seq.set_object("cup");
    s.run(0.2f);
    check(s.seq.grasp().name == kGrasps[find_grasp("wide")].name,
          "a grasp chosen by hand survives the same name again");
    s.seq.set_object("foam");
    s.run(0.04f);
    check(s.seq.grasp().name == kGrasps[0].name, "until the object changes");
  }
  {
    // Three contacts out of two_finger's four joints means both fingers have got there.
    // Two could be one finger, with the other still on its way.
    // to be in contact the way a three finger grasp can.
    Sim s;
    s.seq.command(Command::kSelect, find_grasp("two_finger"));
    s.seq.command(Command::kGrasp);
    s.scene.settled = true;
    s.run(0.04f);
    s.scene.settled = false;
    s.scene.contacts = 2;
    s.run(0.5f);
    check(s.seq.phase() == Phase::kClosing, "two contacts could be one finger: two_finger keeps closing");

    Sim t;
    t.seq.command(Command::kGrasp);
    t.scene.settled = true;
    t.run(0.04f);
    t.scene.settled = false;
    t.scene.contacts = 2;
    t.run(0.5f);
    check(t.seq.phase() == Phase::kClosing, "nor does tripod");
  }

  std::printf("\ntrapped poses\n");
  {
    constexpr float kDeg = 3.14159265358979323846f / 180.0f;
    const auto pose_from_deg = [&](const float * deg, float * out) {
      for (int i = 0; i < gripper_servo::kJoints; ++i) { out[i] = deg[i] * kDeg; }
    };
    for (const Grasp & grasp : kGrasps) {
      for (float closure : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) {
        float pose[gripper_servo::kJoints];
        grasp_pose(grasp, closure, pose);
        char what[64];
        std::snprintf(what, sizeof(what), "%s at %.0f%% closure is fine", grasp.name, closure * 100.0f);
        check(!pose_trapped(pose), what);
      }
    }
    // Measured while gripping. Cutting the torque here would drop the object.
    const float gripping[][gripper_servo::kJoints] = {
      {9.2f, -30.0f, 3.7f, -30.7f, 1.8f, -31.6f, 51.4f, -51.6f},    // bear, envelop
      {1.0f, -27.2f, -2.9f, -30.6f, -2.5f, -30.8f, 51.5f, -51.6f},  // foam, tripod
      {3.0f, -14.9f, -2.9f, -29.8f, -1.8f, -26.8f, 89.5f, -89.6f},  // cup, pinch
    };
    for (const auto & deg : gripping) {
      float pose[gripper_servo::kJoints];
      pose_from_deg(deg, pose);
      check(!pose_trapped(pose), "a pose measured on a real object is fine");
    }
    float limit[gripper_servo::kJoints] = {-0.122f, -0.486f, -0.233f, -0.486f, -0.233f, -0.486f,
                                           0.908f, -0.908f};
    check(!pose_trapped(limit), "where Servo stops a jog at the model limit is fine");
    for (int i = 0; i < 6; ++i) { limit[i] -= 0.0044f; }
    check(!gripper_servo::feasible(limit) && !pose_trapped(limit),
          "and so is reading three ticks past it");

    const auto squeezed = [](const Grasp & grasp, float fraction, float * out) {
      for (int i = 0; i < 6; ++i) { out[i] = fraction * reach_table::kFlexionLower[i]; }
      out[6] = grasp.index_spread_deg * kDeg;
      out[7] = grasp.middle_spread_deg * kDeg;
    };
    float pose[gripper_servo::kJoints];
    squeezed(kGrasps[0], 0.5f, pose);
    check(!gripper_servo::feasible(pose) && pose_trapped(pose), "fingers squeezed together are trapped");
    squeezed(kGrasps[0], 1.0f, pose);
    check(gripper_servo::feasible(pose) && pose_trapped(pose),
          "and so are fingers past the band, which the model calls clear");
  }

  std::printf("\nstartup\n");
  {
    Sim s;
    s.object_at(0.05f);
    s.scene.settled = true;
    s.run(3.0f);
    check(!s.seq.has_goal(), "no goal until told, even with an object present");
  }

  std::printf("\nfollowing the object in\n");
  // Moves the object in a straight line, one TOF reading per tick.
  const auto approach = [](Sim & s, float from, float to, float speed) {
    const int ticks = static_cast<int>(std::fabs(from - to) / speed / kDt + 0.5f);
    for (int i = 1; i <= ticks; ++i) {
      s.object_at(from + (to - from) * static_cast<float>(i) / static_cast<float>(ticks));
      s.run(kDt);
    }
  };
  {
    Sim s;
    s.seq.command(Command::kSelect, 1);
    s.seq.command(Command::kArm);
    s.object_at(0.07f);
    s.run(0.5f);
    check(s.seq.phase() == Phase::kArmed, "does not follow while the preshape is still moving");
    s.scene.settled = true;
    s.object_at(0.3f);
    s.run(1.0f);
    check(s.seq.phase() == Phase::kArmed && s.seq.closure() == 0.0f, "far away, nothing moves");
    s.nothing_seen();
    s.run(1.0f);
    check(s.seq.phase() == Phase::kArmed, "nor with nothing there");
  }
  {
    // Nothing blocks the TOF here, so the whole band is followed.
    Sim s;
    s.store.set_blind("tripod", 1.0f);
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.2f);
    s.run(0.2f);
    approach(s, 0.2f, 0.09f, 0.05f);
    check(s.seq.phase() == Phase::kArmed && s.seq.closure() == 0.0f, "waits for the band");
    approach(s, 0.09f, 0.07f, 0.005f);
    check(s.seq.phase() == Phase::kFollowing, "in the band it follows");
    check(std::fabs(s.seq.closure() - 0.5f) < 0.06f, "half way in, half closed");
    const float there = s.seq.closure();
    s.run(2.0f);
    check(std::fabs(s.seq.closure() - 0.54f) < 0.03f && s.seq.phase() == Phase::kFollowing,
          "the object stops, the hand stops");
    approach(s, 0.07f, 0.08f, 0.005f);
    check(s.seq.closure() < there, "the object backs off, the hand opens");
    approach(s, 0.08f, 0.12f, 0.005f);
    check(s.seq.phase() == Phase::kArmed && s.seq.closure() == 0.0f,
          "it leaves, the hand is back at the preshape, waiting");
    approach(s, 0.12f, 0.07f, 0.01f);
    check(s.seq.phase() == Phase::kFollowing, "it comes back, the hand follows again");
    s.nothing_seen();
    s.run(3.0f);
    check(s.seq.closure() == 0.0f && s.seq.phase() == Phase::kArmed, "it vanishes, the hand opens");

    approach(s, 0.12f, 0.05f, 0.01f);
    check(s.seq.phase() == Phase::kHolding && s.seq.closure() == 1.0f,
          "at the touch distance, with no touch point yet, it closes all the way");
    s.scene.contacts = 3;
    s.run(0.5f);
    check(s.seq.phase() == Phase::kHolding, "and contact holds");
  }
  {
    // A touch closure means the fingers arrive there, not at the end of their travel.
    Sim s;
    s.store.set_blind("tripod", 1.0f);
    s.store.set_touch_closure("foam", 0.6f);
    s.seq.set_object("foam");
    s.run(0.04f);
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.2f);
    s.run(0.2f);
    approach(s, 0.2f, 0.07f, 0.01f);
    check(std::fabs(s.seq.closure() - 0.3f) < 0.05f, "half way in, half way to the touch closure");
    approach(s, 0.07f, 0.0515f, 0.005f);
    check(s.seq.phase() == Phase::kClosing, "at the touch distance it finishes closing");
    s.scene.contacts = 3;
    s.run(0.5f);
    check(s.seq.phase() == Phase::kHolding && s.seq.closure() < 0.75f, "and it touches about there");
  }
  {
    Sim s;
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.0f);
    s.run(3.0f);
    check(s.seq.phase() == Phase::kArmed && s.seq.closure() == 0.0f,
          "a valid 0 mm that never came through the band closes nothing");
    s.object_at(0.2f);
    s.run(0.2f);
    s.object_at(0.06f);
    s.run(kDt);
    s.object_at(0.2f);
    s.run(0.2f);
    check(s.seq.phase() == Phase::kArmed, "nor does a single reading in the band");
  }
  {
    // Fast enough that only the filter could keep a stray reading from moving it.
    SequencerConfig config;
    config.fastest_close_s = 0.1f;
    Sim s(config);
    s.store.set_blind("tripod", 1.0f);
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.07f);
    s.run(1.0f);
    const float there = s.seq.closure();
    s.object_at(0.0f);
    s.run(kDt * 2.0f);
    check(std::fabs(s.seq.closure() - there) < 0.02f, "two stray readings while following move nothing");
  }
  {
    // The rest of the way the TOF is reading the fingers.
    Sim s;
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.2f);
    s.run(0.2f);
    approach(s, 0.2f, 0.09f, 0.05f);
    approach(s, 0.09f, 0.08f, 0.005f);
    check(s.seq.phase() == Phase::kFollowing, "before the fingers block the TOF it follows");
    approach(s, 0.08f, 0.07f, 0.005f);
    check(s.seq.phase() == Phase::kClosing, "once they do it carries on by itself");
    check(std::strstr(s.seq.note(), "cannot see") != nullptr, "and says why");
    const float from = s.seq.closure();
    s.object_at(0.01f);
    s.run(1.0f);
    const float rate = s.seq.closure() - from;
    check(std::fabs(rate - 0.125f) < 0.03f, "at the rate the object was coming: 5 mm/s of 40");
    s.nothing_seen();
    s.run(1.0f);
    check(s.seq.closure() > from + 2.0f * 0.09f, "whatever the TOF reads from here");
    s.scene.contacts = 3;
    s.run(0.5f);
    check(s.seq.phase() == Phase::kHolding, "and the contacts stop it");
  }
  {
    Sim s;
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.2f);
    s.run(0.2f);
    approach(s, 0.2f, 0.09f, 0.05f);
    approach(s, 0.09f, 0.0775f, 0.001f);
    check(s.seq.phase() == Phase::kClosing, "an object creeping in still gets there");
    const float from = s.seq.closure();
    s.object_at(0.0f);
    s.run(1.0f);
    check(std::fabs(s.seq.closure() - from - 1.0f / 12.0f) < 0.03f,
          "an object creeping in does not leave the hand creeping forever");
  }
  {
    Sim s;
    s.store.set_blind("tripod", 1.0f);
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.2f);
    s.run(0.2f);
    s.object_at(0.05f);
    s.run(1.0f);
    check(s.seq.closure() < 0.36f, "however fast it comes, the fingers keep to the fastest close");
  }
  {
    Sim s;
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.05f);
    s.run(5.0f);
    check(s.seq.closure() > 0.9f, "an object that was already there still gets closed on");
  }
  {
    Sim s;
    s.store.set_blind("tripod", 1.0f);
    s.seq.command(Command::kArm);
    s.scene.settled = true;
    s.object_at(0.07f);
    s.run(1.0f);
    s.seq.command(Command::kSelect, 1);
    check(s.seq.grasp().name == kGrasps[0].name, "the grasp cannot change while following");
    s.seq.command(Command::kGrasp);
    check(s.seq.phase() == Phase::kClosing && s.seq.closure() > 0.1f, "g closes from where it is");
    s.seq.command(Command::kRelease);
    s.seq.command(Command::kArm);
    s.run(1.0f);
    s.seq.command(Command::kDisarm);
    check(s.seq.phase() == Phase::kPreshape && s.seq.closure() == 0.0f, "disarming opens it");
  }
  {
    SequencerConfig config;
    config.touch_m = 0.05f;
    Sim s(config);
    s.seq.command(Command::kCalibrate);
    check(s.seq.touch_m() == 0.05f, "calibrating with no range changes nothing");
    s.object_at(0.2f);
    s.run(0.08f);
    s.seq.command(Command::kCalibrate);
    check(std::fabs(s.seq.touch_m() - 0.2f) < 1e-4f, "calibrate takes the current range");
    check(std::fabs(s.seq.start_m() - 0.24f) < 1e-4f, "and the band moves with it");
  }

  std::printf("\nlearning where it touches\n");
  {
    Sim s;
    s.scene.settled = true;
    s.object_at(0.06f);
    s.seq.set_object("cup");
    s.run(0.2f);
    s.seq.command(Command::kCalibrate);
    s.seq.command(Command::kGrasp);
    s.run(2.0f);
    const float at_contact = s.seq.closure();
    s.scene.contacts = 3;
    s.run(0.6f);
    check(s.seq.phase() == Phase::kHolding, "c, then g, closes on it");
    const float learned = s.store.find_object("cup")->touch_closure;
    check(std::fabs(learned - at_contact) < 0.03f, "saving the closure at first contact");
    check(std::fabs(s.seq.touch_closure() - learned) < 1e-4f, "and using it");
    check(std::strstr(s.seq.note(), "touches at") != nullptr, "and says so");

    s.seq.command(Command::kRelease);
    s.scene.contacts = 0;
    s.seq.command(Command::kGrasp);
    s.run(1.0f);
    s.scene.contacts = 3;
    s.run(0.6f);
    check(std::fabs(s.store.find_object("cup")->touch_closure - learned) < 1e-4f,
          "g on its own does not overwrite it");
  }
  {
    Sim s;
    s.scene.settled = true;
    s.object_at(0.06f);
    s.seq.set_object("cup");
    s.run(0.2f);
    s.seq.command(Command::kCalibrate);
    s.seq.command(Command::kGrasp);
    s.run(5.0f);
    check(s.seq.phase() == Phase::kHolding && s.store.find_object("cup")->touch_closure == 0.0f,
          "closing on nothing teaches nothing");
    check(s.seq.touch_closure() == 1.0f, "so it keeps the default");
  }
  {
    Sim s;
    s.scene.settled = true;
    s.object_at(0.06f);
    s.seq.set_object("cup");
    s.run(0.2f);
    s.seq.command(Command::kCalibrate);
    s.seq.command(Command::kGrasp);
    s.run(1.0f);
    s.seq.command(Command::kRelease);
    s.run(0.2f);
    s.seq.command(Command::kGrasp);
    s.run(1.0f);
    s.scene.contacts = 3;
    s.run(0.6f);
    check(s.store.find_object("cup")->touch_closure > 0.0f, "released half way, the next g still teaches");
  }
  {
    GraspStore store;
    store.set_touch_closure("cup", 0.8f);
    store.put_object("cup", "pinch");
    check(store.find_object("cup")->touch_closure == 0.8f, "the same grasp keeps the touch closure");
    store.put_object("cup", "wide");
    check(store.find_object("cup")->touch_closure == 0.0f, "another grasp forgets it");
    Sim s;
    s.store.set_touch_closure("cup", 0.8f);
    s.seq.set_object("cup");
    s.run(0.04f);
    check(s.seq.touch_closure() == 0.8f, "selecting the object uses it");
    s.seq.command(Command::kSelect, find_grasp("wide"));
    check(s.seq.touch_closure() == 1.0f, "a grasp chosen by hand does not");
  }

  std::printf("\nmeasuring where the TOF is blocked\n");
  {
    Sim s;
    s.nothing_seen();
    s.seq.command(Command::kMeasure);
    s.run(1.0f);
    check(s.seq.phase() == Phase::kMeasuring && s.seq.closure() == 0.0f,
          "waits for the fingers to settle");
    s.scene.settled = true;
    s.run(4.3f);
    check(s.seq.closure() > 0.4f && s.seq.closure() < 0.56f, "closes slowly with nothing seen");
    s.object_at(0.06f);
    s.run(0.2f);
    check(s.seq.phase() == Phase::kPreshape && s.seq.closure() == 0.0f, "a finger ends it, opening again");
    const float blind = s.store.blind_measured(0);
    check(blind > 0.4f && blind < 0.56f, "saving where that was, less a margin");
    check(s.seq.blind_closure() == blind, "and following uses it");
  }
  {
    Sim s;
    s.scene.settled = true;
    s.nothing_seen();
    s.seq.command(Command::kMeasure);
    s.run(9.0f);
    check(s.store.blind_measured(0) == 1.0f && s.seq.phase() == Phase::kPreshape,
          "a grasp that never blocks it is recorded as such");
  }
  {
    Sim s;
    s.scene.settled = true;
    s.object_at(0.1f);
    s.run(0.2f);
    s.seq.command(Command::kMeasure);
    s.run(1.0f);
    check(s.store.blind_measured(0) < 0.0f && s.seq.phase() == Phase::kPreshape &&
          std::strstr(s.seq.note(), "in front of") != nullptr,
          "something already in front of it refuses");
  }
  {
    Sim s;
    s.scene.settled = true;
    s.nothing_seen();
    s.seq.command(Command::kMeasure);
    s.run(2.0f);
    s.scene.contacts = 1;
    s.run(0.2f);
    check(s.store.blind_measured(0) < 0.0f && s.seq.closure() == 0.0f, "a finger touching something refuses");
  }
  {
    Sim s;
    s.scene.settled = true;
    s.nothing_seen();
    s.seq.command(Command::kMeasure);
    s.run(1.0f);
    s.seq.command(Command::kRelease);
    check(s.seq.phase() == Phase::kPreshape && s.seq.closure() == 0.0f, "release abandons it");
    check(s.store.blind_measured(0) < 0.0f, "saving nothing");
  }
  {
    GraspStore store;
    AddedGrasp taught;
    std::snprintf(taught.name, sizeof(taught.name), "taught");
    taught.index_spread_deg = 90.0f;
    taught.shape = kTwoFinger;
    store.put_grasp(taught);
    const int slot = store.find_grasp("taught");
    check(store.blind_measured(slot) < 0.0f && store.blind_closure(slot) > 0.0f,
          "unmeasured uses a cautious default");
    check(store.set_blind("taught", 0.5f) && store.blind_closure(slot) == 0.5f, "measured uses that");
    store.put_grasp(taught);
    check(store.blind_measured(slot) < 0.0f, "reshaping a grasp forgets it");
    check(!store.set_blind("nope", 0.5f) && !store.set_blind("taught", 1.5f), "nonsense is refused");
  }

  std::printf("\nclosing\n");
  {
    Sim s;
    s.seq.command(Command::kSelect, 1);
    s.seq.command(Command::kGrasp);
    check(s.seq.phase() == Phase::kArmed, "grasp waits for the preshape");
    s.run(1.0f);
    check(s.seq.closure() == 0.0f, "no closure while the spread is moving");
    s.scene.settled = true;
    s.run(0.04f);
    check(s.seq.phase() == Phase::kClosing, "grasp closes with no TOF at all");
    s.scene.settled = false;
    s.run(2.0f);
    check(std::fabs(s.seq.closure() - 0.5f) < 0.02f, "closure ramps over close_seconds");
    const float before = s.seq.closure();
    s.scene.contacts = 3;
    s.run(0.2f);
    check(s.seq.phase() == Phase::kClosing, "a brief contact is not a grasp");
    s.scene.contacts = 0;
    s.run(0.04f);
    s.scene.contacts = 3;
    s.run(0.2f);
    check(s.seq.phase() == Phase::kClosing, "contact has to persist");
    s.run(0.2f);
    check(s.seq.phase() == Phase::kHolding, "persistent contact holds");
    const float held = s.seq.closure();
    check(held > before, "closure kept advancing until then");
    s.run(2.0f);
    check(s.seq.closure() == held, "closure stops once holding");
    s.scene.contacts = 0;
    s.run(2.0f);
    check(s.seq.phase() == Phase::kHolding, "keeps holding if contact drops");
  }
  {
    Sim s;
    s.seq.command(Command::kGrasp);
    s.scene.settled = true;
    s.run(5.0f);
    check(s.seq.phase() == Phase::kHolding, "out of travel with no contact keeps the grip");
    check(s.seq.closure() == 1.0f, "fully closed");
    float pose[gripper_servo::kJoints];
    check(s.seq.has_goal(), "and keeps issuing the goal");
    s.seq.pose(pose);
    check(pose[1] < -0.5f, "at the closed pose");
  }
  {
    Sim s;
    s.seq.command(Command::kGrasp);
    s.scene.settled = true;
    s.run(0.08f);
    s.scene.settled = false;
    s.scene.blocked = true;
    s.run(0.04f);
    check(s.seq.phase() == Phase::kHolding, "blocked stops the closure");
  }

  std::printf("\nrelease\n");
  {
    Sim s;
    s.seq.command(Command::kSelect, 1);
    s.seq.command(Command::kGrasp);
    s.scene.settled = true;
    s.scene.contacts = 3;
    s.object_at(0.05f);
    s.run(1.0f);
    check(s.seq.phase() == Phase::kHolding, "holding");
    s.seq.command(Command::kSelect, 0);
    check(s.seq.phase() == Phase::kHolding && s.seq.grasp().name == kGrasps[1].name,
          "cannot change grasp while holding");
    s.seq.command(Command::kRelease);
    check(s.seq.phase() == Phase::kPreshape && s.seq.closure() == 0.0f, "release opens");
    check(s.seq.grasp().name == kGrasps[1].name, "and keeps the grasp");
    s.scene.contacts = 0;
    s.run(5.0f);
    check(s.seq.phase() == Phase::kPreshape, "does not re-grasp the object it just let go of");
  }
  {
    Sim s;
    s.seq.command(Command::kGrasp);
    s.scene.settled = true;
    s.run(1.0f);
    s.seq.command(Command::kStop);
    check(!s.seq.has_goal(), "stop issues no goal, so Servo's watchdog holds");
  }
  std::printf("\nstore\n");
  const char * const store_path = "/tmp/grasp_check_store.txt";
  AddedGrasp mine;
  std::snprintf(mine.name, sizeof(mine.name), "mine");
  mine.index_spread_deg = 0.0f;
  mine.shape = kTwoFinger;
  mine.contacts = 2;
  char why[64] = "";
  {
    check(valid_name("mug_2-a") && !valid_name("") && !valid_name("a b") &&
          !valid_name("123456789012345678901234"), "names are one short word");
    check(check_grasp(mine, why, sizeof(why)), "a grasp copied from two_finger passes");

    AddedGrasp bad = mine;
    std::snprintf(bad.name, sizeof(bad.name), "stop");
    check(!check_grasp(bad, why, sizeof(why)), "a command word is not a grasp name");
    bad = mine;
    bad.shape.closed_deg[1] = -200.0f;
    check(!check_grasp(bad, why, sizeof(why)) && std::strstr(why, "C_3_Joint") != nullptr,
          "outside the joint limits is refused, naming the joint");
    bad = mine;
    bad.index_spread_deg = std::nanf("");
    check(!check_grasp(bad, why, sizeof(why)), "so is a NaN");
    bad = mine;
    bad.flex_current = 2000.0f;
    check(!check_grasp(bad, why, sizeof(why)), "and more current than the motors allow");

    bad = mine;
    bad.index_spread_deg = 0.0f;
    bad.middle_spread_deg = 0.0f;
    for (int i = 0; i < 6; ++i) {
      bad.shape.open_deg[i] = 0.0f;
      bad.shape.closed_deg[i] = reach_table::kFlexionLower[i] * 180.0f / 3.14159265f;
    }
    check(!check_grasp(bad, why, sizeof(why)) && std::strstr(why, "self-collision") != nullptr,
          "closing into itself is refused, saying how far it got");
  }
  {
    GraspStore store;
    const uint32_t start = store.revision();
    check(store.object_count() == 3 && store.find_object("cup")->touch_mm == 0.0f,
          "starts with the default objects, uncalibrated");
    check(store.put_grasp(mine) == nullptr && store.has_grasp(store.find_grasp("mine")),
          "a grasp can be added");
    const int slot = store.find_grasp("mine");
    check(store.is_added(slot) && store.grasp(slot).shape != nullptr, "and has its shape");
    AddedGrasp copy = mine;
    std::snprintf(copy.name, sizeof(copy.name), "tripod");
    check(store.put_grasp(copy) != nullptr, "a built in name is refused");
    check(store.remove_grasp("tripod") != nullptr, "a built in grasp cannot be deleted");

    check(store.put_object("mug", "mine") == nullptr, "an object can use it");
    check(store.put_object("box", "nope") != nullptr, "a grasp that does not exist is refused");
    check(store.remove_grasp("mine") != nullptr, "a grasp in use cannot be deleted");
    check(store.set_touch("mug", 72.0f) && store.find_object("mug")->touch_mm == 72.0f,
          "an object takes a trigger distance");
    check(!store.set_touch("nope", 72.0f), "only an object that exists");
    check(store.put_object("mug", "pinch") == nullptr &&
          store.find_object("mug")->touch_mm == 72.0f, "a new grasp keeps the distance");
    check(store.set_blind("pinch", 0.37f), "a grasp takes its blocked closure");
    check(store.revision() > start, "every change counts");

    check(store.put_object("mug", "mine") == nullptr && store.set_touch_closure("mug", 0.84f) &&
          store.save(store_path), "saves");
    GraspStore loaded;
    check(loaded.load(store_path), "loads");
    check(std::fabs(loaded.find_object("mug")->touch_closure - 0.84f) < 1e-3f,
          "with where it touches");
    check(std::fabs(loaded.blind_closure(loaded.find_grasp("pinch")) - 0.37f) < 1e-3f,
          "and the blocked closure comes back");
    check(loaded.object_count() == 4 && loaded.find_object("mug")->touch_mm == 72.0f &&
          std::strcmp(loaded.find_object("mug")->grasp, "mine") == 0,
          "the object comes back with its grasp and distance");
    const Grasp back = loaded.grasp(loaded.find_grasp("mine"));
    check(back.index_spread_deg == 0.0f && back.contacts == 2 &&
          back.shape->closed_deg[3] == kTwoFinger.closed_deg[3], "the grasp comes back whole");

    check(store.remove_object("mug") == nullptr && store.remove_grasp("mine") == nullptr &&
          store.remove_object("bear") == nullptr, "objects and grasps can be deleted");
    check(store.save(store_path) && loaded.load(store_path) && loaded.object_count() == 2 &&
          loaded.find_object("bear") == nullptr, "a deleted default stays deleted");
    check(GraspStore().find_grasp("mine") < 0, "and an empty store has none of it");
  }
  {
    GraspStore store;
    check(store.load("/tmp/grasp_check_no_such_file") && store.object_count() == 3,
          "no file leaves the defaults");
    std::FILE * file = std::fopen(store_path, "w");
    std::fputs("not a store\n", file);
    std::fclose(file);
    check(!store.load(store_path) && store.object_count() == 3, "a foreign file is refused");

    file = std::fopen(store_path, "w");
    std::fputs("gripper_store 1\nnonsense\nobject mug nope 72\nobject cup pinch nan\n"
               "grasp bad 0 0 0 0 0 0 0 0 0 0 0 0 -500 0 0 0 0\nobject foam tripod 37\n"
               "object bear envelop 18 2\nblind tripod 7\nblind nope 0.5\n", file);
    std::fclose(file);
    check(store.load(store_path) && store.object_count() == 1 &&
          store.find_object("foam")->touch_mm == 37.0f && store.find_grasp("bad") < 0,
          "rows that make no sense are skipped, the rest kept");
    check(store.find_object("foam")->touch_closure == 0.0f,
          "a row saved before touch closures existed still loads");
    check(store.blind_measured(0) < 0.0f, "and so does a store with no blocked closures");
    std::remove(store_path);
  }
  {
    StoreRequest request;
    check(!parse_store_command("arm", request) && !parse_store_command("tripod", request),
          "other commands are left alone");
    check(parse_store_command("object_save mug pinch", request) &&
          request.action == StoreAction::kPutObject && std::strcmp(request.name, "mug") == 0 &&
          std::strcmp(request.target, "pinch") == 0, "object_save");
    check(parse_store_command("object_save mug", request) &&
          request.action == StoreAction::kRejected, "without a grasp is rejected");
    check(parse_store_command("object_delete mug", request) &&
          request.action == StoreAction::kRemoveObject, "object_delete");
    check(parse_store_command(
            "grasp_save g 90 0 0 0 0 0 30 0 -3.8 -36 -8.6 -36 30 0 0 0 2", request) &&
          request.action == StoreAction::kPutGrasp && request.grasp.contacts == 2 &&
          request.grasp.shape.closed_deg[3] == -36.0f, "grasp_save");
    check(parse_store_command("grasp_save g 90 0 0", request) &&
          request.action == StoreAction::kRejected, "with too few fields is rejected");
    check(parse_store_command(
            "grasp_save g 90 0 0 0 0 0 30 0 -3.8 -500 -8.6 -36 30 0 0 0 2", request) &&
          request.action == StoreAction::kRejected && std::strstr(request.why, "limits") != nullptr,
          "and an impossible one says why");
  }
  {
    GraspStore store;
    store.put_grasp(mine);
    store.set_touch("cup", 72.0f);
    char row[400];
    int rows = 0;
    bool end_seen = false;
    bool fits = true;
    while (catalog_row(store, rows, row, sizeof(row))) {
      end_seen = std::strncmp(row, "catalog end", 11) == 0;
      fits = fits && std::strlen(row) < sizeof(row) - 1;
      if (rows == 1) {
        check(std::strncmp(row, "catalog grasp tripod 0 52.0 -52.0 0.0", 37) == 0,
              "a curl is spelled out as a shape, so it can be copied");
      }
      if (std::strncmp(row, "catalog object cup", 18) == 0) {
        check(std::strcmp(row, "catalog object cup pinch 72 0.000") == 0, "objects carry their distance");
      }
      ++rows;
    }
    check(end_seen && rows == 2 + kGraspCount + 1 + 3 && fits, "the catalog ends and every row fits");
  }
  std::printf("\ncalibrating an object\n");
  {
    Sim s;
    s.scene.settled = true;
    s.object_at(0.072f);
    s.seq.set_object("cup");
    s.run(0.2f);
    s.seq.command(Command::kCalibrate);
    check(std::fabs(s.store.find_object("cup")->touch_mm - 72.0f) < 0.5f, "calibrate saves it");
    check(std::fabs(s.seq.touch_m() - 0.072f) < 1e-4f, "and uses it");

    s.seq.set_object("foam");
    s.run(0.2f);
    check(std::fabs(s.seq.touch_m() - 0.05f) < 1e-4f,
          "an uncalibrated object has the default");
    check(s.store.find_object("foam")->touch_mm == 0.0f, "and is left uncalibrated");

    s.seq.set_object("cup");
    s.run(0.2f);
    check(std::fabs(s.seq.touch_m() - 0.072f) < 1e-4f, "going back brings it back");

    s.seq.command(Command::kGrasp);
    s.run(1.0f);
    const uint32_t revision = s.store.revision();
    s.seq.command(Command::kCalibrate);
    check(s.seq.phase() == Phase::kClosing && s.store.revision() == revision,
          "not while closing: the TOF is looking at the fingers");
    s.seq.command(Command::kRelease);

    s.scene.settled = false;
    s.run(0.2f);
    s.seq.command(Command::kCalibrate);
    check(s.store.revision() == revision, "not while the fingers are moving");

    s.scene.settled = true;
    s.nothing_seen();
    s.run(1.0f);
    s.seq.command(Command::kCalibrate);
    check(s.store.revision() == revision && std::strstr(s.seq.note(), "no valid range") != nullptr,
          "not from a reading that has gone stale");
  }
  {
    Sim s;
    s.object_at(0.04f);
    s.run(0.2f);
    const uint32_t revision = s.store.revision();
    s.seq.command(Command::kCalibrate);
    check(std::fabs(s.seq.touch_m() - 0.04f) < 1e-4f && s.store.revision() == revision,
          "with no object it applies, and there is nothing to save it to");
  }
  {
    Sim s;
    s.scene.settled = true;
    StoreRequest request;
    parse_store_command("grasp_save mine 90 0 0 0 0 0 30 0 -3.8 -36 -8.6 -36 30 0 0 0 2", request);
    check(s.seq.apply(request, false) == nullptr, "a grasp arrives");
    parse_store_command("object_save mug mine", request);
    check(s.seq.apply(request, false) == nullptr, "an object arrives");
    s.run(0.04f);
    check(std::strcmp(s.seq.object(), "mug") == 0 && std::strcmp(s.seq.grasp().name, "mine") == 0 &&
          s.seq.phase() == Phase::kPreshape, "and the hand goes to its preshape");

    parse_store_command("grasp_delete mine", request);
    check(s.seq.apply(request, false) != nullptr && s.store.find_grasp("mine") >= 0,
          "the grasp in the hand cannot be deleted");

    float pose[gripper_servo::kJoints];
    s.seq.command(Command::kGrasp);
    s.run(0.08f);
    parse_store_command("object_save mug tripod", request);
    check(s.seq.apply(request, false) != nullptr, "an object cannot change while closing");
    s.run(5.0f);
    s.seq.pose(pose);
    check(std::fabs(pose[1] - -36.0f * 3.14159265f / 180.0f) < 1e-3f,
          "and an added grasp closes to its own pose");
    s.seq.command(Command::kRelease);

    parse_store_command("object_save mug pinch", request);
    check(s.seq.apply(request, false) == nullptr, "free again, the object changes grasp");
    s.run(0.04f);
    check(std::strcmp(s.seq.grasp().name, "pinch") == 0, "and the hand follows at once");
    parse_store_command("object_save box pinch", request);
    s.seq.command(Command::kStop);
    check(s.seq.apply(request, true) == nullptr, "without torque it is saved");
    s.run(0.04f);
    check(std::strcmp(s.seq.object(), "mug") == 0, "but the hand does not go anywhere");

    parse_store_command("object_delete mug", request);
    check(s.seq.apply(request, false) == nullptr && s.seq.object()[0] == '\0' &&
          std::fabs(s.seq.touch_m() - 0.05f) < 1e-4f,
          "deleting the object in hand clears it");
    parse_store_command("object_delete mug", request);
    check(s.seq.apply(request, false) != nullptr, "deleting it twice says so");
  }
  std::printf("\n%s\n\n", failures == 0 ? "all passed" : "FAILED");
  return failures == 0 ? 0 : 1;
}
