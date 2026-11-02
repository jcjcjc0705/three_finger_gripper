#include "grasp_sequencer.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "servo/collision.hpp"
#include "grasp_store.hpp"
#include "reach_table.hpp"

namespace grasping
{
namespace
{

constexpr int kFewestRangeSamples = 3;
// Longer than this without a valid reading, and the object is not known to be there.
constexpr double kLostRangeS = 0.5;
// A reading this far below the touch distance has not come through the band, as an
// object does: the sensor sometimes reports a valid 0 mm with nothing there.
constexpr float kBelowTouchM = 0.01f;
// Near enough to count as there, or an object stopping just short never gets gripped.
constexpr float kTouchToleranceM = 0.003f;
constexpr float kReopenHysteresis = 0.05f;
constexpr float kRateTimeConstantS = 0.4f;
// Slow, because the first reading on a finger takes a few samples to get through.
constexpr float kMeasureSeconds = 8.0f;
constexpr double kMeasureBaselineS = 0.3;
// Nearer than this, with the hand empty, can only be a finger.
constexpr float kFingerRangeM = 0.2f;
constexpr float kBlindMargin = 0.03f;

}  // namespace

int find_grasp(const char * name)
{
  for (int i = 0; i < kGraspCount; ++i) {
    if (std::strcmp(kGrasps[i].name, name) == 0) { return i; }
  }
  return -1;
}

float closure_limit(const Grasp & grasp)
{
  if (grasp.shape != nullptr) { return 1.0f; }
  return std::min(reach_table::max_closure(grasp.index_spread_deg),
                  reach_table::max_closure(grasp.middle_spread_deg));
}

void grasp_pose(const Grasp & grasp, float closure, float * out)
{
  constexpr float kDeg = 3.14159265358979323846f / 180.0f;
  const float fraction = std::min(std::max(closure, 0.0f), 1.0f) * closure_limit(grasp);
  for (int i = 0; i < 6; ++i) {
    if (grasp.shape == nullptr) {
      out[i] = fraction * reach_table::kFlexionLower[i];
    } else {
      const float open = grasp.shape->open_deg[i];
      out[i] = (open + fraction * (grasp.shape->closed_deg[i] - open)) * kDeg;
    }
  }
  out[6] = grasp.index_spread_deg * kDeg;
  out[7] = grasp.middle_spread_deg * kDeg;
}

bool pose_trapped(const float * position)
{
  // A setpoint Servo stopped right at the limit reads a few ticks past it. That is
  // not trapped, and cutting the torque for it would drop whatever is in the hand.
  constexpr float kTolerance = 0.03f;
  float probe[gripper_servo::kJoints];
  float open[gripper_servo::kJoints];
  for (int i = 0; i < gripper_servo::kJoints; ++i) {
    probe[i] = position[i];
    open[i] = position[i];
    if (i < 6) {
      probe[i] = position[i] > kTolerance ? position[i] - kTolerance
                 : position[i] < -kTolerance ? position[i] + kTolerance : 0.0f;
      open[i] = 0.0f;
    }
  }
  if (!gripper_servo::feasible(probe)) { return true; }
  float out[gripper_servo::kJoints];
  return gripper_servo::advance_feasible(probe, open, out) < 1.0f;
}

const char * phase_name(Phase phase)
{
  switch (phase) {
    case Phase::kPreshape: return "preshape";
    case Phase::kArmed: return "armed";
    case Phase::kFollowing: return "following";
    case Phase::kMeasuring: return "measuring";
    case Phase::kClosing: return "closing";
    case Phase::kHolding: return "holding";
    default: return "idle";
  }
}

GraspSequencer::GraspSequencer(
  const SequencerConfig & config, GraspStore & store, int initial_grasp)
: config_(config), store_(store), default_touch_m_(config.touch_m),
  grasp_(initial_grasp)
{
}

Grasp GraspSequencer::grasp() const
{
  return store_.grasp(grasp_);
}

float GraspSequencer::blind_closure() const
{
  return store_.blind_closure(grasp_);
}

bool GraspSequencer::busy() const
{
  return phase_ == Phase::kFollowing || phase_ == Phase::kClosing ||
         phase_ == Phase::kHolding || phase_ == Phase::kMeasuring;
}

void GraspSequencer::refresh_touch_closure()
{
  // Closure means something else in another grasp, so only the object's own counts.
  const ObjectRow * row = object_[0] != '\0' ? store_.find_object(object_) : nullptr;
  touch_closure_ = row != nullptr && row->touch_closure > 0.0f &&
                   std::strcmp(row->grasp, grasp().name) == 0 ? row->touch_closure : 1.0f;
}

void GraspSequencer::note_range(const Tick & tick)
{
  if (!tick.tof_fresh || !tick.tof_valid) { return; }
  if (tick.now - range_at_ > kLostRangeS) { sample_count_ = 0; }
  samples_[sample_next_] = tick.tof_m;
  sample_next_ = (sample_next_ + 1) % kRangeSamples;
  if (sample_count_ < kRangeSamples) { ++sample_count_; }
  range_at_ = tick.now;
}

bool GraspSequencer::range_known() const
{
  return sample_count_ >= kFewestRangeSamples && now_ - range_at_ <= kLostRangeS;
}

float GraspSequencer::filtered_range() const
{
  // The median: a wrap-around or a stray 0 mm is one sample, and a mean would move.
  float sorted[kRangeSamples];
  std::copy(samples_, samples_ + sample_count_, sorted);
  std::sort(sorted, sorted + sample_count_);
  return sorted[sample_count_ / 2];
}

float GraspSequencer::command_close_rate() const
{
  const float seconds = grasp().close_seconds;
  return 1.0f / (seconds > 0.0f ? seconds : config_.close_seconds);
}

void GraspSequencer::start_closing(float rate)
{
  phase_ = Phase::kClosing;
  closing_rate_ = rate;
  contact_since_ = -1.0;
}

void GraspSequencer::set_note(const char * text)
{
  std::snprintf(note_, sizeof(note_), "%s", text);
}

void GraspSequencer::pose(float * out) const
{
  grasp_pose(grasp(), closure_, out);
}

void GraspSequencer::select(int slot)
{
  if (busy()) {
    set_note("release first");
    return;
  }
  grasp_ = slot;
  phase_ = Phase::kPreshape;
  closure_ = 0.0f;
  close_now_ = false;
  calibrated_ = false;
  learning_ = false;
  refresh_touch_closure();
  std::snprintf(note_, sizeof(note_), "%s preshape", grasp().name);
}

void GraspSequencer::set_object(const char * name)
{
  std::snprintf(wanted_object_, sizeof(wanted_object_), "%s", name);
}

void GraspSequencer::reselect_object(const char * name)
{
  object_[0] = '\0';
  set_object(name);
}

void GraspSequencer::track_object()
{
  if (wanted_object_[0] == '\0' || std::strcmp(wanted_object_, object_) == 0) { return; }
  if (busy()) { return; }

  const ObjectRow * row = store_.find_object(wanted_object_);
  const int slot = row != nullptr ? store_.find_grasp(row->grasp) : -1;
  if (slot < 0) {
    std::snprintf(note_, sizeof(note_), "unknown object %s", wanted_object_);
    wanted_object_[0] = '\0';
    return;
  }
  std::memcpy(object_, wanted_object_, sizeof(object_));
  config_.touch_m = row->touch_mm > 0.0f ? row->touch_mm / 1000.0f : default_touch_m_;
  select(slot);
  std::snprintf(note_, sizeof(note_), "%s: %s preshape", object_, grasp().name);
}

const char * GraspSequencer::apply(const StoreRequest & request, bool manual)
{
  const char * why = nullptr;
  switch (request.action) {
    case StoreAction::kRejected:
      why = request.why;
      break;
    case StoreAction::kPutGrasp:
      why = busy() && std::strcmp(grasp().name, request.grasp.name) == 0
              ? "the hand is using it, release first" : store_.put_grasp(request.grasp);
      break;
    case StoreAction::kRemoveGrasp:
      why = std::strcmp(grasp().name, request.name) == 0
              ? "the hand is using it, pick another first" : store_.remove_grasp(request.name);
      break;
    case StoreAction::kPutObject:
      why = busy() ? "release first" : store_.put_object(request.name, request.target);
      if (why == nullptr && !manual) { reselect_object(request.name); }
      break;
    case StoreAction::kRemoveObject:
      why = store_.remove_object(request.name);
      if (why == nullptr) {
        if (std::strcmp(object_, request.name) == 0) {
          object_[0] = '\0';
          config_.touch_m = default_touch_m_;
          refresh_touch_closure();
        }
        if (std::strcmp(wanted_object_, request.name) == 0) { wanted_object_[0] = '\0'; }
      }
      break;
  }
  if (why != nullptr) { set_note(why); }
  return why;
}

void GraspSequencer::calibrate()
{
  // A reading older than this says where something used to be, and it gets saved.
  constexpr double kFreshRangeS = 0.5;

  if (busy()) {
    set_note("open the hand first: closed, the TOF sees the fingers");
    return;
  }
  if ((phase_ == Phase::kPreshape || phase_ == Phase::kArmed) && !settled_) {
    set_note("wait for the fingers to settle");
    return;
  }
  if (sample_count_ == 0 || now_ - range_at_ > kFreshRangeS) {
    set_note("no valid range to calibrate against");
    return;
  }
  const float range = filtered_range();
  config_.touch_m = range;
  const double mm = static_cast<double>(range) * 1000.0;
  if (object_[0] != '\0' && store_.set_touch(object_, static_cast<float>(mm))) {
    calibrated_ = true;
    std::snprintf(note_, sizeof(note_), "%s: closes at %.0f mm, saved", object_, mm);
  } else {
    std::snprintf(note_, sizeof(note_), "touch distance %.0f mm, no object to save it to", mm);
  }
}

void GraspSequencer::command(Command command, int grasp)
{
  switch (command) {
    case Command::kSelect: select(grasp); break;
    case Command::kArm:
      if (busy()) {
        set_note("release first");
        break;
      }
      phase_ = Phase::kArmed;
      closure_ = 0.0f;
      close_now_ = false;
      set_note("armed, waiting for the object");
      break;
    case Command::kDisarm:
      if (phase_ == Phase::kArmed || phase_ == Phase::kFollowing) {
        phase_ = Phase::kPreshape;
        closure_ = 0.0f;
        close_now_ = false;
        set_note("disarmed");
      }
      break;
    case Command::kGrasp:
      if (phase_ == Phase::kClosing || phase_ == Phase::kHolding ||
          phase_ == Phase::kMeasuring) { break; }
      if (phase_ == Phase::kFollowing) {
        start_closing(command_close_rate());
        set_note("closing on command");
        break;
      }
      // Goes through kArmed so the spread has settled before the fingers curl:
      // the closure limits assume it is not moving.
      phase_ = Phase::kArmed;
      close_now_ = true;
      learning_ = calibrated_;
      calibrated_ = false;
      set_note("closing on command");
      break;
    case Command::kRelease:
      phase_ = Phase::kPreshape;
      closure_ = 0.0f;
      close_now_ = false;
      if (learning_) { calibrated_ = true; }
      learning_ = false;
      set_note("released");
      break;
    case Command::kStop:
      phase_ = Phase::kIdle;
      close_now_ = false;
      if (learning_) { calibrated_ = true; }
      learning_ = false;
      set_note("stopped, holding position");
      break;
    case Command::kMeasure:
      if (busy()) {
        set_note("release first");
        break;
      }
      phase_ = Phase::kMeasuring;
      closure_ = 0.0f;
      close_now_ = false;
      measure_from_ = -1.0;
      set_note("measuring: the hand has to be empty");
      break;
    case Command::kCalibrate:
      calibrate();
      break;
  }
}

void GraspSequencer::track_armed(const Tick & tick)
{
  // The closure limits assume the spread is not moving.
  if (!tick.settled) { return; }
  if (close_now_) {
    close_now_ = false;
    closure_ = 0.0f;
    start_closing(command_close_rate());
    return;
  }
  if (!range_known()) { return; }
  const float range = filtered_range();
  if (range < start_m() && range >= touch_m() - kBelowTouchM) {
    phase_ = Phase::kFollowing;
    follow_rate_ = 0.0f;
    contact_since_ = -1.0;
    std::snprintf(note_, sizeof(note_), "following from %.0f mm",
                  static_cast<double>(range) * 1000.0);
  }
}

void GraspSequencer::track_following(const Tick & tick)
{
  // How far along the band the object is sets how far along its closure the hand is,
  // so it needs no idea how fast the object is coming, and stops when the object does.
  float target = 0.0f;
  if (range_known()) {
    const float along = (start_m() - filtered_range()) / (config_.follow_band_m - kTouchToleranceM);
    target = std::min(std::max(along, 0.0f), 1.0f) * touch_closure_;
  }
  const float before = closure_;
  const float most = tick.dt / config_.fastest_close_s;
  if (target > closure_) {
    closure_ = std::min(target, closure_ + most);
  } else if (target < closure_ - kReopenHysteresis || target <= 0.0f) {
    // The hysteresis is for wobble. An object that has gone has gone.
    closure_ = std::max(target, closure_ - most);
  }
  if (tick.dt > 0.0f) {
    const float rate = (closure_ - before) / tick.dt;
    follow_rate_ += (rate - follow_rate_) * tick.dt / (kRateTimeConstantS + tick.dt);
  }

  if (in_contact(tick)) { return; }
  if (tick.blocked) {
    phase_ = Phase::kHolding;
    set_note("stopped: self-collision limit, check the reach table");
    return;
  }
  // Past here the TOF reads the fingers, not the object. Assume it keeps coming as it
  // was: the contacts still decide when to stop.
  if (closure_ >= std::min(blind_closure(), touch_closure_) - 1e-3f) {
    const float slowest = 1.0f / config_.slowest_blind_s;
    const float fastest = 1.0f / config_.fastest_close_s;
    start_closing(std::min(std::max(follow_rate_, slowest), fastest));
    std::snprintf(note_, sizeof(note_), "closing on from %.0f%%, the TOF cannot see it",
                  static_cast<double>(closure_) * 100.0);
  } else if (closure_ <= 0.0f && target <= 0.0f) {
    phase_ = Phase::kArmed;
    set_note("armed, the object left");
  }
}

bool GraspSequencer::in_contact(const Tick & tick)
{
  const Grasp current = grasp();
  if (tick.contacts < (current.contacts > 0 ? current.contacts : config_.contacts_needed)) {
    contact_since_ = -1.0;
    return false;
  }
  if (contact_since_ < 0.0) {
    contact_since_ = tick.now;
    contact_closure_ = closure_;
  }
  if (tick.now - contact_since_ >= static_cast<double>(config_.contact_hold_s)) {
    phase_ = Phase::kHolding;
    std::snprintf(note_, sizeof(note_), "holding, %d joints in contact", tick.contacts);
    learn_touch(contact_closure_);
  }
  return true;
}

void GraspSequencer::learn_touch(float closure)
{
  if (!learning_) { return; }
  learning_ = false;
  if (object_[0] != '\0' && store_.set_touch_closure(object_, closure)) {
    touch_closure_ = closure;
    std::snprintf(note_, sizeof(note_), "%s: touches at %.0f%%, saved", object_,
                  static_cast<double>(closure) * 100.0);
  }
}

void GraspSequencer::track_closing(const Tick & tick)
{
  closure_ = std::min(1.0f, closure_ + tick.dt * closing_rate_);

  if (in_contact(tick)) { return; }

  if (tick.blocked) {
    // The reach table should have kept us clear of this.
    phase_ = Phase::kHolding;
    set_note("stopped: self-collision limit, check the reach table");
  } else if (closure_ >= 1.0f && tick.settled) {
    // Soft objects do not always reach the contact threshold, so running out of
    // travel does not mean the hand is empty. Keep the grip.
    phase_ = Phase::kHolding;
    std::snprintf(note_, sizeof(note_), "closed fully, %d joints in contact", tick.contacts);
    if (tick.contacts > 0) {
      learn_touch(1.0f);
    } else {
      learning_ = false;
    }
  }
}

void GraspSequencer::finish_measuring(float closure)
{
  store_.set_blind(grasp().name, closure);
  phase_ = Phase::kPreshape;
  closure_ = 0.0f;
  if (closure >= 1.0f) {
    std::snprintf(note_, sizeof(note_), "%s: fingers never block the TOF, saved", grasp().name);
  } else {
    std::snprintf(note_, sizeof(note_), "%s: TOF blocked from %.0f%%, saved", grasp().name,
                  static_cast<double>(closure) * 100.0);
  }
}

void GraspSequencer::track_measuring(const Tick & tick)
{
  if (measure_from_ < 0.0) {
    if (tick.settled) { measure_from_ = tick.now; }
    return;
  }
  // Before the fingers move: anything near now is not a finger.
  const bool near = range_known() && filtered_range() < kFingerRangeM;
  if (closure_ == 0.0f && tick.now - measure_from_ < kMeasureBaselineS) { return; }
  if (closure_ == 0.0f && near) {
    phase_ = Phase::kPreshape;
    set_note("not measured: something is in front of the TOF");
    return;
  }
  if (tick.contacts > 0 || tick.blocked) {
    phase_ = Phase::kPreshape;
    closure_ = 0.0f;
    set_note("not measured: a finger touched something, empty the hand");
    return;
  }
  if (near) {
    finish_measuring(std::max(closure_ - kBlindMargin, 0.0f));
    return;
  }
  closure_ = std::min(1.0f, closure_ + tick.dt / kMeasureSeconds);
  if (closure_ >= 1.0f && tick.settled) { finish_measuring(1.0f); }
}

void GraspSequencer::update(const Tick & tick)
{
  now_ = tick.now;
  settled_ = tick.settled;
  note_range(tick);
  track_object();
  if (phase_ == Phase::kArmed) {
    track_armed(tick);
  } else if (phase_ == Phase::kFollowing) {
    track_following(tick);
  } else if (phase_ == Phase::kClosing) {
    track_closing(tick);
  } else if (phase_ == Phase::kMeasuring) {
    track_measuring(tick);
  }
}

}  // namespace grasping
