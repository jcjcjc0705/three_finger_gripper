// The grasp strategy: which pose, when to close, when it has hold of something.
//
// No hardware and no ROS, so test/grasp_check.cpp can drive it on a PC. It only
// produces goals; Servo still owns trajectory, joint limits and self-collision.
#ifndef GRASP_SEQUENCER_HPP_
#define GRASP_SEQUENCER_HPP_

#include <cstddef>

#include "servo/gripper_model.hpp"

namespace grasping
{

/// Flexion joints in kJointNames order, in degrees: thumb, index, middle, each
/// proximal then distal. Positive proximal leans a finger out of the way.
struct Shape
{
  float open_deg[6];
  float closed_deg[6];
};

inline constexpr Shape kEnvelop{
  {35.0f, 0.0f, 35.0f, 0.0f, 35.0f, 0.0f},
  {-3.6f, -33.6f, -8.0f, -33.6f, -8.0f, -33.6f},
};
inline constexpr Shape kPinchWide{
  {35.0f, 0.0f, 35.0f, 0.0f, 35.0f, 0.0f},
  {0.0f, -36.0f, 0.0f, -36.0f, 0.0f, -36.0f},
};
// Index and middle, which face each other at spread 0 with their curling planes 0.4 mm
// apart. The thumb cannot do this: its plane is 20 mm from the index finger's, so the
// two tips pass each other. It leans out and stays there.
inline constexpr Shape kTwoFinger{
  {30.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {30.0f, 0.0f, -24.0f, -24.0f, -24.0f, -24.0f},
};

/// A row here is all a new grasp needs. test/grasp_check.cpp then checks that it
/// closes without self-collision and can be reached from every other grasp.
struct Grasp
{
  const char * name;
  float index_spread_deg;
  float middle_spread_deg;
  /// nullptr: every finger curls together, as far as the reach table allows.
  const Shape * shape{nullptr};
  // 0 keeps the default: the joint's Goal Current, --close-seconds, --contacts.
  float flex_current{0.0f};
  float close_seconds{0.0f};
  int contacts{0};
};

inline constexpr Grasp kGrasps[] = {
  {"tripod", 52.0f, -52.0f},   // three tips evenly spaced, for round objects
  {"pinch", 90.0f, -90.0f},    // index and middle together against the thumb
  {"wide", 0.0f, 0.0f},        // fingers apart, widest opening
  {"envelop", 52.0f, -52.0f, &kEnvelop},        // opens to about 190 mm, for large soft objects
  {"pinch_wide", 90.0f, -90.0f, &kPinchWide},   // pinch that opens to about 190 mm
  // Three contacts out of its four joints means both fingers have got there.
  {"two_finger", 0.0f, 0.0f, &kTwoFinger},      // index against middle, for small objects
};
inline constexpr int kGraspCount = sizeof(kGrasps) / sizeof(kGrasps[0]);

struct ObjectGrasp
{
  const char * object;
  const char * grasp;
};

/// What an empty store starts with: see grasp_store.hpp. /gripper/object names the
/// object; whatever recognises it is someone else's job.
inline constexpr ObjectGrasp kObjects[] = {
  {"cup", "pinch"},
  {"foam", "tripod"},
  {"bear", "envelop"},
};

/// Index into kGrasps, or -1. GraspStore::find_grasp also finds the added ones.
int find_grasp(const char * name);

/// How far along its range a grasp may close: for a curl, the smaller of the two
/// fingers' limits, since both have to fit.
float closure_limit(const Grasp & grasp);

/// Joint vector for `grasp` closed by `closure` (0..1) of its usable range.
void grasp_pose(const Grasp & grasp, float closure, float * out);

/// True for a measured pose Servo can never move away from: self-colliding, or so
/// far past the collision band that opening the fingers would have to cross it.
bool pose_trapped(const float * position);

enum class Phase { kIdle, kPreshape, kArmed, kFollowing, kClosing, kHolding, kMeasuring };
const char * phase_name(Phase phase);

enum class Command
{
  kSelect, kArm, kDisarm, kGrasp, kRelease, kStop, kCalibrate, kMeasure,
};

struct SequencerConfig
{
  // What the TOF reads with the object where the fingers should meet it. A default,
  // not a derived value: the TOF's pose in the URDF is a guess, so calibrate on the
  // real object.
  float touch_m{0.05f};
  // The fingers start closing this far before that, so that they arrive together.
  float follow_band_m{0.04f};
  // A full closure takes at least this long, however fast the object comes. Closing
  // in about a second loaded the thumb to its ceiling before the others arrived.
  float fastest_close_s{3.0f};
  // And at most this long once the TOF can no longer see the object.
  float slowest_blind_s{12.0f};
  float close_seconds{4.0f};
  int contacts_needed{3};
  float contact_hold_s{0.3f};
};

class GraspStore;
struct StoreRequest;

struct Tick
{
  double now{0.0};
  float dt{0.0f};
  bool tof_fresh{false};
  bool tof_valid{false};
  float tof_m{0.0f};
  int contacts{0};
  bool blocked{false};
  bool settled{false};   ///< Servo holding and nothing moving
};

class GraspSequencer
{
public:
  GraspSequencer(const SequencerConfig & config, GraspStore & store, int initial_grasp = 0);

  /// `grasp` is a slot in the store, for kSelect.
  void command(Command command, int grasp = 0);
  void update(const Tick & tick);

  /// Moves to that object's preshape, unless the hand is busy with a grasp, when
  /// it waits for the release. Sending the same name again changes nothing, so a
  /// recogniser can repeat it every frame, and a grasp chosen by hand stays chosen.
  void set_object(const char * name);
  const char * object() const { return object_; }
  /// The same, even if it is already the object, for when its row has changed.
  void reselect_object(const char * name);

  /// Carries out a change to the store. Returns nullptr, or why not.
  const char * apply(const StoreRequest & request, bool manual);

  /// Phase::kIdle issues no goal, so Servo's watchdog holds whatever is there.
  bool has_goal() const { return phase_ != Phase::kIdle; }
  void pose(float * out) const;

  Phase phase() const { return phase_; }
  Grasp grasp() const;
  float closure() const { return closure_; }
  float touch_m() const { return config_.touch_m; }
  float start_m() const { return config_.touch_m + config_.follow_band_m; }
  /// Closure at which the object in hand is touched; 1 until a grasp has shown it.
  float touch_closure() const { return touch_closure_; }
  /// Closure from which the fingers block the TOF, for this grasp.
  float blind_closure() const;
  /// Following, closing, holding or measuring: the fingers are spoken for.
  bool busy() const;
  const char * note() const { return note_; }
  void set_note(const char * text);

private:
  void select(int slot);
  void calibrate();
  void track_object();
  void track_armed(const Tick & tick);
  void track_following(const Tick & tick);
  void track_closing(const Tick & tick);
  void track_measuring(const Tick & tick);
  void note_range(const Tick & tick);
  bool range_known() const;
  float filtered_range() const;
  void refresh_touch_closure();
  void start_closing(float rate);
  float command_close_rate() const;
  /// True while enough joints are in contact; goes to kHolding once that has lasted.
  bool in_contact(const Tick & tick);
  void learn_touch(float closure);
  void finish_measuring(float closure);

  SequencerConfig config_;
  GraspStore & store_;
  float default_touch_m_;
  int grasp_;
  Phase phase_{Phase::kIdle};
  float closure_{0.0f};
  float touch_closure_{1.0f};
  float follow_rate_{0.0f};
  float closing_rate_{0.0f};
  bool close_now_{false};
  bool calibrated_{false};   // calibrated since the last grasp, which will show the touch
  bool learning_{false};
  double contact_since_{-1.0};
  float contact_closure_{0.0f};
  double measure_from_{-1.0};
  static constexpr int kRangeSamples = 5;
  float samples_[kRangeSamples]{};
  int sample_count_{0};
  int sample_next_{0};
  double range_at_{-100.0};
  double now_{0.0};
  bool settled_{false};
  char note_[64]{};
  char object_[24]{};
  char wanted_object_[24]{};
};

}  // namespace grasping

#endif  // GRASP_SEQUENCER_HPP_
