// What the hand has been taught: the objects it knows, each with its grasp and the
// range to close at, and grasps added on top of kGrasps. Kept on the board's NAND, so
// it is still there after a power cut.
//
// No hardware and no ROS, so test/grasp_check.cpp can drive it on a PC.
#ifndef GRASP_STORE_HPP_
#define GRASP_STORE_HPP_

#include <cstddef>
#include <cstdint>

#include "grasp_sequencer.hpp"

namespace grasping
{

inline constexpr std::size_t kNameSize = 24;
inline constexpr int kMaxAddedGrasps = 16;
inline constexpr int kMaxObjects = 32;
/// kGrasps first, then the added ones. A grasp keeps its slot until it is deleted.
inline constexpr int kGraspSlots = kGraspCount + kMaxAddedGrasps;

/// A grasp added at run time, spelled out in degrees. 0 keeps a default, as in Grasp.
struct AddedGrasp
{
  char name[kNameSize]{};
  float index_spread_deg{0.0f};
  float middle_spread_deg{0.0f};
  Shape shape{};
  float flex_current{0.0f};
  float close_seconds{0.0f};
  int contacts{0};
};

struct ObjectRow
{
  char name[kNameSize]{};
  char grasp[kNameSize]{};
  float touch_mm{0.0f};        ///< what the TOF reads where the fingers meet it, 0 until calibrated
  float touch_closure{0.0f};   ///< how far this grasp has closed by then, 0 until a grasp shows it
};

/// Letters, digits, _ and -, so that a name is one word in every text format here.
bool valid_name(const char * name);

/// Inside the joint limits, closes without self-collision, and can open again.
/// Slow enough to keep off the control thread: 50 or so collision checks.
bool check_grasp(const AddedGrasp & grasp, char * why, std::size_t size);

class GraspStore
{
public:
  /// kObjects, uncalibrated, and nothing added.
  GraspStore();

  bool has_grasp(int slot) const;
  /// Points into this store, so it does not outlive a change to that slot.
  Grasp grasp(int slot) const;
  int find_grasp(const char * name) const;
  bool is_added(int slot) const { return slot >= kGraspCount; }
  /// Closure from which the fingers block the TOF, or a cautious default if unmeasured.
  float blind_closure(int slot) const;
  /// The same, or -1 if it has not been measured.
  float blind_measured(int slot) const { return blind_[slot]; }

  int object_count() const { return object_count_; }
  const ObjectRow & object(int index) const { return objects_[index]; }
  const ObjectRow * find_object(const char * name) const;

  // Each returns nullptr, or why not.
  const char * put_grasp(const AddedGrasp & grasp);
  const char * remove_grasp(const char * name);
  /// Keeps the object's touch distance if it already has one. A different grasp
  /// forgets its touch closure, which only meant something for the old one.
  const char * put_object(const char * name, const char * grasp);
  const char * remove_object(const char * name);
  bool set_touch(const char * object, float mm);
  bool set_touch_closure(const char * object, float closure);
  bool set_blind(const char * grasp, float closure);

  /// Counts the changes above, so that a change can be noticed and saved.
  uint32_t revision() const { return revision_; }

  /// All or nothing, even if the power goes half way.
  bool save(const char * path) const;
  /// A missing file leaves the defaults and returns true. False: the file is there
  /// but is not one of these. Rows that make no sense are skipped with a warning.
  bool load(const char * path);

private:
  float blind_[kGraspSlots];
  bool used_[kMaxAddedGrasps]{};
  AddedGrasp added_[kMaxAddedGrasps]{};
  int object_count_{0};
  ObjectRow objects_[kMaxObjects]{};
  uint32_t revision_{0};
};

enum class StoreAction { kPutGrasp, kRemoveGrasp, kPutObject, kRemoveObject, kRejected };

struct StoreRequest
{
  StoreAction action{StoreAction::kRejected};
  AddedGrasp grasp{};
  char name[kNameSize]{};
  char target[kNameSize]{};
  char why[64]{};
};

/// False for text that is none of these. One that cannot be carried out comes back
/// as kRejected with the reason. Runs check_grasp, so not on the control thread.
///   grasp_save <name> <index> <middle> <6 open> <6 closed> <current> <seconds> <contacts>
///   grasp_delete <name>
///   object_save <name> <grasp>
///   object_delete <name>
bool parse_store_command(const char * text, StoreRequest & out);

/// One row at a time, for whatever draws a menu. False past the last.
///   catalog begin <revision>
///   catalog grasp <name> <0 built in, 1 added> <index> <middle> <6 open> <6 closed>
///                 <current> <seconds> <contacts> <blind closure, -1 unmeasured>
///   catalog object <name> <grasp> <touch_mm> <touch closure>
///   catalog end <revision> <grasps> <objects>
bool catalog_row(const GraspStore & store, int index, char * out, std::size_t size);

}  // namespace grasping

#endif  // GRASP_STORE_HPP_
