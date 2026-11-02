#include "grasp_store.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "servo/collision.hpp"

namespace grasping
{
namespace
{

constexpr const char * kHeader = "gripper_store 1";
constexpr float kMaxFlexCurrent = 800.0f;   // the motors' Current Limit
constexpr float kMinCloseSeconds = 0.5f;
constexpr float kMaxCloseSeconds = 60.0f;
constexpr float kMaxTouchMm = 2000.0f;
// For a grasp not measured yet. The lowest measured on any grasp is 29.5%.
constexpr float kDefaultBlindClosure = 0.3f;
constexpr int kClosureSteps = 50;
constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
// A pose read back from one decimal of a degree can land this far past a limit it
// was sitting on. Servo clamps to the limit anyway.
constexpr float kLimitSlackRad = 0.002f;

// A grasp name also works as a command, so it cannot be one of the others.
constexpr const char * kCommandWords[] = {
  "arm", "disarm", "grasp", "release", "open", "stop", "calibrate", "photo", "catalog",
  "measure",
  "grasp_save", "grasp_delete", "object_save", "object_delete",
};

void copy_name(char * out, const char * name)
{
  std::snprintf(out, kNameSize, "%s", name);
}

Grasp as_grasp(const AddedGrasp & grasp)
{
  return Grasp{grasp.name, grasp.index_spread_deg, grasp.middle_spread_deg, &grasp.shape,
               grasp.flex_current, grasp.close_seconds, grasp.contacts};
}

bool to_float(const char * token, float & out)
{
  if (token == nullptr) { return false; }
  char * end = nullptr;
  out = std::strtof(token, &end);
  return end != token && *end == '\0';
}

/// name, spreads, 6 open, 6 closed, current, seconds, contacts. Whitespace separated.
bool parse_grasp_fields(char * text, AddedGrasp & out)
{
  char * save = nullptr;
  const char * name = strtok_r(text, " \t\r\n", &save);
  if (name == nullptr || std::strlen(name) >= kNameSize) { return false; }
  copy_name(out.name, name);
  float * const fields[] = {
    &out.index_spread_deg, &out.middle_spread_deg,
    &out.shape.open_deg[0], &out.shape.open_deg[1], &out.shape.open_deg[2],
    &out.shape.open_deg[3], &out.shape.open_deg[4], &out.shape.open_deg[5],
    &out.shape.closed_deg[0], &out.shape.closed_deg[1], &out.shape.closed_deg[2],
    &out.shape.closed_deg[3], &out.shape.closed_deg[4], &out.shape.closed_deg[5],
    &out.flex_current, &out.close_seconds,
  };
  for (float * field : fields) {
    if (!to_float(strtok_r(nullptr, " \t\r\n", &save), *field)) { return false; }
  }
  const char * contacts = strtok_r(nullptr, " \t\r\n", &save);
  if (contacts == nullptr) { return false; }
  out.contacts = std::atoi(contacts);
  return strtok_r(nullptr, " \t\r\n", &save) == nullptr;
}

bool inside_limits(const float * pose, char * why, std::size_t size, const char * which)
{
  for (int i = 0; i < gripper_servo::kJoints; ++i) {
    const auto & model = gripper_servo::kJointModel[i];
    // Written so that a NaN fails too.
    if (!(pose[i] >= model.lower - kLimitSlackRad && pose[i] <= model.upper + kLimitSlackRad)) {
      std::snprintf(why, size, "%s %s is outside its limits", which, gripper_servo::kJointNames[i]);
      return false;
    }
  }
  return true;
}

}  // namespace

bool valid_name(const char * name)
{
  const std::size_t length = std::strlen(name);
  if (length == 0 || length >= kNameSize) { return false; }
  for (std::size_t i = 0; i < length; ++i) {
    const unsigned char c = static_cast<unsigned char>(name[i]);
    if (!std::isalnum(c) && c != '_' && c != '-') { return false; }
  }
  return true;
}

bool check_grasp(const AddedGrasp & grasp, char * why, std::size_t size)
{
  if (!valid_name(grasp.name)) {
    std::snprintf(why, size, "name: up to 23 letters, digits, _ or -");
    return false;
  }
  for (const char * word : kCommandWords) {
    if (std::strcmp(word, grasp.name) == 0) {
      std::snprintf(why, size, "%s is a command, pick another name", word);
      return false;
    }
  }
  if (!(grasp.flex_current >= 0.0f && grasp.flex_current <= kMaxFlexCurrent)) {
    std::snprintf(why, size, "current: 0 to %.0f", static_cast<double>(kMaxFlexCurrent));
    return false;
  }
  if (grasp.close_seconds != 0.0f && !(grasp.close_seconds >= kMinCloseSeconds &&
                                       grasp.close_seconds <= kMaxCloseSeconds))
  {
    std::snprintf(why, size, "seconds: 0, or %.1f to %.0f",
                  static_cast<double>(kMinCloseSeconds), static_cast<double>(kMaxCloseSeconds));
    return false;
  }
  if (grasp.contacts < 0 || grasp.contacts > 6) {
    std::snprintf(why, size, "contacts: 0 to 6");
    return false;
  }

  const Grasp shape = as_grasp(grasp);
  float pose[gripper_servo::kJoints];
  grasp_pose(shape, 0.0f, pose);
  if (!inside_limits(pose, why, size, "open")) { return false; }
  grasp_pose(shape, 1.0f, pose);
  if (!inside_limits(pose, why, size, "closed")) { return false; }

  for (int step = 0; step <= kClosureSteps; ++step) {
    const float closure = static_cast<float>(step) / kClosureSteps;
    grasp_pose(shape, closure, pose);
    if (!gripper_servo::feasible(pose)) {
      std::snprintf(why, size, "self-collision at %.0f%% closure",
                    static_cast<double>(closure) * 100.0);
      return false;
    }
  }
  grasp_pose(shape, 1.0f, pose);
  if (pose_trapped(pose)) {
    std::snprintf(why, size, "closed, the fingers could not open again");
    return false;
  }
  return true;
}

GraspStore::GraspStore()
{
  for (float & blind : blind_) { blind = -1.0f; }
  for (const ObjectGrasp & row : kObjects) {
    copy_name(objects_[object_count_].name, row.object);
    copy_name(objects_[object_count_].grasp, row.grasp);
    ++object_count_;
  }
}

bool GraspStore::has_grasp(int slot) const
{
  if (slot < 0 || slot >= kGraspSlots) { return false; }
  return slot < kGraspCount || used_[slot - kGraspCount];
}

Grasp GraspStore::grasp(int slot) const
{
  return slot < kGraspCount ? kGrasps[slot] : as_grasp(added_[slot - kGraspCount]);
}

int GraspStore::find_grasp(const char * name) const
{
  const int builtin = grasping::find_grasp(name);
  if (builtin >= 0) { return builtin; }
  for (int i = 0; i < kMaxAddedGrasps; ++i) {
    if (used_[i] && std::strcmp(added_[i].name, name) == 0) { return kGraspCount + i; }
  }
  return -1;
}

float GraspStore::blind_closure(int slot) const
{
  return blind_[slot] >= 0.0f ? blind_[slot] : kDefaultBlindClosure;
}

const ObjectRow * GraspStore::find_object(const char * name) const
{
  for (int i = 0; i < object_count_; ++i) {
    if (std::strcmp(objects_[i].name, name) == 0) { return &objects_[i]; }
  }
  return nullptr;
}

const char * GraspStore::put_grasp(const AddedGrasp & grasp)
{
  if (grasping::find_grasp(grasp.name) >= 0) { return "built in, pick another name"; }
  int free_slot = -1;
  for (int i = kMaxAddedGrasps - 1; i >= 0; --i) {
    if (!used_[i]) { free_slot = i; }
    if (used_[i] && std::strcmp(added_[i].name, grasp.name) == 0) {
      free_slot = i;
      break;
    }
  }
  if (free_slot < 0) { return "no room for another grasp"; }
  used_[free_slot] = true;
  added_[free_slot] = grasp;
  blind_[kGraspCount + free_slot] = -1.0f;   // a different shape blocks the TOF differently
  ++revision_;
  return nullptr;
}

const char * GraspStore::remove_grasp(const char * name)
{
  if (grasping::find_grasp(name) >= 0) { return "built in, cannot be deleted"; }
  const int slot = find_grasp(name);
  if (slot < 0) { return "no such grasp"; }
  for (int i = 0; i < object_count_; ++i) {
    if (std::strcmp(objects_[i].grasp, name) == 0) { return "an object is using it"; }
  }
  used_[slot - kGraspCount] = false;
  added_[slot - kGraspCount] = AddedGrasp{};
  blind_[slot] = -1.0f;
  ++revision_;
  return nullptr;
}

const char * GraspStore::put_object(const char * name, const char * grasp)
{
  if (!valid_name(name)) { return "name: up to 23 letters, digits, _ or -"; }
  if (find_grasp(grasp) < 0) { return "no such grasp"; }
  ObjectRow * row = const_cast<ObjectRow *>(find_object(name));
  if (row == nullptr) {
    if (object_count_ >= kMaxObjects) { return "no room for another object"; }
    row = &objects_[object_count_++];
    *row = ObjectRow{};
    copy_name(row->name, name);
  } else if (std::strcmp(row->grasp, grasp) != 0) {
    row->touch_closure = 0.0f;
  }
  copy_name(row->grasp, grasp);
  ++revision_;
  return nullptr;
}

const char * GraspStore::remove_object(const char * name)
{
  const ObjectRow * row = find_object(name);
  if (row == nullptr) { return "no such object"; }
  const int index = static_cast<int>(row - objects_);
  for (int i = index; i + 1 < object_count_; ++i) { objects_[i] = objects_[i + 1]; }
  --object_count_;
  objects_[object_count_] = ObjectRow{};
  ++revision_;
  return nullptr;
}

bool GraspStore::set_touch(const char * object, float mm)
{
  ObjectRow * row = const_cast<ObjectRow *>(find_object(object));
  if (row == nullptr || !(mm >= 0.0f && mm <= kMaxTouchMm)) { return false; }
  row->touch_mm = mm;
  ++revision_;
  return true;
}

bool GraspStore::set_touch_closure(const char * object, float closure)
{
  ObjectRow * row = const_cast<ObjectRow *>(find_object(object));
  if (row == nullptr || !(closure >= 0.0f && closure <= 1.0f)) { return false; }
  row->touch_closure = closure;
  ++revision_;
  return true;
}

bool GraspStore::set_blind(const char * grasp, float closure)
{
  const int slot = find_grasp(grasp);
  if (slot < 0 || !(closure >= 0.0f && closure <= 1.0f)) { return false; }
  blind_[slot] = closure;
  ++revision_;
  return true;
}

bool GraspStore::save(const char * path) const
{
  const std::string temp = std::string(path) + ".new";
  std::FILE * file = std::fopen(temp.c_str(), "w");
  if (file == nullptr) { return false; }

  std::fprintf(file, "%s\n", kHeader);
  for (int i = 0; i < kMaxAddedGrasps; ++i) {
    if (!used_[i]) { continue; }
    const AddedGrasp & g = added_[i];
    std::fprintf(file, "grasp %s %.2f %.2f", g.name, static_cast<double>(g.index_spread_deg),
                 static_cast<double>(g.middle_spread_deg));
    for (const float v : g.shape.open_deg) { std::fprintf(file, " %.2f", static_cast<double>(v)); }
    for (const float v : g.shape.closed_deg) { std::fprintf(file, " %.2f", static_cast<double>(v)); }
    std::fprintf(file, " %.0f %.2f %d\n", static_cast<double>(g.flex_current),
                 static_cast<double>(g.close_seconds), g.contacts);
  }
  for (int slot = 0; slot < kGraspSlots; ++slot) {
    if (has_grasp(slot) && blind_[slot] >= 0.0f) {
      std::fprintf(file, "blind %s %.3f\n", grasp(slot).name, static_cast<double>(blind_[slot]));
    }
  }
  for (int i = 0; i < object_count_; ++i) {
    std::fprintf(file, "object %s %s %.0f %.3f\n", objects_[i].name, objects_[i].grasp,
                 static_cast<double>(objects_[i].touch_mm),
                 static_cast<double>(objects_[i].touch_closure));
  }

  bool ok = std::fflush(file) == 0 && ::fsync(fileno(file)) == 0;
  ok = std::fclose(file) == 0 && ok;
  ok = ok && std::rename(temp.c_str(), path) == 0;
  if (!ok) {
    ::unlink(temp.c_str());
    return false;
  }
  // The rename has to reach the disk too, or a power cut brings the old file back.
  const std::string whole(path);
  const std::size_t slash = whole.rfind('/');
  const std::string directory = slash == std::string::npos ? "." : whole.substr(0, slash + 1);
  const int fd = ::open(directory.c_str(), O_RDONLY);
  if (fd >= 0) {
    ok = ::fsync(fd) == 0;
    ::close(fd);
  }
  return ok;
}

bool GraspStore::load(const char * path)
{
  std::FILE * file = std::fopen(path, "r");
  if (file == nullptr) { return errno == ENOENT; }

  char line[400];
  if (std::fgets(line, sizeof(line), file) == nullptr ||
      std::strncmp(line, kHeader, std::strlen(kHeader)) != 0)
  {
    std::fclose(file);
    return false;
  }

  // The file is the truth from here on, or a deleted object would come back.
  *this = GraspStore();
  for (int i = 0; i < object_count_; ++i) { objects_[i] = ObjectRow{}; }
  object_count_ = 0;

  while (std::fgets(line, sizeof(line), file) != nullptr) {
    char kept[sizeof(line)];
    std::memcpy(kept, line, sizeof(line));
    kept[std::strcspn(kept, "\r\n")] = '\0';

    bool ok = false;
    if (std::strncmp(line, "grasp ", 6) == 0) {
      AddedGrasp grasp;
      char why[64];
      ok = parse_grasp_fields(line + 6, grasp) && check_grasp(grasp, why, sizeof(why)) &&
           put_grasp(grasp) == nullptr;
    } else if (std::strncmp(line, "object ", 7) == 0) {
      char * save = nullptr;
      const char * name = strtok_r(line + 7, " \t\r\n", &save);
      const char * grasp = strtok_r(nullptr, " \t\r\n", &save);
      float mm = 0.0f;
      ok = name != nullptr && grasp != nullptr &&
           to_float(strtok_r(nullptr, " \t\r\n", &save), mm) &&
           mm >= 0.0f && mm <= kMaxTouchMm;
      // Rows saved before there was a touch closure stop at the distance.
      float closure = 0.0f;
      const char * last = ok ? strtok_r(nullptr, " \t\r\n", &save) : nullptr;
      if (last != nullptr) { ok = to_float(last, closure) && closure >= 0.0f && closure <= 1.0f; }
      ok = ok && put_object(name, grasp) == nullptr && set_touch(name, mm) &&
           set_touch_closure(name, closure);
    } else if (std::strncmp(line, "blind ", 6) == 0) {
      char * save = nullptr;
      const char * name = strtok_r(line + 6, " \t\r\n", &save);
      float closure = 0.0f;
      ok = name != nullptr && to_float(strtok_r(nullptr, " \t\r\n", &save), closure) &&
           set_blind(name, closure);
    }
    if (!ok) { std::fprintf(stderr, "%s: skipping \"%s\"\n", path, kept); }
  }
  std::fclose(file);
  revision_ = 0;
  return true;
}

bool parse_store_command(const char * text, StoreRequest & out)
{
  char line[320];
  if (std::snprintf(line, sizeof(line), "%s", text) >= static_cast<int>(sizeof(line))) {
    line[0] = '\0';
  }
  out = StoreRequest{};

  char * rest = std::strchr(line, ' ');
  if (rest != nullptr) { *rest++ = '\0'; }
  const std::string verb(line);

  if (verb == "grasp_save") {
    if (rest == nullptr || !parse_grasp_fields(rest, out.grasp)) {
      std::snprintf(out.why, sizeof(out.why), "grasp_save needs 18 fields");
    } else if (check_grasp(out.grasp, out.why, sizeof(out.why))) {
      out.action = StoreAction::kPutGrasp;
    }
    return true;
  }
  if (verb == "grasp_delete" || verb == "object_delete" || verb == "object_save") {
    char * save = nullptr;
    const char * first = rest != nullptr ? strtok_r(rest, " \t\r\n", &save) : nullptr;
    const char * second = first != nullptr ? strtok_r(nullptr, " \t\r\n", &save) : nullptr;
    const bool two = verb == "object_save";
    if (first == nullptr || std::strlen(first) >= kNameSize || (two && second == nullptr) ||
        (two && std::strlen(second) >= kNameSize) || (!two && second != nullptr))
    {
      std::snprintf(out.why, sizeof(out.why), "%s: wrong arguments", verb.c_str());
      return true;
    }
    copy_name(out.name, first);
    if (two) { copy_name(out.target, second); }
    out.action = verb == "object_save" ? StoreAction::kPutObject
                 : verb == "object_delete" ? StoreAction::kRemoveObject
                 : StoreAction::kRemoveGrasp;
    return true;
  }
  return false;
}

bool catalog_row(const GraspStore & store, int index, char * out, std::size_t size)
{
  int grasps = 0;
  for (int slot = 0; slot < kGraspSlots; ++slot) {
    if (store.has_grasp(slot)) { ++grasps; }
  }

  if (index == 0) {
    std::snprintf(out, size, "catalog begin %u", static_cast<unsigned>(store.revision()));
    return true;
  }
  int row = index - 1;
  if (row < grasps) {
    int slot = 0;
    for (int seen = -1; slot < kGraspSlots; ++slot) {
      if (store.has_grasp(slot) && ++seen == row) { break; }
    }
    const Grasp g = store.grasp(slot);
    float open[gripper_servo::kJoints];
    float closed[gripper_servo::kJoints];
    grasp_pose(g, 0.0f, open);
    grasp_pose(g, 1.0f, closed);
    int used = std::snprintf(out, size, "catalog grasp %s %d %.1f %.1f", g.name,
                             store.is_added(slot) ? 1 : 0, static_cast<double>(g.index_spread_deg),
                             static_cast<double>(g.middle_spread_deg));
    for (int i = 0; i < 6; ++i) {
      // Adding zero turns the -0.0 of a joint that does not move into 0.0.
      used += std::snprintf(out + used, size - used, " %.1f",
                            static_cast<double>(open[i] * kRadToDeg + 0.0f));
    }
    for (int i = 0; i < 6; ++i) {
      used += std::snprintf(out + used, size - used, " %.1f",
                            static_cast<double>(closed[i] * kRadToDeg + 0.0f));
    }
    std::snprintf(out + used, size - used, " %.0f %.1f %d %.3f", static_cast<double>(g.flex_current),
                  static_cast<double>(g.close_seconds), g.contacts,
                  static_cast<double>(store.blind_measured(slot)));
    return true;
  }
  row -= grasps;
  if (row < store.object_count()) {
    const ObjectRow & object = store.object(row);
    std::snprintf(out, size, "catalog object %s %s %.0f %.3f", object.name, object.grasp,
                  static_cast<double>(object.touch_mm), static_cast<double>(object.touch_closure));
    return true;
  }
  if (row == store.object_count()) {
    std::snprintf(out, size, "catalog end %u %d %d", static_cast<unsigned>(store.revision()),
                  grasps, store.object_count());
    return true;
  }
  return false;
}

}  // namespace grasping
