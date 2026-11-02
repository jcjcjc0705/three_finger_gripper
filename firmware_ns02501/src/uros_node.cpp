// The gripper as a ROS 2 node, running entirely on the board.
//
// Motors on /dev/ttyS1, TOF on /dev/i2c-2, camera on USB, and a micro-ROS
// client talking UDP to an Agent on the PC. The Agent bridges to DDS and
// decides nothing; goals come in, state goes out, and the control loop never
// leaves the board.
//
//   ./uros_node --agent 192.168.0.1
//   ./uros_node --agent 192.168.0.1 --dry-run     torque never enabled
//   ./uros_node --agent 192.168.0.1 --no-camera
//
// Publishes                                  Subscribes
//   /gripper/joint_states  JointState          /gripper/command    String
//   /gripper/status        String              /gripper/motor_cmd  String
//   /gripper/motor_reply   String              /gripper/object     String
//   /gripper/motor_diag    Float32MultiArray   temperature, voltage, hardware
//   /tof/range             Range               error and torque, per joint
//   /gripper/goal          Float32MultiArray   position (rad) and current ceiling
//                                              per joint, then 1 if a goal is active
//
// Commands:
//   a name from kGrasps     choose a grasp and move to its preshape
//   arm | disarm            close as the object comes: the nearer, the further closed
//   grasp                   close now, whatever the TOF says
//   release | open          back to the preshape
//   stop                    hold where it is, issuing no goals
//   calibrate               the current range is where the fingers meet the object,
//                           saved with the object if one is chosen. The next grasp
//                           saves how far it had closed when they did
//   measure                 close an empty hand slowly, saving where the fingers
//                           start to block the TOF for this grasp
//   photo
//   catalog                 every grasp and object, answered on /gripper/status
//   grasp_save, grasp_delete, object_save, object_delete
//                           change what the hand knows, see grasp_store.hpp. Answered
//                           on /gripper/status with "store ok" or "store err", once
//                           the change is on NAND
//
// /gripper/object names what is in front of the hand, and the hand moves to that
// object's preshape, with the distance it was calibrated at. Recognising it
// is not done here; publish the name from whatever does.
//
// /gripper/motor_cmd is for dynamixel_tools, through its board_bridge. See
// motor_debug.hpp. Those requests run between bus transactions on the control
// thread. While any motor has no torque, grasp commands are ignored and that
// motor's setpoint follows it, so enabling torque does not move anything.
//
// If the fingers end up where the collision model says Servo can never get them out
// of, every motor loses torque and /gripper/status says fault=self_collision until
// they have been moved apart by hand. Torque then has to be turned back on.
//
// Four threads, so that nothing slow reaches the motors. Control owns the bus
// and the servo; TOF owns I2C; camera owns V4L2; comm owns every micro-ROS call,
// which are not thread safe in this build. They only meet in Shared.
//
// The camera is deliberately not a topic. A 30 KB JPEG through XRCE-DDS at a
// 512 byte MTU is hundreds of fragments, and it would share the link with the
// goals and the state. Frames go to gripper_sensors/palm_stream over TCP instead,
// and "photo" writes one to a file and reports the path on /gripper/status.

#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#include "camera/v4l2_capture.hpp"
#include "dynamixel/dynamixel_bus.hpp"
#include "grasp_sequencer.hpp"
#include "grasp_store.hpp"
#include "gripper_joints.hpp"
#include "servo/servo.hpp"
#include "motor_debug.hpp"
#include "stream_server.hpp"
#include "tof/tof_i2c.hpp"

using dxl::DynamixelBus;
using dxl::JointFeedback;

namespace
{

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

constexpr int kFlexJoints = 6;
constexpr int kJoints = gripper_servo::kJoints;

double monotonic_now()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

void sleep_until(double deadline)
{
  const double remaining = deadline - monotonic_now();
  if (remaining <= 0.0) { return; }
  struct timespec ts;
  ts.tv_sec = static_cast<time_t>(remaining);
  ts.tv_nsec = static_cast<long>((remaining - static_cast<double>(ts.tv_sec)) * 1e9);
  nanosleep(&ts, nullptr);
}

// ---------------------------------------------------------------------------
// Message storage.
//
// This build sets RCUTILS_AVOID_DYNAMIC_ALLOCATION, so nothing here allocates:
// every string and sequence has to be pointed at storage that already exists,
// with its capacity filled in, before the first publish or the first message
// received. Getting this wrong does not fail loudly -- a publisher writes
// through a null pointer, a subscription takes the first message and dies.
// ---------------------------------------------------------------------------

/// Points a ROS string at a fixed buffer and fills it with `text`.
void bind_string(rosidl_runtime_c__String & s, char * storage, std::size_t capacity,
                 const char * text = "")
{
  s.data = storage;
  s.capacity = capacity;
  const std::size_t len = std::strlen(text);
  s.size = len < capacity - 1 ? len : capacity - 1;
  std::memcpy(storage, text, s.size);
  storage[s.size] = '\0';
}

/// Replaces a bound string's contents, clipped to the capacity it was given.
void set_string(rosidl_runtime_c__String & s, const char * text)
{
  const std::size_t len = std::strlen(text);
  s.size = len < s.capacity - 1 ? len : s.capacity - 1;
  std::memcpy(s.data, text, s.size);
  s.data[s.size] = '\0';
}

char g_command_storage[320];
std_msgs__msg__String g_command_msg;

char g_status_storage[400];
std_msgs__msg__String g_status_msg;

sensor_msgs__msg__JointState g_joint_msg;
char g_joint_frame[32];
rosidl_runtime_c__String g_joint_names[kJoints];
char g_joint_name_storage[kJoints][24];
double g_joint_position[kJoints];
double g_joint_velocity[kJoints];
double g_joint_effort[kJoints];

sensor_msgs__msg__Range g_range_msg;
char g_range_frame[32];

char g_object_storage[64];
std_msgs__msg__String g_object_msg;

char g_motor_cmd_storage[128];
std_msgs__msg__String g_motor_cmd_msg;

char g_motor_reply_storage[128];
std_msgs__msg__String g_motor_reply_msg;

constexpr int kDiagFields = 4;   // temperature, voltage, hardware error, torque
float g_motor_diag_data[kJoints * kDiagFields];
std_msgs__msg__Float32MultiArray g_motor_diag_msg;

constexpr int kGoalFields = kJoints * 2 + 1;   // position, current ceiling, active
float g_goal_data[kGoalFields];
std_msgs__msg__Float32MultiArray g_goal_msg;

/// Commands from the topic. The callback runs on the comm thread, so it only
/// queues them and control acts on them. A queue rather than a flag: `pinch`
/// followed straight away by `arm` must not lose the first.
template <typename T>
class Queue
{
public:
  bool push(const T & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ == kCapacity) { return false; }
    items_[(head_ + count_) % kCapacity] = item;
    ++count_;
    return true;
  }

  bool pop(T & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ == 0) { return false; }
    out = items_[head_];
    head_ = (head_ + 1) % kCapacity;
    --count_;
    return true;
  }

private:
  static constexpr int kCapacity = 8;
  std::mutex mutex_;
  T items_[kCapacity]{};
  int head_{0};
  int count_{0};
};

struct GraspCommand
{
  grasping::Command command{grasping::Command::kStop};
  /// For kSelect. Only control can look it up: the store is its own.
  char name[grasping::kNameSize]{};
};

struct StoreReply
{
  char text[96]{};
};

Queue<GraspCommand> g_commands;
Queue<grasping::StoreRequest> g_store_requests;
Queue<StoreReply> g_store_replies;
Queue<motor_debug::Request> g_motor_requests;
Queue<motor_debug::Reply> g_motor_replies;

/// The latest jog target. Not queued: a held key would fill the queue, and only
/// where it ended up matters.
struct JogTarget
{
  std::mutex mutex;
  bool pending{false};
  float position[kJoints]{};
};
JogTarget g_jog;

/// The latest object named. Control takes it when it is free to act on it.
struct ObjectName
{
  std::mutex mutex;
  bool pending{false};
  char name[24]{};
};
ObjectName g_object;

std::atomic<bool> g_photo_request{false};
std::atomic<bool> g_catalog_request{false};
char g_last_command[sizeof(g_command_storage)] = "";

void on_command(const void * msgin)
{
  using grasping::Command;
  const auto * msg = static_cast<const std_msgs__msg__String *>(msgin);
  const std::size_t n = msg->data.size < sizeof(g_last_command) - 1
                          ? msg->data.size : sizeof(g_last_command) - 1;
  std::memcpy(g_last_command, msg->data.data, n);
  g_last_command[n] = '\0';

  const std::string cmd(g_last_command);
  grasping::StoreRequest request;
  if (grasping::parse_store_command(g_last_command, request)) {
    if (!g_store_requests.push(request)) {
      StoreReply reply;
      std::snprintf(reply.text, sizeof(reply.text), "store err queue full");
      g_store_replies.push(reply);
    }
  }
  else if (cmd == "arm") { g_commands.push({Command::kArm}); }
  else if (cmd == "disarm") { g_commands.push({Command::kDisarm}); }
  else if (cmd == "grasp") { g_commands.push({Command::kGrasp}); }
  else if (cmd == "release" || cmd == "open") { g_commands.push({Command::kRelease}); }
  else if (cmd == "stop") { g_commands.push({Command::kStop}); }
  else if (cmd == "calibrate") { g_commands.push({Command::kCalibrate}); }
  else if (cmd == "measure") { g_commands.push({Command::kMeasure}); }
  else if (cmd == "photo") { g_photo_request = true; }
  else if (cmd == "catalog") { g_catalog_request = true; }
  else {
    GraspCommand select{Command::kSelect};
    std::snprintf(select.name, sizeof(select.name), "%s", g_last_command);
    g_commands.push(select);
  }
  std::printf("<- %s\n", g_last_command);
  std::fflush(stdout);
}

void on_object(const void * msgin)
{
  const auto * msg = static_cast<const std_msgs__msg__String *>(msgin);
  std::lock_guard<std::mutex> lock(g_object.mutex);
  const std::size_t n = msg->data.size < sizeof(g_object.name) - 1
                          ? msg->data.size : sizeof(g_object.name) - 1;
  std::memcpy(g_object.name, msg->data.data, n);
  g_object.name[n] = '\0';
  g_object.pending = true;
}

void on_motor_cmd(const void * msgin)
{
  const auto * msg = static_cast<const std_msgs__msg__String *>(msgin);
  char text[sizeof(g_motor_cmd_storage)];
  const std::size_t n = msg->data.size < sizeof(text) - 1 ? msg->data.size : sizeof(text) - 1;
  std::memcpy(text, msg->data.data, n);
  text[n] = '\0';

  motor_debug::Request request;
  float jog[kJoints];
  const motor_debug::Parsed parsed = motor_debug::parse(text, request, jog);
  if (parsed == motor_debug::Parsed::kJog) {
    std::lock_guard<std::mutex> lock(g_jog.mutex);
    std::memcpy(g_jog.position, jog, sizeof(jog));
    g_jog.pending = true;
    return;
  }

  const bool queued = parsed == motor_debug::Parsed::kRequest && g_motor_requests.push(request);
  if (!queued) {
    motor_debug::Reply reply;
    reply.seq = request.seq;
    std::snprintf(reply.text, sizeof(reply.text), "%s",
                  parsed == motor_debug::Parsed::kBad ? "cannot parse the request" : "queue full");
    g_motor_replies.push(reply);
  }
}

const char * servo_name(gripper_servo::Status status)
{
  switch (status) {
    case gripper_servo::Status::kTracking: return "tracking";
    case gripper_servo::Status::kHolding: return "holding";
    case gripper_servo::Status::kBlocked: return "blocked";
    default: return "idle";
  }
}

void sleep_seconds(double seconds)
{
  sleep_until(monotonic_now() + seconds);
}

struct Options
{
  std::string agent_ip{"192.168.0.1"};
  std::string agent_port{"8888"};
  std::string node_name{"gripper"};
  std::string outdir{"/tmp/grasp"};
  std::string store_path{"/usr/bin/gripper/gripper_store.txt"};
  double rate_hz{30.0};
  bool use_camera{true};
  int stream_port{9000};
  bool dry_run{false};
  int initial_grasp{0};
  grasping::SequencerConfig sequencer;
};

/// Everything the threads exchange. Held only long enough to copy.
struct Shared
{
  std::mutex mutex;

  uint32_t state_seq{0};
  float position[kJoints]{};
  float velocity[kJoints]{};
  float current[kJoints]{};
  grasping::Phase phase{grasping::Phase::kIdle};
  char grasp[grasping::kNameSize]{};
  char object[grasping::kNameSize]{};
  float closure{0.0f};
  float touch_mm{0.0f};
  float start_mm{0.0f};
  float touch_closure{1.0f};
  float blind_closure{0.0f};
  bool goal_active{false};
  float goal_position[kJoints]{};
  float goal_current[kJoints]{};
  char note[64]{};
  grasping::GraspStore store;
  uint32_t store_revision{0};
  const char * torque{"off"};
  int hardware_faults{0};
  bool self_collision{false};
  float temperature[kJoints]{};
  float voltage[kJoints]{};
  float hardware_error[kJoints]{};
  float torque_on[kJoints]{};
  gripper_servo::Status servo{gripper_servo::Status::kIdle};
  bool goal_stale{false};
  int contacts{0};
  float reach{1.0f};
  char stalled[kJoints + 1]{};
  float loop_hz{0.0f};
  uint32_t bus_errors{0};

  uint32_t tof_seq{0};
  tof::Reading tof;
  uint32_t tof_errors{0};

  uint32_t photo_seq{0};
  char photo_text[200]{};
  float capture_fps{0.0f};
  float stream_fps{0.0f};
};

void usage()
{
  std::printf(
    "usage: uros_node [--agent IP] [--port PORT] [--name NAME]\n"
    "                 [--port-dxl PATH] [--baud RATE] [--rate HZ]\n"
    "                 [--bus PATH] [--video PATH] [--video-size WxH] [--video-buffers N]\n"
    "                 [--stream-port PORT] [--outdir DIR] [--store PATH]\n"
    "                 [--grasp NAME] [--contacts N] [--close-seconds S]\n"
    "                 [--touch-mm MM] [--band-mm MM] [--fastest-close-s S]\n"
    "                 [--stall-current F] [--stall-release F]\n"
    "                 [--no-camera] [--dry-run]\n"
    "\n"
    "  --agent      Agent address, default 192.168.0.1. The client connects\n"
    "               out, so this has to be fixed\n"
    "  --rate       control loop, default 30, which comes out at 25 Hz\n"
    "  --video-size MJPG frame size, default 640x480. 1280x720 needs --video-buffers 2\n"
    "  --stream-port  TCP port for gripper_sensors/palm_stream, default 9000, 0 is off\n"
    "  --store      what the hand has been taught, default\n"
    "               /usr/bin/gripper/gripper_store.txt. Has to be on NAND to survive a power cut\n"
    "  --grasp      a name from kGrasps, default tripod. Moves nothing until told\n"
    "  --contacts   flexion joints that must report contact, default 3\n"
    "  --close-seconds     time to close fully, default 4\n"
    "  --touch-mm  what the TOF reads where the fingers meet an uncalibrated object, default 50\n"
    "  --band-mm   start closing this far before that, default 40\n"
    "  --fastest-close-s   a full closure takes at least this long, default 3\n"
    "  --stall-current     fraction of the current ceiling that counts as contact,\n"
    "                      default 0.5\n"
    "  --stall-release     fraction below which contact lets go, default 0.3\n"
    "  --dry-run    run everything, never enable torque. Note this also means\n"
    "               contact can never be detected: no torque, no current\n");
}

// ---------------------------------------------------------------------------
// TOF thread
// ---------------------------------------------------------------------------

void tof_loop(tof::TofI2c & sensor, Shared & shared)
{
  bool up = true;
  while (!g_stop) {
    if (!up) {
      sleep_seconds(1.0);
      up = sensor.open();
      if (up) { std::printf("tof: back\n"); std::fflush(stdout); }
      continue;
    }
    tof::Reading reading;
    bool fresh = false;
    if (!sensor.read(reading, 100, fresh)) {
      std::fprintf(stderr, "tof: %s\n", sensor.last_error().c_str());
      sensor.close();
      up = false;
      std::lock_guard<std::mutex> lock(shared.mutex);
      ++shared.tof_errors;
      continue;
    }
    if (fresh) {
      std::lock_guard<std::mutex> lock(shared.mutex);
      shared.tof = reading;
      ++shared.tof_seq;
    }
  }
}

// ---------------------------------------------------------------------------
// Camera thread
// ---------------------------------------------------------------------------

void camera_loop(camera::V4L2Capture & cam, const Options & opt, Shared & shared)
{
  stream::Server server;
  const bool streaming = opt.stream_port > 0 && server.listen(opt.stream_port);
  if (opt.stream_port > 0 && !streaming) {
    std::fprintf(stderr, "no camera stream, cannot listen on port %d: %s\n",
                 opt.stream_port, std::strerror(errno));
  }

  std::size_t photo_count = 0;
  uint32_t stream_seq = 0;
  int captured_count = 0;
  int sent_count = 0;
  double window_start = monotonic_now();

  while (!g_stop) {
    // Dequeuing continuously is what makes a photo current: left alone the
    // buffers fill at startup and every photo is that old frame.
    camera::Frame frame;
    const bool captured = cam.capture(frame, 200);

    if (streaming) { server.poll_accept(0); }
    if (captured) {
      ++captured_count;
      const uint64_t stamp = static_cast<uint64_t>(frame.timestamp.tv_sec) * 1000000000ULL +
                             static_cast<uint64_t>(frame.timestamp.tv_usec) * 1000ULL;
      if (streaming && server.send_jpeg(stream_seq, stamp,
                                        static_cast<uint16_t>(cam.negotiated_width()),
                                        static_cast<uint16_t>(cam.negotiated_height()),
                                        frame.data, frame.size))
      {
        ++stream_seq;
        ++sent_count;
      }
    }
    const double now = monotonic_now();
    if (now - window_start >= 1.0) {
      std::lock_guard<std::mutex> lock(shared.mutex);
      shared.capture_fps = static_cast<float>(captured_count / (now - window_start));
      shared.stream_fps = static_cast<float>(sent_count / (now - window_start));
      captured_count = 0;
      sent_count = 0;
      window_start = now;
    }

    if (!g_photo_request.exchange(false)) {
      if (!captured) { sleep_seconds(0.2); }
      continue;
    }

    char text[sizeof(shared.photo_text)];
    if (!captured) {
      std::snprintf(text, sizeof(text), "photo failed: %.160s", cam.last_error().c_str());
    } else {
      char path[160];
      std::snprintf(path, sizeof(path), "%.120s/uros_%03zu.jpg",
                    opt.outdir.c_str(), ++photo_count);
      if (std::FILE * f = std::fopen(path, "wb")) {
        std::fwrite(frame.data, 1, frame.size, f);
        std::fclose(f);
        std::snprintf(text, sizeof(text), "photo %s (%zu bytes)", path, frame.size);
      } else {
        std::snprintf(text, sizeof(text), "photo failed: %.160s", std::strerror(errno));
      }
    }
    std::lock_guard<std::mutex> lock(shared.mutex);
    std::memcpy(shared.photo_text, text, sizeof(text));
    ++shared.photo_seq;
  }
}

// ---------------------------------------------------------------------------
// Comm thread
// ---------------------------------------------------------------------------

/// Fills the messages' fixed storage. Done once: it outlives every connection.
void bind_messages()
{
  bind_string(g_command_msg.data, g_command_storage, sizeof(g_command_storage));
  bind_string(g_status_msg.data, g_status_storage, sizeof(g_status_storage));

  bind_string(g_joint_msg.header.frame_id, g_joint_frame, sizeof(g_joint_frame), "palm");
  g_joint_msg.name.data = g_joint_names;
  g_joint_msg.name.size = kJoints;
  g_joint_msg.name.capacity = kJoints;
  for (int i = 0; i < kJoints; ++i) {
    bind_string(g_joint_names[i], g_joint_name_storage[i], sizeof(g_joint_name_storage[i]),
                gripper_joints::kJointDefs[i].name);
  }
  g_joint_msg.position.data = g_joint_position;
  g_joint_msg.position.size = g_joint_msg.position.capacity = kJoints;
  g_joint_msg.velocity.data = g_joint_velocity;
  g_joint_msg.velocity.size = g_joint_msg.velocity.capacity = kJoints;
  g_joint_msg.effort.data = g_joint_effort;
  g_joint_msg.effort.size = g_joint_msg.effort.capacity = kJoints;

  bind_string(g_range_msg.header.frame_id, g_range_frame, sizeof(g_range_frame), "palm_tof");
  g_range_msg.radiation_type = 1;   // INFRARED
  g_range_msg.field_of_view = 0.47f;  // ~27 deg, the VL53L1X's default ROI
  g_range_msg.min_range = 0.04f;
  g_range_msg.max_range = 1.3f;       // short mode

  bind_string(g_object_msg.data, g_object_storage, sizeof(g_object_storage));
  bind_string(g_motor_cmd_msg.data, g_motor_cmd_storage, sizeof(g_motor_cmd_storage));
  bind_string(g_motor_reply_msg.data, g_motor_reply_storage, sizeof(g_motor_reply_storage));
  g_motor_diag_msg.data.data = g_motor_diag_data;
  g_motor_diag_msg.data.size = g_motor_diag_msg.data.capacity = kJoints * kDiagFields;
  g_goal_msg.data.data = g_goal_data;
  g_goal_msg.data.size = g_goal_msg.data.capacity = kGoalFields;
}

/// One session with the Agent, torn down and rebuilt whenever the Agent goes away.
struct Link
{
  rclc_support_t support;
  rcl_node_t node;
  rcl_publisher_t joint_pub, range_pub, status_pub, motor_reply_pub, motor_diag_pub, goal_pub;
  rcl_subscription_t command_sub, object_sub, motor_cmd_sub;
  rclc_executor_t executor;
  bool support_up{false}, node_up{false}, joint_up{false}, range_up{false};
  bool status_up{false}, motor_reply_up{false}, motor_diag_up{false}, goal_up{false};
  bool command_up{false}, object_up{false}, motor_cmd_up{false}, executor_up{false};
};

/// A publish waits for its acknowledgement this long. Reliable QoS is what a
/// plain ROS 2 subscriber matches, and the default of a second is too long even
/// on this thread: a dead link should be noticed within a few publishes.
constexpr int kPublishTimeoutMs = 100;
constexpr int kLostAfterFailures = 3;

void destroy_link(Link & link)
{
  // With the Agent gone each destroy would otherwise wait out its own timeout.
  if (link.support_up) {
    rmw_uros_set_context_entity_destroy_session_timeout(
      rcl_context_get_rmw_context(&link.support.context), 0);
  }
  rcl_ret_t ignored = RCL_RET_OK;
  if (link.executor_up) { ignored += rclc_executor_fini(&link.executor); }
  if (link.motor_cmd_up) { ignored += rcl_subscription_fini(&link.motor_cmd_sub, &link.node); }
  if (link.object_up) { ignored += rcl_subscription_fini(&link.object_sub, &link.node); }
  if (link.command_up) { ignored += rcl_subscription_fini(&link.command_sub, &link.node); }
  if (link.goal_up) { ignored += rcl_publisher_fini(&link.goal_pub, &link.node); }
  if (link.motor_diag_up) { ignored += rcl_publisher_fini(&link.motor_diag_pub, &link.node); }
  if (link.motor_reply_up) { ignored += rcl_publisher_fini(&link.motor_reply_pub, &link.node); }
  if (link.status_up) { ignored += rcl_publisher_fini(&link.status_pub, &link.node); }
  if (link.range_up) { ignored += rcl_publisher_fini(&link.range_pub, &link.node); }
  if (link.joint_up) { ignored += rcl_publisher_fini(&link.joint_pub, &link.node); }
  if (link.node_up) { ignored += rcl_node_fini(&link.node); }
  if (link.support_up) { ignored += rclc_support_fini(&link.support); }
  (void)ignored;
  link = Link{};
}

bool create_publisher(
  rcl_publisher_t & pub, rcl_node_t & node, const rosidl_message_type_support_t * type,
  const char * topic)
{
  pub = rcl_get_zero_initialized_publisher();
  if (rclc_publisher_init_default(&pub, &node, type, topic) != RCL_RET_OK) { return false; }
  rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&pub), kPublishTimeoutMs);
  return true;
}

bool create_link(
  Link & link, const Options & opt, rcl_init_options_t & init_options,
  rcl_allocator_t & allocator)
{
  link = Link{};
  link.support_up = rclc_support_init_with_options(
    &link.support, 0, nullptr, &init_options, &allocator) == RCL_RET_OK;
  if (!link.support_up) { return false; }

  link.node = rcl_get_zero_initialized_node();
  link.node_up = rclc_node_init_default(
    &link.node, opt.node_name.c_str(), "", &link.support) == RCL_RET_OK;
  if (!link.node_up) { return false; }

  // Timestamps that mean something on the PC. Without this the board's clock
  // starts at the epoch and every message looks decades stale.
  if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
    std::fprintf(stderr, "warning: clock not synced with the Agent\n");
  }

  link.joint_up = create_publisher(
    link.joint_pub, link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/gripper/joint_states");
  if (!link.joint_up) { return false; }
  link.range_up = create_publisher(
    link.range_pub, link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "/tof/range");
  if (!link.range_up) { return false; }
  link.status_up = create_publisher(
    link.status_pub, link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/gripper/status");
  if (!link.status_up) { return false; }
  link.motor_reply_up = create_publisher(
    link.motor_reply_pub, link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/gripper/motor_reply");
  if (!link.motor_reply_up) { return false; }
  link.motor_diag_up = create_publisher(
    link.motor_diag_pub, link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/gripper/motor_diag");
  if (!link.motor_diag_up) { return false; }
  link.goal_up = create_publisher(
    link.goal_pub, link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/gripper/goal");
  if (!link.goal_up) { return false; }

  link.command_sub = rcl_get_zero_initialized_subscription();
  link.command_up = rclc_subscription_init_default(
    &link.command_sub, &link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/gripper/command") == RCL_RET_OK;
  if (!link.command_up) { return false; }
  link.object_sub = rcl_get_zero_initialized_subscription();
  link.object_up = rclc_subscription_init_default(
    &link.object_sub, &link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/gripper/object") == RCL_RET_OK;
  if (!link.object_up) { return false; }
  link.motor_cmd_sub = rcl_get_zero_initialized_subscription();
  link.motor_cmd_up = rclc_subscription_init_default(
    &link.motor_cmd_sub, &link.node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/gripper/motor_cmd") == RCL_RET_OK;
  if (!link.motor_cmd_up) { return false; }

  link.executor = rclc_executor_get_zero_initialized_executor();
  link.executor_up = rclc_executor_init(
    &link.executor, &link.support.context, 3, &allocator) == RCL_RET_OK;
  if (!link.executor_up) { return false; }
  return rclc_executor_add_subscription(
           &link.executor, &link.command_sub, &g_command_msg, &on_command,
           ON_NEW_DATA) == RCL_RET_OK &&
         rclc_executor_add_subscription(
           &link.executor, &link.object_sub, &g_object_msg, &on_object,
           ON_NEW_DATA) == RCL_RET_OK &&
         rclc_executor_add_subscription(
           &link.executor, &link.motor_cmd_sub, &g_motor_cmd_msg, &on_motor_cmd,
           ON_NEW_DATA) == RCL_RET_OK;
}

void comm_loop(const Options & opt, Shared & shared)
{
  bind_messages();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
    std::fprintf(stderr, "rcl_init_options_init failed; no ROS 2 this run\n");
    return;
  }
  rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  if (rmw_uros_options_set_udp_address(
      opt.agent_ip.c_str(), opt.agent_port.c_str(), rmw_options) != RMW_RET_OK)
  {
    std::fprintf(stderr, "bad Agent address %s:%s; no ROS 2 this run\n",
                 opt.agent_ip.c_str(), opt.agent_port.c_str());
    return;
  }

  Link link;
  bool connected = false;
  int failures = 0;
  uint32_t state_seen = 0, tof_seen = 0, photo_seen = 0;
  grasping::GraspStore store_copy;
  uint32_t store_seen = 0;
  {
    std::lock_guard<std::mutex> lock(shared.mutex);
    store_copy = shared.store;
    store_seen = shared.store_revision;
  }
  bool store_ok = true;
  int catalog_at = -1;
  bool self_collision_seen = false;
  double next_status = 0.0;
  double next_diag = 0.0;
  double next_goal = 0.0;
  double next_ping = 0.0;

  const auto publish = [&](const rcl_publisher_t & pub, const void * msg) {
    failures = rcl_publish(&pub, msg, nullptr) == RCL_RET_OK ? 0 : failures + 1;
  };

  while (!g_stop) {
    if (!connected) {
      // _options, not the plain ping: with no session the plain one uses the
      // transport's default address and would never send anything here.
      if (rmw_uros_ping_agent_options(200, 1, rmw_options) != RMW_RET_OK) {
        sleep_seconds(0.3);
        continue;
      }
      if (!create_link(link, opt, init_options, allocator)) {
        std::fprintf(stderr, "could not set up the node, retrying\n");
        destroy_link(link);
        sleep_seconds(0.5);
        continue;
      }
      connected = true;
      failures = 0;
      next_ping = monotonic_now() + 1.0;
      std::printf("agent %s:%s connected, node \"%s\" up\n",
                  opt.agent_ip.c_str(), opt.agent_port.c_str(), opt.node_name.c_str());
      std::fflush(stdout);
    }

    rclc_executor_spin_some(&link.executor, RCL_MS_TO_NS(5));

    // Before looking for a change to save: a reply taken here has its change in
    // shared already, so it can say truthfully that it is on NAND.
    StoreReply store_reply;
    const bool have_store_reply = g_store_replies.pop(store_reply);

    std::unique_lock<std::mutex> lock(shared.mutex);
    const bool store_changed = shared.store_revision != store_seen;
    if (store_changed) {
      store_copy = shared.store;
      store_seen = shared.store_revision;
    }
    const bool new_state = shared.state_seq != state_seen;
    const bool new_tof = shared.tof_seq != tof_seen;
    const bool new_photo = shared.photo_seq != photo_seen;
    state_seen = shared.state_seq;
    tof_seen = shared.tof_seq;
    photo_seen = shared.photo_seq;
    const tof::Reading reading = shared.tof;
    for (int i = 0; i < kJoints; ++i) {
      g_joint_position[i] = shared.position[i];
      g_joint_velocity[i] = shared.velocity[i];
      // Dynamixel current in its own units, not newton-metres. JointState has
      // nowhere better to put it.
      g_joint_effort[i] = shared.current[i];
    }
    const double now = monotonic_now();
    // Without waiting for the next one: this is an alarm.
    if (shared.self_collision != self_collision_seen) {
      self_collision_seen = shared.self_collision;
      next_status = 0.0;
    }
    if (now >= next_status) {
      std::snprintf(g_status_storage, sizeof(g_status_storage),
                    "phase=%s object=%s grasp=%s closure=%.2f servo=%s%s contacts=%d reach=%.2f "
                    "stalled=%s torque=%s loop=%.1f cam=%.1f stream=%.1f bus_err=%u "
                    "tof_err=%u hw_err=%d touch=%.0f start=%.0f touch_closure=%.2f blind=%.2f "
                    "rev=%u store=%s fault=%s note=%s",
                    grasping::phase_name(shared.phase),
                    shared.object[0] != '\0' ? shared.object : "-", shared.grasp, shared.closure,
                    servo_name(shared.servo), shared.goal_stale ? "(stale)" : "",
                    shared.contacts, shared.reach, shared.stalled,
                    shared.torque, shared.loop_hz, shared.capture_fps,
                    shared.stream_fps, shared.bus_errors, shared.tof_errors,
                    shared.hardware_faults, shared.touch_mm, shared.start_mm,
                    shared.touch_closure, shared.blind_closure, store_seen,
                    store_ok ? "ok" : "failed",
                    shared.self_collision ? "self_collision" : "none",
                    shared.note);
      g_status_msg.data.size = std::strlen(g_status_storage);
    }
    const bool new_diag = now >= next_diag;
    if (new_diag) {
      for (int i = 0; i < kJoints; ++i) {
        float * row = g_motor_diag_data + i * kDiagFields;
        row[0] = shared.temperature[i];
        row[1] = shared.voltage[i];
        row[2] = shared.hardware_error[i];
        row[3] = shared.torque_on[i];
      }
    }
    const bool new_goal = now >= next_goal;
    if (new_goal) {
      for (int i = 0; i < kJoints; ++i) {
        g_goal_data[i] = shared.goal_position[i];
        g_goal_data[kJoints + i] = shared.goal_current[i];
      }
      g_goal_data[kJoints * 2] = shared.goal_active ? 1.0f : 0.0f;
    }
    char photo_text[sizeof(shared.photo_text)];
    if (new_photo) { std::memcpy(photo_text, shared.photo_text, sizeof(photo_text)); }
    lock.unlock();

    if (store_changed) {
      const double started = monotonic_now();
      store_ok = store_copy.save(opt.store_path.c_str());
      if (store_ok) {
        std::printf("store: saved in %.0f ms\n", (monotonic_now() - started) * 1000.0);
        std::fflush(stdout);
      } else {
        std::fprintf(stderr, "store: cannot write %s: %s\n",
                     opt.store_path.c_str(), std::strerror(errno));
      }
    }
    if (have_store_reply) {
      if (!store_ok) { std::snprintf(store_reply.text, sizeof(store_reply.text),
                                     "store err could not write it to NAND"); }
      set_string(g_status_msg.data, store_reply.text);
      publish(link.status_pub, &g_status_msg);
    }

    const int64_t stamp = rmw_uros_epoch_nanos();
    const int32_t sec = static_cast<int32_t>(stamp / 1000000000LL);
    const uint32_t nsec = static_cast<uint32_t>(stamp % 1000000000LL);

    if (new_state) {
      g_joint_msg.header.stamp.sec = sec;
      g_joint_msg.header.stamp.nanosec = nsec;
      publish(link.joint_pub, &g_joint_msg);
    }
    if (new_tof) {
      g_range_msg.header.stamp.sec = sec;
      g_range_msg.header.stamp.nanosec = nsec;
      // Anything the sensor could not measure becomes +inf rather than a number
      // nobody should act on, which is what sensor_msgs/Range documents.
      g_range_msg.range = reading.valid()
                            ? static_cast<float>(reading.metres())
                            : std::numeric_limits<float>::infinity();
      publish(link.range_pub, &g_range_msg);
    }
    if (now >= next_status) {
      publish(link.status_pub, &g_status_msg);
      next_status = now + 0.2;
    }
    if (new_diag) {
      publish(link.motor_diag_pub, &g_motor_diag_msg);
      next_diag = now + 0.5;
    }
    if (new_goal) {
      publish(link.goal_pub, &g_goal_msg);
      next_goal = now + 0.1;
    }
    motor_debug::Reply reply;
    while (g_motor_replies.pop(reply)) {
      motor_debug::format_reply(reply, g_motor_reply_storage, sizeof(g_motor_reply_storage));
      g_motor_reply_msg.data.size = std::strlen(g_motor_reply_storage);
      publish(link.motor_reply_pub, &g_motor_reply_msg);
    }
    if (!opt.use_camera && g_photo_request.exchange(false)) {
      set_string(g_status_msg.data, "photo: camera disabled");
      publish(link.status_pub, &g_status_msg);
    }
    if (new_photo) {
      set_string(g_status_msg.data, photo_text);
      publish(link.status_pub, &g_status_msg);
    }
    // One row each time round, so that a long catalog does not hold up everything else.
    if (g_catalog_request.exchange(false)) { catalog_at = 0; }
    if (catalog_at >= 0) {
      char text[sizeof(g_status_storage)];
      if (grasping::catalog_row(store_copy, catalog_at, text, sizeof(text))) {
        set_string(g_status_msg.data, text);
        publish(link.status_pub, &g_status_msg);
        ++catalog_at;
      } else {
        catalog_at = -1;
      }
    }

    bool lost = failures >= kLostAfterFailures;
    if (!lost && now >= next_ping) {
      lost = rmw_uros_ping_agent(100, 2) != RMW_RET_OK;
      next_ping = now + 1.0;
    }
    if (lost) {
      std::fprintf(stderr, "agent lost, still controlling locally; reconnecting\n");
      destroy_link(link);
      connected = false;
    }
  }

  if (connected) { destroy_link(link); }
  const rcl_ret_t ignored = rcl_init_options_fini(&init_options);
  (void)ignored;
}

// ---------------------------------------------------------------------------
// Control, on the main thread
// ---------------------------------------------------------------------------

/// One motor velocity unit is 0.024 rad/s; below that this would never trip.
constexpr float kSettleSpeed = 0.05f;

/// What motor debugging needs from the bus and the servo.
class BoardMotors : public motor_debug::Hardware
{
public:
  BoardMotors(
    DynamixelBus & bus, gripper_servo::Servo & servo, const gripper_servo::Feedback & fb,
    float * written_current)
  : bus_(bus), servo_(servo), fb_(fb), written_current_(written_current)
  {
  }

  bool ping(uint8_t id) override
  {
    uint16_t model = 0;
    return bus_.ping(id, model);
  }
  bool reboot(uint8_t id) override { return bus_.reboot(id); }
  bool read(uint8_t id, uint16_t address, uint8_t size, int64_t & value) override
  {
    return bus_.read_register(id, address, size, value);
  }
  bool write(uint8_t id, uint16_t address, uint8_t size, int64_t value) override
  {
    return bus_.write_register(id, address, size, value);
  }
  bool zero(uint8_t id) override { return bus_.set_zero(id); }
  bool restore_ram(int joint) override
  {
    // The motor's Goal Current no longer matches what was last sent to it.
    written_current_[joint] = -1.0f;
    return bus_.restore_ram_parameters(gripper_joints::kJointDefs[joint].id);
  }
  bool hold_here(int joint) override
  {
    servo_.resync(joint, fb_);
    std::vector<double> positions(kJoints, std::numeric_limits<double>::quiet_NaN());
    positions[joint] = static_cast<double>(fb_.position[joint]);
    return bus_.write_positions(positions);
  }
  const char * last_error() override { return bus_.last_error().c_str(); }

private:
  DynamixelBus & bus_;
  gripper_servo::Servo & servo_;
  const gripper_servo::Feedback & fb_;
  float * written_current_;
};

void control_loop(
  const Options & opt, DynamixelBus & bus, std::vector<JointFeedback> & raw,
  gripper_servo::Servo & servo, gripper_servo::Feedback & fb, grasping::GraspStore & store,
  Shared & shared)
{
  grasping::GraspSequencer sequencer(opt.sequencer, store, opt.initial_grasp);
  uint32_t store_revision = store.revision();
  int contacts = 0;
  uint32_t tof_seen = 0;

  const double period = 1.0 / opt.rate_hz;
  float written_current[kJoints];
  for (auto & c : written_current) { c = -1.0f; }

  BoardMotors board_motors(bus, servo, fb, written_current);
  uint8_t motor_ids[kJoints];
  for (int i = 0; i < kJoints; ++i) { motor_ids[i] = gripper_joints::kJointDefs[i].id; }
  motor_debug::MotorDebug motors(board_motors, motor_ids, opt.dry_run);
  bool was_manual = false;
  bool jog_active = false;
  float jog_target[kJoints]{};

  double previous = monotonic_now();
  double window_start = previous;
  int window_cycles = 0;
  float loop_hz = 0.0f;
  uint32_t bus_errors = 0;
  bool bus_ok = true;

  while (!g_stop) {
    const double cycle_start = monotonic_now();

    const bool read_ok = bus.read_all(raw);
    if (read_ok) {
      for (int i = 0; i < kJoints; ++i) {
        fb.position[i] = static_cast<float>(raw[i].position);
        fb.velocity[i] = static_cast<float>(raw[i].velocity);
        fb.current[i] = static_cast<float>(raw[i].current);
        motors.observe_error(i, static_cast<int>(raw[i].error));
      }
      const bool was_colliding = motors.self_collision();
      motors.observe_pose(grasping::pose_trapped(fb.position), cycle_start);
      if (motors.self_collision() != was_colliding) {
        if (motors.self_collision()) {
          std::fprintf(stderr, "self-collision pose: torque off, move the fingers apart by hand\n");
        } else {
          std::printf("self-collision pose cleared: turn torque back on to resume\n");
          std::fflush(stdout);
        }
      }
    }

    // Even when the read failed: a motor that is not answering is when these
    // are wanted. A reboot also fails the read, until the motor is back.
    motor_debug::Reply reply;
    motor_debug::Request request;
    bool answered = motors.poll(cycle_start, reply);
    const bool rebooting = motors.busy();
    if (!answered && !rebooting && g_motor_requests.pop(request)) {
      answered = motors.start(request, read_ok, cycle_start, reply);
    }
    if (answered) { g_motor_replies.push(reply); }

    // A failed read skips the cycle rather than ending the node. The motors keep
    // holding their last goal on their own, and exiting would drop the torque
    // and whatever is in the hand.
    if (!read_ok) {
      if (!rebooting) {
        ++bus_errors;
        if (bus_ok) { std::fprintf(stderr, "read failed: %s\n", bus.last_error().c_str()); }
        bus_ok = false;
      }
      {
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.bus_errors = bus_errors;
      }
      sleep_until(cycle_start + period);
      continue;
    }
    if (!bus_ok) { std::printf("bus: back\n"); std::fflush(stdout); }
    bus_ok = true;

    // Without torque a motor goes wherever it is pushed, so its setpoint has to
    // follow. Grasping needs every finger, so that stops too; the others keep
    // their setpoints and whatever they are gripping.
    const bool manual = !opt.dry_run && !motors.all_on();
    if (manual) {
      if (!was_manual) {
        sequencer.command(grasping::Command::kStop);
        jog_active = false;
      }
      for (int i = 0; i < kJoints; ++i) {
        if (!motors.torque_on(i)) { servo.resync(i, fb); }
      }
    }
    was_manual = manual;

    const double now = monotonic_now();
    // The trajectory limits scale with dt, so a long gap would otherwise let the
    // setpoint leap straight to the goal.
    const double elapsed = now - previous;
    const float dt = static_cast<float>(elapsed < 3.0 * period ? elapsed : 3.0 * period);
    previous = now;

    contacts = 0;
    for (int i = 0; i < kFlexJoints; ++i) {
      if (servo.stalled(i)) { ++contacts; }
    }

    GraspCommand queued;
    while (g_commands.pop(queued)) {
      const grasping::Command command = queued.command;
      if (command == grasping::Command::kMeasure && opt.dry_run) {
        sequencer.set_note("measuring needs the fingers to move, not in a dry run");
        continue;
      }
      const bool moves = command != grasping::Command::kStop &&
                         command != grasping::Command::kDisarm &&
                         command != grasping::Command::kCalibrate;
      if (manual && moves) { continue; }
      int slot = 0;
      if (command == grasping::Command::kSelect) {
        slot = store.find_grasp(queued.name);
        if (slot < 0) {
          char note[64];
          std::snprintf(note, sizeof(note), "unknown command %.40s", queued.name);
          sequencer.set_note(note);
          continue;
        }
      }
      if (command != grasping::Command::kCalibrate) { jog_active = false; }
      sequencer.command(command, slot);
    }

    // One at a time, in the order that lets the reply go out after the change is saved.
    grasping::StoreRequest store_request;
    StoreReply store_reply;
    const bool have_store_request = g_store_requests.pop(store_request);
    if (have_store_request) {
      const char * why = sequencer.apply(store_request, manual);
      std::snprintf(store_reply.text, sizeof(store_reply.text), "store %s %s",
                    why != nullptr ? "err" : "ok", why != nullptr ? why : "saved");
    }
    if (store.revision() != store_revision) {
      store_revision = store.revision();
      std::lock_guard<std::mutex> lock(shared.mutex);
      shared.store = store;
      shared.store_revision = store_revision;
    }
    if (have_store_request) { g_store_replies.push(store_reply); }

    if (!manual) {
      std::lock_guard<std::mutex> lock(g_object.mutex);
      if (g_object.pending) {
        g_object.pending = false;
        sequencer.set_object(g_object.name);
      }
    }

    {
      std::lock_guard<std::mutex> lock(g_jog.mutex);
      if (g_jog.pending) {
        g_jog.pending = false;
        if (sequencer.busy()) {
          sequencer.set_note("jog ignored: release first");
        } else if (!manual) {
          sequencer.command(grasping::Command::kStop);
          sequencer.set_note("jogging");
          std::memcpy(jog_target, g_jog.position, sizeof(jog_target));
          jog_active = true;
        }
      }
    }

    grasping::Tick tick;
    {
      std::lock_guard<std::mutex> lock(shared.mutex);
      tick.tof_fresh = shared.tof_seq != tof_seen;
      tof_seen = shared.tof_seq;
      tick.tof_valid = shared.tof.valid();
      tick.tof_m = static_cast<float>(shared.tof.metres());
    }
    bool still = servo.status() == gripper_servo::Status::kHolding;
    for (int i = 0; i < kJoints; ++i) {
      if (std::fabs(fb.velocity[i]) >= kSettleSpeed) { still = false; }
    }
    tick.now = now;
    tick.dt = dt;
    tick.contacts = contacts;
    tick.blocked = servo.status() == gripper_servo::Status::kBlocked;
    tick.settled = still;
    sequencer.update(tick);
    // An object that has just moved the hand to its preshape ends a jog.
    if (sequencer.has_goal()) { jog_active = false; }

    // Issued every cycle, like the sequencer's: Servo's watchdog would otherwise
    // stop a long move half way.
    gripper_servo::Goal goal{};
    const bool goal_active = jog_active || sequencer.has_goal();
    if (goal_active) {
      if (jog_active) {
        std::memcpy(goal.position, jog_target, sizeof(goal.position));
      } else {
        sequencer.pose(goal.position);
      }
      const float flex_current = sequencer.grasp().flex_current;
      for (int i = 0; i < kJoints; ++i) {
        goal.current_limit[i] = i < kFlexJoints && flex_current > 0.0f
                                  ? flex_current
                                  : static_cast<float>(gripper_joints::kJointDefs[i].goal_current);
      }
      servo.set_goal(goal, now);
    }
    const gripper_servo::Command cmd = servo.step(fb, dt, now);

    if (!opt.dry_run) {
      std::vector<double> currents(kJoints, std::numeric_limits<double>::quiet_NaN());
      bool changed = false;
      for (int i = 0; i < kJoints; ++i) {
        if (cmd.current_limit[i] != written_current[i]) {
          currents[i] = static_cast<double>(cmd.current_limit[i]);
          written_current[i] = cmd.current_limit[i];
          changed = true;
        }
      }
      if (changed) { bus.write_currents(currents); }
      std::vector<double> positions(kJoints);
      for (int i = 0; i < kJoints; ++i) {
        positions[i] = static_cast<double>(cmd.position[i]);
      }
      bus.write_positions(positions);
    }

    ++window_cycles;
    if (now - window_start >= 1.0) {
      loop_hz = static_cast<float>(window_cycles / (now - window_start));
      window_start = now;
      window_cycles = 0;
    }

    {
      std::lock_guard<std::mutex> lock(shared.mutex);
      for (int i = 0; i < kJoints; ++i) {
        shared.position[i] = fb.position[i];
        shared.velocity[i] = fb.velocity[i];
        shared.current[i] = fb.current[i];
        shared.stalled[i] = servo.stalled(i) ? 'X' : '.';
        shared.temperature[i] = static_cast<float>(raw[i].temperature);
        shared.voltage[i] = static_cast<float>(raw[i].voltage);
        shared.hardware_error[i] = static_cast<float>(static_cast<int>(raw[i].error) & 0xFF);
        shared.torque_on[i] = motors.torque_on(i) ? 1.0f : 0.0f;
      }
      shared.stalled[kJoints] = '\0';
      shared.phase = sequencer.phase();
      std::snprintf(shared.grasp, sizeof(shared.grasp), "%s", sequencer.grasp().name);
      std::memcpy(shared.object, sequencer.object(), sizeof(shared.object));
      shared.closure = sequencer.closure();
      shared.touch_mm = sequencer.touch_m() * 1000.0f;
      shared.start_mm = sequencer.start_m() * 1000.0f;
      shared.touch_closure = sequencer.touch_closure();
      shared.blind_closure = sequencer.blind_closure();
      shared.goal_active = goal_active;
      std::memcpy(shared.goal_position, goal.position, sizeof(shared.goal_position));
      std::memcpy(shared.goal_current, goal.current_limit, sizeof(shared.goal_current));
      shared.self_collision = motors.self_collision();
      if (motors.self_collision()) {
        std::snprintf(shared.note, sizeof(shared.note),
                      "self-collision pose, torque off: move the fingers apart by hand");
      } else if (motors.resume_needed()) {
        std::snprintf(shared.note, sizeof(shared.note),
                      "pose clear: turn torque back on to resume");
      } else if (manual) {
        std::snprintf(shared.note, sizeof(shared.note),
                      "%d motors without torque, grasp commands ignored",
                      kJoints - motors.count_on());
      } else {
        std::memcpy(shared.note, sequencer.note(), sizeof(shared.note));
      }
      shared.torque = motors.all_on() ? "on" : (motors.any_on() ? "partial" : "off");
      shared.hardware_faults = motors.faults();
      shared.servo = servo.status();
      shared.goal_stale = servo.goal_stale();
      shared.contacts = contacts;
      shared.reach = servo.reach();
      shared.loop_hz = loop_hz;
      ++shared.state_seq;
    }

    sleep_until(cycle_start + period);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  std::string dxl_port = "/dev/ttyS1";
  int baud = 1000000;
  std::string i2c_bus = "/dev/i2c-2";
  std::string video = "/dev/video0";
  uint32_t video_width = 640;
  uint32_t video_height = 480;
  uint32_t video_buffers = 3;
  float stall_current = 0.5f;
  float stall_release = 0.3f;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--agent" && i + 1 < argc) { opt.agent_ip = argv[++i]; }
    else if (a == "--port" && i + 1 < argc) { opt.agent_port = argv[++i]; }
    else if (a == "--name" && i + 1 < argc) { opt.node_name = argv[++i]; }
    else if (a == "--port-dxl" && i + 1 < argc) { dxl_port = argv[++i]; }
    else if (a == "--baud" && i + 1 < argc) { baud = std::atoi(argv[++i]); }
    else if (a == "--rate" && i + 1 < argc) { opt.rate_hz = std::atof(argv[++i]); }
    else if (a == "--bus" && i + 1 < argc) { i2c_bus = argv[++i]; }
    else if (a == "--video" && i + 1 < argc) { video = argv[++i]; }
    else if (a == "--outdir" && i + 1 < argc) { opt.outdir = argv[++i]; }
    else if (a == "--store" && i + 1 < argc) { opt.store_path = argv[++i]; }
    else if (a == "--video-size" && i + 1 < argc) {
      if (std::sscanf(argv[++i], "%ux%u", &video_width, &video_height) != 2) {
        usage();
        return 1;
      }
    }
    else if (a == "--video-buffers" && i + 1 < argc) {
      video_buffers = static_cast<uint32_t>(std::atoi(argv[++i]));
    }
    else if (a == "--stream-port" && i + 1 < argc) { opt.stream_port = std::atoi(argv[++i]); }
    else if (a == "--grasp" && i + 1 < argc) {
      const std::string name = argv[++i];
      opt.initial_grasp = -1;
      for (int g = 0; g < grasping::kGraspCount; ++g) {
        if (name == grasping::kGrasps[g].name) { opt.initial_grasp = g; }
      }
      if (opt.initial_grasp < 0) { usage(); return 1; }
    }
    else if (a == "--touch-mm" && i + 1 < argc) {
      opt.sequencer.touch_m = std::atof(argv[++i]) / 1000.0f;
    }
    else if (a == "--band-mm" && i + 1 < argc) {
      opt.sequencer.follow_band_m = std::atof(argv[++i]) / 1000.0f;
    }
    else if (a == "--fastest-close-s" && i + 1 < argc) {
      opt.sequencer.fastest_close_s = std::atof(argv[++i]);
    }
    else if (a == "--close-seconds" && i + 1 < argc) {
      opt.sequencer.close_seconds = std::atof(argv[++i]);
    }
    else if (a == "--contacts" && i + 1 < argc) {
      opt.sequencer.contacts_needed = std::atoi(argv[++i]);
    }
    else if (a == "--stall-current" && i + 1 < argc) { stall_current = std::atof(argv[++i]); }
    else if (a == "--stall-release" && i + 1 < argc) { stall_release = std::atof(argv[++i]); }
    else if (a == "--no-camera") { opt.use_camera = false; }
    else if (a == "--dry-run") { opt.dry_run = true; }
    else { usage(); return (a == "-h" || a == "--help") ? 0 : 1; }
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  tof::TofI2c sensor(i2c_bus, 0x29, tof::DistanceMode::Short, 33, 33);
  if (!sensor.open()) {
    std::fprintf(stderr, "tof: %s\n", sensor.last_error().c_str());
    return 1;
  }

  camera::V4L2Capture cam(video);
  if (opt.use_camera) {
    if (::mkdir(opt.outdir.c_str(), 0755) != 0 && errno != EEXIST) {
      std::fprintf(stderr, "cannot create %s: %s\n", opt.outdir.c_str(), std::strerror(errno));
      return 1;
    }
    if (!cam.open() || !cam.start(camera::string_to_fourcc("MJPG"), video_width, video_height,
                                  video_buffers)) {
      std::fprintf(stderr, "camera: %s\n", cam.last_error().c_str());
      return 1;
    }
  }

  DynamixelBus bus(dxl_port, baud);
  if (!bus.open()) {
    std::fprintf(stderr, "motors: %s\n", bus.last_error().c_str());
    return 1;
  }
  if (!bus.configure(gripper_joints::make_joint_specs())) {
    std::fprintf(stderr, "configure failed: %s\n", bus.last_error().c_str());
    return 1;
  }

  std::vector<JointFeedback> raw;
  if (!bus.read_all(raw)) {
    std::fprintf(stderr, "initial read failed: %s\n", bus.last_error().c_str());
    return 1;
  }
  gripper_servo::Feedback fb{};
  for (int i = 0; i < kJoints; ++i) {
    fb.position[i] = static_cast<float>(raw[i].position);
    fb.velocity[i] = static_cast<float>(raw[i].velocity);
    fb.current[i] = static_cast<float>(raw[i].current);
  }

  gripper_servo::Servo servo;
  gripper_servo::Config servo_config;
  servo_config.stall_current = stall_current;
  servo_config.stall_release = stall_release;
  servo.configure(servo_config);
  servo.reset(fb);

  std::printf("motors  %d on %s\ntof     %s\ncamera  %s\n",
              kJoints, dxl_port.c_str(), i2c_bus.c_str(),
              opt.use_camera ? video.c_str() : "(disabled)");

  if (!opt.dry_run && grasping::pose_trapped(fb.position)) {
    std::fprintf(stderr, "self-collision pose: torque stays off, move the fingers apart by hand\n");
  } else if (!opt.dry_run) {
    for (const auto & def : gripper_joints::kJointDefs) {
      if (!bus.set_torque(def.id, true)) {
        std::fprintf(stderr, "torque enable failed on %s: %s\n",
                     def.name, bus.last_error().c_str());
        return 1;
      }
    }
  } else {
    std::printf("dry run: torque stays off, so contact can never be detected\n");
  }
  std::fflush(stdout);

  grasping::GraspStore store;
  if (!store.load(opt.store_path.c_str())) {
    // Kept rather than overwritten by the next save, in case it can be rescued.
    const std::string kept = opt.store_path + ".bad";
    std::rename(opt.store_path.c_str(), kept.c_str());
    std::fprintf(stderr, "%s is not a store, moved to %s\n", opt.store_path.c_str(), kept.c_str());
  }
  std::printf("store   %s, %d objects\n", opt.store_path.c_str(), store.object_count());
  std::fflush(stdout);

  Shared shared;
  shared.store = store;
  shared.store_revision = store.revision();
  std::thread tof_thread(tof_loop, std::ref(sensor), std::ref(shared));
  std::thread comm_thread(comm_loop, std::cref(opt), std::ref(shared));
  std::thread camera_thread;
  if (opt.use_camera) {
    camera_thread = std::thread(camera_loop, std::ref(cam), std::cref(opt), std::ref(shared));
  }

  control_loop(opt, bus, raw, servo, fb, store, shared);

  // Torque first: nothing below may delay letting go of the motors.
  std::printf("\nshutting down.\n");
  if (!opt.dry_run) {
    for (const auto & def : gripper_joints::kJointDefs) { bus.set_torque(def.id, false); }
  }
  bus.close();

  if (camera_thread.joinable()) { camera_thread.join(); }
  comm_thread.join();
  tof_thread.join();
  sensor.close();
  if (opt.use_camera) { cam.stop(); cam.close(); }
  return 0;
}
