// micro-ROS bring-up: prove the link to the Agent before anything touches
// hardware.
//
// Subscribes to a command topic and prints what arrives; publishes a counter
// so the other direction is visible too. No motors, no TOF, no camera --
// when this works, everything after it is ordinary work, and when it does
// not, the fault is the network, the Agent or the build, never the gripper.
//
//   ./uros_hello --agent 192.168.0.1
//   ./uros_hello --agent 192.168.0.1 --port 8888 --name gripper
//
// On the PC, with the Agent running:
//   docker run --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v4
//   ros2 topic pub -1 /gripper/command std_msgs/msg/String "{data: hello}"
//   ros2 topic echo /gripper/heartbeat

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

namespace
{

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

double monotonic_now()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

/// Storage for the incoming string. micro-ROS is built with
/// RCUTILS_AVOID_DYNAMIC_ALLOCATION, so a subscription never allocates for
/// its message: the buffer has to exist before the first message arrives, and
/// a message longer than this is dropped rather than growing it. Forgetting
/// this is the classic micro-ROS crash -- the subscription registers fine and
/// then writes into a null pointer on the first publish.
char g_command_storage[256];
std_msgs__msg__String g_command_msg;

std_msgs__msg__Int32 g_heartbeat_msg;
std::size_t g_received = 0;

void on_command(const void * msgin)
{
  const auto * msg = static_cast<const std_msgs__msg__String *>(msgin);
  ++g_received;
  // .data is not guaranteed null-terminated, so the length has to come from
  // .size rather than from strlen.
  std::printf("[%6zu] command: \"%.*s\"  (%u bytes)\n", g_received,
              static_cast<int>(msg->data.size), msg->data.data,
              static_cast<unsigned>(msg->data.size));
  std::fflush(stdout);
}

void usage()
{
  std::printf(
    "usage: uros_hello [--agent IP] [--port PORT] [--name NAME] [--seconds N]\n"
    "\n"
    "  --agent    the Agent's address, default 192.168.0.1 (the PC this board\n"
    "             is cabled to). The client connects out; the Agent never\n"
    "             connects in, so this address has to be fixed\n"
    "  --port     default 8888\n"
    "  --name     node name, default gripper_hello\n"
    "  --seconds  run for this long, default 0 (until Ctrl-C)\n");
}

}  // namespace

/// Anything that fails during setup is fatal and worth naming: a silent
/// failure here looks identical to a network problem.
#define CHECK(call, what)                                                   \
  do {                                                                      \
    const rcl_ret_t rc_ = (call);                                           \
    if (rc_ != RCL_RET_OK) {                                                \
      std::fprintf(stderr, "%s failed (rcl_ret=%d)\n", (what), int(rc_));   \
      return 1;                                                             \
    }                                                                       \
  } while (0)

int main(int argc, char ** argv)
{
  std::string agent_ip = "192.168.0.1";
  std::string agent_port = "8888";
  std::string node_name = "gripper_hello";
  double seconds = 0.0;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--agent" && i + 1 < argc) { agent_ip = argv[++i]; }
    else if (a == "--port" && i + 1 < argc) { agent_port = argv[++i]; }
    else if (a == "--name" && i + 1 < argc) { node_name = argv[++i]; }
    else if (a == "--seconds" && i + 1 < argc) { seconds = std::atof(argv[++i]); }
    else { usage(); return (a == "-h" || a == "--help") ? 0 : 1; }
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  CHECK(rcl_init_options_init(&init_options, allocator), "rcl_init_options_init");

  rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  CHECK(rmw_uros_options_set_udp_address(agent_ip.c_str(), agent_port.c_str(), rmw_options),
        "rmw_uros_options_set_udp_address");

  // Ping first. rclc_support_init_with_options() against an Agent that is not
  // there fails in a way that says nothing useful, so ask plainly and say what
  // is wrong while there is still something to say.
  //
  // _options is the whole point: plain rmw_uros_ping_agent() pings the
  // transport's *default* address and cannot see the one just set on
  // rmw_options. Called that way it never puts a packet on the wire, and the
  // failure is indistinguishable from an Agent that is not running -- tcpdump
  // on the Agent's side shows nothing at all, which sends you hunting the
  // network instead of this line.
  std::printf("looking for the Agent at %s:%s ...\n", agent_ip.c_str(), agent_port.c_str());
  bool found = false;
  for (int attempt = 0; attempt < 30 && !g_stop; ++attempt) {
    if (rmw_uros_ping_agent_options(200, 1, rmw_options) == RMW_RET_OK) {
      found = true;
      break;
    }
  }
  if (!found) {
    std::fprintf(stderr,
                 "no Agent answered at %s:%s\n"
                 "  - is it running?  docker run --rm --net=host \\\n"
                 "      microros/micro-ros-agent:jazzy udp4 --port %s -v4\n"
                 "  - can this board reach it?  ping %s\n",
                 agent_ip.c_str(), agent_port.c_str(), agent_port.c_str(), agent_ip.c_str());
    return 1;
  }
  std::printf("Agent found.\n");

  rclc_support_t support;
  CHECK(rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator),
        "rclc_support_init_with_options");

  rcl_node_t node;
  CHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support),
        "rclc_node_init_default");

  rcl_subscription_t command_sub;
  CHECK(rclc_subscription_init_default(
          &command_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/gripper/command"),
        "subscription /gripper/command");

  rcl_publisher_t heartbeat_pub;
  CHECK(rclc_publisher_init_default(
          &heartbeat_pub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/gripper/heartbeat"),
        "publisher /gripper/heartbeat");

  g_command_msg.data.data = g_command_storage;
  g_command_msg.data.size = 0;
  g_command_msg.data.capacity = sizeof(g_command_storage);
  g_heartbeat_msg.data = 0;

  rclc_executor_t executor;
  CHECK(rclc_executor_init(&executor, &support.context, 1, &allocator),
        "rclc_executor_init");
  CHECK(rclc_executor_add_subscription(&executor, &command_sub, &g_command_msg,
                                       &on_command, ON_NEW_DATA),
        "rclc_executor_add_subscription");

  std::printf("node \"%s\" up.\n"
              "  subscribing  /gripper/command    std_msgs/String\n"
              "  publishing   /gripper/heartbeat  std_msgs/Int32 at 1 Hz\n"
              "Ctrl-C to stop.\n\n",
              node_name.c_str());

  const double started = monotonic_now();
  double next_beat = started;

  while (!g_stop && (seconds <= 0.0 || monotonic_now() - started < seconds)) {
    // Bounded, not blocking: the real node will run this inside a 30 Hz
    // control loop, where an executor that waits is an executor that makes
    // the fingers stutter.
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));

    const double now = monotonic_now();
    if (now >= next_beat) {
      const rcl_ret_t rc = rcl_publish(&heartbeat_pub, &g_heartbeat_msg, nullptr);
      if (rc != RCL_RET_OK) {
        std::fprintf(stderr, "publish failed (rcl_ret=%d) -- Agent gone?\n", int(rc));
      }
      ++g_heartbeat_msg.data;
      next_beat = now + 1.0;
    }
  }

  std::printf("\nshutting down. %zu command(s) received, %d heartbeat(s) sent.\n",
              g_received, g_heartbeat_msg.data);

  // These are marked warn_unused_result and a (void) cast does not satisfy
  // GCC, so the results are collected. Nothing is done with them: the process
  // is exiting and a failed teardown leaves nothing to salvage -- but a
  // non-zero total is worth seeing if this ever starts hanging on exit.
  rcl_ret_t fini = RCL_RET_OK;
  fini += rcl_subscription_fini(&command_sub, &node);
  fini += rcl_publisher_fini(&heartbeat_pub, &node);
  fini += rcl_node_fini(&node);
  fini += rclc_executor_fini(&executor);
  fini += rclc_support_fini(&support);
  if (fini != RCL_RET_OK) {
    std::fprintf(stderr, "(teardown reported %d, ignored)\n", int(fini));
  }
  return 0;
}
