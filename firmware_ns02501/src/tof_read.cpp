// TOF bring-up tool for the ns02501/NT98532 board: the VL53L1X directly on
// /dev/i2c-2, through ST's ULD.
//
//   ./tof_read                          five seconds of readings
//   ./tof_read --short                  short mode, what uros_node runs
//   ./tof_read --seconds 0              until Ctrl-C
//
// Reports the frame rate as well as the values, because a TOF that answers but
// at 3 Hz instead of 30 is a different problem from one that does not answer.

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include "tof/tof_i2c.hpp"

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

void usage()
{
  std::printf(
    "usage: tof_read [--bus PATH] [--addr 0x29] [--short] [--budget MS]\n"
    "                [--seconds N] [--quiet]\n"
    "\n"
    "  --bus      default /dev/i2c-2, which is EXT_SDA/EXT_SCL on this board.\n"
    "             Needs the i2c pinmux set -- see HARDWARE.md\n"
    "  --addr     7-bit, default 0x29, as i2cdetect prints it\n"
    "  --short    short distance mode: ~1.3 m ceiling, far better in ambient\n"
    "  --budget   timing budget in ms, default 33 (~30 Hz)\n"
    "  --seconds  how long to read, default 5. 0 runs until Ctrl-C\n"
    "  --quiet    print only the summary, not every reading\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  double seconds = 5.0;
  bool quiet = false;
  std::string bus = "/dev/i2c-2";
  int address = 0x29;
  bool short_mode = false;
  int budget_ms = 33;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--seconds" && i + 1 < argc) { seconds = std::atof(argv[++i]); }
    else if (arg == "--quiet") { quiet = true; }
    else if (arg == "--bus" && i + 1 < argc) { bus = argv[++i]; }
    else if (arg == "--addr" && i + 1 < argc) {
      address = static_cast<int>(std::strtol(argv[++i], nullptr, 0));
    }
    else if (arg == "--short") { short_mode = true; }
    else if (arg == "--budget" && i + 1 < argc) { budget_ms = std::atoi(argv[++i]); }
    else {
      usage();
      return (arg == "-h" || arg == "--help") ? 0 : 1;
    }
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::size_t total = 0;
  std::size_t valid = 0;
  int min_mm = 0;
  int max_mm = 0;
  int last_mm = 0;
  int last_status = -1;

  const double start = monotonic_now();
  const double deadline = start + seconds;
  auto keep_going = [&] {
    return !g_stop && (seconds <= 0.0 || monotonic_now() < deadline);
  };

  int rc = 0;

  tof::TofI2c sensor(bus, static_cast<uint8_t>(address),
                     short_mode ? tof::DistanceMode::Short : tof::DistanceMode::Long,
                     budget_ms, budget_ms);
  if (!sensor.open()) {
    std::fprintf(stderr, "%s\n", sensor.last_error().c_str());
    std::fprintf(stderr,
                 "(is the i2c pinmux set? `echo i2c 0x10000 > "
                 "/proc/nvt_info/nvt_pinmux/pinmux_set`, then check\n"
                 " `i2cdetect -y -r 2 0x28 0x2a` still shows 29 -- see HARDWARE.md)\n");
    return 1;
  }
  std::printf("reading %s addr 0x%02x, %s mode, %d ms budget%s\n",
              bus.c_str(), address, short_mode ? "short" : "long", budget_ms,
              seconds > 0 ? "" : " (Ctrl-C to stop)");

  while (keep_going()) {
    tof::Reading reading;
    bool fresh = false;
    if (!sensor.read(reading, 200, fresh)) {
      std::fprintf(stderr, "%s\n", sensor.last_error().c_str());
      rc = 1;
      break;
    }
    if (!fresh) { continue; }  // slower than expected, not an error

    if (total == 0) {
      min_mm = max_mm = reading.distance_mm;
    } else {
      if (reading.distance_mm < min_mm) { min_mm = reading.distance_mm; }
      if (reading.distance_mm > max_mm) { max_mm = reading.distance_mm; }
    }
    ++total;
    if (reading.valid()) { ++valid; }
    last_mm = reading.distance_mm;
    last_status = reading.status;

    // The diagnostics are what distinguish the ways a reading goes bad.
    if (!quiet) {
      std::printf("%6d mm  status=%d%s  signal=%u amb=%u spads=%u\n",
                  reading.distance_mm, reading.status,
                  reading.valid() ? "" : "  (INVALID -- sensor could not measure)",
                  sensor.signal_per_spad_kcps(), sensor.ambient_kcps(), sensor.spad_count());
    }
  }

  const double elapsed = monotonic_now() - start;

  std::printf("\n--- summary ---\n");
  std::printf("  elapsed     %.2f s\n", elapsed);
  std::printf("  readings    %zu  (%.1f Hz, expect ~30)\n",
              total, elapsed > 0 ? total / elapsed : 0.0);
  std::printf("  valid       %zu  invalid %zu\n", valid, total - valid);
  if (total > 0) {
    std::printf("  distance    %d..%d mm, last %d mm (status %d)\n",
                min_mm, max_mm, last_mm, last_status);
  }

  if (rc != 0) { return rc; }
  if (total == 0) {
    std::printf("\nNo measurements. The sensor booted and configured, so it is\n"
                "there and the bus works -- it is ranging that is not finishing.\n");
    return 1;
  }
  return 0;
}
