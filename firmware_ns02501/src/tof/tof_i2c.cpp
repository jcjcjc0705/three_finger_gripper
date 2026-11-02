#include "tof/tof_i2c.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <utility>

// Not wrapped in extern "C": the ULD header carries no linkage guards and its
// .cpp is built as C++ here, so the names must mangle the same way on both
// sides. (vl53l1_platform.h, included through it, has its own extern "C" and
// matches the platform implementation.)
#include "core/VL53L1X_api.h"

namespace tof
{
namespace
{

double monotonic_ms()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) * 1e3 + static_cast<double>(ts.tv_nsec) / 1e6;
}

void sleep_ms(int ms)
{
  struct timespec ts;
  ts.tv_sec = ms / 1000;
  ts.tv_nsec = static_cast<long>(ms % 1000) * 1000000L;
  nanosleep(&ts, nullptr);
}

/// The datasheet allows ~1.2 ms from power-on; a second is generous enough
/// that anything longer means the sensor is not really there.
constexpr double kBootTimeoutMs = 1000.0;

}  // namespace

TofI2c::TofI2c(std::string device, uint8_t address7, DistanceMode mode,
               int timing_budget_ms, int inter_measurement_ms)
: device_(std::move(device)),
  dev_(static_cast<uint16_t>(address7) << 1),
  mode_(mode),
  timing_budget_ms_(timing_budget_ms),
  inter_measurement_ms_(inter_measurement_ms)
{
}

TofI2c::~TofI2c() { close(); }

bool TofI2c::open()
{
  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    last_error_ = "cannot open " + device_ + ": " + std::strerror(errno);
    return false;
  }
  VL53L1_SetI2CFd(fd_);

  // Wait for the boot flag before touching anything else. VL53L1X_SensorInit()
  // busy-waits on data-ready with no timeout of its own, so calling it against
  // a sensor that never answers would hang the process outright -- this check
  // is what keeps that from happening.
  const double deadline = monotonic_ms() + kBootTimeoutMs;
  uint8_t booted = 0;
  while (booted == 0) {
    if (VL53L1X_BootState(dev_, &booted) != 0) {
      last_error_ = "no answer from the sensor on " + device_ +
                    " (does i2cdetect still show 0x29? is the pinmux still set?)";
      close();
      return false;
    }
    if (booted != 0) { break; }
    if (monotonic_ms() > deadline) {
      last_error_ = "sensor on " + device_ + " acknowledges but never reports booted";
      close();
      return false;
    }
    sleep_ms(2);
  }

  if (VL53L1X_SensorInit(dev_) != 0) {
    last_error_ = "VL53L1X_SensorInit failed on " + device_;
    close();
    return false;
  }
  if (VL53L1X_SetDistanceMode(dev_, static_cast<uint16_t>(mode_)) != 0 ||
      VL53L1X_SetTimingBudgetInMs(dev_, static_cast<uint16_t>(timing_budget_ms_)) != 0 ||
      VL53L1X_SetInterMeasurementInMs(dev_, static_cast<uint32_t>(inter_measurement_ms_)) != 0)
  {
    last_error_ = "failed to configure the sensor on " + device_;
    close();
    return false;
  }
  if (VL53L1X_StartRanging(dev_) != 0) {
    last_error_ = "VL53L1X_StartRanging failed on " + device_;
    close();
    return false;
  }
  ranging_ = true;
  return true;
}

void TofI2c::close()
{
  if (fd_ >= 0) {
    // Leaving it ranging would keep the laser firing after we exit, and the
    // next run would then find it mid-measurement.
    if (ranging_) { VL53L1X_StopRanging(dev_); }
    VL53L1_SetI2CFd(-1);
    ::close(fd_);
    fd_ = -1;
  }
  ranging_ = false;
}

bool TofI2c::read(Reading & out, int timeout_ms, bool & fresh)
{
  fresh = false;
  if (fd_ < 0) {
    last_error_ = "bus not open";
    return false;
  }

  const double deadline = monotonic_ms() + timeout_ms;
  uint8_t ready = 0;
  while (true) {
    if (VL53L1X_CheckForDataReady(dev_, &ready) != 0) {
      last_error_ = "I2C error polling data-ready on " + device_;
      return false;
    }
    if (ready != 0) { break; }
    if (monotonic_ms() >= deadline) { return true; }  // slow, not broken
    sleep_ms(1);
  }

  // One 17-byte read for status, distance and the diagnostics, rather than
  // three separate transactions that could straddle two measurements.
  VL53L1X_Result_t result{};
  if (VL53L1X_GetResult(dev_, &result) != 0) {
    last_error_ = "I2C error reading the result on " + device_;
    return false;
  }
  if (VL53L1X_ClearInterrupt(dev_) != 0) {
    last_error_ = "I2C error clearing the interrupt on " + device_;
    return false;
  }

  out.distance_mm = static_cast<int>(result.Distance);
  out.status = static_cast<int>(result.Status);
  ambient_kcps_ = result.Ambient;
  signal_per_spad_kcps_ = result.SigPerSPAD;
  spad_count_ = result.NumSPADs;
  fresh = true;
  return true;
}

}  // namespace tof
