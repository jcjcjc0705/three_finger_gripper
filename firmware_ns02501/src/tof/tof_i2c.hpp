// Palm TOF over the board's own I2C. ST's ULD translates the device's raw range
// status into a 0-means-valid form.
//
// Wiring and pinmux for this board are in HARDWARE.md -- EXT_SDA/EXT_SCL on
// the expansion header are I2C3_1, which is /dev/i2c-2.
#ifndef TOF__TOF_I2C_HPP_
#define TOF__TOF_I2C_HPP_

#include <cstdint>
#include <string>

namespace tof
{

/// Anything else means the sensor could not make the measurement: too far, too
/// reflective, ambient light. uros_node publishes these as +inf rather than
/// passing a distance nobody should act on.
constexpr int kValidStatus = 0;

struct Reading
{
  int distance_mm{0};
  int status{-1};
  bool valid() const { return status == kValidStatus; }
  double metres() const { return distance_mm / 1000.0; }
};

/// ST's two presets. Short is good to ~1.3 m but much less bothered by
/// ambient light, which is the tradeoff that matters for a palm sensor
/// looking at something a few centimetres away.
enum class DistanceMode
{
  Short = 1,
  Long = 2,
};

class TofI2c
{
public:
  /// `address7` is the 7-bit address i2cdetect prints (0x29), not the 0x52
  /// the ULD uses internally.
  ///
  /// Timing budget and inter-measurement period both default to 33 ms, about 30 Hz.
  explicit TofI2c(std::string device = "/dev/i2c-2", uint8_t address7 = 0x29,
                  DistanceMode mode = DistanceMode::Long,
                  int timing_budget_ms = 33, int inter_measurement_ms = 33);
  ~TofI2c();

  TofI2c(const TofI2c &) = delete;
  TofI2c & operator=(const TofI2c &) = delete;

  /// Opens the bus, waits for the sensor to boot, runs the ULD's
  /// initialisation and starts continuous ranging.
  bool open();
  void close();

  /// Waits up to timeout_ms for one fresh measurement. Returns false on an
  /// I2C error; a timeout yields true with `fresh` false, because a sensor
  /// that is merely slow is a different fault from one that is not answering.
  bool read(Reading & out, int timeout_ms, bool & fresh);

  /// Diagnostics from the most recent successful read. When status is
  /// non-zero these are what say why: little signal and high ambient is a
  /// dark or distant target, plenty of signal is usually a wrap-around.
  uint16_t ambient_kcps() const { return ambient_kcps_; }
  uint16_t signal_per_spad_kcps() const { return signal_per_spad_kcps_; }
  uint16_t spad_count() const { return spad_count_; }

  const std::string & last_error() const { return last_error_; }

private:
  std::string device_;
  uint16_t dev_;  ///< 8-bit address, the form the ULD passes around
  DistanceMode mode_;
  int timing_budget_ms_;
  int inter_measurement_ms_;
  int fd_{-1};
  bool ranging_{false};
  uint16_t ambient_kcps_{0};
  uint16_t signal_per_spad_kcps_{0};
  uint16_t spad_count_{0};
  std::string last_error_;
};

}  // namespace tof

#endif  // TOF__TOF_I2C_HPP_
