// Generated from the gripper's collision model.
//
// How far the hand can close, in a straight line, at each spread angle.
#ifndef REACH_TABLE_HPP_
#define REACH_TABLE_HPP_

namespace reach_table
{

/// Flexion joint lower limits in joint order; a closure fraction scales them.
inline constexpr float kFlexionLower[6] = {-0.2217f, -2.0944f, -0.4992f, -2.0944f, -0.4992f, -2.0944f};

struct Sample
{
  float spread_deg;
  float closure;
};

inline constexpr int kSamples = 37;

/// Safe closure fraction, already backed off 0.03 from the measured edge.
inline constexpr Sample kTable[kSamples] = {
  {-90.0f, 0.960f},
  {-85.0f, 0.960f},
  {-80.0f, 0.960f},
  {-75.0f, 0.960f},
  {-70.0f, 0.960f},
  {-65.0f, 0.960f},
  {-60.0f, 0.960f},
  {-55.0f, 0.960f},
  {-50.0f, 0.960f},
  {-45.0f, 0.960f},
  {-40.0f, 0.960f},
  {-35.0f, 0.960f},
  {-30.0f, 0.960f},
  {-25.0f, 0.960f},
  {-20.0f, 0.510f},
  {-15.0f, 0.440f},
  {-10.0f, 0.410f},
  {-5.0f, 0.390f},
  {0.0f, 0.370f},
  {5.0f, 0.350f},
  {10.0f, 0.340f},
  {15.0f, 0.330f},
  {20.0f, 0.320f},
  {25.0f, 0.310f},
  {30.0f, 0.300f},
  {35.0f, 0.290f},
  {40.0f, 0.290f},
  {45.0f, 0.280f},
  {50.0f, 0.280f},
  {55.0f, 0.280f},
  {60.0f, 0.280f},
  {65.0f, 0.290f},
  {70.0f, 0.290f},
  {75.0f, 0.290f},
  {80.0f, 0.290f},
  {85.0f, 0.300f},
  {90.0f, 0.300f},
};

/// The smaller of the two bracketing samples, not an interpolation: the limit
/// has a cliff in it, and interpolating across that hands back a value the hand
/// cannot reach.
inline float max_closure(float spread_deg)
{
  if (spread_deg <= kTable[0].spread_deg) { return kTable[0].closure; }
  if (spread_deg >= kTable[kSamples - 1].spread_deg) { return kTable[kSamples - 1].closure; }
  for (int i = 0; i + 1 < kSamples; ++i) {
    if (spread_deg >= kTable[i].spread_deg && spread_deg <= kTable[i + 1].spread_deg) {
      return kTable[i].closure < kTable[i + 1].closure ? kTable[i].closure : kTable[i + 1].closure;
    }
  }
  return 0.0f;
}

}  // namespace reach_table

#endif  // REACH_TABLE_HPP_
