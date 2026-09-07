#include "gripper_servo/collision.hpp"

#include <cmath>

namespace gripper_servo
{
namespace
{

constexpr int kMarchSteps = 24;      ///< samples along the segment
constexpr int kBisectionSteps = 5;   ///< refines one march interval

void multiply(const float * a, const float * b, float * out)
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      float v = (c == 3) ? a[r * 4 + 3] : 0.0f;
      for (int k = 0; k < 3; ++k) {
        v += a[r * 4 + k] * b[k * 4 + c];
      }
      out[r * 4 + c] = v;
    }
  }
}

/// Rotation of `angle` about `axis`, as a 3x4 with zero translation.
void rotation(const float * axis, float angle, float * out)
{
  const float c = std::cos(angle), s = std::sin(angle), t = 1.0f - c;
  const float x = axis[0], y = axis[1], z = axis[2];
  out[0] = t * x * x + c;      out[1] = t * x * y - s * z;  out[2] = t * x * z + s * y;
  out[4] = t * x * y + s * z;  out[5] = t * y * y + c;      out[6] = t * y * z - s * x;
  out[8] = t * x * z - s * y;  out[9] = t * y * z + s * x;  out[10] = t * z * z + c;
  out[3] = out[7] = out[11] = 0.0f;
}

void apply(const Transform & t, const float * p, float * out)
{
  for (int r = 0; r < 3; ++r) {
    out[r] = t.m[r * 4 + 0] * p[0] + t.m[r * 4 + 1] * p[1] +
             t.m[r * 4 + 2] * p[2] + t.m[r * 4 + 3];
  }
}

float dot(const float * a, const float * b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float clamp(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

/// Squared distance between segments p1q1 and p2q2 (Ericson, Real-Time
/// Collision Detection). Squared so the hot path has no sqrt.
float segment_distance_sq(const float * p1, const float * q1,
                          const float * p2, const float * q2)
{
  float d1[3], d2[3], r[3];
  for (int i = 0; i < 3; ++i) {
    d1[i] = q1[i] - p1[i];
    d2[i] = q2[i] - p2[i];
    r[i] = p1[i] - p2[i];
  }
  const float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  float s = 0.0f, t = 0.0f;

  if (a <= 1e-12f && e <= 1e-12f) {
    return dot(r, r);
  }
  if (a <= 1e-12f) {
    t = clamp(f / e, 0.0f, 1.0f);
  } else {
    const float c = dot(d1, r);
    if (e <= 1e-12f) {
      s = clamp(-c / a, 0.0f, 1.0f);
    } else {
      const float b = dot(d1, d2);
      const float denom = a * e - b * b;
      s = (denom > 1e-12f) ? clamp((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
      t = (b * s + f) / e;
      if (t < 0.0f) {
        t = 0.0f;
        s = clamp(-c / a, 0.0f, 1.0f);
      } else if (t > 1.0f) {
        t = 1.0f;
        s = clamp((b - c) / a, 0.0f, 1.0f);
      }
    }
  }

  float diff[3];
  for (int i = 0; i < 3; ++i) {
    diff[i] = r[i] + s * d1[i] - t * d2[i];
  }
  return dot(diff, diff);
}

}  // namespace

void forward_kinematics(const float * joints, Transform * out_links)
{
  for (int i = 0; i < 12; ++i) {
    out_links[0].m[i] = 0.0f;
  }
  out_links[0].m[0] = out_links[0].m[5] = out_links[0].m[10] = 1.0f;

  for (int n = 1; n < kLinks; ++n) {
    const int link = kFkOrder[n];
    const JointModel & j = kJointModel[link - 1];
    float rot[12], local[12];
    rotation(j.axis, joints[link - 1], rot);
    multiply(j.origin, rot, local);
    multiply(out_links[j.parent].m, local, out_links[link].m);
  }
}

bool feasible(const float * joints)
{
  Transform links[kLinks];
  forward_kinematics(joints, links);

  float a[kCapsuleCount][3], b[kCapsuleCount][3];
  for (int i = 0; i < kCapsuleCount; ++i) {
    apply(links[kCapsules[i].link], kCapsules[i].a, a[i]);
    apply(links[kCapsules[i].link], kCapsules[i].b, b[i]);
  }

  for (int i = 0; i < kCapsuleCount; ++i) {
    for (int k = i + 1; k < kCapsuleCount; ++k) {
      if (!kMonitored[kCapsules[i].link][kCapsules[k].link]) { continue; }
      const float reach = kCapsules[i].radius + kCapsules[k].radius;
      if (segment_distance_sq(a[i], b[i], a[k], b[k]) < reach * reach) {
        return false;
      }
    }
  }
  return true;
}

float advance_feasible(const float * from, const float * to, float * out)
{
  // The whole segment has to be clear, not just its end. Curling the fingers
  // proportionally from a tripod spread is feasible at 0-30% and again at 100%,
  // but collides in between: testing only the goal would walk the setpoint
  // straight through that band.
  float lo = 0.0f;
  float hi = -1.0f;
  float probe[kJoints];

  for (int step = 1; step <= kMarchSteps; ++step) {
    const float f = static_cast<float>(step) / kMarchSteps;
    for (int i = 0; i < kJoints; ++i) {
      probe[i] = from[i] + f * (to[i] - from[i]);
    }
    if (!feasible(probe)) {
      hi = f;
      break;
    }
    lo = f;
  }

  if (hi < 0.0f) {
    for (int i = 0; i < kJoints; ++i) { out[i] = to[i]; }
    return 1.0f;
  }

  for (int step = 0; step < kBisectionSteps; ++step) {
    const float mid = 0.5f * (lo + hi);
    for (int i = 0; i < kJoints; ++i) {
      probe[i] = from[i] + mid * (to[i] - from[i]);
    }
    if (feasible(probe)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  for (int i = 0; i < kJoints; ++i) {
    out[i] = from[i] + lo * (to[i] - from[i]);
  }
  return lo;
}

}  // namespace gripper_servo
