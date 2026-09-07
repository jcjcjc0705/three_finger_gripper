// Self-collision check over the generated capsule model.
//
// No ROS, no allocation, no exceptions -- this compiles into the NT98532
// firmware unchanged.
#pragma once

#include "gripper_servo/gripper_model.hpp"

namespace gripper_servo
{

/// Row-major 3x4 rigid transform.
struct Transform
{
  float m[12];
};

/// Forward kinematics for every link, in the order kFkOrder resolves.
void forward_kinematics(const float * joints, Transform * out_links);

/// True when no monitored capsule pair overlaps.
bool feasible(const float * joints);

/// Furthest point along `from` -> `to` whose whole prefix is feasible, written
/// to `out`. Returns the fraction reached: 1 means the entire segment is clear,
/// 0 means it cannot move at all.
///
/// Feasibility is not monotonic along a straight line in joint space, so the
/// segment is marched rather than bisected from the endpoint.
///
/// Advancing the target rather than rejecting it keeps a policy that asks for
/// something unreachable moving instead of stalling.
float advance_feasible(const float * from, const float * to, float * out);

}  // namespace gripper_servo
