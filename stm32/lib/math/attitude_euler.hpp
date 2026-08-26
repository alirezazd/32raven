// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace math {

struct EulerZyx {
  float roll;   // rad, body X
  float pitch;  // rad, body Y
  float yaw;    // rad, body Z
};

// ZYX extraction from the blackboard's world-to-body quaternion. Every wire
// format that carries attitude as angles reads it from here, because a second
// copy is a second chance for the sign convention to drift.
//
// Pitch saturates instead of handing NaN to asin: the input is a unit
// quaternion only to float precision, so the argument can leave [-1, 1] by an
// epsilon at exactly the attitude where yaw is degenerate anyway.
inline EulerZyx EulerZyxFromQuaternion(const Eigen::Quaternionf &q) {
  const float sin_roll = 2.0f * ((q.w() * q.x()) + (q.y() * q.z()));
  const float cos_roll = 1.0f - (2.0f * ((q.x() * q.x()) + (q.y() * q.y())));
  float sin_pitch = 2.0f * ((q.w() * q.y()) - (q.z() * q.x()));
  sin_pitch = (sin_pitch > 1.0f) ? 1.0f : ((sin_pitch < -1.0f) ? -1.0f : sin_pitch);
  const float sin_yaw = 2.0f * ((q.w() * q.z()) + (q.x() * q.y()));
  const float cos_yaw = 1.0f - (2.0f * ((q.y() * q.y()) + (q.z() * q.z())));
  return EulerZyx{
      .roll = std::atan2(sin_roll, cos_roll),
      .pitch = std::asin(sin_pitch),
      .yaw = std::atan2(sin_yaw, cos_yaw),
  };
}

}  // namespace math
