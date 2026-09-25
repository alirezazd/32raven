// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cmath>

namespace math {

constexpr float kGravityMps2 = 9.80665f;

// Height above where `reference_pa` was read, by the ISA troposphere. With
// 101325 Pa it is a standard day's height above sea level.
inline float PressureHeight(float pressure_pa, float reference_pa) {
  constexpr float kIsaScaleHeightM = 44330.77f;
  constexpr float kIsaExponent = 0.190263f;
  return kIsaScaleHeightM *
         (1.0f - std::pow(pressure_pa / reference_pa, kIsaExponent));
}

}  // namespace math
