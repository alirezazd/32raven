// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

// FlightMode — the operator-selected control law currently in effect.
//   kAcro       : sticks → angular-rate setpoint (current Phase A path)
//   kStabilize  : sticks → desired-attitude quaternion → attitude
//                 controller → angular-rate setpoint

#pragma once

#include <cstdint>

enum class FlightMode : std::uint8_t {
  kAcro = 0,
  kStabilize,
};
