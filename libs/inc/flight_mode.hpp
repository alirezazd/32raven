// FlightMode — the operator-selected control law currently in effect.
//   kAcro       : sticks → angular-rate setpoint (current Phase A path)
//   kStabilize  : sticks → desired-attitude quaternion → attitude
//                 controller → angular-rate setpoint
// TODO: Future modes (altitude hold, position hold, RTH, mission)

#pragma once

#include <cstdint>

enum class FlightMode : std::uint8_t {
  kAcro = 0,
  kStabilize,
};
