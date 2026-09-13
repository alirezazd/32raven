// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <mavlink.h>

#include <cstdint>

// The bridge speaks as the vehicle's autopilot; a ground station addresses
// parameters, calibration and modes there, so it is not ours to choose.
inline constexpr uint8_t kMavlinkComponentId = MAV_COMP_ID_AUTOPILOT1;

struct MavlinkConfig {
  // Distinguishes this vehicle among any sharing a link or a ground
  // station, which is the one thing here a build gets to decide.
  uint8_t sysid = 0;

  struct Tx {
    struct Periods {
      uint16_t hb_ms = 0;
      uint16_t gps_ms = 0;
      uint16_t att_ms = 0;
      uint16_t gpos_ms = 0;
      uint16_t batt_ms = 0;
      uint16_t rc_ms = 0;
      uint16_t esc_ms = 0;
    } periods;
    // What the telem UART's radio carries over the air, vehicle to ground,
    // in bytes per second; 0 leaves the line rate as the limit. Declared,
    // because a radio never reports it and the build checks the ladder
    // against it.
    uint32_t link_air_bytes_per_s = 0;

    struct Schedule {
      uint16_t hb_deadline_ms = 0;
    } schedule;
  } tx;
};
