#pragma once

#include <array>
#include <cstdint>

#include "stm32_limits.hpp"

// ---------------------------------------------------------
// POD (Plain Old Data) Sensor Packets
// ---------------------------------------------------------

struct GpsData {
  uint64_t timestamp_us;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;  // PVT validity flags

  uint32_t tAcc;  // ns
  int32_t lat;    // deg * 1e7
  int32_t lon;    // deg * 1e7
  int32_t alt;    // mm (MSL)
  uint32_t hAcc;  // mm
  uint32_t vAcc;  // mm
  uint16_t vel;   // cm/s
  uint16_t hdg;   // cdeg
  uint8_t num_sats;
  uint8_t fix_type;  // 0-1: no fix, 2: 2D, 3: 3D

  // Quality metrics (DOP)
  uint16_t gDOP;  // Geometric DOP [0.01]
  uint16_t pDOP;  // Position DOP [0.01]
  uint16_t hDOP;  // Horizontal DOP [0.01]
  uint16_t vDOP;  // Vertical DOP [0.01]

  // Covariance (for Kalman filtering)
  uint8_t posCovValid;  // Position covariance valid flag
  uint8_t velCovValid;  // Velocity covariance valid flag
  float posCovNN;       // Position covariance North-North [m²]
  float posCovEE;       // Position covariance East-East [m²]
  float posCovDD;       // Position covariance Down-Down [m²]

  bool updated;  // Freshness flag
};

struct BatteryData {
  float voltage;
  float current;
  float mah_drawn;
  uint8_t percentage;  // 0-100
};

struct RcData {
  uint32_t timestamp_us = 0;
  std::array<uint16_t, stm32_limits::kRcEnabledChannelCount> channels{};
  uint16_t roll_us = 0;
  uint16_t pitch_us = 0;
  uint16_t yaw_us = 0;
  uint16_t throttle_us = 0;
  bool rx_online = false;
  bool tx_online = false;
  bool updated = false;
};

// ---------------------------------------------------------
// Central Data Store (Blackboard)
// ---------------------------------------------------------

class VehicleState {
 public:
  // --- WRITERS (Called by Drivers) ---

  void UpdateGps(const GpsData &data) {
    gps_ = data;
    gps_.updated = true;
  }

  void UpdateBattery(const BatteryData &data) { bat_ = data; }
  void UpdateRc(const RcData &data) {
    rc_ = data;
    rc_.updated = true;
  }

  // --- READERS (Called by Logic/Consumers) ---

  // Fast access for Control Loop (High Frequency)
  const GpsData &GetGps() const { return gps_; }
  const BatteryData &GetBattery() const { return bat_; }
  const RcData &GetRc() const { return rc_; }

  // Polling access for Telemetry/Log (Low Frequency)
  // Returns true if new data was available since last pop
  bool PopGps(GpsData &out) {
    out = gps_;
    if (gps_.updated) {
      gps_.updated = false;
      return true;
    }
    return false;
  }

  bool PopRc(RcData &out) {
    out = rc_;
    if (rc_.updated) {
      rc_.updated = false;
      return true;
    }
    return false;
  }

 private:
  GpsData gps_{};
  BatteryData bat_{};
  RcData rc_{};
};
