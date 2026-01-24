#pragma once

#include <cstdint>

// ---------------------------------------------------------
// POD (Plain Old Data) Sensor Packets
// ---------------------------------------------------------

struct ImuData {
  uint32_t timestamp_us;
  float accel[3]; // m/s^2
  float gyro[3];  // rad/s
  float mag[3];   // uT
  bool valid;
};

struct GpsData {
  uint32_t timestamp_us;
  int32_t lat;  // deg * 1e7
  int32_t lon;  // deg * 1e7
  int32_t alt;  // mm (MSL)
  uint16_t vel; // cm/s
  uint16_t hdg; // cdeg
  uint8_t num_sats;
  uint8_t fix_type; // 0-1: no fix, 2: 2D, 3: 3D
  bool updated;     // Freshness flag
};

struct BatteryData {
  float voltage;
  float current;
  float mah_drawn;
};

// ---------------------------------------------------------
// Central Data Store (Blackboard)
// ---------------------------------------------------------

class VehicleState {
public:
  // --- WRITERS (Called by Drivers) ---

  void UpdateImu(const ImuData &data) {
    // Atomic copy usually safe for struct on 32-bit if aligned,
    // but for larger structs in RTOS preemption, consider critical section.
    // For bare metal/cooperative, this is fine.
    imu_ = data;
  }

  void UpdateGps(const GpsData &data) {
    gps_ = data;
    gps_.updated = true;
  }

  void UpdateBattery(const BatteryData &data) { bat_ = data; }

  // --- READERS (Called by Logic/Consumers) ---

  // Fast access for Control Loop (High Frequency)
  const ImuData &GetImu() const { return imu_; }
  const BatteryData &GetBattery() const { return bat_; }

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

private:
  ImuData imu_{};
  GpsData gps_{};
  BatteryData bat_{};
};
