// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <optional>

#include "flight_mode.hpp"
#include "message.hpp"
#include "stm32_limits.hpp"

// POD (Plain Old Data) Sensor Packets

inline constexpr uint16_t kImuMaxSamples =
    stm32_limits::kIcm42688pMaxWatermarkRecords;

// One FIFO read, in the chip's own counts: the axis map is a signed
// permutation, so it applies losslessly to the integers, and the two scales are
// what turn a count into SI. Sized by the watermark: a burst spanning several
// reads would have to average `dt` and coarsen the scale.
//
// int32 because HiRes samples are 20 bits. Narrowing to 16 would fit only
// +/-125 dps and +/-1 g, so even a board at rest would spend most bursts on a
// 16x coarser fallback scale.
//
// Per-sample stamps are not carried: sample i sat at
// `timestamp_us - (count-1-i) * dt_us`, which is also how a ULog reader expands
// the record built from these fields.
struct ImuBurst {
  uint64_t timestamp_us = 0;  // the burst's newest sample
  uint32_t device_id = 0;
  float dt_us = 0.0f;        // between samples, from the chip's own timestamp
  float gyro_scale = 0.0f;   // count -> rad/s
  float accel_scale = 0.0f;  // count -> m/s^2
  uint8_t count = 0;
  int32_t gyro[3][kImuMaxSamples]{};
  int32_t accel[3][kImuMaxSamples]{};
};

// A mailbox, not a plain store: the interrupt sets `fresh` and will not write
// while it is set; the consumer clears it once it has finished reading.
struct ImuBurstSlot {
  volatile bool fresh = false;
  ImuBurst burst{};
};

struct GpsData {
  uint32_t timestamp_us;
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
};

struct BatteryData {
  // When the conversion was *started*: publish trails it by up to the whole
  // decimation period.
  uint32_t timestamp_us = 0;
  float voltage;
  // Absent when the board has no current sense. A bare float could only have
  // said zero, which is also what a pack at rest reads.
  std::optional<float> current;
  std::optional<float> mah_drawn;
  uint8_t percentage;  // 0-100
};

struct EscTelemetryMotorData {
  uint32_t timestamp_us = 0;
  float voltage = 0.0f;
  // Absent together: AM32 derives the consumption by integrating the same
  // reading, so an ESC with no shunt fabricates both or neither.
  std::optional<float> current;
  std::optional<uint16_t> consumption_mah;
  uint32_t electrical_rpm = 0;
  uint32_t rpm = 0;
  int16_t temperature_c = 0;
  bool valid = false;
};

struct EscTelemetryData {
  std::array<EscTelemetryMotorData, 4> motors{};
  uint8_t valid_mask = 0;
  uint32_t frame_count = 0;
  uint32_t crc_error_count = 0;
  uint32_t unassigned_frame_count = 0;
  uint32_t rx_drop_bytes = 0;
  uint32_t rx_dma_error_count = 0;
  uint32_t uart_error_count = 0;
};

// Written on every burst, so `timestamp_us` doubles as the sample path's
// heartbeat. Counters only: the thresholds live with Sentinel.
struct ImuHealth {
  uint32_t timestamp_us = 0;
  uint32_t publish_count = 0;  // bursts handed to the control loop
  // Any exit from a sample interrupt that did not publish, which is what the
  // driver's recovery window counts. Summed from the four below, so it always
  // agrees with them.
  uint32_t path_faults = 0;
  uint32_t true_overruns = 0;  // interrupt arrived with a transfer in flight
  uint32_t dma_start_fails = 0;
  uint32_t spi_errors = 0;
  uint32_t parse_fails = 0;
  // Records the chip filled with its no-fresh-data sentinel. Separate from
  // parse_fails because the cause is a configuration mismatch, not lost framing.
  uint32_t invalid_samples = 0;
  uint32_t dropped_records = 0;  // discarded by a FIFO flush, counted exactly
  // Published but never claimed, because the control loop still held the
  // previous burst. In samples, so it compares directly against the ODR.
  uint32_t missed_samples = 0;
  uint8_t last_bad_header = 0;
};

// Die temperature, ~1 Hz: a ~90 s thermal constant does not belong on the fast
// path. Unguarded on purpose -- a torn read is off by millikelvin.
struct ImuTemperature {
  uint32_t timestamp_us = 0;
  float celsius = 0.0f;
};

// Accumulated by the control tick and published whole, so the blackboard
// only ever receives a finished value. It only grows: the reader deltas
// against its own snapshot rather than clearing it, which would race the
// tick still adding to it.
struct ControlLoopLoad {
  uint32_t busy_cycles = 0;
};

struct EstimatorState {
  uint64_t timestamp_us = 0;
  // Both averaged over the burst the control tick consumed.
  Eigen::Vector3f gyro_body_rad_s = Eigen::Vector3f::Zero();
  Eigen::Vector3f accel_body_mps2 = Eigen::Vector3f::Zero();
  // Body attitude in world (NED).
  Eigen::Quaternionf attitude_world_to_body = Eigen::Quaternionf::Identity();
};

struct RcData {
  uint32_t timestamp_us = 0;
  // As the receiver reported them; calibration stops at the four axes below.
  std::array<uint16_t, message::kRcChannelCount> channels_raw{};
  uint16_t roll_us = 0;
  uint16_t pitch_us = 0;
  uint16_t yaw_us = 0;
  uint16_t throttle_us = 0;
};

// CRSF LINK_STATISTICS (0x14). Its own frame, so its own timestamp -- on the
// channels' it could read as fresh as the sticks while being arbitrarily old.
struct CrsfLinkData {
  uint32_t timestamp_us = 0;
  uint8_t uplink_rssi_ant1_dbm = 0;
  uint8_t uplink_rssi_ant2_dbm = 0;
  uint8_t uplink_link_quality = 0;
  int8_t uplink_snr_db = 0;
  uint8_t active_antenna = 0;
  uint8_t rf_mode = 0;
  uint8_t uplink_tx_power_index = 0;
  uint8_t downlink_rssi_dbm = 0;
  uint8_t downlink_link_quality = 0;
  int8_t downlink_snr_db = 0;
};

// The USB bench session. Two writers, never concurrent: MspService in ESC
// config, MscService in MSC. `timestamp_us` is when a field last *changed*.
struct UsbStatusData {
  uint32_t timestamp_us = 0;
  uint32_t msp_requests = 0;
  uint32_t msp_replies = 0;
  uint32_t four_way_requests = 0;
  uint32_t four_way_replies = 0;
  bool attached = false;
  bool configured = false;
  bool port_open = false;
  bool esc_config_granted = false;
  bool msc_active = false;
};

class SharedState {
 public:
  // WRITERS (Called by Drivers)

  void UpdateGps(const GpsData &data) { gps_ = data; }
  void UpdateBattery(const BatteryData &data) { bat_ = data; }
  void UpdateEscTelemetry(const EscTelemetryData &data) { esc_ = data; }
  void UpdateRc(const RcData &data) { rc_ = data; }
  void UpdateCrsfLink(const CrsfLinkData &data) { crsf_link_ = data; }
  // Written from the TIM5 interrupt. Milliseconds in one word so the store
  // cannot be observed torn, which a 64-bit microsecond count would be.
  void UpdateUptimeMs(uint32_t uptime_ms) { uptime_ms_ = uptime_ms; }
  // Monotonic counters, so a reader preempted mid-copy is off by at most one
  // increment on one field -- not worth a seqlock.
  void UpdateImuHealth(const ImuHealth &data) { imu_health_ = data; }
  void UpdateControlLoopLoad(const ControlLoopLoad &data) {
    control_loop_load_ = data;
  }
  void UpdateMainTickCount(uint32_t count) { main_tick_count_ = count; }
  void UpdateImuTemperature(const ImuTemperature &data) { imu_temp_ = data; }

  void UpdateEstimate(const EstimatorState &data) { estimate_ = data; }
  void UpdateUsbStatus(const UsbStatusData &data) { usb_ = data; }
  void SetFlightMode(FlightMode mode) { mode_ = mode; }
  // The state machine owns this: it is set wherever the control tick hook is.
  void SetControlLoopRunning(bool running) { control_loop_running_ = running; }

  // READERS (Called by Logic/Consumers)

  // Fast access for Control Loop (High Frequency)
  const GpsData &GetGps() const { return gps_; }
  const BatteryData &GetBattery() const { return bat_; }
  const EscTelemetryData &GetEscTelemetry() const { return esc_; }
  const RcData &GetRc() const { return rc_; }
  const CrsfLinkData &GetCrsfLink() const { return crsf_link_; }
  // Monotonic since boot; wraps at 49.7 days rather than TIM2's 71.6 min.
  uint32_t UptimeMs() const { return uptime_ms_; }
  const ImuHealth &GetImuHealth() const { return imu_health_; }
  const ControlLoopLoad &GetControlLoopLoad() const {
    return control_loop_load_;
  }
  uint32_t MainTickCount() const { return main_tick_count_; }
  const ImuTemperature &GetImuTemperature() const { return imu_temp_; }

  const EstimatorState &GetEstimate() const { return estimate_; }
  const UsbStatusData &GetUsbStatus() const { return usb_; }
  FlightMode GetFlightMode() const { return mode_; }
  bool IsArmed() const { return armed_; }
  bool IsControlLoopRunning() const { return control_loop_running_; }
  // message::kVehicleFailsafeFlag*. Readable from the control loop, which is
  // what keeps it here rather than behind a Sentinel accessor.
  uint32_t FailsafeFlags() const { return failsafe_flags_; }

  const ImuBurstSlot &GetImuBurstSlot() const { return imu_slot_; }

 private:
  friend class Icm42688p;
  friend class Ahrs;
  ImuBurstSlot &ImuBurstMailbox() { return imu_slot_; }

  // Sentinel is the only writer of both: changing the arm state goes through
  // Sentinel::RequestArm, which carries the interlock and the stop frames.
  friend class Sentinel;
  void SetArmed(bool armed) { armed_ = armed; }
  void SetFailsafeFlags(uint32_t flags) { failsafe_flags_ = flags; }

  GpsData gps_{};
  BatteryData bat_{};
  EscTelemetryData esc_{};
  RcData rc_{};
  CrsfLinkData crsf_link_{};
  uint32_t uptime_ms_ = 0;
  ImuHealth imu_health_{};
  ControlLoopLoad control_loop_load_{};
  uint32_t main_tick_count_ = 0;
  ImuTemperature imu_temp_{};
  alignas(8) ImuBurstSlot imu_slot_{};
  EstimatorState estimate_{};
  UsbStatusData usb_{};
  FlightMode mode_ = FlightMode::kAcro;
  uint32_t failsafe_flags_ = 0;
  bool armed_ = false;
  bool control_loop_running_ = false;
};
