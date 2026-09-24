// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "sensor_health_monitor.hpp"

#include <cstddef>

#include "stm32_config.hpp"
#include "time_base.hpp"

namespace {

// Long enough that a single retried DMA error does not hold a sensor
// unhealthy across a whole GCS refresh, short enough to catch a repeating one.
constexpr uint32_t kFaultWindowUs = 1000000u;

// ~100 burst periods, well past Sentinel's stall window: a path that stalls
// and is recovered should not blink the health bit on its way back.
constexpr uint32_t kImuFreshTimeoutUs = 100000u;

// Some fifty conversions, so a read landing between two never reads as a loss.
constexpr uint32_t kMagFreshTimeoutUs = 1000000u;

// Each consumer sets its own bound; this one asks whether the receiver is
// still delivering.
constexpr uint32_t kRcFreshTimeoutUs = 1500000u;

// Sized for an idle disarmed ESC that still answers, some sixty turns of the
// request cycle; a running motor stamps far faster.
constexpr uint32_t kEscFreshTimeoutUs = 1000000u;

// Arrival rather than the error counters: four talkers on one unterminated
// wire corrupt a frame now and then whether or not anything is wrong.
uint8_t EscOnlineMask(const EscTelemetryData &esc, uint32_t now_us) {
  uint8_t online = 0;
  for (size_t i = 0; i < esc.motors.size(); ++i) {
    if ((esc.valid_mask & (1u << i)) != 0u &&
        ElapsedMicros(now_us, esc.motors[i].timestamp_us) <=
            kEscFreshTimeoutUs) {
      online |= static_cast<uint8_t>(1u << i);
    }
  }
  return online;
}

}  // namespace

SensorHealthMonitor &SensorHealthMonitor::GetInstance() {
  static SensorHealthMonitor instance;
  return instance;
}

void SensorHealthMonitor::Init(SharedState &blackboard) {
  blackboard_ = &blackboard;
}

void SensorHealthMonitor::Poll(uint32_t now_us) {
  UpdateFaultWindows(now_us);

  const SharedState &blackboard = *blackboard_;
  SensorHealth health{};

  const ImuHealth &imu = blackboard.GetImuHealth();
  health.imu.present = imu.timestamp_us != 0u;
  health.imu.healthy =
      health.imu.present && IsHealthy(FaultSource::kImu) &&
      ElapsedMicros(now_us, imu.timestamp_us) <= kImuFreshTimeoutUs;

  const GpsData &gps = blackboard.GetGps();
  health.gps.present = gps.timestamp_us != 0u;
  health.gps.healthy =
      health.gps.present && gps.fix_type >= 2u &&
      IsHealthy(FaultSource::kGps) &&
      ElapsedMicros(now_us, gps.timestamp_us) <= kGpsFreshTimeoutUs;

  const BatteryData &battery = blackboard.GetBattery();
  health.battery.present = battery.voltage > 0.0f;
  health.battery.healthy =
      health.battery.present && IsHealthy(FaultSource::kBattery) &&
      ElapsedMicros(now_us, battery.timestamp_us) <= kBatteryFreshTimeoutUs;

  const RcData &rc = blackboard.GetRc();
  health.rc.present = rc.timestamp_us != 0u;
  health.rc.healthy =
      health.rc.present && IsHealthy(FaultSource::kRc) &&
      ElapsedMicros(now_us, rc.timestamp_us) <= kRcFreshTimeoutUs;

  const EscTelemetryData &esc = blackboard.GetEscTelemetry();
  health.esc_online = EscOnlineMask(esc, now_us);
  health.esc.present = esc.valid_mask != 0u;
  health.esc.healthy =
      health.esc.present && health.esc_online == esc.valid_mask;

  // Healthy is a calibrated compass reading the field it was calibrated in:
  // uncalibrated, it has no strength to be held against.
  const MagnetometerData &mag = blackboard.GetMagnetometer();
  health.mag.present = mag.timestamp_us != 0u;
  health.mag.healthy =
      health.mag.present && blackboard.GetMagCalibration().calibrated &&
      !blackboard.GetEstimate().mag.interference &&
      ElapsedMicros(now_us, mag.timestamp_us) <= kMagFreshTimeoutUs;

  blackboard_->UpdateSensorHealth(health);
}

void SensorHealthMonitor::UpdateFaultWindows(uint32_t now_us) {
  if ((now_us - last_window_us_) < kFaultWindowUs) {
    return;
  }
  last_window_us_ = now_us;

  const SystemHealth &health = blackboard_->GetSystemHealth();

  // Ordered by FaultSource: transport plus parser, as garbage over a clean UART
  // is not healthy. path_faults already sums both; an ADC has no parser.
  const std::array<uint32_t, std::to_underlying(FaultSource::kCount)> totals = {
      blackboard_->GetImuHealth().path_faults,
      health.gps_uart.Total() + blackboard_->GetGps().checksum_failures,
      health.rc_uart.Total() + blackboard_->GetCrsfLink().checksum_failures,
      health.batt_adc.Total(),
  };

  for (size_t i = 0; i < totals.size(); ++i) {
    fault_windows_[i].healthy = totals[i] == fault_windows_[i].last_total;
    fault_windows_[i].last_total = totals[i];
  }
}
