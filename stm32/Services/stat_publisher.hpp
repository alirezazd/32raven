// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "icm42688p.hpp"

struct AppContext;
struct BatteryData;

namespace message {
struct SystemStatusMsg;
struct UsbStatusMsg;
struct VehicleStatusMsg;
}  // namespace message

class StatPublisher {
 public:
  static StatPublisher &GetInstance() {
    static StatPublisher instance;
    return instance;
  }

  void PublishTelemetry(AppContext &ctx, uint32_t now_us,
                        uint32_t loop_counter);

  // Rate-limits itself on the link's exchange cadence, faster while frames
  // are moving.
  void PublishUsbStatus(AppContext &ctx, uint32_t now_us);

 private:
  StatPublisher() = default;
  ~StatPublisher() = default;
  StatPublisher(const StatPublisher &) = delete;
  StatPublisher &operator=(const StatPublisher &) = delete;

  static uint16_t BatteryVoltageMv(const BatteryData &battery);
  static int16_t BatteryCurrentCa(const BatteryData &battery);
  static int8_t BatteryRemainingPct(const BatteryData &battery);
  static message::SystemStatusMsg BuildSystemStatusMsg(AppContext &ctx,
                                                       uint32_t now_us,
                                                       uint32_t loop_counter,
                                                       const Icm42688p &imu);
  static message::VehicleStatusMsg BuildVehicleStatusMsg(AppContext &ctx);
  static message::UsbStatusMsg BuildUsbStatusMsg(AppContext &ctx);

  uint32_t usb_status_sent_us_ = 0;
  uint8_t usb_status_rx_frames_ = 0;
  uint8_t usb_status_tx_frames_ = 0;
};
