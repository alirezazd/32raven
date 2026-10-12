// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "shared_state.hpp"
#include "uart.hpp"
#include "usb_cdc.hpp"

// u-center to the GPS module: the USB COM port and UART2 joined byte for byte,
// at whatever baud the host opens the port at. The M10 parser is not polled
// for the session, and the session ends in a reset so boot re-applies the
// module's configuration whatever the host changed.
class GpsPassthroughService {
 public:
  static GpsPassthroughService &GetInstance();

  void Poll(uint32_t now_us);

  // Refuses while armed.
  void SetGpsPassthroughMode(bool enabled);
  bool GpsPassthroughGranted() const { return granted_; }

 private:
  friend class System;
  void Init(UsbCdc &usb, Uart2 &uart, SharedState &blackboard);

  GpsPassthroughService() = default;
  ~GpsPassthroughService() = default;
  GpsPassthroughService(const GpsPassthroughService &) = delete;
  GpsPassthroughService &operator=(const GpsPassthroughService &) = delete;

  void PublishUsbStatus(uint32_t now_us);
  void FollowHostBaud();
  void PumpToGps();
  void PumpToHost();

  UsbCdc *usb_ = nullptr;
  Uart2 *uart_ = nullptr;
  SharedState *blackboard_ = nullptr;
  bool initialized_ = false;
  bool granted_ = false;
  // What UART2 runs at; zero until the session applies the host's rate.
  uint32_t uart_baud_ = 0;
  uint32_t to_gps_bytes_ = 0;
  uint32_t to_host_bytes_ = 0;
};
