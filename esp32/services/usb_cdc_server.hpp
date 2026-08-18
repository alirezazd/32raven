// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "mavlink_transport.hpp"

extern "C" {
}

// USB Serial/JTAG transport for the ESP32-C3 native USB peripheral. Selected
// when ESP32_MAVLINK_TRANSPORT_USB_CDC=y; the host sees /dev/ttyACM0. Same
// Send/Receive contract as UdpServer; MAVLink drives it through the
// IMavlinkTransport interface.
class UsbCdcServer : public IMavlinkTransport {
 public:
  struct Config {
    size_t rx_buffer_bytes = 1024;
    size_t tx_buffer_bytes = 1024;
  };

  static UsbCdcServer &GetInstance();

  int Receive(std::span<uint8_t> dst) override;
  int Send(std::span<const uint8_t> bytes) override;
  bool IsReady() const override;
  void ClearPeer() override;

 private:
  friend class System;
  void Init(const Config &cfg);

  UsbCdcServer() = default;
  ~UsbCdcServer() = default;
  UsbCdcServer(const UsbCdcServer &) = delete;
  UsbCdcServer &operator=(const UsbCdcServer &) = delete;

  Config cfg_{};
  bool driver_installed_ = false;
};
