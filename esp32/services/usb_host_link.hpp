// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "host_link.hpp"

class UsbCdcServer;

// HostLink over the bridge's own USB port, the one make flash-esp32 uses.
// One byte stream carries both channels in sequence: verbs as lines until a
// BEGIN is accepted, then exactly the announced image, then lines again. The
// IDF driver drops what its ring cannot hold rather than stalling the host,
// so the image is paced: the host sends one chunk and waits for OK, and OK
// goes out once the whole chunk has been read out of the driver's ring. That
// ring is the only place the image waits.
class UsbHostLink final : public HostLink {
 public:
  static UsbHostLink &GetInstance();

  // Bytes the host sends between acknowledgements; tools/esp32_client.py
  // mirrors it.
  static constexpr size_t kChunkBytes = 512;

  // Drops what a host sent while another page held the port, so the next
  // BEGIN is the first one answered. A no-op while running: Program hands
  // the port back with the conversation, and its Status, intact.
  void Start();
  void Stop();
  void Poll() override;
  void SendCtrlLine(const char *line) override;
  [[nodiscard]] size_t ReadDataRx(std::span<uint8_t> dst) override;

 private:
  friend class System;
  void Init(UsbCdcServer &usb);
  void DiscardDataRx() override;
  void Drain();

  UsbHostLink() = default;
  ~UsbHostLink() override = default;
  UsbHostLink(const UsbHostLink &) = delete;
  UsbHostLink &operator=(const UsbHostLink &) = delete;

  UsbCdcServer *usb_ = nullptr;
  bool running_ = false;
  bool attached_ = false;
  // Image bytes read out since BEGIN; the data phase ends when it reaches
  // the announced size.
  uint32_t received_ = 0;
};
