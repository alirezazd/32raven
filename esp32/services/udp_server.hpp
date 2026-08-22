// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <span>

#include "esp32_limits.hpp"
#include "mavlink_transport.hpp"
#include "ring_buffer.hpp"
#include "wifi.hpp"

extern "C" {
}

#include "wifi.hpp"

class UdpServer : public IMavlinkTransport {
 public:
  struct Config {
    uint16_t port = 14550;
    uint32_t upload_cap_kbits = 0;
    uint32_t download_cap_kbits = 0;
    uint32_t overflow_threshold = 16;
  };

  static UdpServer &GetInstance();

  // Failure is tolerated by design and leaves the socket closed.
  void Start();
  void Stop();
  void ClearPeer() override;

  int Receive(std::span<uint8_t> dst) override;
  int Send(std::span<const uint8_t> bytes) override;
  bool IsReady() const override;

 private:
  WifiController *wifi_ = nullptr;
  friend class System;
  void Init(const Config &cfg, WifiController &wifi);
  static bool SetNonblock(int fd);
  // The two scalars move together, so they travel together.
  struct TokenBucket {
    uint32_t tokens_bytes = 0;
    int64_t last_refill_us = 0;
  };
  static TokenBucket RefillTokens(TokenBucket bucket, uint32_t bytes_per_s,
                                  uint32_t burst_bytes);
  void ResetShaperState();

  Config cfg_{};
  int fd_ = -1;
  bool running_ = false;
  uint32_t peer_ipv4_ = 0;
  uint16_t peer_port_ = 0;
  static constexpr uint32_t kUploadBufferBytes =
      static_cast<uint32_t>(esp32_limits::kUdpServerUploadBufferBytes);
  static constexpr uint32_t kDownloadBufferBytes =
      static_cast<uint32_t>(esp32_limits::kUdpServerDownloadBufferBytes);
  bool upload_cap_enabled_ = false;
  uint32_t upload_cap_bytes_per_s_ = 0;
  TokenBucket upload_bucket_{};
  uint32_t upload_overflow_count_ = 0;
  RingBuffer<uint8_t, esp32_limits::kUdpServerUploadBufferBytes + 1>
      upload_shaper_buffer_;
  bool download_cap_enabled_ = false;
  uint32_t download_cap_bytes_per_s_ = 0;
  TokenBucket download_bucket_{};
  uint32_t download_overflow_count_ = 0;
  RingBuffer<uint8_t, esp32_limits::kUdpServerDownloadBufferBytes + 1>
      download_shaper_buffer_;

  UdpServer() = default;
  ~UdpServer() = default;
  UdpServer(const UdpServer &) = delete;
  UdpServer &operator=(const UdpServer &) = delete;
};
