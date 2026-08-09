// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "hal/gpio_types.h"

enum class UartInstance : uint8_t { kFcLink, kTelem };

enum class UartParity : uint8_t {
  kNone = 0,
  kEven = 1,
  kOdd = 2,
};

struct UartConfig {
  static constexpr int kMinBufferSize = 256;

  struct Pins {
    gpio_num_t tx_gpio = GPIO_NUM_NC;
    gpio_num_t rx_gpio = GPIO_NUM_NC;
  };

  struct Line {
    uint32_t baud_rate = 0;
    UartParity parity = UartParity::kNone;
  };

  struct Buffers {
    int rx_bytes = 0;
    int tx_bytes = 0;
  };

  Pins pins{};
  Line line{};
  Buffers buffers{};
};

template <UartInstance Inst>
class Uart {
 public:
  static Uart &GetInstance() {
    static Uart instance;
    return instance;
  }
  int WriteBytes(std::span<const uint8_t> bytes);
  // Not a WriteBytes overload: on a uint16_t crc, WriteBytes(crc) would then
  // compile and silently send one truncated byte.
  int WriteByte(uint8_t byte);
  [[nodiscard]] int ReadBytes(std::span<uint8_t> bytes, uint32_t timeout_ms = 0);
  // nullopt on timeout.
  [[nodiscard]] std::optional<uint8_t> ReadByte(uint32_t timeout_ms = 0);
  size_t BufferedRxBytes() const;
  void Flush();
  void DrainTx(uint32_t timeout_ms);
  void SetBaudRate(uint32_t baud_rate);
  const UartConfig &GetConfig() const { return cfg_; }

 private:
  friend class System;
  void Init(const UartConfig &cfg);
  UartConfig cfg_{};
  Uart() = default;
  ~Uart() = default;
  Uart(const Uart &) = delete;
  Uart &operator=(const Uart &) = delete;
};

using UartFcLink = Uart<UartInstance::kFcLink>;
using UartTelem = Uart<UartInstance::kTelem>;
