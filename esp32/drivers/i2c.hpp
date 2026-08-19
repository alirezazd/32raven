// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <span>

#include "driver/i2c_master.h"
#include "hal/gpio_types.h"

enum class I2cInstance : uint8_t { kDisplay };

struct I2cConfig {
  struct Pins {
    gpio_num_t sda_gpio = GPIO_NUM_NC;
    gpio_num_t scl_gpio = GPIO_NUM_NC;
  };

  struct Bus {
    uint32_t transfer_timeout_ms = 100;
  };

  Pins pins{};
  Bus bus{};
};

struct I2cDeviceConfig {
  struct Timing {
    uint32_t clock_hz = 100000;
    uint32_t scl_wait_us = 0;
  };

  uint16_t address = 0;
  Timing timing{};
  bool disable_ack_check = false;
};

template <I2cInstance Inst>
class I2c {
 public:
  static I2c &GetInstance() {
    static I2c instance;
    return instance;
  }

  i2c_master_dev_handle_t AddDevice(const I2cDeviceConfig &cfg);
  bool Probe(uint16_t address) const;

  void Transmit(i2c_master_dev_handle_t device,
                std::span<const uint8_t> bytes) const;
  void MultiBufferTransmit(
      i2c_master_dev_handle_t device,
      std::span<i2c_master_transmit_multi_buffer_info_t> buffers) const;
  void TransmitReceive(i2c_master_dev_handle_t device,
                       std::span<const uint8_t> write_buffer,
                       std::span<uint8_t> read_buffer) const;

 private:
  void Init(const I2cConfig &cfg);
  friend class System;

  I2cConfig cfg_{};
  i2c_master_bus_handle_t bus_ = nullptr;
  I2c() = default;
  ~I2c() = default;
  I2c(const I2c &) = delete;
  I2c &operator=(const I2c &) = delete;
};

using I2cDisplay = I2c<I2cInstance::kDisplay>;
