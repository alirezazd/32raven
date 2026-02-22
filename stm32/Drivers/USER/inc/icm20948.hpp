#pragma once

#include "board.h"
#include "icm20948_reg.hpp"
#include <cstdint>

class GPIO;

class Icm20948 {
public:
  struct Config {
    size_t dma_buf_size;
    size_t fifo_size;

    struct Accel {
      uint8_t range;
      uint8_t dlpf_config;
      uint8_t sample_rate_div;
    } accel;

    struct Gyro {
      uint8_t range;
      uint8_t dlpf_config;
      uint8_t sample_rate_div;
    } gyro;

    uint8_t mag_rate;
    uint8_t spi_prescaler; // Cast from SpiPrescaler
    uint8_t who_am_i;

    static constexpr float kAccelScale = 9.80665f / 2048.0f;
    static constexpr float kGyroScale = 0.01745329f / 16.4f;
  };

  static Icm20948 &GetInstance() {
    static Icm20948 instance;
    return instance;
  }

  // Called from EXTI15_10_IRQHandler
  void OnDrdyIrq(uint32_t timestamp);

  uint32_t GetLastDrdyTime() const { return last_drdy_time_; }

private:
  friend class System;
  Icm20948() = default;
  ~Icm20948() = default;

  Icm20948(const Icm20948 &) = delete;
  Icm20948 &operator=(const Icm20948 &) = delete;

  void Init(GPIO &gpio, const Config &config);
  void SelectRegisterBank(uint8_t bank);
  void WriteReg(uint8_t bank, uint8_t reg, uint8_t val, bool verify = false,
                uint8_t verify_mask = 0xFF);
  uint8_t ReadReg(uint8_t bank, uint8_t reg);

  void WriteRegRaw(uint8_t reg, uint8_t val);
  uint8_t ReadRegRaw(uint8_t reg);

  // Configuration helpers
  void ConfigureAccel();
  void ConfigureGyro();

  Config config_;
  GPIO *gpio_ = nullptr;
  volatile uint8_t last_bank_ = 0;
  volatile uint32_t last_drdy_time_ = 0;
  volatile uint32_t missed_drdy_count_ = 0;
  bool initialized_ = false;

  // DMA Buffer: 1 byte command + data
  static constexpr size_t kDmaBufSize = 32;
  uint8_t dma_tx_buf_[kDmaBufSize];
  uint8_t dma_rx_buf_[kDmaBufSize];

  static void OnDmaComplete(void *user, bool ok);
};
