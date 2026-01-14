#pragma once

#include "board.h"
#include <cstdint>

class Icm20948 {
public:
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

  void Init();
  void WriteReg(uint8_t reg, uint8_t val);
  uint8_t ReadReg(uint8_t reg);

  volatile uint32_t last_drdy_time_ = 0;
  volatile uint32_t missed_drdy_count_ = 0;
  bool initialized_ = false;

  // DMA Buffer: 1 byte command + data (size TBD, say 32 for now)
  static constexpr size_t kDmaBufSize = 32;
  uint8_t dma_tx_buf_[kDmaBufSize];
  uint8_t dma_rx_buf_[kDmaBufSize];

  static void OnDmaComplete(void *user, bool ok);
};
