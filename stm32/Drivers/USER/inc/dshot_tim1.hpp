#pragma once
#include <cstdint>

struct DShotTim1Timings {
  uint16_t arr;
  uint16_t t1h;
  uint16_t t0h;
};

enum class DShotMode : uint8_t { kDshot150, kDshot300, kDshot600 };

class DShotTim1 {
 public:
  static DShotTim1 &GetInstance() {
    static DShotTim1 instance;
    return instance;
  }

  struct Config {
    DShotMode mode;
  };

  static bool IsBusy() { return GetInstance().busy_; }

  static const DShotTim1Timings &Timings() { return GetInstance().timings_; }

  static bool SendBits(const uint16_t *interleaved_ccr, uint16_t total_bits) {
    return GetInstance().SendBitsImpl(interleaved_ccr, total_bits);
  }

  void FinishAndIdle();

  bool IsInitialized() const { return initialized_; }
  uint32_t DmaStartFailCount() const { return dma_start_fail_count_; }

 private:
  friend class System;

  DShotTim1() = default;
  DShotTim1(const DShotTim1 &) = delete;
  DShotTim1 &operator=(const DShotTim1 &) = delete;

  void Init(const Config &config);
  bool SendBitsImpl(const uint16_t *interleaved_ccr, uint16_t total_bits);

  void DmaInit();
  void Tim1Init(uint16_t period);
  void StartOutputsOnce();
  bool StartTransfer(const uint16_t *buf, uint32_t count_words);

  static constexpr uint8_t kMotors = 4;

  DShotTim1Timings timings_{};
  volatile bool busy_ = false;
  volatile uint32_t dma_start_fail_count_ = 0;
  bool initialized_ = false;
};
