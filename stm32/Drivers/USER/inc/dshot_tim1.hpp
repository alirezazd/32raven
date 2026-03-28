#pragma once
#include <cstdint>

extern "C" volatile uint32_t g_dshot_dma_done;

struct DShotTim1Timings {
  uint16_t arr;
  uint16_t t1h;
  uint16_t t0h;
};

enum class DShotMode : uint8_t { DSHOT150, DSHOT300, DSHOT600 };

class DShotTim1 {
public:
  static DShotTim1 &getInstance() {
    static DShotTim1 instance;
    return instance;
  }

  struct Config {
    DShotMode mode;
  };

  static bool isBusy() { return getInstance().busy_; }

  static const DShotTim1Timings &timings() { return getInstance().timings_; }

  static bool sendBits(const uint16_t *interleaved_ccr, uint16_t total_bits) {
    return getInstance().SendBitsImpl(interleaved_ccr, total_bits);
  }

  void finishAndIdle();

private:
  friend class System;
  static void init(const Config &config) { getInstance().Init(config); }

  DShotTim1() = default;
  DShotTim1(const DShotTim1 &) = delete;
  DShotTim1 &operator=(const DShotTim1 &) = delete;

  void Init(const Config &config);
  bool SendBitsImpl(const uint16_t *interleaved_ccr, uint16_t total_bits);

  void DmaInit();
  void Tim1Init(uint16_t period);
  void ConfigureBurstMode();
  void StartOutputsOnce();
  void StartTransfer(const uint16_t *buf, uint32_t count_words);

  static constexpr uint8_t kMotors = 4;

  DShotTim1Timings timings_{};
  volatile bool busy_ = false;
  bool initialized_ = false;
};
