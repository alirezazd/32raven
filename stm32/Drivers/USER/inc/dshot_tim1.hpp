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

  static bool sendBits(const uint16_t *interleaved_ccr, uint16_t totalBits) {
    return getInstance()._sendBits(interleaved_ccr, totalBits);
  }

  void finishAndIdle();

private:
  friend class System;
  static void init(const Config &config) { getInstance()._init(config); }

  DShotTim1() = default;
  DShotTim1(const DShotTim1 &) = delete;
  DShotTim1 &operator=(const DShotTim1 &) = delete;

  void _init(const Config &config);
  bool _sendBits(const uint16_t *interleaved_ccr, uint16_t totalBits);

  void dma_init_();
  void tim1_init_(uint16_t period);
  void configureBurstMode_();
  void startOutputsOnce_();
  void startTransfer_(const uint16_t *buf, uint32_t count_words);

  static constexpr uint8_t kMotors = 4;

  DShotTim1Timings timings_{};
  volatile bool busy_ = false;
  bool initialized_ = false;
};
