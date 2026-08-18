// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include <cstdint>

#include "stm32_limits.hpp"

struct DShotTim1Timings {
  uint16_t arr;
  uint16_t t1h;
  uint16_t t0h;
};

enum class DShotMode : uint8_t { kDshot150, kDshot300, kDshot600 };

class DShotTim1 {
 public:
  static DShotTim1 &GetInstance();

  struct Config {
    DShotMode mode;
    // TIM1's kernel clock. The bit period is this divided by the DShot rate,
    // so it belongs to the clock tree rather than to the mode: an APB2
    // divider past /2 halves it, and a period frozen against 168 MHz would
    // put a slower rate on the wire than the mode names.
    uint32_t timer_clock_hz;
  };

  // Wire rates. Fixed by the protocol, unlike the tick counts they resolve to.
  static constexpr uint32_t BitRateHz(DShotMode mode) {
    switch (mode) {
      case DShotMode::kDshot150:
        return 150000u;
      case DShotMode::kDshot300:
        return 300000u;
      case DShotMode::kDshot600:
        return 600000u;
    }
    return 0u;
  }

  // The generator rejects a rate whose bit period falls outside these, so the
  // runtime check cannot disagree with the build-time one.
  static constexpr uint32_t kMinPeriodTicks = stm32_limits::kDshotMinPeriodTicks;
  static constexpr uint32_t kMaxPeriodTicks = stm32_limits::kDshotMaxPeriodTicks;

  static bool IsBusy() { return GetInstance().busy_; }

  static const DShotTim1Timings &Timings() { return GetInstance().timings_; }

  static bool SendBits(const uint16_t *interleaved_ccr, uint16_t total_bits) {
    return GetInstance().SendBitsImpl(interleaved_ccr, total_bits);
  }

  void FinishAndIdle();

  // AM32 reboots half a second into silence and beeps until it ends, so a
  // disarmed wire has to keep saying "stop".
  void KeepAlive();

  // A stop frame is sixteen low pulses, and a low line tells an AM32
  // bootloader to hand over to firmware -- so passthrough mutes this.
  void SuspendKeepAlive() { keep_alive_suspended_ = true; }
  void ResumeKeepAlive() { keep_alive_suspended_ = false; }

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

  // Well under AM32's ~0.5 s silence-to-reboot, well over any real gap.
  static constexpr uint32_t kKeepAliveSilenceUs = 100000u;
  // Motor stop is value 0, telemetry 0, CRC 0 -- sixteen '0' bits -- plus two
  // trailing zero slots that park the lines low.
  static constexpr uint16_t kStopFrameBits = 18;

  DShotTim1Timings timings_{};
  uint16_t stop_frame_[kStopFrameBits * kMotors] = {};
  volatile uint32_t last_frame_us_ = 0;
  volatile bool keep_alive_suspended_ = false;
  volatile bool busy_ = false;
  volatile uint32_t dma_start_fail_count_ = 0;
  bool initialized_ = false;
};
