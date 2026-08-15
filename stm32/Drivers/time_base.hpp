// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#ifndef USER_DRIVERS_TIMEBASE_HPP
#define USER_DRIVERS_TIMEBASE_HPP

#include <cstdint>

#include "shared_state.hpp"
#include "stm32f4xx.h"  // for TIM2 (used inline by Micros())

// Wide on purpose: a microsecond count overflows 32 bits at 4295 seconds, so
// multiplying in the argument's own width truncates a long interval and then
// widens the wrong answer. Callers that hold the result in a uint32_t are
// saying the interval is short.
constexpr uint64_t SecondsToMicros(uint64_t s) { return s * 1000000ull; }
constexpr uint64_t MillisToMicros(uint64_t ms) { return ms * 1000ull; }

struct TimeBaseConfig {
  struct Tim2 {
    uint32_t prescaler;  // e.g. 83 -> 1 MHz tick if TIM2CLK = 84 MHz
    uint32_t period;     // typically 0xFFFFFFFF
  } tim2;

  struct Tim5 {
    // TIM5 periodic scheduler tick
    uint32_t prescaler;
    uint32_t period;
    bool autoreload_preload;
  } tim5;
};

class TimeBase {
 public:
  using Config = TimeBaseConfig;

  __attribute__((always_inline)) inline uint32_t Micros()
      const {  // direct 32-bit counter read; inlined for fast-path overhead
    return TIM2->CNT;
  }

  void DelayMicros(uint32_t us) const;

  // TIM5 scheduler tick helpers
  uint32_t ConsumeTim5Ticks() const;

 private:
  friend class System;
  void Init(const Config &config, SharedState &blackboard);

  static TimeBase &GetInstance();

  TimeBase() = default;
  ~TimeBase() = default;

  TimeBase(const TimeBase &) = delete;
  TimeBase &operator=(const TimeBase &) = delete;

  bool initialized_ = false;
};

#endif  // USER_DRIVERS_TIMEBASE_HPP
