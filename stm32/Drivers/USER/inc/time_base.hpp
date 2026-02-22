#ifndef USER_DRIVERS_TIMEBASE_HPP
#define USER_DRIVERS_TIMEBASE_HPP

#include "stm32f4xx_hal.h"
#include <cstdint>

#define SECONDS_TO_MICROS(s) ((s) * 1000000u)
#define MILLIS_TO_MICROS(ms) ((ms) * 1000u)

struct TimeBaseConfig {
  struct Tim2 {
    uint32_t prescaler;    // e.g. 83 -> 1 MHz tick if TIM2CLK = 84 MHz
    uint32_t period;       // typically 0xFFFFFFFF
    uint32_t compensation; // PPS capture compensation in microseconds
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

  uint32_t Micros() const;
  uint64_t MicrosCorrected() const;

  void DelayMicros(uint32_t us) const;

  // TIM5 scheduler tick helpers
  uint32_t ConsumeTim5Ticks() const;

  bool IsGpsSynchronized() const;
  uint32_t GetDriftMicros() const;

private:
  friend class System;
  void Init(const Config &config);

  static TimeBase &GetInstance();

  TimeBase() = default;
  ~TimeBase() = default;

  TimeBase(const TimeBase &) = delete;
  TimeBase &operator=(const TimeBase &) = delete;

  bool initialized_ = false;
  uint32_t compensation_ = 0;
};

#endif // USER_DRIVERS_TIMEBASE_HPP
