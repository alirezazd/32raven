#ifndef USER_DRIVERS_TIMEBASE_HPP
#define USER_DRIVERS_TIMEBASE_HPP

#include "stm32f4xx_hal.h"
#include <cstdint>

constexpr uint32_t SECONDS_TO_MICROS(uint32_t s) { return s * 1000000u; }
constexpr uint32_t MILLIS_TO_MICROS(uint32_t ms) { return ms * 1000u; }

struct TimeBaseConfig {
  uint32_t prescaler; // e.g. 83 -> 1 MHz tick if TIM2CLK = 84 MHz
  uint32_t period;    // typically 0xFFFFFFFF
};

class TimeBase {
public:
  using Config = TimeBaseConfig;

  void Init(const Config &config);
  uint32_t Micros() const;

private:
  friend class System;

  static TimeBase &GetInstance() {
    static TimeBase instance;
    return instance;
  }

  TimeBase() = default;
  ~TimeBase() = default;

  TimeBase(const TimeBase &) = delete;
  TimeBase &operator=(const TimeBase &) = delete;

  bool initialized_ = false;
};

#endif // USER_DRIVERS_TIMEBASE_HPP
