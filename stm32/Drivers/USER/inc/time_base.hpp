#ifndef USER_DRIVERS_TIMEBASE_HPP
#define USER_DRIVERS_TIMEBASE_HPP

#include "stm32f4xx_hal.h"
#include <cstdint>

#define SECONDS_TO_MICROS(s) ((s) * 1000000u)
#define MILLIS_TO_MICROS(ms) ((ms) * 1000u)

struct TimeBaseConfig {
  uint32_t prescaler; // e.g. 83 -> 1 MHz tick if TIM2CLK = 84 MHz
  uint32_t period;    // typically 0xFFFFFFFF
};

class TimeBase {
public:
  using Config = TimeBaseConfig;

  uint32_t Micros() const;
  void DelayMicros(uint32_t us) const;

private:
  friend class System;
  void Init(const Config &config);

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
