#ifndef CORE_SYSTEM_HPP
#define CORE_SYSTEM_HPP

#include "GPIO.hpp"
#include "LED.hpp"
#include "TimeBase.hpp"
#include "board.h"

class System {
public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  struct Config {
    RCC_OscInitTypeDef osc;
    RCC_ClkInitTypeDef clk;
    uint32_t flashLatency;
    uint32_t voltageScaling;
  };

  void Init(const Config &config);

  LED &Led() { return LED::GetInstance(); }
  GPIO &Gpio() { return GPIO::GetInstance(); }
  TimeBase &Time() { return TimeBase::GetInstance(); }

private:
  bool initialized_ = false;
  System();
  ~System() {}
  System(const System &) = delete;
  System &operator=(const System &) = delete;
  void ConfigureSystemClock(const Config &config);
};

#define MICROS() (System::GetInstance().Time().Micros())
#endif // CORE_SYSTEM_HPP