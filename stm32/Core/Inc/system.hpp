#ifndef CORE_SYSTEM_HPP
#define CORE_SYSTEM_HPP

#include "board.h"
#include "button.hpp"

#include "dshot_tim1.hpp"
#include "gpio.hpp"
#include "led.hpp"
#include "spi.hpp"
#include "time_base.hpp"
#include "uart.hpp"

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

  Spi &GetSpi() { return Spi::GetInstance(); }
  GPIO &Gpio() { return GPIO::GetInstance(); }
  TimeBase &Time() { return TimeBase::GetInstance(); }
  Button &Btn() { return Button::GetInstance(); }
  Uart<UartInstance::kUart1> &GetUart() {
    return Uart<UartInstance::kUart1>::GetInstance();
  }

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