#ifndef CORE_SYSTEM_HPP
#define CORE_SYSTEM_HPP

#include "board.h"
#include "button.hpp"
#include "dshot_tim1.hpp"
#include "gpio.hpp"
#include "led.hpp"
#include "m9n.hpp"
#include "m9n_service.hpp"
#include "spi.hpp"
#include "time_base.hpp"
#include "uart.hpp"
#include "user_config.hpp" // For SystemConfig

class System {
public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  void Init(const SystemConfig &config);

  LED &Led() { return LED::GetInstance(); }

  Spi &GetSpi() { return Spi::GetInstance(); }
  GPIO &Gpio() { return GPIO::GetInstance(); }
  TimeBase &Time() { return TimeBase::GetInstance(); }
  Button &Btn() { return Button::GetInstance(); }
  Uart<UartInstance::kUart1> &GetUart() {
    return Uart<UartInstance::kUart1>::GetInstance();
  }
  // Alias for clarity (Console)
  Uart<UartInstance::kUart1> &GetUart1() {
    return Uart<UartInstance::kUart1>::GetInstance();
  }
  // GPS UART
  Uart<UartInstance::kUart2> &GetUart2() {
    return Uart<UartInstance::kUart2>::GetInstance();
  }

  M9N &GetGps() { return M9N::GetInstance(); }
  M9NService &ServiceM9N() { return m9n_service_; }

private:
  bool initialized_ = false;
  M9NService m9n_service_;

  System();
  ~System() {}
  System(const System &) = delete;
  System &operator=(const System &) = delete;
  void ConfigureSystemClock(const SystemConfig &config);
};

#define MICROS() (System::GetInstance().Time().Micros())
#endif // CORE_SYSTEM_HPP