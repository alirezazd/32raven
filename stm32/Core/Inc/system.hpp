#ifndef CORE_SYSTEM_HPP
#define CORE_SYSTEM_HPP

#include "board.h"
#include "button.hpp"
#include "command_handler.hpp"
#include "dshot_tim1.hpp"
#include "fc_link.hpp"
#include "gpio.hpp"
#include "icm42688p.hpp"
#include "led.hpp"
#include "m9n.hpp"
#include "m9n_service.hpp"
#include "spi.hpp"
#include "time_base.hpp"
#include "uart.hpp"
#include "user_config.hpp" // For SystemConfig
#include "vehicle_state.hpp"

class System {
public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  // INITIALIZATION ORDER: Reorder these enum values to change init sequence
  enum class Component {
    kTimeBase = 0,
    kGpio,
    kLed,
    kUart1,

    // SECONDARY DRIVERS
    kSpi1,
    kDshot,
    kButton,
    kUart2,
    kM9n,
    kIcm42688p,
    kCount // Must be last
  };

  void Init(const SystemConfig &config);

  LED &Led() { return LED::GetInstance(); }

  Spi1 &GetSpi() { return Spi1::GetInstance(); }
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
  Icm20948 &GetImu() { return Icm20948::GetInstance(); }
  Icm42688p &GetImu42688p() { return Icm42688p::GetInstance(); }

  VehicleState &GetVehicleState() { return vehicle_state_; }
  FcLink &GetFcLink() { return FcLink::GetInstance(); }
  CommandHandler &GetCommandHandler() { return CommandHandler::GetInstance(); }

private:
  void InitComponent(Component c);

  bool initialized_ = false;
  M9NService m9n_service_;
  VehicleState vehicle_state_;

  System();
  ~System() {}
  System(const System &) = delete;
  System &operator=(const System &) = delete;
  void ConfigureSystemClock(const SystemConfig &config);
};

#define MICROS() (System::GetInstance().Time().Micros())
#endif // CORE_SYSTEM_HPP
