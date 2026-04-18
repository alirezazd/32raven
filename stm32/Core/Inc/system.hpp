#pragma once

#include "battery.hpp"
#include "board.h"
#include "button.hpp"
#include "command_handler.hpp"
#include "dshot_tim1.hpp"
#include "ee.hpp"
#include "fc_link.hpp"
#include "gpio.hpp"
#include "led.hpp"
#include "m10.hpp"
#include "m10_service.hpp"
#include "rc_receiver.hpp"
#include "spi.hpp"
#include "time_base.hpp"
#include "uart.hpp"
#include "user_config.hpp"  // For SystemConfig
#include "vehicle_state.hpp"

class System {
 public:
  static System &GetInstance() {
    static System instance;
    return instance;
  }

  // Component identifiers used by InitComponent().
  enum class Component {
    kTimeBase,
    kGpio,
    kSpi1,
    kEe,
    kBattery,
    kUart6,
    kRcReceiver,
    kLed,
    kUart1,

    // SECONDARY DRIVERS
    kSpi2,
    kDshot,
    kButton,
    kUart2,
    kM10,
    kIcm42688p
  };

  void Init(const SystemConfig &config);

  LED &Led() { return LED::GetInstance(); }

  Spi1 &GetSpi1() { return Spi1::GetInstance(); }
  Spi2 &GetSpi() { return Spi2::GetInstance(); }
  GPIO &Gpio() { return GPIO::GetInstance(); }
  TimeBase &Time() { return TimeBase::GetInstance(); }
  EE &GetEe() { return EE::GetInstance(); }
  Battery &GetBattery() { return Battery::GetInstance(); }
  Button &Btn() { return Button::GetInstance(); }
  Uart1 &GetUart() { return Uart1::GetInstance(); }
  // Alias for clarity (Console)
  Uart1 &GetUart1() { return Uart1::GetInstance(); }
  // GPS UART
  Uart2 &GetUart2() { return Uart2::GetInstance(); }
  Uart6 &GetUart6() { return Uart6::GetInstance(); }

  M10 &GetGps() { return M10::GetInstance(); }
  RcReceiver &GetRcReceiver() { return RcReceiver::GetInstance(); }
  M10Service &ServiceGps() { return gps_service_; }
  Icm42688p &GetImu42688p() { return Icm42688p::GetInstance(); }

  VehicleState &GetVehicleState() { return vehicle_state_; }
  FcLink &GetFcLink() { return FcLink::GetInstance(); }
  CommandHandler &GetCommandHandler() { return CommandHandler::GetInstance(); }

 private:
  void InitComponent(Component c);

  bool initialized_ = false;
  M10Service gps_service_;
  VehicleState vehicle_state_;

  System();
  ~System() {}
  System(const System &) = delete;
  System &operator=(const System &) = delete;
  void ConfigureSystemClock(const SystemConfig &config);
};

#define MICROS() (System::GetInstance().Time().Micros())
