// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "ahrs.hpp"
#include "attitude_controller.hpp"
#include "battery.hpp"
#include "button.hpp"
#include "command_handler.hpp"
#include "crsf_link_service.hpp"
#include "esc_bootloader.hpp"
#include "esc_service.hpp"
#include "fc_link.hpp"
#include "four_way_service.hpp"
#include "gpio.hpp"
#include "icm42688p.hpp"
#include "led.hpp"
#include "m10_service.hpp"
#include "msp_service.hpp"
#include "multirotor_mixer.hpp"
#include "rate_controller.hpp"
#include "rcc.hpp"
#include "rc_receiver.hpp"
#include "stat_publisher.hpp"
#include "time_base.hpp"
#include "uart.hpp"
#include "vehicle_state.hpp"
#include "watchdog.hpp"

class System {
 public:
  static System &GetInstance();

  // Component identifiers used by InitComponent().
  enum class Component : uint8_t {
    kRcc,
    kTimeBase,
    kGpio,
    kSpi1,
    kEe,
    kBattery,
    kUart6,
    kRcReceiver,
    kCrsfLink,
    kLed,
    kUart1,

    // SECONDARY DRIVERS
    kSpi2,
    kDshot,
    kEscTelemetry,
    kEscService,
    kUartSoft,
    kEscBootloader,
    kFourWayService,
    kMspService,
    kUsbCdc,
    kButton,
    kUart2,
    kM10,
    kIcm42688p,
    kMultirotorMixer,
    kAhrs,
    kRateController,
    kAttitudeController,

    kCount,  // sentinel: keep last
  };

  void Init();
  void SuspendFlightComponents();
  void ResumeFlightComponents();
  LED &Led() { return LED::GetInstance(); }

  ::Rcc &Rcc() { return ::Rcc::GetInstance(); }
  GPIO &Gpio() { return GPIO::GetInstance(); }
  TimeBase &Time() { return TimeBase::GetInstance(); }
  Watchdog &Wdg() { return Watchdog::GetInstance(); }
  Battery &Batt() { return Battery::GetInstance(); }
  Button &Btn() { return Button::GetInstance(); }
  Uart1 &FcUart() { return Uart1::GetInstance(); }
  Uart2 &GpsUart() { return Uart2::GetInstance(); }
  Uart6 &RcUart() { return Uart6::GetInstance(); }
  RcReceiver &RcRx() { return RcReceiver::GetInstance(); }
  CrsfLinkService &CrsfLinkSvc() { return crsf_link_service_; }
  EscService &EscSvc() { return esc_service_; }
  MspService &MspSvc() { return msp_service_; }
  FourWayService &FourWaySvc() { return four_way_service_; }
  EscBootloader &EscBootSvc() { return esc_bootloader_; }
  M10Service &GpsSvc() { return gps_service_; }
  Icm42688p &Imu() { return Icm42688p::GetInstance(); }
  multirotor_mixer::Mixer &MixerSvc() { return mixer_; }
  Ahrs &AhrsSvc() { return ahrs_; }
  RateController &RateControllerSvc() { return rate_controller_; }
  AttitudeController &AttitudeControllerSvc() { return attitude_controller_; }

  VehicleState &Vehicle() { return vehicle_state_; }
  FcLink &FcLinkSvc() { return FcLink::GetInstance(); }
  CommandHandler &GetCommandHandler() { return CommandHandler::GetInstance(); }
  StatPublisher &StatPubSvc() { return StatPublisher::GetInstance(); }

 private:
  void InitComponent(Component c);
  static void CoreInit();  // flash cache + NVIC priority grouping
  bool initialized_ = false;
  M10Service gps_service_;
  VehicleState vehicle_state_;
  CrsfLinkService crsf_link_service_;
  EscService esc_service_;
  MspService msp_service_;
  FourWayService four_way_service_;
  EscBootloader esc_bootloader_;
  multirotor_mixer::Mixer mixer_;
  Ahrs ahrs_;
  RateController rate_controller_;
  AttitudeController attitude_controller_;

  // Empty by design — call Init() explicitly to bring up the chip.
  System() = default;
  ~System() = default;
  System(const System &) = delete;
  System &operator=(const System &) = delete;
};
