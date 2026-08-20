// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "system.hpp"

#include <array>
#include <cstddef>

#include "button.hpp"
#include "dshot_tim1.hpp"
#include "error_code.hpp"
#include "gpio.hpp"
#include "irq_priority.hpp"
#include "led.hpp"
#include "panic.hpp"
#include "rcc.hpp"
#include "sdio.hpp"
#include "spi.hpp"
#include "stm32_config.hpp"
#include "time_base.hpp"
#include "uart.hpp"
#include "usb_cdc.hpp"
#include "watchdog.hpp"

namespace {
// Bring-up order, which deliberately differs from declaration order.
constexpr std::array<System::Component,
                     static_cast<std::size_t>(System::Component::kCount)>
    kInitOrder{
        System::Component::kRcc,
        System::Component::kTimeBase,
        System::Component::kGpio,
        System::Component::kUart1,
        System::Component::kSpi1,
        System::Component::kEe,
        System::Component::kBattery,
        System::Component::kUart6,
        System::Component::kRcReceiver,
        System::Component::kCrsfLink,
        System::Component::kLed,
        System::Component::kSpi2,
        System::Component::kDshot,
        System::Component::kEscTelemetry,
        System::Component::kEscService,
        System::Component::kUsbCdc,
        System::Component::kUartSoft,
        System::Component::kEscBootloader,
        System::Component::kFourWayService,
        System::Component::kMspService,
        System::Component::kButton,
        System::Component::kUart2,
        System::Component::kM10,
        System::Component::kIcm42688p,
        System::Component::kMultirotorMixer,
        System::Component::kAhrs,
        System::Component::kRateController,
        System::Component::kAttitudeController,
        System::Component::kSentinel,
        System::Component::kStatPublisher,
        System::Component::kSdio,
        System::Component::kLogService,
        System::Component::kMscService,
    };

// -Werror=switch already ties Component to InitComponent's switch; this ties it
// to the boot order, so an enumerator that is never brought up fails the build
// instead of shipping with its peripheral clock still gated off.
consteval bool InitOrderCoversEveryComponent() {
  std::array<bool, static_cast<std::size_t>(System::Component::kCount)> seen{};
  for (const System::Component c : kInitOrder) {
    const auto i = static_cast<std::size_t>(c);
    if (i >= seen.size() || seen[i]) {
      return false;
    }
    seen[i] = true;
  }
  for (const bool s : seen) {
    if (!s) {
      return false;
    }
  }
  return true;
}
static_assert(InitOrderCoversEveryComponent());
}  // namespace

// Out-of-line so the `static System` lives in this one TU rather than emitting
// a linkonce/COMDAT copy per includer.
System &System::GetInstance() {
  static System instance;
  return instance;
}

void System::Init() {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSystemReinit);
  }
  initialized_ = true;

  CoreInit();
  NVIC_SetPriority(PendSV_IRQn, irq_priority::kPendSv);

  for (const Component c : kInitOrder) {
    InitComponent(c);
  }

  // Arm the watchdog last so blocking bring-up can't trip it. From here the
  // main loop (and the panic loop) must keep feeding it.
  Wdg().Init();
}

void System::Poll(uint32_t now_us) {
  SentinelSvc().Supervise(now_us);
  Batt().Poll(now_us);
  StatPubSvc().Poll(now_us);
  // Last, so what the publisher just queued goes out on this pass.
  FcLinkSvc().Poll();
}

void System::SuspendFlightComponents() {
  // Order matters only in that the sample interrupt goes first: it is the one
  // that drives the cascade, and the two receive paths merely feed it.
  Imu().SuspendSampling();
  GpsUart().SuspendRx();
  RcUart().SuspendRx();
  (void)EscSvc().StopAll();
}

void System::ResumeFlightComponents() {
  GpsUart().ResumeRx();
  RcUart().ResumeRx();
  Imu().ResumeSampling();
}

void System::CoreInit() {
  FLASH->ACR |= FLASH_ACR_ICEN;
  FLASH->ACR |= FLASH_ACR_DCEN;
  FLASH->ACR |= FLASH_ACR_PRFTEN;
  NVIC_SetPriorityGrouping(irq_priority::kPriorityGrouping);
}

void System::InitComponent(Component c) {
  switch (c) {
    case Component::kRcc:
      ::Rcc::GetInstance().Init(kRccConfig);
      break;
    case Component::kTimeBase:
      System::GetInstance().Time().Init(kTimeBaseConfig, blackboard_);
      break;
    case Component::kGpio:
      GPIO::GetInstance().Init(kGpioDefault);
      break;
    case Component::kSpi1:
      Spi1::GetInstance().Init(kEeSpi1Config);
      break;
    case Component::kEe:
      EE::GetInstance().Init(GPIO::GetInstance(), Spi1::GetInstance());
      break;
    case Component::kBattery:
      Battery::GetInstance().Init(kBatteryConfig, blackboard_);
      break;
    case Component::kUart6:
      Uart6::GetInstance().Init(kUart6Config);
      break;
    case Component::kRcReceiver:
      RcReceiver::GetInstance().Init(kRcReceiverConfig, EE::GetInstance(),
                                     blackboard_);
      break;
    case Component::kCrsfLink:
      crsf_link_service_.Init(kCrsfLinkConfig, Uart6::GetInstance(),
                              blackboard_, RcReceiver::GetInstance());
      break;
    case Component::kSentinel:
      sentinel_.Init(kSentinelConfig, blackboard_, esc_service_,
                     rate_controller_, Icm42688p::GetInstance(),
                     FcLink::GetInstance());
      break;
    case Component::kStatPublisher:
      StatPublisher::GetInstance().Init(
          kStatPublisherConfig, blackboard_, FcLink::GetInstance(),
          crsf_link_service_, TimeBase::GetInstance().Micros());
      break;
    case Component::kLed:
      LED::GetInstance().Init(GPIO::GetInstance(), kLedConfig);
      LED::GetInstance().Set(true);
      break;
    case Component::kUart1:
      Uart1::GetInstance().Init(kUart1Config);
      break;
    case Component::kSpi2:
      Spi2::GetInstance().Init(kSpi2Config);
      break;
    case Component::kDshot:
      DShotTim1::GetInstance().Init(kDshotTim1Config);
      break;
    case Component::kEscTelemetry:
      EscTelemetry::GetInstance().Init(kEscTelemetryConfig, blackboard_);
      break;
    case Component::kEscService:
      esc_service_.Init(kEscServiceConfig, EscTelemetry::GetInstance(),
                        blackboard_);
      break;
    case Component::kUsbCdc:
      UsbCdc::GetInstance().Init(kUsbCdcConfig);
      break;
    case Component::kUartSoft:
      UartSoft::GetInstance().Init(kEscBootloaderUartConfig);
      break;
    case Component::kEscBootloader:
      esc_bootloader_.Init(UartSoft::GetInstance());
      break;
    case Component::kFourWayService:
      four_way_service_.Init(UsbCdc::GetInstance(), esc_bootloader_);
      break;
    case Component::kMspService:
      msp_service_.Init(kMspServiceConfig, UsbCdc::GetInstance(), blackboard_,
                        four_way_service_, esc_service_);
      break;
    case Component::kButton:
      Button::GetInstance().Init(GPIO::GetInstance(), kButtonConfig);
      break;
    case Component::kUart2:
      Uart2::GetInstance().Init(kUart2Config);
      break;
    case Component::kM10:
      M10::GetInstance().Init(kM10Config);
      gps_service_.Init(Uart2::GetInstance(), blackboard_);
      break;
    case Component::kIcm42688p:
      Icm42688p::GetInstance().Init(GPIO::GetInstance(), Spi2::GetInstance(),
                                    EE::GetInstance(), kIcm42688pConfig,
                                    blackboard_);
      break;
    case Component::kMultirotorMixer:
      mixer_.Init(kMultirotorMixerConfig, blackboard_);
      break;
    case Component::kAhrs:
      ahrs_.Init(kAhrsConfig);
      break;
    case Component::kRateController:
      rate_controller_.Init(kRateControllerConfig);
      break;
    case Component::kAttitudeController:
      attitude_controller_.Init(kAttitudeControllerConfig);
      break;
    case Component::kSdio:
      Sdio::GetInstance().Init();
      break;
    case Component::kLogService:
      log_service_.Init(kLogServiceConfig, blackboard_);
      break;
    case Component::kMscService:
      msc_service_.Init(UsbCdc::GetInstance(), log_service_, blackboard_);
      break;
    case Component::kCount:
      break;
  }
}
