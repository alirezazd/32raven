#include "system.hpp"

#include "button.hpp"
#include "dshot_tim1.hpp"
#include "error_code.hpp"
#include "gpio.hpp"
#include "irq_priority.hpp"
#include "led.hpp"
#include "panic.hpp"
#include "spi.hpp"
#include "stm32_config.hpp"
#include "time_base.hpp"
#include "uart.hpp"

System::System() {
  // Constructor does nothing now, explicit init() required.
}

void System::Init(const System::Config &config) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSystemReinit);
  }
  initialized_ = true;

  HAL_Init();
  ConfigureSystemClock(config);
  NVIC_SetPriority(PendSV_IRQn, irq_priority::kPendSv);

  InitComponent(Component::kTimeBase);
  InitComponent(Component::kGpio);
  InitComponent(Component::kUart1);
  InitComponent(Component::kSpi1);
  InitComponent(Component::kEe);
  InitComponent(Component::kBattery);
  InitComponent(Component::kUart6);
  InitComponent(Component::kRcReceiver);
  InitComponent(Component::kCrsfLink);
  InitComponent(Component::kLed);
  InitComponent(Component::kSpi2);
  InitComponent(Component::kDshot);
  InitComponent(Component::kEscTelemetry);
  InitComponent(Component::kEscService);
  InitComponent(Component::kButton);
  InitComponent(Component::kUart2);
  InitComponent(Component::kM10);
  InitComponent(Component::kIcm42688p);
  InitComponent(Component::kMultirotorMixer);
  InitComponent(Component::kAttitudeEstimator);
}

void System::InitComponent(Component c) {
  switch (c) {
    case Component::kTimeBase:
      System::GetInstance().Time().Init(kTimeBaseConfig);
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
      Battery::GetInstance().Init(kBatteryConfig);
      break;
    case Component::kUart6:
      Uart6::GetInstance().Init(kUart6Config);
      break;
    case Component::kRcReceiver:
      RcReceiver::GetInstance().Init(kRcReceiverConfig, EE::GetInstance(),
                                     vehicle_state_);
      break;
    case Component::kCrsfLink:
      crsf_link_service_.Init(kCrsfLinkConfig, Uart6::GetInstance(),
                              vehicle_state_, RcReceiver::GetInstance(),
                              FcLink::GetInstance());
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
      EscTelemetry::GetInstance().Init(kEscTelemetryConfig);
      break;
    case Component::kEscService:
      esc_service_.Init(kEscServiceConfig, EscTelemetry::GetInstance(),
                        vehicle_state_);
      break;
    case Component::kButton:
      Button::GetInstance().Init(GPIO::GetInstance(), kButtonConfig);
      break;
    case Component::kUart2:
      Uart2::GetInstance().Init(kUart2Config);
      break;
    case Component::kM10:
      M10::GetInstance().Init(kM10Config);
      break;
    case Component::kIcm42688p:
      Icm42688p::GetInstance().Init(GPIO::GetInstance(), Spi2::GetInstance(),
                                    EE::GetInstance(), kIcm42688pConfig);
      break;
    case Component::kMultirotorMixer:
      // TODO: migrate to Kconfig (kMultirotorMixerConfig in stm32_config.hpp)
      // once we have a second tunable. For v1 only `idle` exists; hardcoded.
      mixer_.Init({.idle = 0.05f});
      break;
    case Component::kAttitudeEstimator:
      attitude_estimator_.Init({});
      break;
  }
}

void System::ConfigureSystemClock(const System::Config &config) {
  const bool use_hse = config.oscillator == system_clock::Oscillator::kHse;

  RCC_OscInitTypeDef osc{};
  osc.OscillatorType =
      use_hse ? RCC_OSCILLATORTYPE_HSE : RCC_OSCILLATORTYPE_HSI;
  osc.HSEState = use_hse ? RCC_HSE_ON : RCC_HSE_OFF;
  osc.HSIState = use_hse ? RCC_HSI_OFF : RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = use_hse ? RCC_PLLSOURCE_HSE : RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = config.pllm;
  osc.PLL.PLLN = config.plln;
  osc.PLL.PLLP = static_cast<uint32_t>(config.pllp);
  osc.PLL.PLLQ = config.pllq;

  RCC_ClkInitTypeDef clk{};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = static_cast<uint32_t>(config.ahb_divider);
  clk.APB1CLKDivider = static_cast<uint32_t>(config.apb1_divider);
  clk.APB2CLKDivider = static_cast<uint32_t>(config.apb2_divider);

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(static_cast<uint32_t>(config.voltage_scale));

  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    Panic(ErrorCode::Stm32::kRccOscConfigFailed);
  }
  if (HAL_RCC_ClockConfig(&clk, config.flash_latency) != HAL_OK) {
    Panic(ErrorCode::Stm32::kRccClockConfigFailed);
  }
  // HAL_RCC_EnableCSS(); TODO: Enable this when we have a backup power source
}
