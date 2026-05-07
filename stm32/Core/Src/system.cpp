#include "system.hpp"

#include "button.hpp"
#include "dshot_tim1.hpp"
#include "error_code.hpp"
#include "gpio.hpp"
#include "panic.hpp"
// #include "i2c.hpp"
#include "board_config.hpp"
#include "led.hpp"
#include "spi.hpp"
#include "time_base.hpp"
#include "uart.hpp"

System::System() {
  // Constructor does nothing now, explicit init() required.
}

void System::Init(const SystemConfig &config) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kSystemReinit);
  }
  initialized_ = true;

  HAL_Init();
  ConfigureSystemClock(config);
  // Express-lane bottom-half: below IMU EXTI/SPI DMA, above slow/background
  // work.
  NVIC_SetPriority(PendSV_IRQn, 4);

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
  }
}

void System::ConfigureSystemClock(const SystemConfig &config) {
  RCC_OscInitTypeDef rcc_osc_init = config.osc;
  RCC_ClkInitTypeDef rcc_clk_init = config.clk;

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(config.voltage_scaling);

  /** Initializes the RCC Oscillators
   */
  if (HAL_RCC_OscConfig(&rcc_osc_init) != HAL_OK) {
    Panic(ErrorCode::Stm32::kRccOscConfigFailed);
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  if (HAL_RCC_ClockConfig(&rcc_clk_init, config.flash_latency) != HAL_OK) {
    Panic(ErrorCode::Stm32::kRccClockConfigFailed);
  }

  /** Enables the Clock Security System
   */
  // HAL_RCC_EnableCSS(); TODO: Enable this when we have a backup power source
}
