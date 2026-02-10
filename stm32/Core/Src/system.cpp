#include "system.hpp"
#include "button.hpp"

#include "dshot_tim1.hpp"
#include "gpio.hpp"
// #include "i2c.hpp"
#include "board.h"
#include "led.hpp"
#include "spi.hpp"
#include "stm32f4xx.h"
#include "time_base.hpp"
#include "uart.hpp"
#include "user_config.hpp"

// TODO: Use menuconfig to set these values and auto generate the config

System::System() {
  // Constructor does nothing now, explicit init() required.
}

void System::Init(const SystemConfig &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  HAL_Init();
  ConfigureSystemClock(config);

  // Initialize Drivers
  // 1. Core IO/LED first for debugging
  GPIO::GetInstance().Init(kGpioDefault);
  LED::GetInstance().Init(kLedDefault);
  LED::GetInstance().Set(true); // Indicator: Boot started

  Spi<SpiInstance::kSpi1>::GetInstance().Init(kSpi1Config);
  DShotTim1::init(kDshotTim1Default);
  // I2C<I2CInstance::kI2C1>::GetInstance().Init(kI2cDefault);

  // 2. Complex drivers
  System::GetInstance().Time().Init(kTimeBaseDefault);
  Button::GetInstance().Init(kButtonDefault);
  Uart<UartInstance::kUart1>::GetInstance().Init(kUart1Config);
  Uart<UartInstance::kUart2>::GetInstance().Init(kUart2Config);
  M9N::GetInstance().Init();
  // Icm20948::GetInstance().Init(kIcm20948Config);
  Icm42688p::GetInstance().Init(kIcm42688pConfig);

  // Init Complete: LEAVE LED ON for Debugging
  // LED::GetInstance().Set(false);
}

void System::ConfigureSystemClock(const SystemConfig &config) {
  RCC_OscInitTypeDef rcc_osc_init = config.osc;
  RCC_ClkInitTypeDef rcc_clk_init = config.clk;

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(config.voltageScaling);

  /** Initializes the RCC Oscillators
   */
  if (HAL_RCC_OscConfig(&rcc_osc_init) != HAL_OK) {
    ErrorHandler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  if (HAL_RCC_ClockConfig(&rcc_clk_init, config.flashLatency) != HAL_OK) {
    ErrorHandler();
  }

  /** Enables the Clock Security System
   */
  // HAL_RCC_EnableCSS(); TODO: Enable this when we have a backup power source
}

extern "C" void ErrorHandler(void) {
  __disable_irq();
  System::GetInstance().Led().Set(true);
  while (1) {
    __NOP();
  }
}
