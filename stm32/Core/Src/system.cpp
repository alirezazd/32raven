#include "system.hpp"
#include "DShotTim1.hpp"
#include "GPIO.hpp"
#include "I2C.hpp"
#include "LED.hpp"
#include "TimeBase.hpp"
#include "UserConfig.hpp"
#include "board.h"
#include "stm32f4xx.h"

// TODO: Use menuconfig to set these values and auto generate the config

System::System() {
  // Constructor does nothing now, explicit init() required.
}

void System::Init(const Config &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  HAL_Init();
  ConfigureSystemClock(config);

  // Initialize Drivers
  GPIO::GetInstance().Init(kGpioDefault);
  DShotTim1::init(kDshotTim1Default);
  I2C<I2CInstance::kI2C1>::GetInstance().Init(kI2cDefault);
  I2C<I2CInstance::kI2C3>::GetInstance().Init(kI2cDefault);
  System::GetInstance().Time().Init(kTimeBaseDefault);
  LED::GetInstance().Init(kLedDefault);
}

void System::ConfigureSystemClock(const Config &config) {
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
  while (1) {
  }
}
