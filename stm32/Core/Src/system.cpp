#include "system.hpp"
#include "DShotTim1.hpp"
#include "GPIO.hpp"
#include "I2C.hpp"
#include "LED.hpp"
#include "TimeBase.hpp"
#include "UserConfig.hpp"
#include "stm32f4xx.h"

System::System() {
  // Constructor does nothing now, explicit init() required.
}

void System::_init(const Config &config) {
  if (initialized_) {
    ::Error_Handler();
  }
  initialized_ = true;

  HAL_Init();
  System::SystemClock_Config(config);

  // Initialize Drivers
  GPIO::init(GPIO_DEFAULT);
  DShotTim1::init(DSHOT_TIM1_DEFAULT);
  I2C<I2CInstance::I2C_1>::init(I2C_DEFAULT);
  I2C<I2CInstance::I2C_3>::init(I2C_DEFAULT);
  TimeBase::init(TimeBase_DEFAULT);
  LED::init(LED_DEFAULT);
}

void System::SystemClock_Config(const Config &config) {
  RCC_OscInitTypeDef RCC_OscInitStruct = config.osc;
  RCC_ClkInitTypeDef RCC_ClkInitStruct = config.clk;

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(config.voltageScaling);

  /** Initializes the RCC Oscillators
   */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, config.flashLatency) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  // HAL_RCC_EnableCSS(); TODO: Enable this when we have a backup power source
}

extern "C" void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
