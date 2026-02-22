#include "system.hpp"
#include "button.hpp"
#include "panic.hpp"

#include "dshot_tim1.hpp"
#include "gpio.hpp"
// #include "i2c.hpp"
#include "board.h"
#include "icm42688p.hpp"
#include "led.hpp"
#include "spi.hpp"
#include "stm32f4xx.h"
#include "time_base.hpp"
#include "uart.hpp"
#include "user_config.hpp"

// TODO: Use menuconfig to set these values and auto generate the config
// TODO: Need a way to change some parameters at runtime

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

  // Initialize components in enum order (defined in system.hpp)
  for (int i = 0; i < static_cast<int>(Component::kCount); ++i) {
    InitComponent(static_cast<Component>(i));
  }

  // Init Complete: LEAVE LED ON for Debugging
  // LED::GetInstance().Set(false);
}

void System::InitComponent(Component c) {
  switch (c) {
  case Component::kTimeBase:
    System::GetInstance().Time().Init(kTimeBaseDefault);
    break;
  case Component::kGpio:
    GPIO::GetInstance().Init(kGpioDefault);
    break;
  case Component::kLed:
    LED::GetInstance().Init(GPIO::GetInstance(), kLedDefault);
    LED::GetInstance().Set(true);
    break;
  case Component::kUart1:
    Uart<UartInstance::kUart1>::GetInstance().Init(kUart1Config);
    break;
  case Component::kSpi1:
    Spi1::GetInstance().Init(kSpi1Config);
    break;
  case Component::kDshot:
    DShotTim1::init(kDshotTim1Default);
    break;
  case Component::kButton:
    Button::GetInstance().Init(GPIO::GetInstance(), kButtonDefault);
    break;
  case Component::kUart2:
    Uart<UartInstance::kUart2>::GetInstance().Init(kUart2Config);
    break;
  case Component::kM9n:
    M9N::GetInstance().Init(kM9nConfig);
    break;
  case Component::kIcm42688p:
    Icm42688p::GetInstance().Init(GPIO::GetInstance(),
                                  Spi1::GetInstance(),
                                  kIcm42688pConfig);
    break;
  case Component::kCount:
    break;
  }
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
