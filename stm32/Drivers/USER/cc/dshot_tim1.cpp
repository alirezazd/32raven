#include "dshot_tim1.hpp"
#include "DShotCodec.hpp"
#include "stm32f4xx_hal.h"
#include "system.hpp"
#include <cstdint>

extern "C" {
TIM_HandleTypeDef htim1;
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim); // NOLINT
}

// ---------- local helpers ----------

static uint16_t DivRoundU16(uint32_t num, uint32_t den) {
  return static_cast<uint16_t>((num + (den / 2u)) / den);
}

// DShot duty ratios
static constexpr uint32_t kT1Num = 3u;
static constexpr uint32_t kT1Den = 4u; // 75%
static constexpr uint32_t kT0Num = 3u;
static constexpr uint32_t kT0Den = 8u; // 37.5%

// ---------- driver init ----------

void DShotTim1::_init(const Config &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  uint32_t period = 0;
  // SystemClock is 168MHz (HSE=8, PLLM=8, PLLN=336, PLLP=2) -> SYSCLK=168MHz
  // TIM1 is on APB2 (84MHz), but if APB2 pre != 1, TIM1 clk = 2 * APB2 =
  // 168MHz. We assume TIM1 CLK = 168 MHz. DShot600 = 600kHz -> 1.67us. 168e6 /
  // 600e3 = 280 ticks. DShot300 = 300kHz -> 3.33us. 168e6 / 300e3 = 560 ticks.
  // DShot150 = 150kHz -> 6.67us. 168e6 / 150e3 = 1120 ticks.

  switch (config.mode) {
  case DShotMode::DSHOT600:
    period = 280 - 1;
    break;
  case DShotMode::DSHOT300:
    period = 560 - 1;
    break;
  case DShotMode::DSHOT150:
    period = 1120 - 1;
    break;
  }

  dma_init_();
  tim1_init_(static_cast<uint16_t>(period));

  TIM1->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;

  timings_.arr = static_cast<uint16_t>(htim1.Init.Period);
  const uint32_t kPeriodTicks = timings_.arr + 1u;

  timings_.t1h = DivRoundU16(kPeriodTicks * kT1Num, kT1Den);
  timings_.t0h = DivRoundU16(kPeriodTicks * kT0Num, kT0Den);

  // idle low
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  startOutputsOnce_();
  busy_ = false;
}

// ---------- CubeMX-derived init ----------

void DShotTim1::dma_init_() {
  // DMA init moved to Dma driver
}

void DShotTim1::tim1_init_(uint16_t period) {
  TIM_ClockConfigTypeDef clock_source_config = {0};
  TIM_MasterConfigTypeDef master_config = {0};
  TIM_OC_InitTypeDef config_oc = {0};
  TIM_BreakDeadTimeConfigTypeDef break_dead_time_config = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = period;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    ErrorHandler();

  clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &clock_source_config) != HAL_OK)
    ErrorHandler();

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    ErrorHandler();

  master_config.MasterOutputTrigger = TIM_TRGO_RESET;
  master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master_config) != HAL_OK)
    ErrorHandler();

  config_oc.OCMode = TIM_OCMODE_PWM1;
  config_oc.Pulse = 0;
  config_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  config_oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  config_oc.OCFastMode = TIM_OCFAST_DISABLE;
  config_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  config_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &config_oc, TIM_CHANNEL_1) != HAL_OK)
    ErrorHandler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &config_oc, TIM_CHANNEL_2) != HAL_OK)
    ErrorHandler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &config_oc, TIM_CHANNEL_3) != HAL_OK)
    ErrorHandler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &config_oc, TIM_CHANNEL_4) != HAL_OK)
    ErrorHandler();

  break_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;
  break_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;
  break_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;
  break_dead_time_config.DeadTime = 0;
  break_dead_time_config.BreakState = TIM_BREAK_DISABLE;
  break_dead_time_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  break_dead_time_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &break_dead_time_config) != HAL_OK)
    ErrorHandler();

  HAL_TIM_MspPostInit(&htim1);
}

// ---------- runtime ----------

void DShotTim1::startOutputsOnce_() {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Base_Start(&htim1);
}

bool DShotTim1::_sendBits(const uint16_t *interleaved_ccr, uint16_t totalBits) {
  if (!interleaved_ccr || totalBits == 0)
    return false;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (busy_) {
    __set_PRIMASK(primask);
    return false;
  }
  busy_ = true;
  __set_PRIMASK(primask);

  const uint32_t kCountWords = static_cast<uint32_t>(totalBits) * kMotors;
  startTransfer_(interleaved_ccr, kCountWords);
  return true;
}

volatile uint32_t g_dshot_dma_done = 0;

void DShotTim1::startTransfer_(const uint16_t *buf, uint32_t count_words) {
  g_dshot_dma_done = 0;
  if (HAL_DMA_Start_IT(&hdma_tim1_up, reinterpret_cast<uint32_t>(buf),
                       reinterpret_cast<uint32_t>(&TIM1->DMAR),
                       count_words) != HAL_OK) {
    busy_ = false;
    ErrorHandler();
    return;
  }

  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  TIM1->EGR = TIM_EGR_UG;
}

void DShotTim1::finishAndIdle() {
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  (void)HAL_DMA_Abort(&hdma_tim1_up); // safe even if already stopped // NOLINT

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
  TIM1->EGR = TIM_EGR_UG;

  busy_ = false;
}

// ---------- DMA callbacks ----------

extern "C" void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma) { // NOLINT
  if (hdma == &hdma_tim1_up) {
    g_dshot_dma_done = 1;
    DShotTim1::getInstance().finishAndIdle();
  }
}

extern "C" void HAL_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma) { // NOLINT
  if (hdma == &hdma_tim1_up) {
    DShotTim1::getInstance().finishAndIdle();
  }
}
