#include "time_base.hpp"
#include "board.h"
#include "stm32f4xx.h"

// Global handle for HAL compatibility
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

static volatile uint32_t g_tim5_tick_count = 0;

extern "C" void TimeBaseOnTim5Irq(void) { g_tim5_tick_count++; }

TimeBase &TimeBase::GetInstance() {
  static TimeBase instance;
  return instance;
}

void TimeBase::Init(const Config &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  htim2.Instance = TIM2;
  htim2.State = HAL_TIM_STATE_READY;

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  __DSB();

  TIM2->CR1 = 0;
  TIM2->CR1 |= TIM_CR1_URS;

  TIM2->PSC = config.tim2.prescaler;
  TIM2->ARR = config.tim2.period;

  TIM2->EGR = TIM_EGR_UG;
  TIM2->CNT = 0;
  TIM2->CR1 |= TIM_CR1_CEN;

  // TIM5: periodic scheduler tick (default 1kHz)
  htim5.Instance = TIM5;
  htim5.State = HAL_TIM_STATE_READY;

  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  __DSB();

  TIM5->CR1 = 0;
  TIM5->CR1 |= TIM_CR1_URS;
  if (config.tim5.autoreload_preload) {
    TIM5->CR1 |= TIM_CR1_ARPE;
  }

  TIM5->PSC = config.tim5.prescaler;
  TIM5->ARR = config.tim5.period;
  TIM5->DIER |= TIM_DIER_UIE;
  TIM5->SR = 0;

  NVIC_SetPriority(TIM5_IRQn, 7);
  NVIC_EnableIRQ(TIM5_IRQn);

  TIM5->EGR = TIM_EGR_UG;
  TIM5->CNT = 0;
  TIM5->CR1 |= TIM_CR1_CEN;
}

// Raw, monotonic, fast. Wraps about every 71.6 minutes at 1MHz.
uint32_t TimeBase::Micros() const { return TIM2->CNT; }

void TimeBase::DelayMicros(uint32_t us) const {
  uint32_t start = TIM2->CNT;
  while ((TIM2->CNT - start) < us) {
  }
}

uint32_t TimeBase::ConsumeTim5Ticks() const {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t n = g_tim5_tick_count;
  g_tim5_tick_count = 0;
  __set_PRIMASK(primask);
  return n;
}
