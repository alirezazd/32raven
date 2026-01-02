#include "TimeBase.hpp"

#include "board.h"

void TimeBase::_init(const Config &config) {
  if (initialized_) {
    Error_Handler();
  }
  initialized_ = true;

  // Enable TIM2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  __DSB();

  // Stop timer and reset control register
  TIM2->CR1 = 0;

  // Only overflow generates update events
  TIM2->CR1 |= TIM_CR1_URS;

  // Configure prescaler and auto-reload
  TIM2->PSC = config.prescaler;
  TIM2->ARR = config.period;

  // Force update event to load PSC and ARR immediately
  TIM2->EGR = TIM_EGR_UG;

  // Reset counter
  TIM2->CNT = 0;

  // Start timer
  TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t TimeBase::_micros() const {
  // 32-bit read is atomic on Cortex-M4
  return TIM2->CNT;
}
