// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "time_base.hpp"

#include "error_code.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "stm32f4xx.h"

static volatile uint32_t g_tim5_tick_count = 0;

// Uptime lives beside the tick count for the same reason: the vector calls a
// free function, and TimeBase::GetInstance() is System's to hand out.
static SharedState *g_uptime_blackboard = nullptr;
static uint32_t g_uptime_ms = 0;
static uint32_t g_uptime_last_cnt = 0;

// Elapsed TIM2 microseconds, not ticks. Counting ticks would assume every
// interrupt runs: uart_soft masks for roughly a byte at a time, which a faster
// STM32_TIMEBASE_TIM5_TICK_HZ would outlast, and a Cortex-M folds two pendings
// of one interrupt into a single entry. A delta cannot lose time -- a late call
// just measures a longer one -- so the tick rate is free to move.
extern "C" void TimeBaseOnTim5Irq(void) {
  g_tim5_tick_count = g_tim5_tick_count + 1;

  const uint32_t delta_us =
      static_cast<uint32_t>(TIM2->CNT - g_uptime_last_cnt);
  if (delta_us < 1000u) {
    return;
  }
  const uint32_t elapsed_ms = delta_us / 1000u;
  g_uptime_ms += elapsed_ms;
  g_uptime_last_cnt += elapsed_ms * 1000u;
  if (g_uptime_blackboard != nullptr) {
    g_uptime_blackboard->UpdateUptimeMs(g_uptime_ms);
  }
}

TimeBase &TimeBase::GetInstance() {
  static TimeBase instance;
  return instance;
}

void TimeBase::Init(const Config &config, SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kTimInitFailed);
  }
  initialized_ = true;

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  __DSB();

  TIM2->CR1 = 0;
  TIM2->CR1 |= TIM_CR1_URS;

  TIM2->PSC = config.tim2.prescaler;
  TIM2->ARR = config.tim2.period;

  TIM2->EGR = TIM_EGR_UG;
  TIM2->CNT = 0;
  TIM2->CR1 |= TIM_CR1_CEN;

  // TIM5: periodic scheduler tick; the rate comes from the config.
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

  NVIC_SetPriority(TIM5_IRQn, irq_priority::kTimeBaseTim5);
  NVIC_EnableIRQ(TIM5_IRQn);

  TIM5->EGR = TIM_EGR_UG;
  TIM5->CNT = 0;
  TIM5->CR1 |= TIM_CR1_CEN;

  g_uptime_blackboard = &blackboard;
  g_uptime_last_cnt = TIM2->CNT;
}

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
