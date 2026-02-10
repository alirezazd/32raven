#include "time_base.hpp"
#include "board.h"
#include "stm32f4xx.h"
#include "system.hpp"
#include <atomic>
#include <cstdio>
#include <cstring>

// Global handle for HAL compatibility
TIM_HandleTypeDef htim2;

// Time tracking globals
static volatile uint32_t g_pps_drift = 0;
static volatile bool g_gps_synced = false;
static volatile uint32_t g_clock_scaler = 16777216; // 2^24 = 1.0x multiplier
static volatile uint32_t g_last_pps_capture = 0;
static volatile uint32_t g_total_pps_micros = 0;
uint32_t g_pps_compensation = 0;

extern "C" void TimeBaseOnPpsIrq(uint32_t capture_val) {
  static uint32_t prev_capture = 0;
  static bool first = true;

  if (first) {
    prev_capture = capture_val;
    g_last_pps_capture = capture_val;
    first = false;
    return;
  }

  // Calculate ticks since last PPS
  uint32_t delta = capture_val - prev_capture;
  prev_capture = capture_val;

  // Validate delta (1s +/- 10ms tolerance)
  if (delta > 990000 && delta < 1010000) {
    // Calculate absolute drift in microseconds
    int32_t drift = (int32_t)delta - 1000000;
    g_pps_drift = (uint32_t)(drift > 0 ? drift : -drift);

    // Update clock scaler: (Ideal / Actual) * 2^24
    // If clock is fast (delta > 1,000,000), scaler becomes < 1.0
    g_clock_scaler = (uint32_t)(((uint64_t)1000000 << 24) / delta);

    // Update reference point and total time
    g_last_pps_capture = capture_val;
    g_total_pps_micros += 1000000;
    g_gps_synced = true;
  } else {
    // Out of valid range - report raw delta for debugging
    g_pps_drift = delta;
    g_gps_synced = false;
  }
}

TimeBase &TimeBase::GetInstance() {
  static TimeBase instance;
  return instance;
}

void TimeBase::Init(const Config &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;
  compensation_ = config.compensation;
  g_pps_compensation = config.compensation;

  // Set HAL handle instance to avoid HardFault in ISR (calls
  // HAL_TIM_IRQHandler)
  htim2.Instance = TIM2;
  // Initialize state so HAL doesn't think it's uninitialized
  htim2.State = HAL_TIM_STATE_READY;

  // Enable TIM2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  // Enable GPIOA clock (for PA15)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  __DSB();

  // Configure PA15 as TIM2_CH1 (AF1)
  // MODER: 10 (Alternate Function)
  // AFR[1] (AFRH): 0001 (AF1) for Pin 15
  GPIOA->MODER &= ~(3U << (15 * 2));
  GPIOA->MODER |= (2U << (15 * 2));

  GPIOA->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
  GPIOA->AFR[1] |= (1U << ((15 - 8) * 4));

  // Stop timer and reset control register
  TIM2->CR1 = 0;

  // Only overflow generates update events
  TIM2->CR1 |= TIM_CR1_URS;

  // Configure prescaler and auto-reload
  TIM2->PSC = config.prescaler;
  TIM2->ARR = config.period;

  // Input Capture Channel 1 Configuration (PA15)
  // CC1S = 01 (IC1 mapped to TI1), IC1F = 1111 (filter = 15)
  TIM2->CCMR1 =
      (TIM2->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1F)) |
      (1U << TIM_CCMR1_CC1S_Pos) | // CC1S = 01 (Input, IC1 mapped to TI1)
      (15U << TIM_CCMR1_IC1F_Pos); // IC1F = 15 (max filter)

  // CC1P = 0, CC1NP = 0 (Rising edge), CC1E = 1 (Enable capture)
  TIM2->CCER = (TIM2->CCER & ~(TIM_CCER_CC1P | TIM_CCER_CC1NP)) | TIM_CCER_CC1E;

  // Enable CC1 interrupt
  TIM2->DIER |= TIM_DIER_CC1IE;

  // Enable IRQ in NVIC
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);

  // Force update event to load PSC and ARR immediately
  TIM2->EGR = TIM_EGR_UG;

  // Reset counter
  TIM2->CNT = 0;

  // Start timer
  TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t TimeBase::Micros() const {
  uint32_t current_cnt = TIM2->CNT;

  // Calculate ticks since last PPS reference point
  uint32_t ticks_since_pps = current_cnt - g_last_pps_capture;

  // Scale ticks to real microseconds using GPS-corrected scaler
  uint32_t corrected_elapsed =
      (uint32_t)(((uint64_t)ticks_since_pps * g_clock_scaler) >> 24);

  return g_total_pps_micros + corrected_elapsed;
}

void TimeBase::DelayMicros(uint32_t us) const {
  uint32_t start = TIM2->CNT;
  while ((TIM2->CNT - start) < us)
    ;
}

uint32_t TimeBase::GetDriftMicros() const { return g_pps_drift; }

bool TimeBase::IsGpsSynchronized() const { return g_gps_synced; }
