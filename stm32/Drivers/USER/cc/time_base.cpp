#include "time_base.hpp"
#include "board.h"
#include "stm32f4xx.h"
#include "system.hpp"
#include <atomic>
#include <cstdio>
#include <cstring>

// Global handle for HAL compatibility
TIM_HandleTypeDef htim2;

static volatile uint32_t g_pps_drift_abs = 0;
static volatile int32_t g_pps_drift_signed = 0;
static volatile bool g_gps_synced = false;

// 2^24 = 1.0x multiplier
static volatile uint32_t g_clock_scaler = 16777216;

// PPS epoch state
static volatile uint32_t g_last_pps_capture = 0;
static volatile uint64_t g_total_pps_micros = 0;

// sequence lock for coherent reads of epoch state
static volatile uint32_t g_time_seq = 0;

uint32_t g_pps_compensation = 0;

// 1/8 smoothing on scaler updates (runs at 1Hz, trivial cost)
static constexpr uint8_t kScalerAlphaShift = 3;

static inline void TimeSeqBeginWrite() {
  g_time_seq++;
  __DMB();
}
static inline void TimeSeqEndWrite() {
  __DMB();
  g_time_seq++;
}

extern "C" void TimeBaseOnPpsIrq(uint32_t capture_val) {
  static uint32_t prev_capture = 0;
  static bool first = true;

  if (first) {
    prev_capture = capture_val;

    TimeSeqBeginWrite();
    g_last_pps_capture = capture_val;
    g_total_pps_micros = 0;
    g_clock_scaler = 16777216;
    g_gps_synced = false;
    g_pps_drift_abs = 0;
    g_pps_drift_signed = 0;
    TimeSeqEndWrite();

    first = false;
    return;
  }

  const uint32_t kDelta = capture_val - prev_capture;
  prev_capture = capture_val;

  if (kDelta > 990000 && kDelta < 1010000) {
    const int32_t kDrift = (int32_t)kDelta - 1000000;
    const uint32_t kAbsDrift = (uint32_t)(kDrift >= 0 ? kDrift : -kDrift);

    const uint32_t target_scaler =
        (uint32_t)(((uint64_t)1000000 << 24) / kDelta);

    // IIR low-pass: scaler += (target - scaler) / 2^k
    int32_t err = (int32_t)target_scaler - (int32_t)g_clock_scaler;
    uint32_t new_scaler =
        (uint32_t)((int32_t)g_clock_scaler + (err >> kScalerAlphaShift));

    TimeSeqBeginWrite();
    g_pps_drift_signed = kDrift;
    g_pps_drift_abs = kAbsDrift;
    g_clock_scaler = new_scaler;
    g_last_pps_capture = capture_val;
    g_total_pps_micros += 1000000ULL;
    g_gps_synced = true;
    TimeSeqEndWrite();
  } else {
    TimeSeqBeginWrite();
    g_pps_drift_signed = (int32_t)kDelta; // raw for debug
    g_pps_drift_abs = kDelta;
    g_gps_synced = false;
    TimeSeqEndWrite();
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

  htim2.Instance = TIM2;
  htim2.State = HAL_TIM_STATE_READY;

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  __DSB();

  GPIOA->MODER &= ~(3U << (15 * 2));
  GPIOA->MODER |= (2U << (15 * 2));

  GPIOA->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
  GPIOA->AFR[1] |= (1U << ((15 - 8) * 4));

  TIM2->CR1 = 0;
  TIM2->CR1 |= TIM_CR1_URS;

  TIM2->PSC = config.prescaler;
  TIM2->ARR = config.period;

  TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1F)) |
                (1U << TIM_CCMR1_CC1S_Pos) | (3U << TIM_CCMR1_IC1F_Pos);

  TIM2->CCER = (TIM2->CCER & ~(TIM_CCER_CC1P | TIM_CCER_CC1NP)) | TIM_CCER_CC1E;

  TIM2->DIER |= TIM_DIER_CC1IE;

  NVIC_SetPriority(TIM2_IRQn, 6);
  NVIC_EnableIRQ(TIM2_IRQn);

  TIM2->EGR = TIM_EGR_UG;
  TIM2->CNT = 0;
  TIM2->CR1 |= TIM_CR1_CEN;
}

// Raw, monotonic, fast. Wraps about every 71.6 minutes at 1MHz.
uint32_t TimeBase::Micros() const { return TIM2->CNT; }

// GPS disciplined time (your old Micros()).
uint64_t TimeBase::MicrosCorrected() const {
  for (;;) {
    uint32_t s0 = g_time_seq;
    __DMB();
    if (s0 & 1u)
      continue;

    uint32_t last_pps = g_last_pps_capture;
    uint64_t base_us = g_total_pps_micros;
    uint32_t scaler = g_clock_scaler;
    uint32_t cnt = TIM2->CNT;

    __DMB();
    uint32_t s1 = g_time_seq;
    if (s0 != s1)
      continue;

    uint32_t ticks_since_pps = cnt - last_pps;
    uint64_t corrected_elapsed = (((uint64_t)ticks_since_pps * scaler) >> 24);
    return base_us + corrected_elapsed;
  }
}

void TimeBase::DelayMicros(uint32_t us) const {
  uint32_t start = TIM2->CNT;
  while ((TIM2->CNT - start) < us) {
  }
}

bool TimeBase::IsGpsSynchronized() const { return g_gps_synced; }
uint32_t TimeBase::GetDriftMicros() const { return g_pps_drift_abs; }
