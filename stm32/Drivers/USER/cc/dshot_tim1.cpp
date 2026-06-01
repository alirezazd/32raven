#include "dshot_tim1.hpp"

#include <cstdint>

#include "error_code.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "stm32f4xx.h"

// ---------- local helpers ----------

static uint16_t DivRoundU16(uint32_t num, uint32_t den) {
  return static_cast<uint16_t>((num + (den / 2u)) / den);
}

// DShot duty ratios
static constexpr uint32_t kT1Num = 3u;
static constexpr uint32_t kT1Den = 4u;  // 75%
static constexpr uint32_t kT0Num = 3u;
static constexpr uint32_t kT0Den = 8u;  // 37.5%

// TIM1_UP DMA is fixed by the F407 request map to DMA2 Stream5, channel 6.
static constexpr uint32_t kDmaChannel = 6u;

// PWM mode 1 (0b110) in the OCxM field.
static constexpr uint32_t kOcModePwm1 = 0x6u;

// Burst-DMA target: base register = CCR1, length = 4 transfers (CCR1..CCR4).
// DBA is the CCR1 offset from TIMx_CR1 counted in 32-bit words (0x34/4 = 0x0D);
// DBL holds (transfers - 1).
static constexpr uint32_t kDcrDbaCcr1 = 0x0Du;
static constexpr uint32_t kDcrBurst4 = 3u;

static uint16_t DshotPeriodTicks(DShotMode mode) {
  switch (mode) {
    case DShotMode::kDshot600:
      return 280u - 1u;
    case DShotMode::kDshot300:
      return 560u - 1u;
    case DShotMode::kDshot150:
      return 1120u - 1u;
  }
  return 280u - 1u;
}

static inline void Dma2Stream5DisableAndWait() {
  DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream5->CR & DMA_SxCR_EN) {
  }
}

static inline void Dma2Stream5ClearFlags() {
  DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
}

// ---------- driver init ----------

void DShotTim1::Init(const Config &config) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kDshotInitFailed);
  }
  initialized_ = true;
  // SystemClock is 168MHz (HSE=8, PLLM=8, PLLN=336, PLLP=2) -> SYSCLK=168MHz.
  // TIM1 is on APB2 (84MHz); with APB2 pre != 1 the timer clock is 2 x APB2 =
  // 168MHz. DShot600 = 280 ticks, DShot300 = 560, DShot150 = 1120.
  const uint16_t period = DshotPeriodTicks(config.mode);

  DmaInit();
  Tim1Init(period);

  TIM1->DCR =
      (kDcrDbaCcr1 << TIM_DCR_DBA_Pos) | (kDcrBurst4 << TIM_DCR_DBL_Pos);

  timings_.arr = period;
  const uint32_t period_ticks = static_cast<uint32_t>(timings_.arr) + 1u;

  timings_.t1h = DivRoundU16(period_ticks * kT1Num, kT1Den);
  timings_.t0h = DivRoundU16(period_ticks * kT0Num, kT0Den);

  // idle low
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;

  StartOutputsOnce();
  busy_ = false;
}

// ---------- peripheral setup (direct register) ----------

void DShotTim1::DmaInit() {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  (void)RCC->AHB1ENR;  // read-back so the clock is up before first access

  NVIC_SetPriority(DMA2_Stream5_IRQn, irq_priority::kDshotTim1Dma);
  NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

void DShotTim1::Tim1Init(uint16_t period) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  (void)RCC->APB2ENR;

  // Up-counting, edge-aligned, clock division /1. ARR preload on so the period
  // is double-buffered. Counter stays off until StartOutputsOnce().
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->CR2 = 0;   // MMS = reset (TRGO), MSM disabled
  TIM1->SMCR = 0;  // internal clock source
  TIM1->PSC = 0;
  TIM1->ARR = period;
  TIM1->RCR = 0;

  // PWM mode 1 on CH1..CH4 with output-compare preload (OCxPE). The burst DMA
  // rewrites CCRx on every update; preload latches each new value cleanly at
  // the update boundary.
  TIM1->CCMR1 = (kOcModePwm1 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                (kOcModePwm1 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = (kOcModePwm1 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                (kOcModePwm1 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

  // Enable CC outputs, active-high (CCxP = 0). No complementary outputs.
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

  // Advanced-timer main output enable — without MOE the CCx pins stay inert.
  TIM1->BDTR = TIM_BDTR_MOE;

  // Load PSC / ARR / RCR now (HAL's base-init issues the same update event).
  TIM1->EGR = TIM_EGR_UG;
}

// ---------- runtime ----------

void DShotTim1::StartOutputsOnce() {
  // Outputs idle low (CCRx = 0); start the counter free-running. DShot frames
  // are gated afterwards by toggling the update-DMA request (UDE).
  TIM1->CR1 |= TIM_CR1_CEN;
}

bool DShotTim1::SendBitsImpl(const uint16_t *interleaved_ccr,
                             uint16_t total_bits) {
  if (!initialized_ || !interleaved_ccr || total_bits == 0) {
    return false;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (busy_) {
    __set_PRIMASK(primask);
    return false;
  }
  busy_ = true;
  __set_PRIMASK(primask);

  const uint32_t count_words = static_cast<uint32_t>(total_bits) * kMotors;
  if (!StartTransfer(interleaved_ccr, count_words)) {
    busy_ = false;
    return false;
  }
  return true;
}

bool DShotTim1::StartTransfer(const uint16_t *buf, uint32_t count_words) {
  // NDTR is 16-bit; reject anything that would not fit (mirrors HAL's guard).
  if (count_words == 0u || count_words > 0xFFFFu) {
    dma_start_fail_count_++;
    return false;
  }

  // NDTR / PAR / M0AR are read-only while the stream is enabled.
  Dma2Stream5DisableAndWait();
  Dma2Stream5ClearFlags();

  // Mem->periph, 16-bit both ends, memory auto-increment, channel 6, high
  // priority, transfer-complete + transfer-error + direct-mode-error IRQs.
  // The stream is dedicated to DShot, so a full assignment is fine.
  DMA2_Stream5->CR = (kDmaChannel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DIR_0 |
                     DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                     DMA_SxCR_PL_1 | DMA_SxCR_TCIE | DMA_SxCR_TEIE |
                     DMA_SxCR_DMEIE;
  DMA2_Stream5->FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);  // direct mode

  DMA2_Stream5->PAR = reinterpret_cast<uint32_t>(&TIM1->DMAR);
  DMA2_Stream5->M0AR = reinterpret_cast<uint32_t>(buf);
  DMA2_Stream5->NDTR = count_words;

  DMA2_Stream5->CR |= DMA_SxCR_EN;

  // Arm the timer's update-DMA request and kick the first transfer with a
  // manual update event.
  TIM1->DIER |= TIM_DIER_UDE;
  TIM1->EGR = TIM_EGR_UG;
  return true;
}

void DShotTim1::FinishAndIdle() {
  TIM1->DIER &= ~TIM_DIER_UDE;  // stop update-DMA requests
  Dma2Stream5DisableAndWait();  // abort the stream (safe if already stopped)
  Dma2Stream5ClearFlags();

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  TIM1->EGR = TIM_EGR_UG;

  busy_ = false;
}

// ---------- DMA completion (called from DMA2_Stream5_IRQHandler) ----------

extern "C" void DshotTim1DmaComplete(void) {
  DShotTim1::GetInstance().FinishAndIdle();
}

extern "C" void DshotTim1DmaError(void) {
  DShotTim1::GetInstance().FinishAndIdle();
}
