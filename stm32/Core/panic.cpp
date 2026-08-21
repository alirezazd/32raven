// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "panic.hpp"


#include "error_code.hpp"
#include "message.hpp"
#include "stm32f4xx.h"

// TIM2 free-running counter works even with IRQs disabled.
static inline bool Tim2Running() { return (TIM2->CR1 & TIM_CR1_CEN) != 0; }
static inline uint32_t Tim2UsNow() { return TIM2->CNT; }  // assumes 1 MHz tick

// Wait for (*reg & mask) == want_set, bailing after timeout_us.
// Falls back to a bounded spin count when TIM2 is not running.
static bool WaitCondTimeout(volatile uint32_t *reg, uint32_t mask,
                            bool want_set, uint32_t timeout_us) {
  if (Tim2Running()) {
    const uint32_t start_time = Tim2UsNow();
    while (true) {
      const bool is_set = ((*reg) & mask) != 0;
      if (is_set == want_set) return true;
      if ((uint32_t)(Tim2UsNow() - start_time) >= timeout_us) return false;
    }
  } else {
    // Not time-accurate; scale factor conservative under typical -O2 clocks.
    volatile uint32_t spins = (timeout_us * 16U) + 1000U;
    while (spins != 0U) {
      spins = spins - 1U;
      const bool is_set = ((*reg) & mask) != 0;
      if (is_set == want_set) return true;
    }
    return false;
  }
}

// Raw hardware delay using TIM2 (assumes 1MHz tick from init)
static void DelayMs(uint32_t ms) {
  if (!Tim2Running()) {
    for (volatile uint32_t i = 0; i < ms * 10000;) {
      i = i + 1;
    }
    return;
  }
  uint32_t start = TIM2->CNT;
  uint32_t target = ms * 1000;  // 1 MHz = 1 us per tick
  while ((uint32_t)(TIM2->CNT - start) < target);
}

// Raw GPIO LED toggle (PA1, active low on this board)
static void ToggleLed() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // PA1 as output
  GPIOA->MODER &= ~(3U << (1U * 2U));
  GPIOA->MODER |= (1U << (1U * 2U));

  GPIOA->ODR ^= (1U << 1);
}

// Raw UART1 transmit (blocking, but bounded)
static void UartSend(const uint8_t *data, size_t len) {
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // Only transmit if UART is already up (UE) and TX enabled (TE).
  if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE)) !=
      (USART_CR1_UE | USART_CR1_TE)) {
    return;
  }

  // Bound the wait so panic never hangs. At 115200 baud one byte is ~87 us
  // (10 bits), so 2 ms per byte is plenty; TC allows a whole packet to drain.
  constexpr uint32_t txe_timeout_us = 2000;
  constexpr uint32_t tc_timeout_us = 20000;

  for (size_t i = 0; i < len; ++i) {
    if (!WaitCondTimeout(&USART1->SR, USART_SR_TXE, true, txe_timeout_us)) {
      return;  // do not hang panic
    }
    USART1->DR = data[i];
  }

  (void)WaitCondTimeout(&USART1->SR, USART_SR_TC, true, tc_timeout_us);
}

// AM32 reboots half a second after the DShot stream stops and replays its
// startup tune until power-off, so the panic loop keeps sending motor-stop.

// Value 0, telemetry 0, CRC 0: every bit of a motor-stop frame is a '0' bit.
// The trailing zero slots park the lines low; no completion ISR runs here.
static constexpr uint32_t kDshotFrameBits = 16u;
static constexpr uint32_t kDshotGapBits = 2u;
static constexpr uint32_t kDshotMotors = 4u;
static constexpr uint32_t kDshotFrameWords =
    (kDshotFrameBits + kDshotGapBits) * kDshotMotors;
static uint16_t g_dshot_stop_frame[kDshotFrameWords];

// CEN and MOE are set only by DShotTim1::Init and never cleared, so a panic
// before the driver exists refuses rather than faults. Under four-way
// passthrough the pins are GPIO's, out of TIM1's reach.
static bool DshotReady() {
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) == 0u) return false;
  if ((RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN) == 0u) return false;
  if ((TIM1->CR1 & TIM_CR1_CEN) == 0u) return false;
  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0u) return false;
  return true;
}

static void SendDshotStopFrame() {
  if (!DshotReady()) {
    return;
  }

  // '0'-bit duty is 3/8 of the bit period, the driver's rounding included;
  // rebuilt from ARR so nothing here trusts init order.
  const uint32_t period_ticks = (uint32_t)TIM1->ARR + 1u;
  const uint16_t t0h = (uint16_t)(((period_ticks * 3u) + 4u) / 8u);
  for (uint32_t i = 0; i < kDshotFrameBits * kDshotMotors; ++i) {
    g_dshot_stop_frame[i] = t0h;
  }
  for (uint32_t i = kDshotFrameBits * kDshotMotors; i < kDshotFrameWords; ++i) {
    g_dshot_stop_frame[i] = 0u;
  }

  // FinishAndIdle's order: requests off, stream off, flags clear. A stream
  // that will not stop means the motors are not ours to drive; bail.
  TIM1->DIER &= ~TIM_DIER_UDE;
  DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  if (!WaitCondTimeout(&DMA2_Stream5->CR, DMA_SxCR_EN, false, 1000)) {
    return;
  }
  DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

  // Burst target CCR1..CCR4, rewritten in case the fault trampled it.
  TIM1->DCR = (0x0Du << TIM_DCR_DBA_Pos) | (3u << TIM_DCR_DBL_Pos);

  // The driver's transfer setup minus the interrupt enables.
  DMA2_Stream5->CR = (6u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DIR_0 |
                     DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                     DMA_SxCR_PL_1;
  DMA2_Stream5->FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  DMA2_Stream5->PAR = (uint32_t)&TIM1->DMAR;
  DMA2_Stream5->M0AR = (uint32_t)g_dshot_stop_frame;
  DMA2_Stream5->NDTR = kDshotFrameWords;
  DMA2_Stream5->CR |= DMA_SxCR_EN;

  TIM1->DIER |= TIM_DIER_UDE;
  TIM1->EGR = TIM_EGR_UG;
}

static void SendPanicMessage(uint32_t error_code) {
  message::PanicMsg panic_msg = {};
  panic_msg.error_code = error_code;

  message::PacketBuffer<message::PanicMsg> pkt_buf{};
  size_t pkt_len =
      message::Serialize(message::MsgId::kPanic,
                         {reinterpret_cast<const uint8_t *>(&panic_msg),
                          message::PayloadLength<message::PanicMsg>()},
                         pkt_buf);

  if (pkt_len == 0) {
    return;
  }

  UartSend(pkt_buf.data(), pkt_len);
}

// C-callable entry for the fault vectors in stm32f4xx_it.c: a bare while(1)
// there says nothing, and fault priority masks the keep-alive tick.
extern "C" void PanicHardFault(void) {
  Panic(ErrorCode::Stm32::kHardFault);
}

[[noreturn]] void PanicImpl(uint32_t code) {
  // Fail safe: stop any in-flight frame and latch the lines low. Gated on the
  // TIM1 clock — touching a clock-gated peripheral (panic before the motor
  // driver is up) would fault.
  if (RCC->APB2ENR & RCC_APB2ENR_TIM1EN) {
    TIM1->DIER &= ~TIM_DIER_UDE;  // stop the burst-DMA frame requests
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
    TIM1->EGR = TIM_EGR_UG;  // latch zeros -> all four ESC lines idle low
  }

  __disable_irq();

  // Motor-stop frames at 100 Hz between blink/report passes (~100 ms). The
  // IWDG refresh keeps this deliberate disarmed state alive — a real hang
  // elsewhere still trips the watchdog — and no-ops if it never started.
  while (true) {
    SendPanicMessage(code);
    ToggleLed();
    IWDG->KR =
        0x0000AAAAu;  // IWDG reload key; raw write, panic depends on no drivers
    for (uint32_t i = 0; i < 10u; ++i) {
      SendDshotStopFrame();
      DelayMs(10);
    }
  }
}
