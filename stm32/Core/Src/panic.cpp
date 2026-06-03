#include "panic.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>

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
    volatile uint32_t spins = timeout_us * 16U + 1000U;
    while (spins--) {
      const bool is_set = ((*reg) & mask) != 0;
      if (is_set == want_set) return true;
    }
    return false;
  }
}

// Raw hardware delay using TIM2 (assumes 1MHz tick from init)
static void DelayMs(uint32_t ms) {
  if (!Tim2Running()) {
    for (volatile uint32_t i = 0; i < ms * 10000; ++i);
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

// Send Epistole panic message with error code
static void SendPanicMessage(uint32_t error_code) {
  message::PanicMsg panic_msg = {};
  panic_msg.error_code = error_code;

  auto pkt_buf = message::MakePacketBuffer(panic_msg);
  size_t pkt_len = message::Serialize(
      message::MsgId::kPanic, (const uint8_t *)&panic_msg,
      message::PayloadLength<message::PanicMsg>(), pkt_buf.data());

  // Reject a bogus length so UartSend can't overrun the buffer.
  if (pkt_len == 0 || pkt_len > pkt_buf.size()) {
    return;
  }

  UartSend(pkt_buf.data(), pkt_len);
}

void PanicImpl(uint32_t code) {
  // Fail safe: drive the DShot/TIM1 lines idle low so any panic disarms the
  // motors (ESCs failsafe on signal loss). Gated on the TIM1 clock — touching a
  // clock-gated peripheral (panic before the motor driver is up) would fault.
  if (RCC->APB2ENR & RCC_APB2ENR_TIM1EN) {
    TIM1->DIER &= ~TIM_DIER_UDE;  // stop the burst-DMA frame requests
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
    TIM1->EGR = TIM_EGR_UG;  // latch zeros -> all four ESC lines idle low
  }

  __disable_irq();

  // Blink/report every ~100 ms, refreshing IWDG each pass so this deliberate
  // disarmed state persists rather than being reset out of it (a real hang
  // elsewhere still trips the watchdog). No-op if the watchdog never started.
  while (true) {
    SendPanicMessage(code);
    ToggleLed();
    IWDG->KR =
        0x0000AAAAu;  // IWDG reload key; raw write, panic depends on no drivers
    DelayMs(100);
  }
}
