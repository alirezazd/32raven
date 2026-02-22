#include "panic.hpp"
#include "board.h"
#include "message.hpp"
#include "stm32f4xx.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

// -----------------------------
// Helpers
// -----------------------------

// Return a monotonically increasing microsecond-ish counter if TIM2 is running.
// This works even with IRQs disabled.
static inline bool Tim2Running() { return (TIM2->CR1 & TIM_CR1_CEN) != 0; }
static inline uint32_t Tim2UsNow() { return TIM2->CNT; } // assumes 1 MHz tick

// Wait until (cond) is true, but bail out after timeout_us.
// If TIM2 is not running, falls back to a bounded spin count.
static bool WaitCondTimeout(volatile uint32_t *reg, uint32_t mask,
                            bool want_set, uint32_t timeout_us) {
  if (Tim2Running()) {
    const uint32_t kStartTime = Tim2UsNow();
    while (true) {
      const bool kIsSet = ((*reg) & mask) != 0;
      if (kIsSet == want_set)
        return true;
      if ((uint32_t)(Tim2UsNow() - kStartTime) >= timeout_us)
        return false;
    }
  } else {
    // Bounded loop fallback. Not time-accurate, but guarantees progress.
    // Scale factor chosen to be conservative under typical -O2 clocks.
    volatile uint32_t spins = timeout_us * 16U + 1000U;
    while (spins--) {
      const bool kIsSet = ((*reg) & mask) != 0;
      if (kIsSet == want_set)
        return true;
    }
    return false;
  }
}

// Raw hardware delay using TIM2 (assumes 1MHz tick from init)
static void DelayMs(uint32_t ms) {
  if (!Tim2Running()) {
    // TIM2 not running, fallback to busy loop
    for (volatile uint32_t i = 0; i < ms * 10000; ++i)
      ;
    return;
  }
  uint32_t start = TIM2->CNT;
  uint32_t target = ms * 1000; // 1MHz = 1us per tick
  while ((uint32_t)(TIM2->CNT - start) < target)
    ;
}

// Raw GPIO LED toggle (PA1, active low on this board)
static void ToggleLed() {
  // Ensure GPIOA clock enabled
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // Configure PA1 as output if not already
  GPIOA->MODER &= ~(3U << (1U * 2U));
  GPIOA->MODER |= (1U << (1U * 2U));

  // Toggle (active low, so invert logic)
  GPIOA->ODR ^= (1U << 1);
}

// Raw UART1 transmit (blocking, but bounded)
static void UartSend(const uint8_t *data, size_t len) {
  // Ensure USART1 clock enabled
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // Only transmit if UART is already initialized and TX enabled.
  if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE)) !=
      (USART_CR1_UE | USART_CR1_TE)) {
    return;
  }

  // Timeouts chosen to avoid hanging forever in panic.
  // At 115200 baud, one byte is ~87 us on the wire (10 bits), so 2 ms is
  // plenty.
  constexpr uint32_t kTxeTimeoutUs = 2000;
  constexpr uint32_t kTcTimeoutUs = 20000; // allow a whole packet to drain

  for (size_t i = 0; i < len; ++i) {
    // Wait for TXE
    if (!WaitCondTimeout(&USART1->SR, USART_SR_TXE, true, kTxeTimeoutUs)) {
      return; // bail out, do not hang panic
    }
    USART1->DR = data[i];
  }

  // Wait for TC (transmission complete)
  (void)WaitCondTimeout(&USART1->SR, USART_SR_TC, true, kTcTimeoutUs);
}

// Send Epistole panic message with error code
static void SendPanicMessage(ErrorCode error_code) {
  // Create PanicMsg
  message::PanicMsg panic_msg = {};
  panic_msg.error_code = error_code;

  // Serialize Epistole message
  auto pkt_buf = message::MakePacketBuffer(panic_msg);
  size_t pkt_len =
      message::Serialize(message::MsgId::kPanic, (const uint8_t *)&panic_msg,
                         sizeof(message::PanicMsg), pkt_buf.data());

  // Guard: if Serialize returns something bogus, do not overrun UART loop
  if (pkt_len == 0 || pkt_len > pkt_buf.size()) {
    return;
  }

  // Send via UART1
  UartSend(pkt_buf.data(), pkt_len);
}

void Panic(ErrorCode code) {
  // Disable interrupts to prevent further corruption
  __disable_irq();

  // Toggle LED and send panic message every 500ms (as currently coded)
  while (true) {
    SendPanicMessage(code);
    ToggleLed();
    DelayMs(40);
  }
}
