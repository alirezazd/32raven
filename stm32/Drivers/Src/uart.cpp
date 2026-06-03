#include "uart.hpp"

#include <cstring>

#include "error_code.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "stm32f4xx.h"
#include "system.hpp"

namespace {
// Mask IRQ priorities 5 and lower while updating shared UART TX state.
static constexpr uint32_t kMaskPri = (5u << (8u - __NVIC_PRIO_BITS)) & 0xFFu;

struct BasepriGuard {
  uint32_t old;
  explicit BasepriGuard(uint32_t new_basepri) : old(__get_BASEPRI()) {
    __set_BASEPRI(new_basepri);
    __DSB();
    __ISB();
  }
  ~BasepriGuard() {
    __set_BASEPRI(old);
    __DSB();
    __ISB();
  }
};
}  // namespace

// TX DMA stream/channel mapping (matches CubeMX MSP):
// USART1_TX -> DMA2_Stream7 Channel 4
// USART2_TX -> DMA1_Stream6 Channel 4
// USART6_TX -> DMA2_Stream6 Channel 5

static inline void DmaDisableAndWait(DMA_Stream_TypeDef *s) {
  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {
  }
}

template <UartInstance Inst>
static inline bool UartTransmitDma(const uint8_t *buf, uint16_t len) {
  if (!buf || len == 0) {
    return false;
  }

  USART_TypeDef *uart = nullptr;
  DMA_Stream_TypeDef *s = nullptr;
  DMA_TypeDef *dma = nullptr;
  uint32_t clear_flags = 0;

  if constexpr (Inst == UartInstance::kUart1) {
    uart = USART1;
    s = DMA2_Stream7;
    dma = DMA2;
    clear_flags = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 |
                  DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;
  } else if constexpr (Inst == UartInstance::kUart2) {
    uart = USART2;
    s = DMA1_Stream6;
    dma = DMA1;
    clear_flags = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                  DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
  } else {
    static_assert(Inst == UartInstance::kUart6, "Invalid Uart Instance");
    uart = USART6;
    s = DMA2_Stream6;
    dma = DMA2;
    clear_flags = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                  DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
  }

  DmaDisableAndWait(s);
  dma->HIFCR = clear_flags;

  s->PAR = reinterpret_cast<uint32_t>(&uart->DR);
  s->M0AR = reinterpret_cast<uint32_t>(buf);
  s->NDTR = len;

  // Mem->Periph, MINC, byte sizes, TC/TE/DME interrupts. DIR_0 = mem-to-periph.
  uint32_t cr = s->CR;
  cr &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
          DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_CT | DMA_SxCR_DBM);
  const uint32_t channel = (Inst == UartInstance::kUart6) ? 5u : 4u;
  cr |= (channel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DIR_0 | DMA_SxCR_MINC |
        DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
  s->CR = cr;

  // Direct mode (FIFO disabled)
  s->FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

  uart->CR3 |= USART_CR3_DMAT;
  s->CR |= DMA_SxCR_EN;
  return true;
}

// No HAL handle/DMA globals: this driver never calls HAL_UART_Init, so the
// CubeMX MSP is unreachable and --gc-sections drops it. Pin and DMA setup live
// in kGpioDefault and StartRxDma() / UartTransmitDma().

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
USART_TypeDef *Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::UartReg() {
  if constexpr (Inst == UartInstance::kUart1) {
    return USART1;
  } else if constexpr (Inst == UartInstance::kUart2) {
    return USART2;
  } else {
    static_assert(Inst == UartInstance::kUart6, "Invalid Uart Instance");
    return USART6;
  }
}

namespace {

uint32_t Apb1Hz() {
  const uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
  const uint32_t div = ppre1 < 4u ? 1u : (1u << (ppre1 - 3u));
  return SystemCoreClock / div;
}

uint32_t Apb2Hz() {
  const uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
  const uint32_t div = ppre2 < 4u ? 1u : (1u << (ppre2 - 3u));
  return SystemCoreClock / div;
}

// USARTDIV mantissa[15:4] | fraction[3:0] (or [2:0] when OVER8). The x100
// scaling keeps the fractional part precise without floating point.
uint32_t ComputeUartBrr(uint32_t pclk_hz, uint32_t baud_rate, bool over8) {
  const uint32_t scale = over8 ? 2u : 4u;
  const uint64_t div_x100 =
      (static_cast<uint64_t>(pclk_hz) * 25u) / (scale * baud_rate);
  const uint32_t mantissa = static_cast<uint32_t>(div_x100 / 100u);
  const uint32_t frac_units = over8 ? 8u : 16u;
  const uint32_t frac_x100 =
      static_cast<uint32_t>(div_x100 - mantissa * 100u) * frac_units + 50u;
  const uint32_t fraction = frac_x100 / 100u;
  if (over8) {
    return (mantissa << 4u) | ((fraction & 0xF8u) << 1u) | (fraction & 0x07u);
  }
  return (mantissa << 4u) | (fraction & 0x0Fu);
}

}  // namespace

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::Init(
    const UartConfig &config) {
  if constexpr (Inst == UartInstance::kUart1) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;
    NVIC_SetPriority(DMA2_Stream2_IRQn, irq_priority::kUart1Dma);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_SetPriority(DMA2_Stream7_IRQn, irq_priority::kUart1Dma);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  } else if constexpr (Inst == UartInstance::kUart2) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;
    NVIC_SetPriority(DMA1_Stream5_IRQn, irq_priority::kUart2Dma);
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    NVIC_SetPriority(DMA1_Stream6_IRQn, irq_priority::kUart2Dma);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  } else if constexpr (Inst == UartInstance::kUart6) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;
    NVIC_SetPriority(DMA2_Stream1_IRQn, irq_priority::kUart6Dma);
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    NVIC_SetPriority(DMA2_Stream6_IRQn, irq_priority::kUart6Dma);
    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  }
  if (initialized_) {
    return;
  }
  initialized_ = true;

  USART_TypeDef *uart = UartReg();
  uint32_t pclk_hz = 0;
  if constexpr (Inst == UartInstance::kUart1) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    pclk_hz = Apb2Hz();
  } else if constexpr (Inst == UartInstance::kUart2) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;
    pclk_hz = Apb1Hz();
  } else {
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    (void)RCC->APB2ENR;
    pclk_hz = Apb2Hz();
  }

  // Disable while reprogramming so writes to BRR / CR1-3 take effect cleanly.
  uart->CR1 &= ~USART_CR1_UE;
  // config.* enum values are raw register bit patterns (e.g. UART_PARITY_EVEN
  // == USART_CR1_PCE), OR'd straight into CR1/CR2/CR3.
  uart->CR1 = static_cast<uint32_t>(config.word_length) |
              static_cast<uint32_t>(config.parity) |
              static_cast<uint32_t>(config.over_sampling) |
              static_cast<uint32_t>(config.mode);
  uart->CR2 = static_cast<uint32_t>(config.stop_bits);
  uart->CR3 = static_cast<uint32_t>(config.hw_flow_control);
  const bool over8 = config.over_sampling == UartOverSampling::k8;
  uart->BRR = ComputeUartBrr(pclk_hz, config.baud_rate, over8);
  uart->CR1 |= USART_CR1_UE;

  StartRxDma();
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::SetBaudRate(
    uint32_t baud_rate) {
  if (!initialized_) return;

  USART_TypeDef *uart = UartReg();
  uint32_t pclk_hz = 0;
  if constexpr (Inst == UartInstance::kUart1 || Inst == UartInstance::kUart6) {
    pclk_hz = Apb2Hz();
  } else {
    pclk_hz = Apb1Hz();
  }
  const bool over8 = (uart->CR1 & USART_CR1_OVER8) != 0u;

  // Briefly drop UE to apply BRR atomically; framing config (CR1/2/3) is
  // unchanged so we can leave the rest of CR1 alone.
  const uint32_t cr1 = uart->CR1;
  uart->CR1 = cr1 & ~USART_CR1_UE;
  uart->BRR = ComputeUartBrr(pclk_hz, baud_rate, over8);
  uart->CR1 = cr1;

  StartRxDma();
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::Send(const char *str) {
  Send((const uint8_t *)str, strlen(str));
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::Send(const uint8_t *data,
                                                           size_t len) {
  for (size_t i = 0; i < len; i++) {
    // Non-blocking: drop if full
    if (!tx_buffer_.Push(data[i])) {
      tx_drop_bytes_++;
      // Can't log via UART (buffer full); signal overflow on LED instead.
      System::GetInstance().Led().Set(true);
      // TODO: Handle UART TX buffer overflow correctly in future
    }
  }

  if (!tx_busy_) {
    FlushTx();
  }
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
bool Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::Read(uint8_t &out) {
  return rx_ring_.Pop(out);
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::FlushRx() {
  rx_ring_.Clear();
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::FlushTx() {
  const uint8_t *ptr = nullptr;
  size_t len = 0;

  {
    BasepriGuard g(kMaskPri);
    if (tx_busy_) {
      return;
    }

    len = tx_buffer_.ContiguousReadable(ptr);

    if (len > 0 && ptr != nullptr) {
      if (len > 0xFFFFu) len = 0xFFFFu;
      tx_busy_ = true;
      last_dma_len_ = static_cast<uint16_t>(len);
    } else {
      // Nothing queued: clear DMAT so a stale request can't re-trigger.
      UartReg()->CR3 &= ~USART_CR3_DMAT;
      return;
    }
  }  // interrupts unmasked here

  const bool ok = UartTransmitDma<Inst>(ptr, static_cast<uint16_t>(len));
  if (!ok) {
    BasepriGuard g2(kMaskPri);
    tx_busy_ = false;
    last_dma_len_ = 0;
  }
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::IrqHandler() {
  // DMA done: consume what was sent, then drain remaining (wrap or new data).
  tx_buffer_.Consume(last_dma_len_);
  last_dma_len_ = 0;
  tx_busy_ = false;

  FlushTx();
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::HandleRxDmaError(
    uint32_t isr_flags) {
  rx_dma_err_++;
  System::GetInstance().Led().Set(true);

  // RX DMA recovery: disable stream, clear flags, reload PAR/M0AR/NDTR,
  // re-enable.
  USART_TypeDef *uart = nullptr;
  DMA_Stream_TypeDef *s = nullptr;
  DMA_TypeDef *dma = nullptr;
  uint32_t clear_flags = 0;

  if constexpr (Inst == UartInstance::kUart1) {
    uart = USART1;
    s = DMA2_Stream2;
    dma = DMA2;
    clear_flags = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                  DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
  } else if constexpr (Inst == UartInstance::kUart2) {
    uart = USART2;
    s = DMA1_Stream5;
    dma = DMA1;
    clear_flags = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                  DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
  } else {
    static_assert(Inst == UartInstance::kUart6, "Invalid Uart Instance");
    uart = USART6;
    s = DMA2_Stream1;
    dma = DMA2;
    clear_flags = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                  DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
  }

  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {
  }

  // Low/high flag register selected by stream index (Stream5 lives in high).
  if constexpr (Inst == UartInstance::kUart1) {
    dma->LIFCR = clear_flags;
  } else if constexpr (Inst == UartInstance::kUart2) {
    dma->HIFCR = clear_flags;
  } else {
    dma->LIFCR = clear_flags;
  }

  // NDTR may be corrupt/zero; reload all stream addresses to be safe.
  s->PAR = reinterpret_cast<uint32_t>(&uart->DR);
  s->M0AR = reinterpret_cast<uint32_t>(rx_dma_buf_);
  s->NDTR = RxDmaSize;

  // DMA restarts writing at index 0; resync pointer (lossy but safe).
  rx_last_pos_ = 0;

  s->CR |= DMA_SxCR_EN;
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::HandleDmaError(
    uint32_t isr_flags) {
  tx_dma_err_++;
  System::GetInstance().Led().Set(true);

  // Retry Strategy
  if (tx_retry_count_ < 3) {
    tx_retry_count_++;
    {
      BasepriGuard g(kMaskPri);
      tx_busy_ = false;
      // Keep last_dma_len_ intact so FlushTx resends the same buffer.
    }
    FlushTx();
    return;
  }

  // Retries exhausted: drop the offending packet to avoid an infinite loop.
  tx_retry_count_ = 0;
  tx_buffer_.Consume(last_dma_len_);

  {
    BasepriGuard g(kMaskPri);
    tx_busy_ = false;
    last_dma_len_ = 0;
  }

  char msg[] = "DMA ERR: 00000000\r\n";
  const char hex[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) {
    msg[16 - i] = hex[isr_flags & 0xF];
    isr_flags >>= 4;
  }
  Send(msg);
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::StartRxDma() {
  USART_TypeDef *uart = nullptr;
  DMA_Stream_TypeDef *s = nullptr;
  DMA_TypeDef *dma = nullptr;
  uint32_t clear_flags = 0;

  if constexpr (Inst == UartInstance::kUart1) {
    uart = USART1;
    s = DMA2_Stream2;
    dma = DMA2;
    // Streams 0..3 use LIFCR; 4..7 use HIFCR.
    clear_flags = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                  DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
  } else if constexpr (Inst == UartInstance::kUart2) {
    uart = USART2;
    s = DMA1_Stream5;
    dma = DMA1;
    clear_flags = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                  DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
  } else {
    static_assert(Inst == UartInstance::kUart6, "Invalid Uart Instance");
    uart = USART6;
    s = DMA2_Stream1;
    dma = DMA2;
    clear_flags = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                  DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
  }

  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {
  }

  if constexpr (Inst == UartInstance::kUart1) {
    dma->LIFCR = clear_flags;
  } else if constexpr (Inst == UartInstance::kUart2) {
    dma->HIFCR = clear_flags;
  } else {
    dma->LIFCR = clear_flags;
  }

  s->PAR = reinterpret_cast<uint32_t>(&uart->DR);
  s->M0AR = reinterpret_cast<uint32_t>(rx_dma_buf_);
  s->NDTR = RxDmaSize;
  rx_last_pos_ = 0;  // DMA starts from index 0

  // Circular periph-to-mem with MINC. DIR=00 (P2M) is the default, left unset.
  uint32_t cr = s->CR;
  cr &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
          DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DBM | DMA_SxCR_CT);

  const uint32_t channel = (Inst == UartInstance::kUart6) ? 5UL : 4UL;
  cr |= (channel << DMA_SxCR_CHSEL_Pos);
  cr |= DMA_SxCR_MINC;
  cr |= DMA_SxCR_CIRC;

  // TC/HT drive the ring drain; TE/DME feed rx_dma_err_.
  cr |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;

  s->CR = cr;

  // Enable stream before the peripheral DMA request (DMAR).
  s->CR |= DMA_SxCR_EN;
  uart->CR3 |= USART_CR3_DMAR;

  uart->CR1 |= USART_CR1_IDLEIE;
  // RXNEIE is unset (DMA RX), so FE/NE/ORE need EIE (CR3) and PE needs PEIE
  // (CR1) to raise interrupts.
  uart->CR3 |= USART_CR3_EIE;
  uart->CR1 |= USART_CR1_PEIE;

  if constexpr (Inst == UartInstance::kUart1) {
    NVIC_SetPriority(USART1_IRQn, irq_priority::kUart1);
    NVIC_EnableIRQ(USART1_IRQn);
  } else if constexpr (Inst == UartInstance::kUart2) {
    NVIC_SetPriority(USART2_IRQn, irq_priority::kUart2);
    NVIC_EnableIRQ(USART2_IRQn);
  } else {
    NVIC_SetPriority(USART6_IRQn, irq_priority::kUart6);
    NVIC_EnableIRQ(USART6_IRQn);
  }
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::OnUartInterrupt() {
  USART_TypeDef *uart = nullptr;
  if constexpr (Inst == UartInstance::kUart1) {
    uart = USART1;
  } else if constexpr (Inst == UartInstance::kUart2) {
    uart = USART2;
  } else {
    uart = USART6;
  }

  uint32_t sr = uart->SR;
  bool drain = false;

  if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    if (sr & USART_SR_ORE) uart_ore_err_++;
    if (sr & USART_SR_FE) uart_fe_err_++;
    if (sr & USART_SR_NE) uart_ne_err_++;
    if (sr & USART_SR_PE) uart_pe_err_++;
    drain = true;
  }

  if (sr & USART_SR_IDLE) {
    last_idle_time_ = System::GetInstance().Time().Micros();
    drain = true;
  }

  // SR read above + DR read below is the required sequence to clear these
  // flags.
  if (drain) {
    volatile uint32_t tmp = uart->DR;
    (void)tmp;
    DrainRx();
  }
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::OnRxHalfCplt() {
  DrainRx();
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::OnRxCplt() {
  DrainRx();
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::DrainRx() {
  DMA_Stream_TypeDef *s = nullptr;
  if constexpr (Inst == UartInstance::kUart1) {
    s = DMA2_Stream2;
  } else if constexpr (Inst == UartInstance::kUart2) {
    s = DMA1_Stream5;
  } else {
    s = DMA2_Stream1;
  }

  // NDTR counts DOWN from Size to 0
  const uint16_t current_ndtr = s->NDTR;
  uint16_t head_pos = RxDmaSize - current_ndtr;
  if (head_pos >= RxDmaSize) {
    head_pos = 0;
  }

  if (head_pos == rx_last_pos_) {
    return;
  }

  if (head_pos > rx_last_pos_) {
    // Contiguous: single span.
    const size_t len = head_pos - rx_last_pos_;
    size_t written = rx_ring_.PushBlock(&rx_dma_buf_[rx_last_pos_], len);
    rx_drop_bytes_ += (len - written);
  } else {
    // Wrapped: copy tail-to-end, then start-to-head.
    const size_t len1 = RxDmaSize - rx_last_pos_;
    size_t w1 = rx_ring_.PushBlock(&rx_dma_buf_[rx_last_pos_], len1);
    rx_drop_bytes_ += (len1 - w1);

    const size_t len2 = head_pos;
    size_t w2 = rx_ring_.PushBlock(&rx_dma_buf_[0], len2);
    rx_drop_bytes_ += (len2 - w2);
  }

  rx_last_pos_ = head_pos;
}

// Explicit instantiations
template class Uart<UartInstance::kUart1, kUartTxBufSize, kUartRxDmaSize,
                    kUartRxRingSize>;
template class Uart<UartInstance::kUart2, kUartTxBufSize, kUartRxDmaSize,
                    kUartRxRingSize>;
template class Uart<UartInstance::kUart6, kUartTxBufSize, kUartRxDmaSize,
                    kUartRxRingSize>;

extern "C" {
void Uart1DmaTxComplete() { Uart1::GetInstance().IrqHandler(); }

void Uart2DmaTxComplete() { Uart2::GetInstance().IrqHandler(); }

void Uart6DmaTxComplete() { Uart6::GetInstance().IrqHandler(); }

void Uart1DmaError(uint32_t isr_flags) {
  Uart1::GetInstance().HandleDmaError(isr_flags);
}

void Uart2DmaError(uint32_t isr_flags) {
  Uart2::GetInstance().HandleDmaError(isr_flags);
}

void Uart6DmaError(uint32_t isr_flags) {
  Uart6::GetInstance().HandleDmaError(isr_flags);
}

void Uart1RxDmaIrq() {
  // Unused
}

void Uart1RxDmaError(uint32_t isr_flags) {
  Uart1::GetInstance().HandleRxDmaError(isr_flags);
}

void Uart2RxDmaError(uint32_t isr_flags) {
  Uart2::GetInstance().HandleRxDmaError(isr_flags);
}

void Uart6RxDmaError(uint32_t isr_flags) {
  Uart6::GetInstance().HandleRxDmaError(isr_flags);
}

// Wrappers for interrupts

void Uart1OnUartInterrupt() { Uart1::GetInstance().OnUartInterrupt(); }

void Uart2OnUartInterrupt() { Uart2::GetInstance().OnUartInterrupt(); }

void Uart6OnUartInterrupt() { Uart6::GetInstance().OnUartInterrupt(); }

void Uart1OnRxHalfCplt() { Uart1::GetInstance().OnRxHalfCplt(); }

void Uart1OnRxCplt() { Uart1::GetInstance().OnRxCplt(); }

void Uart2OnRxHalfCplt() { Uart2::GetInstance().OnRxHalfCplt(); }

void Uart6OnRxHalfCplt() { Uart6::GetInstance().OnRxHalfCplt(); }

void Uart2OnRxCplt() { Uart2::GetInstance().OnRxCplt(); }

void Uart6OnRxCplt() { Uart6::GetInstance().OnRxCplt(); }
}
