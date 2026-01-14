#include "uart.hpp"
#include "system.hpp"
#include "user_config.hpp"
#include <cstring>

#include "critical_section.hpp"
#include "stm32f4xx.h"

// NOTE: These streams match your CubeMX MSP:
// USART1_TX -> DMA2_Stream7 Channel 4
// USART2_TX -> DMA1_Stream6 Channel 4

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
  } else {
    uart = USART2;
    s = DMA1_Stream6;
    dma = DMA1;
    clear_flags = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                  DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
  }

  // Stop stream and clear flags
  DmaDisableAndWait(s);
  dma->HIFCR = clear_flags;

  // Program addresses and length
  s->PAR = reinterpret_cast<uint32_t>(&uart->DR);
  s->M0AR = reinterpret_cast<uint32_t>(buf);
  s->NDTR = len;

  // Configure stream explicitly (minimal set, independent of HAL runtime):
  // Channel 4, Mem->Periph, MINC, byte sizes, TC interrupt enabled.
  uint32_t cr = s->CR;
  cr &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
          DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_CT | DMA_SxCR_DBM);
  cr |= (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DIR_0 | DMA_SxCR_MINC |
        DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
  s->CR = cr;

  // Direct mode (FIFO disabled)
  s->FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

  // Enable UART TX DMA requests
  uart->CR3 |= USART_CR3_DMAT;

  // Start
  s->CR |= DMA_SxCR_EN;
  return true;
}

extern "C" {
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
UART_HandleTypeDef *
Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::GetHandle() {
  if constexpr (Inst == UartInstance::kUart1) {
    return &huart1;
  } else {
    static_assert(Inst == UartInstance::kUart2, "Invalid Uart Instance");
    return &huart2;
  }
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::Init(
    const UartConfig &config) {
  if constexpr (Inst == UartInstance::kUart1) {
    __HAL_RCC_DMA2_CLK_ENABLE();
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    /* DMA2_Stream7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  } else if constexpr (Inst == UartInstance::kUart2) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    /* DMA1_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  }
  if (initialized_) {
    return;
  }
  initialized_ = true;

  UART_HandleTypeDef *handle = GetHandle();
  if (!handle) {
    ErrorHandler();
    return;
  }

  if constexpr (Inst == UartInstance::kUart1) {
    handle->Instance = USART1;
  } else if constexpr (Inst == UartInstance::kUart2) {
    handle->Instance = USART2;
  }

  handle->Init.BaudRate = config.baudRate;
  handle->Init.WordLength = config.wordLength;
  handle->Init.StopBits = config.stopBits;
  handle->Init.Parity = config.parity;
  handle->Init.Mode = config.mode;
  handle->Init.HwFlowCtl = config.hwFlowCtl;
  handle->Init.OverSampling = config.overSampling;

  if (HAL_UART_Init(handle) != HAL_OK) {
    ErrorHandler();
  }

  StartRxDma();
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::ReInit(
    const UartConfig &config) {
  initialized_ = false;
  Init(config);
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
      // Overflow: Turn on LED and drop data.
      // We cannot log this error via UART because the buffer is full.
      System::GetInstance().Led().Set(true);
      // TODO: Handle UART TX buffer overflow correctly in future
    }
  }

  // Kick off transmission if not already running
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
      if (len > 0xFFFFu)
        len = 0xFFFFu;
      tx_busy_ = true;
      last_dma_len_ = static_cast<uint16_t>(len);
    } else {
      // No data to send, so we can disable DMAT to save power/clean state
      UART_HandleTypeDef *handle = GetHandle();
      handle->Instance->CR3 &= ~USART_CR3_DMAT;
      return;
    }
  } // interrupts unmasked here

  const bool kOk = UartTransmitDma<Inst>(ptr, static_cast<uint16_t>(len));
  if (!kOk) {
    BasepriGuard g2(kMaskPri);
    tx_busy_ = false;
    last_dma_len_ = 0;
  }
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::IrqHandler() {
  // DMA finished, consume the data we just sent
  tx_buffer_.Consume(last_dma_len_);
  last_dma_len_ = 0;
  tx_busy_ = false;

  // Try to send more (handle wrap-around or new data)
  // Try to send more (handle wrap-around or new data)
  FlushTx();
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::HandleRxDmaError(
    uint32_t isr_flags) {
  rx_dma_err_++;
  System::GetInstance().Led().Set(true);

  // RX DMA Failure Recovery Strategy:
  // 1. Disable Stream
  // 2. Clear Flags
  // 3. Re-configure NDTR/M0AR/PAR
  // 4. Re-enable Stream

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
  } else {
    uart = USART2;
    s = DMA1_Stream5;
    dma = DMA1;
    clear_flags = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                  DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
  }

  // 1. Disable Stream and Wait
  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {
  }

  // 2. Clear Flags (ISR flags passed might need clearing too, but we clear all)
  if constexpr (Inst == UartInstance::kUart1) {
    dma->LIFCR = clear_flags;
  } else {
    dma->HIFCR = clear_flags;
  }

  // 3. Re-configure
  // Note: CR is likely still correct, but NDTR might be corrupted or zero.
  // PAR/M0AR might be fine, but safe to reload.
  s->PAR = reinterpret_cast<uint32_t>(&uart->DR);
  s->M0AR = reinterpret_cast<uint32_t>(rx_dma_buf_);
  s->NDTR = RxDmaSize;

  // Reset our software pointers too to stay in sync with a fresh DMA start?
  // If DMA restarts at index 0, we must assume it writes to index 0 next.
  // However, our RingBuffer might have data.
  // If we reset DMA, it starts writing at 0.
  // We should update rx_last_pos_ to 0. (Lossy recovery, but safe)
  rx_last_pos_ = 0;

  // 4. Re-enable Stream
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
    // Reset state to allow retry
    {
      BasepriGuard g(kMaskPri);
      tx_busy_ = false;
      // Do NOT consume last_dma_len_, so we retry sending it.
      // last_dma_len_ remains valid.
    }
    FlushTx();
    return;
  }

  // Retries failed. Drop packet and report error.
  tx_retry_count_ = 0;

  // Robustness: Drop the packet that caused the error to avoid infinite loops.
  tx_buffer_.Consume(last_dma_len_);

  // Reset state
  {
    BasepriGuard g(kMaskPri);
    tx_busy_ = false;
    last_dma_len_ = 0;
  }

  // Send Error Message
  char msg[] = "DMA ERR: 00000000\r\n";
  const char kHex[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) {
    msg[16 - i] = kHex[isr_flags & 0xF];
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
    // Stream2 is 0..3 Low
    clear_flags = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                  DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
  } else {
    uart = USART2;
    s = DMA1_Stream5;
    dma = DMA1;
    // Stream5 is 4..7 High
    clear_flags = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                  DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
  }

  // 1. Disable Stream
  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {
  }

  // 2. Clear Flags
  if constexpr (Inst == UartInstance::kUart1) {
    dma->LIFCR = clear_flags;
  } else {
    dma->HIFCR = clear_flags;
  }

  // 3. Set PAR (Peripheral Address)
  s->PAR = reinterpret_cast<uint32_t>(&uart->DR);

  // 4. Set M0AR (Memory 0 Address)
  s->M0AR = reinterpret_cast<uint32_t>(rx_dma_buf_);

  // 5. Set NDTR (Number of Data to Transfer)
  s->NDTR = RxDmaSize;
  rx_last_pos_ = 0; // Reset software pointer since DMA starts from 0

  // 6. Configure CR
  // CHSEL=4 (100), MINC, CIRC
  // Priority Medium (optional, default is Low=00)
  uint32_t cr = s->CR;
  // Clear bits
  cr &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
          DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DBM | DMA_SxCR_CT);

  // Set bits
  cr |= (4UL << DMA_SxCR_CHSEL_Pos); // Channel 4
  // DIR is 00 (Periph-to-Mem) by default, no need to set
  cr |= DMA_SxCR_MINC;
  cr |= DMA_SxCR_CIRC;

  // Enable TC (Transfer Complete) and HT (Half Transfer) Interrupts
  // Also RX Error Interrupts are not standardly enabled here, but we can if we
  // want to handle DMA errors. User wanted rx_dma_err_.
  cr |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;

  s->CR = cr;

  // 7. Enable Stream (Must be before Peripheral DMAR)
  s->CR |= DMA_SxCR_EN;

  // 8. Enable DMAR in UART
  uart->CR3 |= USART_CR3_DMAR;

  // 9. Enable IDLE Interrupt
  uart->CR1 |= USART_CR1_IDLEIE;
  // Enable Error Interrupts (PE, ORE, NE, FE) are enabled via PEIE and EIE?
  // HAL usually handles this but we are custom.
  // To get ORE/NE/FE interrupts, we need EIE bit in CR3? Or just PEIE in CR1
  // and RXNEIE implies errors? ORE/FE/NE/PE generate interrupt if RXNEIE is
  // set? No. Reference Manual: PE interrupt enabled by PEIE bit in CR1. ORE,
  // NE, FE interrupts enabled by EIE bit in CR3? Wait, actually ORE/NE/FE use
  // the same vector as RXNE? "The interrupt is generated if the RXNEIE bit is
  // set" for ORE. "The interrupt is generated if the EIE bit is set" for
  // FE/NE/ORE. We want to detect errors. We should enable EIE in CR3? Since we
  // are using DMA (DMAR), we might not have RXNEIE set? We are NOT using
  // RXNEIE. So we probably need EIE in CR3 to get error interrupts.
  uart->CR3 |= USART_CR3_EIE;
  uart->CR1 |= USART_CR1_PEIE;

  // Enable Global Interrupt
  if constexpr (Inst == UartInstance::kUart1) {
    HAL_NVIC_SetPriority(USART1_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  } else {
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

template <UartInstance Inst, size_t TxBufferSize, size_t RxDmaSize,
          size_t RxRingSize>
void Uart<Inst, TxBufferSize, RxDmaSize, RxRingSize>::OnUartInterrupt() {
  USART_TypeDef *uart = nullptr;
  if constexpr (Inst == UartInstance::kUart1) {
    uart = USART1;
  } else {
    uart = USART2;
  }

  uint32_t sr = uart->SR;
  bool drain = false;

  // Check for Errors
  if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    if (sr & USART_SR_ORE)
      uart_ore_err_++;
    if (sr & USART_SR_FE)
      uart_fe_err_++;
    if (sr & USART_SR_NE)
      uart_ne_err_++;
    if (sr & USART_SR_PE)
      uart_pe_err_++;
    drain = true;
  }

  // Check for IDLE line interrupt
  if (sr & USART_SR_IDLE) {
    last_idle_time_ = System::GetInstance().Time().Micros();
    drain = true;
  }

  // Clear flags (SR read above + DR read here) and Drain
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
  } else {
    s = DMA1_Stream5;
  }

  // NDTR counts DOWN from Size to 0
  const uint16_t kCurrentNdtr = s->NDTR;
  uint16_t head_pos = RxDmaSize - kCurrentNdtr;
  if (head_pos >= RxDmaSize) {
    head_pos = 0;
  }

  // No new data
  if (head_pos == rx_last_pos_) {
    return;
  }

  // Calculate new data
  if (head_pos > rx_last_pos_) {
    // Linear capability (Head > Tail)
    const size_t kLen = head_pos - rx_last_pos_;
    size_t written = rx_ring_.PushBlock(&rx_dma_buf_[rx_last_pos_], kLen);
    rx_drop_bytes_ += (kLen - written);
  } else {
    // Wrap-around capability (Head < Tail)

    // 1. Tail to End
    const size_t kLen1 = RxDmaSize - rx_last_pos_;
    size_t w1 = rx_ring_.PushBlock(&rx_dma_buf_[rx_last_pos_], kLen1);
    rx_drop_bytes_ += (kLen1 - w1);

    // 2. Start to Head
    const size_t kLen2 = head_pos;
    size_t w2 = rx_ring_.PushBlock(&rx_dma_buf_[0], kLen2);
    rx_drop_bytes_ += (kLen2 - w2);
  }

  rx_last_pos_ = head_pos;
}

// Explicit Instantiations
// Explicit Instantiations
template class Uart<UartInstance::kUart1, kUartTxBufSize, kUartRxDmaSize,
                    kUartRxRingSize>;
template class Uart<UartInstance::kUart2, kUartTxBufSize, kUartRxDmaSize,
                    kUartRxRingSize>;

extern "C" {
void Uart1DmaTxComplete() {
  Uart<UartInstance::kUart1, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .IrqHandler();
}

void Uart2DmaTxComplete() {
  Uart<UartInstance::kUart2, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .IrqHandler();
}

void Uart1DmaError(uint32_t isr_flags) {
  Uart<UartInstance::kUart1, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .HandleDmaError(isr_flags);
}

void Uart2DmaError(uint32_t isr_flags) {
  Uart<UartInstance::kUart2, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .HandleDmaError(isr_flags);
}

void Uart1RxDmaIrq() {
  // Unused
}

void Uart1RxDmaError(uint32_t isr_flags) {
  Uart<UartInstance::kUart1, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .HandleRxDmaError(isr_flags);
}

void Uart2RxDmaError(uint32_t isr_flags) {
  Uart<UartInstance::kUart2, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .HandleRxDmaError(isr_flags);
}

// Wrappers for interrupts

void Uart1OnUartInterrupt() {
  Uart<UartInstance::kUart1, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .OnUartInterrupt();
}

void Uart2OnUartInterrupt() {
  Uart<UartInstance::kUart2, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .OnUartInterrupt();
}

void Uart1OnRxHalfCplt() {
  Uart<UartInstance::kUart1, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .OnRxHalfCplt();
}

void Uart1OnRxCplt() {
  Uart<UartInstance::kUart1, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .OnRxCplt();
}

void Uart2OnRxHalfCplt() {
  Uart<UartInstance::kUart2, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .OnRxHalfCplt();
}

void Uart2OnRxCplt() {
  Uart<UartInstance::kUart2, kUartTxBufSize, kUartRxDmaSize,
       kUartRxRingSize>::GetInstance()
      .OnRxCplt();
}
}