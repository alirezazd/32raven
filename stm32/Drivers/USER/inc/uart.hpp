#pragma once

#include <cstddef>
#include <cstdint>

#include "board.h"
#include "ring_buffer.hpp"

enum class UartInstance { kUart1, kUart2, kUart6 };

enum class UartWordLength : uint32_t {
  k8Bits = UART_WORDLENGTH_8B,
  k9Bits = UART_WORDLENGTH_9B,
};

enum class UartStopBits : uint32_t {
  k1 = UART_STOPBITS_1,
  k2 = UART_STOPBITS_2,
};

enum class UartParity : uint32_t {
  kNone = UART_PARITY_NONE,
  kEven = UART_PARITY_EVEN,
  kOdd = UART_PARITY_ODD,
};

enum class UartMode : uint32_t {
  kRx = UART_MODE_RX,
  kTx = UART_MODE_TX,
  kTxRx = UART_MODE_TX_RX,
};

enum class UartHwFlowControl : uint32_t {
  kNone = UART_HWCONTROL_NONE,
  kRts = UART_HWCONTROL_RTS,
  kCts = UART_HWCONTROL_CTS,
  kRtsCts = UART_HWCONTROL_RTS_CTS,
};

enum class UartOverSampling : uint32_t {
  k16 = UART_OVERSAMPLING_16,
  k8 = UART_OVERSAMPLING_8,
};

struct UartConfig {
  uint32_t baud_rate;
  UartWordLength word_length;
  UartStopBits stop_bits;
  UartParity parity;
  UartMode mode;
  UartHwFlowControl hw_flow_control;
  UartOverSampling over_sampling;
};

constexpr size_t kUartTxBufSize = 256;
constexpr size_t kUartRxDmaSize = 128;
constexpr size_t kUartRxRingSize = 1024;

template <UartInstance Inst, size_t TxBufferSize = kUartTxBufSize,
          size_t RxDmaSize = kUartRxDmaSize,
          size_t RxRingSize = kUartRxRingSize>
class Uart {
 public:
  static Uart &GetInstance() {
    static Uart instance;
    return instance;
  }

  void Send(const char *str);
  void Send(const uint8_t *data, size_t len);
  bool Read(uint8_t &out);
  void FlushRx();
  void SetBaudRate(uint32_t baud_rate);
  size_t TxFree() const { return tx_buffer_.Capacity() - tx_buffer_.Available(); }

  // Called from ISR
  void IrqHandler();
  void HandleDmaError(uint32_t isr_flags);
  void HandleRxDmaError(uint32_t isr_flags);

  // RX Controls
  void StartRxDma();
  uint64_t GetLastRxTime() const { return last_idle_time_; }

  // ISR Callbacks
  void OnUartInterrupt();
  void OnRxHalfCplt();
  void OnRxCplt();

 private:
  friend class System;
  void Init(const UartConfig &config);

  Uart() = default;
  ~Uart() = default;

  UART_HandleTypeDef *GetHandle();
  bool initialized_ = false;

  RingBuffer<uint8_t, TxBufferSize> tx_buffer_;
  volatile bool tx_busy_ = false;
  uint16_t last_dma_len_ = 0;
  void FlushTx();

  // RX DMA
  uint8_t rx_dma_buf_[RxDmaSize];
  uint16_t rx_last_pos_ = 0;
  RingBuffer<uint8_t, RxRingSize> rx_ring_;
  volatile uint64_t last_idle_time_ = 0;

  void DrainRx();

  // Error Counters
  volatile uint32_t tx_drop_bytes_ = 0;
  volatile uint32_t tx_dma_err_ = 0;
  uint8_t tx_retry_count_ = 0;  // For TX DMA retries
  volatile uint32_t rx_drop_bytes_ = 0;
  volatile uint32_t rx_dma_err_ = 0;
  volatile uint32_t uart_ore_err_ = 0;
  volatile uint32_t uart_fe_err_ = 0;
  volatile uint32_t uart_ne_err_ = 0;
  volatile uint32_t uart_pe_err_ = 0;
};

using Uart1 = Uart<UartInstance::kUart1>;
using Uart2 = Uart<UartInstance::kUart2>;
using Uart6 = Uart<UartInstance::kUart6>;
