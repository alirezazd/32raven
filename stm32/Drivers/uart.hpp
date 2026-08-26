// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "outcome.hpp"
#include "ring_buffer.hpp"
#include "shared_state.hpp"
#include "stm32f4xx.h"

enum class UartInstance { kUart1, kUart2, kUart6 };

// Underlying values are raw USART_CRx register bit patterns from CMSIS;
// the driver ORs them straight into the registers, so they must match the
// silicon, not any HAL convention.
enum class UartWordLength : uint32_t {
  k8Bits = 0u,
  k9Bits = USART_CR1_M,
};

enum class UartStopBits : uint32_t {
  k1 = 0u,
  k2 = USART_CR2_STOP_1,
};

enum class UartParity : uint32_t {
  kNone = 0u,
  kEven = USART_CR1_PCE,
  kOdd = USART_CR1_PCE | USART_CR1_PS,
};

enum class UartMode : uint32_t {
  kRx = USART_CR1_RE,
  kTx = USART_CR1_TE,
  kTxRx = USART_CR1_RE | USART_CR1_TE,
};

enum class UartHwFlowControl : uint32_t {
  kNone = 0u,
  kRts = USART_CR3_RTSE,
  kCts = USART_CR3_CTSE,
  kRtsCts = USART_CR3_RTSE | USART_CR3_CTSE,
};

enum class UartOverSampling : uint32_t {
  k16 = 0u,
  k8 = USART_CR1_OVER8,
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

  // All or nothing. Pushing only what fits would put a frame's opening bytes
  // on the wire without its close, which every peer here reads as a checksum
  // failure rather than as a gap -- and the ESP32 panics past a threshold of
  // those. Callers streaming rather than framing can still chunk to TxFree().
  [[nodiscard]] Outcome Send(const char *str);
  [[nodiscard]] Outcome Send(const uint8_t *data, size_t len);
  // nullopt when the ring is empty.
  [[nodiscard]] std::optional<uint8_t> ReadByte();
  void FlushRx();
  void SuspendRx();
  void ResumeRx();
  void SetBaudRate(uint32_t baud_rate);
  size_t TxFree() const {
    return tx_buffer_.Capacity() - tx_buffer_.Available();
  }
  size_t TxPending() const { return tx_buffer_.Available(); }

  // Since-boot totals; SharedState::SystemHealth documents how to read them.
  UartFaults GetFaults() const {
    return UartFaults{.tx_drops = tx_drop_bytes_,
                      .tx_dma_errors = tx_dma_err_,
                      .rx_dma_errors = rx_dma_err_};
  }

  // Called from ISR
  void IrqHandler();
  void HandleDmaError(uint32_t isr_flags);
  void HandleRxDmaError();

  // RX Controls
  void StartRxDma();
  uint32_t GetLastRxTime() const { return last_idle_time_; }

  // ISR Callbacks
  void OnUartInterrupt();
  void OnRxHalfCplt();
  void OnRxCplt();

 private:
  friend class System;
  void Init(const UartConfig &config);

  Uart() = default;
  ~Uart() = default;
  Uart(const Uart &) = delete;
  Uart &operator=(const Uart &) = delete;

  USART_TypeDef *UartReg();
  bool initialized_ = false;

  RingBuffer<uint8_t, TxBufferSize> tx_buffer_;
  volatile bool tx_busy_ = false;
  uint16_t last_dma_len_ = 0;
  void FlushTx();

  // RX DMA
  uint8_t rx_dma_buf_[RxDmaSize];
  uint16_t rx_last_pos_ = 0;
  RingBuffer<uint8_t, RxRingSize> rx_ring_;
  volatile uint32_t last_idle_time_ = 0;

  void DrainRx();

  // Error Counters
  volatile uint32_t tx_drop_bytes_ = 0;
  volatile uint32_t tx_dma_err_ = 0;
  uint8_t tx_retry_count_ = 0;
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
