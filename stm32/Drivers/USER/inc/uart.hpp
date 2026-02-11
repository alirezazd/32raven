#pragma once

#include "board.h"
#include "user_config.hpp" // For kUartDefault and UartConfig
#include <cstddef>
#include <cstdint>

enum class UartInstance { kUart1, kUart2 };

#include "ring_buffer.hpp"

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
  friend class M9N; // Allow M9N::ConfigureOnce to reinit UART at different baud
                    // rates
  void Init(const UartConfig &config);
  void ReInit(const UartConfig &config);

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
  uint8_t tx_retry_count_ = 0; // For TX DMA retries
  volatile uint32_t rx_drop_bytes_ = 0;
  volatile uint32_t rx_dma_err_ = 0;
  volatile uint32_t uart_ore_err_ = 0;
  volatile uint32_t uart_fe_err_ = 0;
  volatile uint32_t uart_ne_err_ = 0;
  volatile uint32_t uart_pe_err_ = 0;
};
