// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32f4xx.h"
#include "uart.hpp"

// Bit-banged half-duplex UART for pins with no USART alternate function.
//
// The four DShot outputs are why this exists: ST's pin data gives
// PE9/PE11/PE13/PE14 only TIM1 and FSMC, so the ESC bootloader link -- 8N1 on
// the same wire that carries DShot -- has no hardware option.
//
// Mirrors Uart where the two can agree (Init through System, Send/Read/FlushRx,
// SetBaudRate, the same error-counter shape). Three things it cannot mirror:
//
//   * Transfers are synchronous. There is no DMA and no ring, so Send blocks
//     for one bit period per bit and Read blocks until a byte or a timeout.
//   * The wire is shared. Send drives the line and returns it to input, so a
//     reply can only be read after the request is fully out.
//   * The pin is borrowed. Open/Close bind and release it; Uart owns its pins
//     for the life of the firmware, this one hands them back to TIM1.
//
// Timing is busy-wait against TIM2 and tolerates no preemption: a bit is 52 us
// at 19200 baud while the IMU alone interrupts every 125 us, so a byte would be
// corrupted four times over. Every byte runs with interrupts masked, and the
// caller must have stopped whatever cannot survive that -- Open takes the pin,
// it does not arbitrate for it.
struct UartSoftConfig {
  uint32_t baud_rate;
  // Eight data bits is the one thing this cannot vary, so unlike UartConfig
  // there is no word_length to ask for nine.
  UartParity parity;
  UartStopBits stop_bits;
  // How long Read waits for a falling edge. A bootloader answers within a few
  // bit times; an absent or unpowered ESC never answers at all.
  uint32_t start_bit_timeout_us;
};

class UartSoft {
 public:
  static UartSoft &GetInstance();

  // Binds the wire. Left driven high, which is idle, so the far end sees idle
  // rather than a spurious start bit the moment the pin is claimed.
  // restore_af is what Close puts the pin back to -- the alternate function
  // that owned it, so the borrow is symmetric and the caller cannot forget.
  void Open(GPIO_TypeDef *port, uint16_t pin, uint8_t restore_af);

  void Close();

  bool IsOpen() const { return open_; }

  void Send(const uint8_t *data, size_t len);
  bool ReadByte(uint8_t &out);

  // Stops at the first byte that does not arrive, so a short count means a
  // timeout rather than a partial frame worth keeping.
  size_t ReadBytes(uint8_t *out, size_t len);

  void FlushRx();
  void SetBaudRate(uint32_t baud_rate);

  uint32_t RxTimeoutCount() const { return rx_timeout_err_; }
  uint32_t RxFramingCount() const { return rx_framing_err_; }
  uint32_t RxParityCount() const { return rx_parity_err_; }

 private:
  friend class System;
  void Init(const UartSoftConfig &config);

  UartSoft() = default;
  ~UartSoft() = default;
  UartSoft(const UartSoft &) = delete;
  UartSoft &operator=(const UartSoft &) = delete;

  void SetOutput();
  void SetInput();
  bool ParityBit(uint8_t value) const;
  void ShiftOutByte(uint8_t value);
  bool ShiftInByte(uint8_t &out);

  bool LineHigh() const { return (port_->IDR & pin_) != 0u; }
  void DriveHigh() { port_->BSRR = pin_; }
  void DriveLow() { port_->BSRR = static_cast<uint32_t>(pin_) << 16u; }

  UartSoftConfig cfg_{};
  bool initialized_ = false;

  uint32_t bit_period_us_ = 0;
  // Sampled three quarters into the start bit so every later sample lands mid
  // bit rather than drifting toward an edge.
  uint32_t start_sample_us_ = 0;

  GPIO_TypeDef *port_ = nullptr;
  uint16_t pin_ = 0;
  // MODER, OSPEEDR and PUPDR are two bits per pin, so the shift is the pin
  // number doubled. Decoded once in Open rather than per bit.
  uint32_t moder_shift_ = 0;
  uint8_t pin_number_ = 0;
  uint8_t restore_af_ = 0;
  bool open_ = false;

  uint32_t rx_timeout_err_ = 0;
  uint32_t rx_framing_err_ = 0;
  uint32_t rx_parity_err_ = 0;
};
