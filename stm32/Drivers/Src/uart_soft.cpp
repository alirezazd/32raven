// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "uart_soft.hpp"

#include "error_code.hpp"
#include "gpio.hpp"
#include "panic.hpp"

namespace {

// TIM2 counts microseconds and wraps every ~71 minutes, so deadlines are
// compared as a signed difference rather than by magnitude.
inline bool Reached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

inline uint32_t Micros() { return TIM2->CNT; }

constexpr uint32_t kDataBits = 8u;

// Reciprocal of the tolerated baud error, so 50 is 2%. A frame is sampled at
// its midpoints, so error accumulates over ten bits before the stop bit sample
// walks off the end; 2% leaves that with room to spare.
constexpr uint32_t kMaxBaudDeviationDivisor = 50u;

}  // namespace

void UartSoft::Init(const UartSoftConfig &config) {
  if (initialized_ || config.baud_rate == 0u || config.baud_rate > 1000000u) {
    Panic(ErrorCode::Stm32::kUartSoftReinit);
  }

  // A bit period is a whole number of microseconds, because that is all TIM2
  // resolves, so the rate actually produced is 1e6 / (1e6 / baud). 19200 lands
  // on 52 us and realises 19230, off by 0.16%. 115200 would land on 8 us and
  // realise 125000 -- off by 8.5%, which desynchronises inside one byte and
  // looks like a dead ESC rather than a misconfigured one. Rejected here
  // rather than debugged on a bench.
  const uint32_t realised = 1000000u / (1000000u / config.baud_rate);
  const uint32_t deviation = (realised > config.baud_rate)
                                 ? realised - config.baud_rate
                                 : config.baud_rate - realised;
  if (deviation * kMaxBaudDeviationDivisor > config.baud_rate) {
    Panic(ErrorCode::Stm32::kUartSoftReinit);
  }

  cfg_ = config;
  SetBaudRate(config.baud_rate);
  initialized_ = true;
}

// Even parity makes the count of ones across data plus parity even, odd makes
// it odd, which is the same rule the USART applies in hardware.
bool UartSoft::ParityBit(uint8_t value) const {
  const bool odd_ones = (__builtin_popcount(value) & 1) != 0;
  return (cfg_.parity == UartParity::kOdd) ? !odd_ones : odd_ones;
}

void UartSoft::SetBaudRate(uint32_t baud_rate) {
  cfg_.baud_rate = baud_rate;
  bit_period_us_ = 1000000u / baud_rate;
  start_sample_us_ = bit_period_us_ - (bit_period_us_ / 4u);
}

void UartSoft::Open(GPIO_TypeDef *port, uint16_t pin, uint8_t restore_af) {
  port_ = port;
  pin_ = pin;
  pin_number_ = static_cast<uint8_t>(__builtin_ctz(pin));
  moder_shift_ = static_cast<uint32_t>(pin_number_) * 2u;
  restore_af_ = restore_af;

  // High before the pin becomes an output, so claiming it cannot emit a start
  // bit the far end would try to decode.
  DriveHigh();
  SetOutput();
  open_ = true;
}

void UartSoft::Close() {
  if (!open_) {
    return;
  }

  // Input first, so the pin is never driven by two owners at once during the
  // handover. Then back to the alternate function that had it -- leaving it a
  // plain input would silently mute whatever was driving the wire.
  SetInput();

  const uint32_t afr_index = pin_number_ >> 3u;
  const uint32_t afr_shift = (pin_number_ & 0x7u) * 4u;
  port_->AFR[afr_index] = (port_->AFR[afr_index] & ~(0xFu << afr_shift)) |
                          (static_cast<uint32_t>(restore_af_) << afr_shift);
  port_->MODER = (port_->MODER & ~(0x3u << moder_shift_)) |
                 (GPIO_MODE_AF_PP << moder_shift_);

  open_ = false;
}

void UartSoft::SetOutput() {
  port_->OSPEEDR |= (GPIO_SPEED_FREQ_VERY_HIGH << moder_shift_);
  port_->MODER = (port_->MODER & ~(0x3u << moder_shift_)) |
                 (GPIO_MODE_OUTPUT_PP << moder_shift_);
}

void UartSoft::SetInput() {
  // Pulled up so an ESC that is absent or unpowered reads as idle rather than
  // floating, which would otherwise look like an endless start bit.
  port_->PUPDR =
      (port_->PUPDR & ~(0x3u << moder_shift_)) | (GPIO_PULLUP << moder_shift_);
  port_->MODER &= ~(0x3u << moder_shift_);
}

void UartSoft::WriteByte(uint8_t value) {
  // One guard bit time of idle high, the start bit low, eight data bits least
  // significant first, the parity bit if configured, then the stop bits high.
  // Packed into one word so the loop is a uniform shift rather than a switch
  // per bit position.
  const uint32_t stop_bits = (cfg_.stop_bits == UartStopBits::k2) ? 2u : 1u;
  const bool has_parity = cfg_.parity != UartParity::kNone;
  const uint32_t parity_bits = has_parity ? 1u : 0u;
  const uint32_t total_bits = 1u + 1u + kDataBits + parity_bits + stop_bits;

  uint32_t frame = static_cast<uint32_t>(value) << 2u;
  frame |= 1u;  // guard: line already idle, held one bit time

  uint32_t next = 2u + kDataBits;
  if (has_parity) {
    if (ParityBit(value)) {
      frame |= 1u << next;
    }
    ++next;
  }
  for (uint32_t i = 0; i < stop_bits; ++i) {
    frame |= 1u << (next + i);
  }

  uint32_t deadline = Micros();
  for (uint32_t bit = 0; bit < total_bits; ++bit) {
    if (frame & 1u) {
      DriveHigh();
    } else {
      DriveLow();
    }
    frame >>= 1u;
    deadline += bit_period_us_;
    while (!Reached(Micros(), deadline)) {
    }
  }
}

bool UartSoft::ReadByte(uint8_t &out) {
  const uint32_t timeout_at = Micros() + cfg_.start_bit_timeout_us;
  while (LineHigh()) {
    if (Reached(Micros(), timeout_at)) {
      ++rx_timeout_err_;
      return false;
    }
  }

  // Three quarters into the start bit; every bit period after this lands in
  // the middle of a bit, which is where the sample is furthest from an edge.
  uint32_t deadline = Micros() + start_sample_us_;
  while (!Reached(Micros(), deadline)) {
  }

  const bool has_parity = cfg_.parity != UartParity::kNone;
  uint32_t frame = 0;
  const uint32_t sampled_bits = 1u + kDataBits + (has_parity ? 1u : 0u) + 1u;
  for (uint32_t bit = 0; bit < sampled_bits; ++bit) {
    if (LineHigh()) {
      frame |= 1u << bit;
    }
    if (bit + 1u == sampled_bits) {
      break;
    }
    deadline += bit_period_us_;
    while (!Reached(Micros(), deadline)) {
    }
  }

  // Bit 0 is the start bit and must be low; the last sampled bit is the stop
  // bit and must be high. Either being wrong means the sample points drifted
  // off the far end's timing, and the byte between them is not trustworthy.
  const bool start_ok = (frame & 1u) == 0u;
  const bool stop_ok = (frame & (1u << (sampled_bits - 1u))) != 0u;
  if (!start_ok || !stop_ok) {
    ++rx_framing_err_;
    return false;
  }

  const uint8_t value = static_cast<uint8_t>((frame >> 1u) & 0xFFu);
  if (has_parity) {
    const bool received = (frame & (1u << (1u + kDataBits))) != 0u;
    if (received != ParityBit(value)) {
      // Counted apart from framing: a parity error means the bits arrived on
      // time but one of them is wrong, which points at noise on the wire
      // rather than at a rate mismatch.
      ++rx_parity_err_;
      return false;
    }
  }

  out = value;
  return true;
}

void UartSoft::Send(const uint8_t *data, size_t len) {
  if (!open_ || data == nullptr) {
    return;
  }

  SetOutput();
  for (size_t i = 0; i < len; ++i) {
    // Masked per byte rather than across the whole buffer: a 256-byte page at
    // 19200 baud is 133 ms, and nothing else on this MCU survives being held
    // off that long.
    __disable_irq();
    WriteByte(data[i]);
    __enable_irq();
  }
  // Back to input immediately: the far end answers on this same wire and will
  // start driving it as soon as the last stop bit clears.
  SetInput();
}

bool UartSoft::Read(uint8_t &out) {
  if (!open_) {
    return false;
  }
  __disable_irq();
  const bool ok = ReadByte(out);
  __enable_irq();
  return ok;
}

size_t UartSoft::ReadBlock(uint8_t *out, size_t len) {
  if (!open_ || out == nullptr) {
    return 0;
  }
  size_t got = 0;
  while (got < len) {
    if (!Read(out[got])) {
      break;
    }
    ++got;
  }
  return got;
}

void UartSoft::FlushRx() {
  if (!open_) {
    return;
  }
  // Nothing is buffered -- there is no ring and no DMA -- so the only stale
  // state is a byte still on the wire. Drain until the line has been idle for
  // one full character.
  uint8_t discard = 0;
  while (Read(discard)) {
  }
}
