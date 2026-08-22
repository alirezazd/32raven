// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "esc_telemetry.hpp"

#include <cstring>

#include "error_code.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "rcc.hpp"
#include "stm32f4xx.h"

namespace {

static constexpr uint32_t kUsart3DmaChannel = 4u;

static inline void DmaDisableAndWait(DMA_Stream_TypeDef *stream) {
  stream->CR &= ~DMA_SxCR_EN;
  while ((stream->CR & DMA_SxCR_EN) != 0u) {
  }
}

uint32_t UsartBrr(uint32_t pclk_hz, uint32_t baud_rate) {
  return (pclk_hz + (baud_rate / 2u)) / baud_rate;
}

uint16_t LoadBe16(const uint8_t *data) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8u) |
                               data[1]);
}

uint8_t KissCrc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8u; ++bit) {
      crc = (crc & 0x80u) != 0u ? static_cast<uint8_t>((crc << 1u) ^ 0x07u)
                                : static_cast<uint8_t>(crc << 1u);
    }
  }
  return crc;
}

}  // namespace

EscTelemetry &EscTelemetry::GetInstance() {
  static EscTelemetry instance;
  return instance;
}

void EscTelemetry::Init(const Config &cfg, SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kEscTelemetryInitFailed);
  }
  if (cfg.response_timeout_us == 0u) {
    Panic(ErrorCode::Stm32::kEscTelemetryInitFailed);
  }
  cfg_ = cfg;
  blackboard_ = &blackboard;
  ConfigureUart();
  StartRxDma();
  NVIC_SetPriority(USART3_IRQn, irq_priority::kEscTelemetry);
  NVIC_EnableIRQ(USART3_IRQn);
  NVIC_SetPriority(DMA1_Stream1_IRQn, irq_priority::kEscTelemetry);
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  initialized_ = true;
}

void EscTelemetry::ConfigureUart() {
  // RX pin (mode/speed/pull/AF) is configured by GPIO::Init(); only the
  // peripheral clock and USART3 setup belong here.
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  __DSB();

  USART3->CR1 &= ~USART_CR1_UE;
  USART3->CR1 = 0;
  USART3->CR2 = 0;
  USART3->CR3 = 0;

  const uint32_t pclk1_hz = Rcc::Apb1Hz();
  USART3->BRR = UsartBrr(pclk1_hz, kBaudRate);
  USART3->CR3 = USART_CR3_DMAR | USART_CR3_EIE;
  USART3->CR1 = USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_PEIE | USART_CR1_UE;
}

void EscTelemetry::StartRxDma() {
  DmaDisableAndWait(DMA1_Stream1);
  DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

  DMA1_Stream1->PAR = reinterpret_cast<uintptr_t>(&USART3->DR);
  DMA1_Stream1->M0AR = reinterpret_cast<uintptr_t>(rx_dma_buf_);
  DMA1_Stream1->NDTR = kRxDmaSize;
  rx_last_pos_ = 0;

  uint32_t cr = DMA1_Stream1->CR;
  cr &= ~(DMA_SxCR_CHSEL | DMA_SxCR_DIR | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
          DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DBM | DMA_SxCR_CT);
  cr |= (kUsart3DmaChannel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
        DMA_SxCR_CIRC | DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE |
        DMA_SxCR_DMEIE;
  DMA1_Stream1->CR = cr;
  DMA1_Stream1->FCR &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  DMA1_Stream1->CR |= DMA_SxCR_EN;
}

void EscTelemetry::ExpectMotor(uint8_t motor_index, uint32_t now_us) {
  if (motor_index >= kMotorCount) {
    return;
  }
  expected_motor_ = motor_index;
  expected_since_us_ = now_us;
  if (expected_frame_size_ != kKissFrameSize) {
    expected_frame_size_ = kKissFrameSize;
    frame_len_ = 0;
  }
}

// The reply is 49 bytes rather than 10, so the sliding window has to be resized
// before it arrives; a partial frame from the old size would never match again.
void EscTelemetry::ExpectInfo(uint8_t motor_index, uint32_t now_us) {
  if (motor_index >= kMotorCount) {
    return;
  }
  expected_motor_ = motor_index;
  expected_since_us_ = now_us;
  expected_frame_size_ = kInfoFrameSize;
  frame_len_ = 0;
}

EscTelemetry::Info EscTelemetry::GetInfo(uint8_t motor_index) const {
  if (motor_index >= kMotorCount) {
    return Info{};
  }
  return info_[motor_index];
}

void EscTelemetry::Poll(uint32_t now_us) {
  DrainRx();

  uint8_t byte = 0;
  while (rx_ring_.Pop(byte)) {
    ProcessByte(byte, now_us);
  }

  // A reply that never came would otherwise leave the window sized for info.
  if (expected_frame_size_ == kInfoFrameSize &&
      static_cast<uint32_t>(now_us - expected_since_us_) > kInfoTimeoutUs) {
    expected_motor_ = kNoMotor;
    expected_frame_size_ = kKissFrameSize;
    frame_len_ = 0;
  }

  PublishIfChanged();
}

EscTelemetryData EscTelemetry::BuildEscTelemetryData() const {
  EscTelemetryData out{};
  out.valid_mask = valid_mask_;
  out.frame_count = frame_count_;
  out.crc_error_count = crc_error_count_;
  out.unassigned_frame_count = unassigned_frame_count_;
  out.rx_drop_bytes = rx_drop_bytes_;
  out.rx_dma_error_count = rx_dma_error_count_;
  out.uart_error_count = uart_ore_error_count_ + uart_fe_error_count_ +
                         uart_ne_error_count_ + uart_pe_error_count_;

  for (uint8_t i = 0; i < kMotorCount; ++i) {
    const Sample &src = samples_[i];
    EscTelemetryMotorData &dst = out.motors[i];
    dst.timestamp_us = src.timestamp_us;
    dst.voltage = static_cast<float>(src.voltage_centivolts) * 0.01f;
    if (cfg_.has_current) {
      dst.current = static_cast<float>(src.current_centiamps) * 0.01f;
      dst.consumption_mah = src.consumption_mah;
    } else {
      dst.current = std::nullopt;
      dst.consumption_mah = std::nullopt;
    }
    dst.electrical_rpm = src.electrical_rpm;
    dst.rpm = src.rpm;
    dst.temperature_c = src.temperature_c;
    dst.valid = src.valid;
  }
  return out;
}

void EscTelemetry::PublishIfChanged() {
  if (blackboard_ == nullptr ||
      frame_count_ == blackboard_->GetEscTelemetry().frame_count) {
    return;
  }

  blackboard_->UpdateEscTelemetry(BuildEscTelemetryData());
}

void EscTelemetry::OnUartInterrupt() {
  const uint32_t sr = USART3->SR;
  bool drain = false;

  if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u) {
    // `x = x + 1`, not `x++`: C++20 deprecates ++ on volatile scalars.
    if ((sr & USART_SR_ORE) != 0u) {
      uart_ore_error_count_ = uart_ore_error_count_ + 1u;
    }
    if ((sr & USART_SR_FE) != 0u) {
      uart_fe_error_count_ = uart_fe_error_count_ + 1u;
    }
    if ((sr & USART_SR_NE) != 0u) {
      uart_ne_error_count_ = uart_ne_error_count_ + 1u;
    }
    if ((sr & USART_SR_PE) != 0u) {
      uart_pe_error_count_ = uart_pe_error_count_ + 1u;
    }
    drain = true;
  }

  if ((sr & USART_SR_IDLE) != 0u) {
    drain = true;
  }

  if (drain) {
    volatile uint32_t tmp = USART3->DR;
    (void)tmp;
    DrainRx();
  }
}

void EscTelemetry::OnRxHalfCplt() { DrainRx(); }

void EscTelemetry::OnRxCplt() { DrainRx(); }

void EscTelemetry::HandleRxDmaError(uint32_t isr_flags) {
  (void)isr_flags;
  rx_dma_error_count_ = rx_dma_error_count_ + 1u;  // volatile-safe (C++20)
  StartRxDma();
}

void EscTelemetry::DrainRx() {
  const uint16_t current_ndtr = DMA1_Stream1->NDTR;
  uint16_t head_pos = kRxDmaSize - current_ndtr;
  if (head_pos >= kRxDmaSize) {
    head_pos = 0;
  }

  if (head_pos == rx_last_pos_) {
    return;
  }

  if (head_pos > rx_last_pos_) {
    const size_t len = head_pos - rx_last_pos_;
    const size_t written = rx_ring_.PushBlock(&rx_dma_buf_[rx_last_pos_], len);
    rx_drop_bytes_ += (len - written);
  } else {
    const size_t len1 = kRxDmaSize - rx_last_pos_;
    const size_t written1 =
        rx_ring_.PushBlock(&rx_dma_buf_[rx_last_pos_], len1);
    rx_drop_bytes_ += (len1 - written1);

    const size_t len2 = head_pos;
    const size_t written2 = rx_ring_.PushBlock(&rx_dma_buf_[0], len2);
    rx_drop_bytes_ += (len2 - written2);
  }

  rx_last_pos_ = head_pos;
}

void EscTelemetry::ProcessByte(uint8_t byte, uint32_t now_us) {
  const uint8_t size = expected_frame_size_;
  if (frame_len_ < size) {
    frame_buf_[frame_len_++] = byte;
  } else {
    std::memmove(frame_buf_, frame_buf_ + 1u, size - 1u);
    frame_buf_[size - 1u] = byte;
  }

  if (frame_len_ != size) {
    return;
  }

  if (KissCrc8(frame_buf_, size - 1u) != frame_buf_[size - 1u]) {
    crc_error_count_++;
    return;
  }

  if (size == kInfoFrameSize) {
    PublishInfo();
  } else {
    PublishFrame(now_us);
  }
  frame_len_ = 0;
}

bool EscTelemetry::ExpectedMotorActive(uint32_t now_us) const {
  if (expected_motor_ >= kMotorCount) {
    return false;
  }
  return static_cast<uint32_t>(now_us - expected_since_us_) <=
         cfg_.response_timeout_us;
}

// Byte offsets into AM32's settings page. Bytes 17-46 are identical in layouts
// 2 and 3; below 17 and above 46 they are not, so anything read from outside
// that window needs its own per-version offset.
namespace {
constexpr uint8_t kInfoEepromVersion = 1;
constexpr uint8_t kInfoVersionMajor = 3;
constexpr uint8_t kInfoVersionMinor = 4;
constexpr uint8_t kInfoDirReversed = 17;
constexpr uint8_t kInfoBiDirection = 18;
constexpr uint8_t kInfoMotorKv = 26;
constexpr uint8_t kInfoMotorPoles = 27;
constexpr uint8_t kInfoInputType = 46;

constexpr uint8_t kLayoutVersionMin = 2;
constexpr uint8_t kLayoutVersionMax = 3;
}  // namespace

void EscTelemetry::PublishInfo() {
  const uint8_t motor = expected_motor_;
  expected_motor_ = kNoMotor;
  expected_frame_size_ = kKissFrameSize;
  if (motor >= kMotorCount) {
    unassigned_frame_count_++;
    return;
  }

  Info info{};
  info.eeprom_version = frame_buf_[kInfoEepromVersion];
  info.firmware_major = frame_buf_[kInfoVersionMajor];
  info.firmware_minor = frame_buf_[kInfoVersionMinor];

  // A layout we have not checked the offsets against is reported without them
  // rather than decoded on faith; a wrong pole count is worse than none.
  if (info.eeprom_version >= kLayoutVersionMin &&
      info.eeprom_version <= kLayoutVersionMax) {
    info.reversed = frame_buf_[kInfoDirReversed] != 0u;
    info.bidirectional = frame_buf_[kInfoBiDirection] != 0u;
    info.motor_kv_raw = frame_buf_[kInfoMotorKv];
    info.input_type = frame_buf_[kInfoInputType];
    info.motor_poles = frame_buf_[kInfoMotorPoles];
    info.valid = true;
  }

  info_[motor] = info;
  frame_count_++;
}

void EscTelemetry::PublishFrame(uint32_t now_us) {
  if (!ExpectedMotorActive(now_us)) {
    unassigned_frame_count_++;
    expected_motor_ = kNoMotor;
    return;
  }

  Sample sample{};
  sample.timestamp_us = now_us;
  // KISS telemetry sends temperature as a two's-complement byte, so widening
  // into the int16_t field must sign-extend.
  // NOLINTNEXTLINE(bugprone-signed-char-misuse)
  sample.temperature_c = static_cast<int8_t>(frame_buf_[0]);
  sample.voltage_centivolts = LoadBe16(&frame_buf_[1]);
  sample.current_centiamps = LoadBe16(&frame_buf_[3]);
  sample.consumption_mah = LoadBe16(&frame_buf_[5]);
  sample.erpm_hundreds = LoadBe16(&frame_buf_[7]);
  sample.electrical_rpm = static_cast<uint32_t>(sample.erpm_hundreds) * 100u;
  // Only the ESC knows its pole count, and it is asked for it at startup. Until
  // that lands there is no honest conversion, so rpm stays zero rather than
  // carrying a guess that nothing downstream could tell apart from a reading.
  const Info &info = info_[expected_motor_];
  const uint8_t pole_pairs =
      info.valid ? static_cast<uint8_t>(info.motor_poles / 2u) : 0u;
  sample.rpm = pole_pairs == 0u ? 0u : sample.electrical_rpm / pole_pairs;
  sample.valid = true;

  samples_[expected_motor_] = sample;
  valid_mask_ |= static_cast<uint8_t>(1u << expected_motor_);
  expected_motor_ = kNoMotor;
  frame_count_++;
}

extern "C" {

void EscTelemetryOnUartInterrupt(void) {
  EscTelemetry::GetInstance().OnUartInterrupt();
}

void EscTelemetryOnRxHalfCplt(void) {
  EscTelemetry::GetInstance().OnRxHalfCplt();
}

void EscTelemetryOnRxCplt(void) { EscTelemetry::GetInstance().OnRxCplt(); }

void EscTelemetryRxDmaError(uint32_t isr_flags) {
  EscTelemetry::GetInstance().HandleRxDmaError(isr_flags);
}
}
