#include "esc_telemetry.hpp"

#include <cstring>

#include "error_code.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "stm32f4xx.h"

namespace {

static constexpr uint32_t kUsart3DmaChannel = 4u;

static inline void DmaDisableAndWait(DMA_Stream_TypeDef *stream) {
  stream->CR &= ~DMA_SxCR_EN;
  while ((stream->CR & DMA_SxCR_EN) != 0u) {
  }
}

uint32_t Apb1PrescalerDivisor() {
  const uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
  if (ppre1 < 4u) {
    return 1u;
  }
  return 1u << (ppre1 - 3u);
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

void EscTelemetry::Init(const Config &cfg) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kEscTelemetryInitFailed);
  }
  if (cfg.baud_rate == 0u || cfg.motor_pole_count < 2u ||
      cfg.response_timeout_us == 0u) {
    Panic(ErrorCode::Stm32::kEscTelemetryInitFailed);
  }
  cfg_ = cfg;
  ConfigureUart();
  StartRxDma();
  NVIC_SetPriority(USART3_IRQn, irq_priority::kEscTelemetry);
  NVIC_EnableIRQ(USART3_IRQn);
  NVIC_SetPriority(DMA1_Stream1_IRQn, irq_priority::kEscTelemetry);
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  initialized_ = true;
}

void EscTelemetry::ConfigureUart() {
  // GPIO setup for kEscTlmRx (mode/speed/pull/AF) is done by GPIO::Init()
  // via the kGpioDefault entry. Only the peripheral clock and USART3
  // configuration remain here.
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  __DSB();

  USART3->CR1 &= ~USART_CR1_UE;
  USART3->CR1 = 0;
  USART3->CR2 = 0;
  USART3->CR3 = 0;

  const uint32_t pclk1_hz = SystemCoreClock / Apb1PrescalerDivisor();
  USART3->BRR = UsartBrr(pclk1_hz, cfg_.baud_rate);
  USART3->CR3 = USART_CR3_DMAR | USART_CR3_EIE;
  USART3->CR1 = USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_PEIE | USART_CR1_UE;
}

void EscTelemetry::StartRxDma() {
  DmaDisableAndWait(DMA1_Stream1);
  DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

  DMA1_Stream1->PAR = reinterpret_cast<uint32_t>(&USART3->DR);
  DMA1_Stream1->M0AR = reinterpret_cast<uint32_t>(rx_dma_buf_);
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
  if (!initialized_ || motor_index >= kMotorCount) {
    return;
  }
  expected_motor_ = motor_index;
  expected_since_us_ = now_us;
}

void EscTelemetry::Poll(uint32_t now_us) {
  if (!initialized_) {
    return;
  }

  DrainRx();

  uint8_t byte = 0;
  while (rx_ring_.Pop(byte)) {
    ProcessByte(byte, now_us);
  }
}

EscTelemetry::Snapshot EscTelemetry::GetSnapshot() const {
  Snapshot snapshot{};
  snapshot.motors = samples_;
  snapshot.valid_mask = valid_mask_;
  snapshot.frame_count = frame_count_;
  snapshot.crc_error_count = crc_error_count_;
  snapshot.unassigned_frame_count = unassigned_frame_count_;
  snapshot.rx_drop_bytes = rx_drop_bytes_;
  snapshot.rx_dma_error_count = rx_dma_error_count_;
  snapshot.uart_error_count = uart_ore_error_count_ + uart_fe_error_count_ +
                              uart_ne_error_count_ + uart_pe_error_count_;
  return snapshot;
}

void EscTelemetry::OnUartInterrupt() {
  const uint32_t sr = USART3->SR;
  bool drain = false;

  if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u) {
    if ((sr & USART_SR_ORE) != 0u) {
      uart_ore_error_count_++;
    }
    if ((sr & USART_SR_FE) != 0u) {
      uart_fe_error_count_++;
    }
    if ((sr & USART_SR_NE) != 0u) {
      uart_ne_error_count_++;
    }
    if ((sr & USART_SR_PE) != 0u) {
      uart_pe_error_count_++;
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
  rx_dma_error_count_++;
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
  if (frame_len_ < kKissFrameSize) {
    frame_buf_[frame_len_++] = byte;
  } else {
    std::memmove(frame_buf_, frame_buf_ + 1u, kKissFrameSize - 1u);
    frame_buf_[kKissFrameSize - 1u] = byte;
  }

  if (frame_len_ != kKissFrameSize) {
    return;
  }

  if (KissCrc8(frame_buf_, kKissFrameSize - 1u) !=
      frame_buf_[kKissFrameSize - 1u]) {
    crc_error_count_++;
    return;
  }

  PublishFrame(now_us);
  frame_len_ = 0;
}

bool EscTelemetry::ExpectedMotorActive(uint32_t now_us) const {
  if (expected_motor_ >= kMotorCount) {
    return false;
  }
  return static_cast<uint32_t>(now_us - expected_since_us_) <=
         cfg_.response_timeout_us;
}

void EscTelemetry::PublishFrame(uint32_t now_us) {
  if (!ExpectedMotorActive(now_us)) {
    unassigned_frame_count_++;
    expected_motor_ = kNoMotor;
    return;
  }

  Sample sample{};
  sample.timestamp_us = now_us;
  sample.temperature_c = static_cast<int8_t>(frame_buf_[0]);
  sample.voltage_centivolts = LoadBe16(&frame_buf_[1]);
  sample.current_centiamps = LoadBe16(&frame_buf_[3]);
  sample.consumption_mah = LoadBe16(&frame_buf_[5]);
  sample.erpm_hundreds = LoadBe16(&frame_buf_[7]);
  sample.electrical_rpm = static_cast<uint32_t>(sample.erpm_hundreds) * 100u;
  const uint8_t pole_pairs = cfg_.motor_pole_count / 2u;
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
