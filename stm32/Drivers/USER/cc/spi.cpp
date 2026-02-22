#include "spi.hpp"
#include "system.hpp"

template <SpiInstance Inst> void Spi<Inst>::Init(const SpiConfig &config) {
  /* DMA controller clock enable */
  //__HAL_RCC_DMA2_CLK_ENABLE();
  EnableDmaClk();

  /* DMA interrupt init (now called manually by user to set priority) */

  // Bare Metal Init
  SPI_TypeDef *spi = Hw();

  // Enable Clock
  EnableSpiClk();

  // 1. Disable SPI
  spi->CR1 &= ~SPI_CR1_SPE;

  // 2. Configure CR1
  // Base configuration: Master, 8-bit, Soft NSS (SSM=1, SSI=1)
  uint32_t cr1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;

  // Polarity
  if (config.polarity == SpiPolarity::kHigh) {
    cr1 |= SPI_CR1_CPOL;
  }

  // Phase
  if (config.phase == SpiPhase::k2Edge) {
    cr1 |= SPI_CR1_CPHA;
  }

  // Bit Order
  if (config.bit_order == SpiBitOrder::kLsbFirst) {
    cr1 |= SPI_CR1_LSBFIRST;
  }

  // Baud Rate Prescaler: bits [5:3]
  cr1 |= (static_cast<uint32_t>(config.prescaler) << SPI_CR1_BR_Pos);

  // Data Size (DFF) is 0 for 8-bit, so nothing to add.

  spi->CR1 = cr1;

  // 3. Configure CR2
  // Motorola frame format (TI Mode off), interrupts disabled, etc.
  spi->CR2 = 0;

  // 4. Clear residual flags (OVR) before enabling
  volatile uint32_t tmp __attribute__((unused));
  tmp = spi->DR;
  tmp = spi->SR;

  // 5. Enable SPI
  Enable();
  initialized_ = true;
}

template <SpiInstance Inst> void Spi<Inst>::EnableDmaClk() {
  if constexpr (Inst == SpiInstance::kSpi1) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR; // readback delay
  } else {
    static_assert(Inst == SpiInstance::kSpi1, "Unsupported SPI instance");
  }
}

template <SpiInstance Inst> void Spi<Inst>::EnableSpiClk() {
  if constexpr (Inst == SpiInstance::kSpi1) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR; // readback delay
  } else {
    static_assert(Inst == SpiInstance::kSpi1, "Unsupported SPI instance");
  }
}

template <SpiInstance Inst> void Spi<Inst>::EnableIrqs(uint32_t priority) {
  if constexpr (Inst == SpiInstance::kSpi1) {
    /* DMA2_Stream0_IRQn interrupt configuration */
    NVIC_SetPriority(DMA2_Stream0_IRQn, priority);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    /* DMA2_Stream3_IRQn interrupt configuration */
    NVIC_SetPriority(DMA2_Stream3_IRQn, priority);
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  } else {
    static_assert(Inst == SpiInstance::kSpi1, "Unsupported SPI instance");
  }
}

template <SpiInstance Inst>
void Spi<Inst>::TxRx(const uint8_t *tx, uint8_t *rx, size_t len) {
  if (len == 0)
    return;

  SPI_TypeDef *spi = Hw();
  const uint8_t *tx_ptr = tx;
  uint8_t *rx_ptr = rx;
  size_t i = 0;

  // Ensure SPI is enabled
  if (!(spi->CR1 & SPI_CR1_SPE)) {
    Enable();
  }

  while (i < len) {
    // Wait until TXE (Transmit Empty) is set
    while (!(spi->SR & SPI_SR_TXE)) {
      // Yield or timeout? Blocking for now.
    };

    // Write Data
    *reinterpret_cast<volatile uint8_t *>(&spi->DR) = tx_ptr ? *tx_ptr++ : 0xFF;

    // Wait until RXNE (Receive Not Empty) is set
    while (!(spi->SR & SPI_SR_RXNE)) {
      // Yield or timeout?
    };

    // Read Data
    uint8_t d =
        *reinterpret_cast<volatile uint8_t *>(&spi->DR); // Read clears RXNE
    if (rx_ptr) {
      *rx_ptr++ = d;
    }
    i++;
  }

  // Strictly wait for BSY to clear so CS can be deasserted safely
  while (spi->SR & SPI_SR_BSY) {
  }

  // Clear flags (Hygiene) - only if OVR set
  if (spi->SR & SPI_SR_OVR) {
    volatile uint32_t tmp __attribute__((unused));
    tmp = spi->DR;
    tmp = spi->SR;
  }
}

template <SpiInstance Inst> uint8_t Spi<Inst>::TxRxByte(uint8_t tx) {
  uint8_t rx = 0;
  TxRx(&tx, &rx, 1);
  return rx;
}

template <SpiInstance Inst>
void Spi<Inst>::Write(const uint8_t *tx, size_t len) {
  TxRx(tx, nullptr, len);
}

template <SpiInstance Inst> void Spi<Inst>::Read(uint8_t *rx, size_t len) {
  TxRx(nullptr, rx, len);
}

template <SpiInstance Inst> void Spi<Inst>::SetPrescaler(SpiPrescaler rate) {
  if (busy_) {
    ErrorHandler();
    return;
  }
  SPI_TypeDef *spi = Hw();
  Disable();

  uint32_t cr1 = spi->CR1;
  cr1 &= ~SPI_CR1_BR; // Clear old BR
  cr1 |= (static_cast<uint32_t>(rate) << SPI_CR1_BR_Pos);
  spi->CR1 = cr1;

  Enable();
}

template <SpiInstance Inst> void Spi<Inst>::Enable() {
  Hw()->CR1 |= SPI_CR1_SPE;
}

template <SpiInstance Inst> void Spi<Inst>::Disable() {
  SPI_TypeDef *spi = Hw();
  // 1. Wait for BSY to clear
  while (spi->SR & SPI_SR_BSY) {
  }
  // 2. Disable SPI
  spi->CR1 &= ~SPI_CR1_SPE;

  // 3. Clear OVR (Read DR then SR) unconditionally
  volatile uint32_t tmp __attribute__((unused));
  tmp = spi->DR;
  tmp = spi->SR;
}

static inline void DmaDisableAndWait(DMA_Stream_TypeDef *s) {
  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {
  }
}

template <SpiInstance Inst> bool Spi<Inst>::Busy() const { return busy_; }

template <SpiInstance Inst>
bool Spi<Inst>::StartTxRxDma(const uint8_t *tx, uint8_t *rx, size_t len,
                             SpiDoneCb cb, void *user) {
  if (busy_ || len == 0 || len > 0xFFFF) {
    return false;
  }
  busy_ = true;
  cb_ = cb;
  user_ = user;
  tx_ = tx;
  rx_ = rx;
  len_ = static_cast<uint16_t>(len);

  SPI_TypeDef *spi = Hw();

  // Pre-flight drain: Ensure DR is empty and OVR is cleared
  while (spi->SR & SPI_SR_RXNE) {
    volatile uint32_t tmp = spi->DR;
    (void)tmp;
  }
  // Clear OVR if it was set during drain or before
  if (spi->SR & SPI_SR_OVR) {
    volatile uint32_t tmp = spi->DR;
    tmp = spi->SR;
    (void)tmp;
  }

  DMA_Stream_TypeDef *rx_stream = nullptr;
  DMA_Stream_TypeDef *tx_stream = nullptr;
  DMA_TypeDef *dma = nullptr;
  uint32_t rx_clear_flags = 0;
  uint32_t tx_clear_flags = 0;

  if constexpr (Inst == SpiInstance::kSpi1) {
    // SPI1 RX: DMA2_Stream0_Ch3
    // SPI1 TX: DMA2_Stream3_Ch3
    dma = DMA2;
    rx_stream = DMA2_Stream0;
    tx_stream = DMA2_Stream3;

    // Stream 0 (Low 0-3)
    rx_clear_flags = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                     DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    // Stream 3 (Low 0-3)
    tx_clear_flags = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                     DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

  } else {
    // Unsupported
    return false;
  }

  // 1. Disable Streams
  DmaDisableAndWait(rx_stream);
  DmaDisableAndWait(tx_stream);

  // Hardening: Disable SPI DMA requests to ensure clean state
  spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  // 2. Clear Flags
  dma->LIFCR = rx_clear_flags | tx_clear_flags;

  // Hardening: Clear FCR (Direct Mode default)
  rx_stream->FCR = 0;
  tx_stream->FCR = 0;

  // 3. Configure RX Stream (Stream0)
  rx_stream->PAR = reinterpret_cast<uint32_t>(&spi->DR);
  rx_stream->NDTR = len;
  if (rx) {
    rx_stream->M0AR = reinterpret_cast<uint32_t>(rx);
  } else {
    rx_stream->M0AR = reinterpret_cast<uint32_t>(&dummy_rx_);
  }

  uint32_t rx_cr = 0;
  // Channel 3
  rx_cr |= (3UL << DMA_SxCR_CHSEL_Pos);
  // DIR: 00 (Periph-to-Memory) -> Default
  // Priority: Very High
  rx_cr |= DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
  // MINC: Only if rx buffer provided
  if (rx) {
    rx_cr |= DMA_SxCR_MINC;
  }
  // Interrupts: TCIE, TEIE, DMEIE
  rx_cr |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;

  rx_stream->CR = rx_cr;

  // 4. Configure TX Stream (Stream3)
  tx_stream->PAR = reinterpret_cast<uint32_t>(&spi->DR);
  tx_stream->NDTR = len;
  if (tx) {
    tx_stream->M0AR = reinterpret_cast<uint32_t>(tx);
  } else {
    tx_stream->M0AR = reinterpret_cast<uint32_t>(&dummy_tx_);
  }

  uint32_t tx_cr = 0;
  // Channel 3
  tx_cr |= (3UL << DMA_SxCR_CHSEL_Pos);
  // DIR: 01 (Memory-to-Periph)
  tx_cr |= DMA_SxCR_DIR_0;
  // Priority: Very High
  tx_cr |= DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
  // MINC: Only if tx buffer provided
  if (tx) {
    tx_cr |= DMA_SxCR_MINC;
  }
  // Interrupts: TEIE, DMEIE (TCIE not needed on TX, we assume RX finishes last)
  tx_cr |= DMA_SxCR_TEIE | DMA_SxCR_DMEIE;

  tx_stream->CR = tx_cr;

  // 5. Enable Streams (RX then TX)
  rx_stream->CR |= DMA_SxCR_EN;
  tx_stream->CR |= DMA_SxCR_EN;

  // 6. Enable SPI DMA Request
  spi->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  return true;
}

template <SpiInstance Inst> void Spi<Inst>::OnRxDmaTcIrq() {
  SPI_TypeDef *spi = Hw();
  DMA_Stream_TypeDef *rx_stream = nullptr;
  DMA_Stream_TypeDef *tx_stream = nullptr;
  DMA_TypeDef *dma = nullptr;
  uint32_t rx_clear_flags = 0;

  if constexpr (Inst == SpiInstance::kSpi1) {
    dma = DMA2;
    rx_stream = DMA2_Stream0;
    tx_stream = DMA2_Stream3;
    // Clear ALL flags for both streams to ensure clean state
    rx_clear_flags = (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                      DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0) |
                     (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                      DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3);
  } else {
    static_assert(Inst == SpiInstance::kSpi1, "Unsupported SPI instance");
  }

  // Clear IRQ flags
  dma->LIFCR = rx_clear_flags;

  // 1. Wait for BSY to clear
  // Loop limit for safety? SPI is fast, so this should be quick.
  while (spi->SR & SPI_SR_BSY) {
  }

  // 2. Disable SPI DMA Requests
  spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  // 3. Disable Streams
  DmaDisableAndWait(rx_stream);
  DmaDisableAndWait(tx_stream);

  // Hardening: Clear OVR if present (cheap insurance)
  if (spi->SR & SPI_SR_OVR) {
    volatile uint32_t tmp = spi->DR;
    tmp = spi->SR;
    (void)tmp;
  }

  // 4. Callback
  SpiDoneCb cb = nullptr;
  void *user = nullptr;

  busy_ = false;
  cb = cb_;
  user = user_;

  if (cb) {
    cb(user, true); // ok=true
  }
}

template <SpiInstance Inst> void Spi<Inst>::HandleDmaError(uint32_t isr_flags) {
  // Just clear flags and maybe fail the callback?
  // Implementation similar to OnRxDmaTcIrq but with ok=false
  SPI_TypeDef *spi = Hw();
  DMA_Stream_TypeDef *rx_stream = DMA2_Stream0; // Hardcoded for SPI1
  DMA_Stream_TypeDef *tx_stream = DMA2_Stream3;

  System::GetInstance().Led().Set(true);

  // Disable SPI DMA Requests
  spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  if constexpr (Inst == SpiInstance::kSpi1) {
    DMA2->LIFCR = (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                   DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0) |
                  (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                   DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3);
  } else {
    static_assert(Inst == SpiInstance::kSpi1, "Unsupported SPI instance");
  }

  // Disable Streams
  DmaDisableAndWait(rx_stream);
  DmaDisableAndWait(tx_stream);

  // Hardening: Clear OVR if present
  if (spi->SR & SPI_SR_OVR) {
    volatile uint32_t tmp = spi->DR;
    tmp = spi->SR;
    (void)tmp;
  }

  SpiDoneCb cb = nullptr;
  void *user = nullptr;

  busy_ = false;
  cb = cb_;
  user = user_;

  if (cb) {
    cb(user, false); // ok=false
  }
}

template class Spi<SpiInstance::kSpi1>;

extern "C" {
void Spi1RxDmaComplete() {
  Spi1::GetInstance().OnRxDmaTcIrq();
}
void Spi1DmaError(uint32_t isr) {
  Spi1::GetInstance().HandleDmaError(isr);
}
}
