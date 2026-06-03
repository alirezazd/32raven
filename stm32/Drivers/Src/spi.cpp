#include "spi.hpp"

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"

template <SpiInstance Inst>
void Spi<Inst>::Init(const SpiConfig &config) {
  EnableDmaClk();
  SPI_TypeDef *spi = Hw();

  EnableSpiClk();

  // SPE must be off to write CR1.
  spi->CR1 &= ~SPI_CR1_SPE;

  // Master, 8-bit (DFF=0), software NSS held high (SSM=1, SSI=1).
  uint32_t cr1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;

  if (config.polarity == SpiPolarity::kHigh) {
    cr1 |= SPI_CR1_CPOL;
  }

  if (config.phase == SpiPhase::k2Edge) {
    cr1 |= SPI_CR1_CPHA;
  }

  if (config.bit_order == SpiBitOrder::kLsbFirst) {
    cr1 |= SPI_CR1_LSBFIRST;
  }

  // Baud prescaler: BR bits [5:3].
  cr1 |= (static_cast<uint32_t>(config.prescaler) << SPI_CR1_BR_Pos);

  spi->CR1 = cr1;

  // Motorola frame format, interrupts off.
  spi->CR2 = 0;

  // Read DR then SR to clear any residual OVR before enabling.
  volatile uint32_t tmp __attribute__((unused));
  tmp = spi->DR;
  tmp = spi->SR;

  Enable();
  initialized_ = true;
}

template <SpiInstance Inst>
void Spi<Inst>::EnableDmaClk() {
  if constexpr (Inst == SpiInstance::kSpi2) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;  // readback delay
  }
}

template <SpiInstance Inst>
void Spi<Inst>::EnableSpiClk() {
  if constexpr (Inst == SpiInstance::kSpi1) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;  // readback delay
  } else if constexpr (Inst == SpiInstance::kSpi2) {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    (void)RCC->APB1ENR;  // readback delay
  } else {
    static_assert(Inst == SpiInstance::kSpi1, "Unsupported SPI instance");
  }
}

template <SpiInstance Inst>
void Spi<Inst>::EnableIrqsImpl(uint32_t priority) {
  if constexpr (Inst == SpiInstance::kSpi2) {
    /* DMA1_Stream3_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Stream3_IRQn, priority);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    /* DMA1_Stream4_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Stream4_IRQn, priority);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  }
}

template <SpiInstance Inst>
void Spi<Inst>::TxRx(const uint8_t *tx, uint8_t *rx, size_t len) {
  if (len == 0) return;

  SPI_TypeDef *spi = Hw();
  const uint8_t *tx_ptr = tx;
  uint8_t *rx_ptr = rx;
  size_t i = 0;

  if (!(spi->CR1 & SPI_CR1_SPE)) {
    Enable();
  }

  while (i < len) {
    while (!(spi->SR & SPI_SR_TXE)) {
    };

    // Drive 0xFF when transmit-only so the receiver sees idle MOSI.
    *reinterpret_cast<volatile uint8_t *>(&spi->DR) = tx_ptr ? *tx_ptr++ : 0xFF;

    while (!(spi->SR & SPI_SR_RXNE)) {
    };

    // Reading DR clears RXNE.
    uint8_t d = *reinterpret_cast<volatile uint8_t *>(&spi->DR);
    if (rx_ptr) {
      *rx_ptr++ = d;
    }
    i++;
  }

  // Wait for BSY before the caller deasserts CS.
  while (spi->SR & SPI_SR_BSY) {
  }

  if (spi->SR & SPI_SR_OVR) {
    volatile uint32_t tmp __attribute__((unused));
    tmp = spi->DR;
    tmp = spi->SR;
  }
}

template <SpiInstance Inst>
uint8_t Spi<Inst>::TxRxByte(uint8_t tx) {
  uint8_t rx = 0;
  TxRx(&tx, &rx, 1);
  return rx;
}

template <SpiInstance Inst>
void Spi<Inst>::Write(const uint8_t *tx, size_t len) {
  TxRx(tx, nullptr, len);
}

template <SpiInstance Inst>
void Spi<Inst>::Read(uint8_t *rx, size_t len) {
  TxRx(nullptr, rx, len);
}

template <SpiInstance Inst>
void Spi<Inst>::SetPrescaler(SpiPrescaler rate) {
  if (busy_) {
    Panic(ErrorCode::Stm32::kSpiInitFailed);
    return;
  }
  SPI_TypeDef *spi = Hw();
  Disable();

  uint32_t cr1 = spi->CR1;
  cr1 &= ~SPI_CR1_BR;
  cr1 |= (static_cast<uint32_t>(rate) << SPI_CR1_BR_Pos);
  spi->CR1 = cr1;

  Enable();
}

template <SpiInstance Inst>
void Spi<Inst>::Enable() {
  Hw()->CR1 |= SPI_CR1_SPE;
}

template <SpiInstance Inst>
void Spi<Inst>::Disable() {
  SPI_TypeDef *spi = Hw();
  // Drain in-flight frame before clearing SPE.
  while (spi->SR & SPI_SR_BSY) {
  }
  spi->CR1 &= ~SPI_CR1_SPE;

  // Read DR then SR to clear OVR.
  volatile uint32_t tmp __attribute__((unused));
  tmp = spi->DR;
  tmp = spi->SR;
}

static inline void DmaDisableAndWait(DMA_Stream_TypeDef *s) {
  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {
  }
}

template <SpiInstance Inst>
bool Spi<Inst>::BusyImpl() const {
  return busy_;
}

template <SpiInstance Inst>
bool Spi<Inst>::StartTxRxDmaImpl(const uint8_t *tx, uint8_t *rx, size_t len,
                                 SpiDoneCb cb, void *user) {
  if constexpr (Inst == SpiInstance::kSpi1) {
    (void)tx;
    (void)rx;
    (void)len;
    (void)cb;
    (void)user;
    return false;
  }

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

  // Drain stale RX so DMA starts on a clean DR; then clear OVR.
  while (spi->SR & SPI_SR_RXNE) {
    volatile uint32_t tmp = spi->DR;
    (void)tmp;
  }
  if (spi->SR & SPI_SR_OVR) {
    volatile uint32_t tmp = spi->DR;
    tmp = spi->SR;
    (void)tmp;
  }

  DMA_Stream_TypeDef *rx_stream = nullptr;
  DMA_Stream_TypeDef *tx_stream = nullptr;
  DMA_TypeDef *dma = nullptr;
  uint32_t low_clear_flags = 0;
  uint32_t high_clear_flags = 0;
  uint32_t channel_sel = 0;

  if constexpr (Inst == SpiInstance::kSpi2) {
    // SPI2 RX: DMA1_Stream3_Ch0
    // SPI2 TX: DMA1_Stream4_Ch0
    dma = DMA1;
    rx_stream = DMA1_Stream3;
    tx_stream = DMA1_Stream4;
    channel_sel = 0;
    low_clear_flags = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                      DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
    high_clear_flags = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |
                       DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
  }

  DmaDisableAndWait(rx_stream);
  DmaDisableAndWait(tx_stream);

  // Drop SPI DMA requests while reconfiguring streams.
  spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  dma->LIFCR = low_clear_flags;
  if (high_clear_flags != 0) {
    dma->HIFCR = high_clear_flags;
  }

  // FCR=0 selects direct (non-FIFO) mode.
  rx_stream->FCR = 0;
  tx_stream->FCR = 0;

  // RX stream: periph->mem.
  rx_stream->PAR = reinterpret_cast<uint32_t>(&spi->DR);
  rx_stream->NDTR = len;
  if (rx) {
    rx_stream->M0AR = reinterpret_cast<uint32_t>(rx);
  } else {
    rx_stream->M0AR = reinterpret_cast<uint32_t>(&dummy_rx_);
  }

  uint32_t rx_cr = 0;
  rx_cr |= channel_sel;
  // DIR=00 periph->mem (default); PL=11 very high.
  rx_cr |= DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
  // Increment memory only when a real buffer is supplied.
  if (rx) {
    rx_cr |= DMA_SxCR_MINC;
  }
  rx_cr |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;

  rx_stream->CR = rx_cr;

  // TX stream: mem->periph.
  tx_stream->PAR = reinterpret_cast<uint32_t>(&spi->DR);
  tx_stream->NDTR = len;
  if (tx) {
    tx_stream->M0AR = reinterpret_cast<uint32_t>(tx);
  } else {
    tx_stream->M0AR = reinterpret_cast<uint32_t>(&dummy_tx_);
  }

  uint32_t tx_cr = 0;
  tx_cr |= channel_sel;
  // DIR=01 mem->periph; PL=11 very high.
  tx_cr |= DMA_SxCR_DIR_0;
  tx_cr |= DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
  if (tx) {
    tx_cr |= DMA_SxCR_MINC;
  }
  // No TCIE on TX; completion is driven off RX, which finishes last.
  tx_cr |= DMA_SxCR_TEIE | DMA_SxCR_DMEIE;

  tx_stream->CR = tx_cr;

  // Enable RX before TX so the first received frame can't be dropped.
  rx_stream->CR |= DMA_SxCR_EN;
  tx_stream->CR |= DMA_SxCR_EN;

  spi->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  return true;
}

template <SpiInstance Inst>
void Spi<Inst>::OnRxDmaTcIrqImpl() {
  if constexpr (Inst == SpiInstance::kSpi1) {
    return;
  }

  SPI_TypeDef *spi = Hw();
  DMA_Stream_TypeDef *rx_stream = nullptr;
  DMA_Stream_TypeDef *tx_stream = nullptr;
  DMA_TypeDef *dma = nullptr;
  uint32_t low_clear_flags = 0;
  uint32_t high_clear_flags = 0;

  if constexpr (Inst == SpiInstance::kSpi2) {
    dma = DMA1;
    rx_stream = DMA1_Stream3;
    tx_stream = DMA1_Stream4;
    low_clear_flags = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                      DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
    high_clear_flags = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |
                       DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
  }

  dma->LIFCR = low_clear_flags;
  if (high_clear_flags != 0) {
    dma->HIFCR = high_clear_flags;
  }

  // RX TC fires when the last byte hits memory, but SPI may still be shifting.
  while (spi->SR & SPI_SR_BSY) {
  }

  spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  DmaDisableAndWait(rx_stream);
  DmaDisableAndWait(tx_stream);

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
    cb(user, true);
  }
}

template <SpiInstance Inst>
void Spi<Inst>::HandleDmaErrorImpl(uint32_t isr_flags) {
  if constexpr (Inst == SpiInstance::kSpi1) {
    (void)isr_flags;
    return;
  }

  // Tear down like OnRxDmaTcIrq, but report failure to the callback.
  SPI_TypeDef *spi = Hw();
  DMA_Stream_TypeDef *rx_stream = nullptr;
  DMA_Stream_TypeDef *tx_stream = nullptr;
  DMA_TypeDef *dma = nullptr;
  uint32_t low_clear_flags = 0;
  uint32_t high_clear_flags = 0;

  System::GetInstance().Led().Set(true);

  spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  if constexpr (Inst == SpiInstance::kSpi2) {
    dma = DMA1;
    rx_stream = DMA1_Stream3;
    tx_stream = DMA1_Stream4;
    low_clear_flags = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                      DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
    high_clear_flags = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |
                       DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
  }

  dma->LIFCR = low_clear_flags;
  if (high_clear_flags != 0) {
    dma->HIFCR = high_clear_flags;
  }

  DmaDisableAndWait(rx_stream);
  DmaDisableAndWait(tx_stream);

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
    cb(user, false);
  }
}

template class Spi<SpiInstance::kSpi1>;
template class Spi<SpiInstance::kSpi2>;

extern "C" {
void Spi2RxDmaComplete() { Spi2::GetInstance().OnRxDmaTcIrq(); }
void Spi2DmaError(uint32_t isr) { Spi2::GetInstance().HandleDmaError(isr); }
}
