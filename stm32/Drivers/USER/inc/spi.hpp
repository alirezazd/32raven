#pragma once

#include "stm32f4xx.h"
#include "user_config.hpp" // SpiConfig, SpiPrescaler, SpiPolarity, SpiPhase, SpiBitOrder
#include <cstddef>
#include <cstdint>

enum class SpiInstance { kSpi1 };

template <SpiInstance Inst> class Spi {
public:
  static Spi &GetInstance() {
    static Spi instance;
    return instance;
  }

  void TxRx(const uint8_t *tx, uint8_t *rx, size_t len);
  uint8_t TxRxByte(uint8_t tx);
  void Write(const uint8_t *tx, size_t len);
  void Read(uint8_t *rx, size_t len);

  void SetPrescaler(SpiPrescaler rate);

  bool IsInitialized() const { return initialized_; }

  bool Busy() const;

  using SpiDoneCb = void (*)(void *user, bool ok);

  bool StartTxRxDma(const uint8_t *tx, uint8_t *rx, size_t len, SpiDoneCb cb,
                    void *user);
  void OnRxDmaTcIrq();

  void HandleDmaError(uint32_t isr_flags);

private:
  friend class System;
  void Init(const SpiConfig &config);

  Spi() = default;
  ~Spi() = default;
  Spi(const Spi &) = delete;
  Spi &operator=(const Spi &) = delete;

  static inline SPI_TypeDef *Hw() {
    if constexpr (Inst == SpiInstance::kSpi1)
      return SPI1;
    else
      return nullptr;
  }
  static_assert(Inst == SpiInstance::kSpi1, "Unsupported SPI instance");

  bool initialized_ = false;
  volatile bool busy_ = false;
  const uint8_t *tx_ = nullptr;
  uint8_t *rx_ = nullptr;
  uint16_t len_ = 0;
  SpiDoneCb cb_ = nullptr;
  void *user_ = nullptr;
  uint8_t dummy_tx_ = 0xFF;
  uint8_t dummy_rx_ = 0;

  void Enable();
  void Disable();
};
