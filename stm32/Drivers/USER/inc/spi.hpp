#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "stm32f4xx.h"

enum class SpiInstance { kSpi1, kSpi2 };

enum class SpiPrescaler : uint8_t {
  kDiv2 = 0,
  kDiv4 = 1,
  kDiv8 = 2,
  kDiv16 = 3,
  kDiv32 = 4,
  kDiv64 = 5,
  kDiv128 = 6,
  kDiv256 = 7,
};

enum class SpiPolarity : uint8_t { kLow = 0, kHigh = 1 };
enum class SpiPhase : uint8_t { k1Edge = 0, k2Edge = 1 };
enum class SpiBitOrder : uint8_t { kMsbFirst = 0, kLsbFirst = 1 };

struct SpiConfig {
  SpiPolarity polarity;
  SpiPhase phase;
  SpiPrescaler prescaler;
  SpiBitOrder bit_order;
};

template <SpiInstance Inst>
class Spi {
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
  template <SpiInstance I = Inst,
            typename std::enable_if_t<I == SpiInstance::kSpi2, int> = 0>
  void EnableIrqs(uint32_t priority = 2) {
    EnableIrqsImpl(priority);
  }

  bool IsInitialized() const { return initialized_; }

  template <SpiInstance I = Inst,
            typename std::enable_if_t<I == SpiInstance::kSpi2, int> = 0>
  bool Busy() const {
    return BusyImpl();
  }

  using SpiDoneCb = void (*)(void *user, bool ok);

  template <SpiInstance I = Inst,
            typename std::enable_if_t<I == SpiInstance::kSpi2, int> = 0>
  bool StartTxRxDma(const uint8_t *tx, uint8_t *rx, size_t len, SpiDoneCb cb,
                    void *user) {
    return StartTxRxDmaImpl(tx, rx, len, cb, user);
  }

  template <SpiInstance I = Inst,
            typename std::enable_if_t<I == SpiInstance::kSpi2, int> = 0>
  void OnRxDmaTcIrq() {
    OnRxDmaTcIrqImpl();
  }

  template <SpiInstance I = Inst,
            typename std::enable_if_t<I == SpiInstance::kSpi2, int> = 0>
  void HandleDmaError(uint32_t isr_flags) {
    HandleDmaErrorImpl(isr_flags);
  }

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
    else if constexpr (Inst == SpiInstance::kSpi2)
      return SPI2;
    else
      return nullptr;
  }
  static_assert(Inst == SpiInstance::kSpi1 || Inst == SpiInstance::kSpi2,
                "Unsupported SPI instance");

  bool initialized_ = false;
  volatile bool busy_ = false;
  const uint8_t *tx_ = nullptr;
  uint8_t *rx_ = nullptr;
  uint16_t len_ = 0;
  SpiDoneCb cb_ = nullptr;
  void *user_ = nullptr;
  uint8_t dummy_tx_ = 0xFF;
  uint8_t dummy_rx_ = 0x00;

  void Enable();
  void Disable();
  void EnableDmaClk();
  void EnableSpiClk();
  void EnableIrqsImpl(uint32_t priority);
  bool BusyImpl() const;
  bool StartTxRxDmaImpl(const uint8_t *tx, uint8_t *rx, size_t len,
                        SpiDoneCb cb, void *user);
  void OnRxDmaTcIrqImpl();
  void HandleDmaErrorImpl(uint32_t isr_flags);
};

using Spi1 = Spi<SpiInstance::kSpi1>;
using Spi2 = Spi<SpiInstance::kSpi2>;
