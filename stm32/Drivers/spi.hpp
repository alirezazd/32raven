// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "outcome.hpp"
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

  // Discarding is legitimate -- the bus tallies the timeout either way -- but
  // only the callsite knows whether a truncated command is survivable, so it
  // has to say so.
  [[nodiscard]] Outcome TxRx(const uint8_t *tx, uint8_t *rx, size_t len);
  [[nodiscard]] Outcome WriteBytes(std::span<const uint8_t> tx);
  [[nodiscard]] Outcome ReadBytes(std::span<uint8_t> rx);

  void SetPrescaler(SpiPrescaler rate);
  void EnableIrqs(uint32_t priority)
    requires(Inst == SpiInstance::kSpi2);

  bool IsInitialized() const { return initialized_; }

  // Since-boot bus faults. Counted here rather than by the device, because
  // each is the bus's own event -- a refused start is this bus busy, a
  // transfer error is this bus's DMA, and a timeout is this bus's status flag
  // never arriving. The device reads them back for its own path summary, so
  // there is one count with two readers.
  struct Faults {
    uint32_t start_refused = 0;
    uint32_t dma_errors = 0;
    uint32_t timeouts = 0;
  };

  Faults GetFaults() const {
    return Faults{.start_refused = start_refused_,
                  .dma_errors = dma_errors_,
                  .timeouts = timeouts_};
  }

  bool Busy() const
    requires(Inst == SpiInstance::kSpi2);

  // No user pointer: the one owner of this bus is a singleton, so the callee
  // finds itself.
  using SpiDoneCb = void (*)(bool ok);

  bool StartTxRxDma(const uint8_t *tx, uint8_t *rx, size_t len, SpiDoneCb cb)
    requires(Inst == SpiInstance::kSpi2);

  void OnRxDmaTcIrq()
    requires(Inst == SpiInstance::kSpi2);

  void HandleDmaError()
    requires(Inst == SpiInstance::kSpi2);

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
  // All three are incremented from ISR context or against an in-flight
  // transfer.
  volatile uint32_t start_refused_ = 0;
  volatile uint32_t dma_errors_ = 0;
  volatile uint32_t timeouts_ = 0;
  const uint8_t *tx_ = nullptr;
  uint8_t *rx_ = nullptr;
  uint16_t len_ = 0;
  SpiDoneCb cb_ = nullptr;
  uint8_t dummy_tx_ = 0xFF;
  uint8_t dummy_rx_ = 0x00;

  void Enable();
  void Disable();
  void EnableDmaClk();
  void EnableSpiClk();
};

using Spi1 = Spi<SpiInstance::kSpi1>;
using Spi2 = Spi<SpiInstance::kSpi2>;
