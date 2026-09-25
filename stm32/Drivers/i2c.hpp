// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

#include "gpio.hpp"
#include "shared_state.hpp"
#include "outcome.hpp"
#include "stm32f4xx.h"

enum class I2cInstance : uint8_t { kI2c1 };

// Every device on the bus. Each owns a result slot, and reaches the bus only
// through a Port bound to that slot.
enum class I2cTenant : uint8_t { kQmc5883p, kDps310, kCount };

// What a tenant's last transfer came to. Terminal values persist in its slot
// until its own next Start*, so a consumer polling slower than the bus cannot
// miss its result, and no other tenant's transfer can overwrite it.
enum class I2cTransferStatus : uint8_t {
  kIdle = 0,  // nothing started since Init
  kBusy,
  kComplete,
  kNackAddr,  // no device answered the address: absent or wrong address
  kNackData,  // the device refused mid-payload
  kBusError,  // the transfer stalled with a bus error on record
  kArbitrationLost,
  kTimeout,  // deadline passed; the driver rebuilt the peripheral and, if
             // SDA was held, clocked the bus free
};

struct I2cConfig {
  uint32_t scl_hz;  // at 100 kHz programs standard mode, above it fast mode
  // A floor, not a ceiling: a transfer whose wire time cannot fit gets a
  // deadline computed from its own length instead, so lowering the clock
  // never manufactures timeouts.
  uint32_t transfer_timeout_us;
};

// Sized by the largest single write any intended tenant needs: a VL53-class
// TOF loads a ~93-byte configuration blob in one transfer. The DPS310 and
// MMC5983MA stay under 32.
constexpr size_t kI2cBufSize = 160;

// Per slot, so sized by the largest read rather than the largest write: the
// DPS310's 18 coefficient bytes.
constexpr size_t kI2cRxBufSize = 32;

// Master-only, 7-bit addressing, no DMA. Interrupts move the bytes and
// consumers poll for the outcome: Start* arms a transfer, the EV/ER
// interrupts run it, Poll() reports where it ended and enforces the
// deadline. Buffers are driver-owned on both sides so the interrupt
// handlers never touch caller memory.
template <I2cInstance Inst, size_t BufSize = kI2cBufSize>
class I2c {
 public:
  static I2c &GetInstance() {
    static I2c instance;
    return instance;
  }

  // One tenant's view of the bus: its own transfers and its own result.
  // System hands each driver its port at Init, so which slot a driver reads
  // is fixed by type. Starts from different tenants still take turns on the
  // one engine: a Start while another transfer is on the wire is kRejected.
  template <I2cTenant Tenant>
  class Port {
   public:
    Port() = default;

    // Zero-length is meaningful, not a no-op: START, address, STOP -- an
    // address probe, answered through kComplete / kNackAddr.
    [[nodiscard]] Outcome StartWrite(uint8_t addr7,
                                     std::span<const uint8_t> tx) {
      return bus_->Arm(Tenant, addr7, tx, 0);
    }
    // Write, repeated START, read -- the register-read shape every tenant
    // uses. tx must be non-empty and rx_len nonzero; a bare read has no
    // consumer here.
    [[nodiscard]] Outcome StartWriteRead(uint8_t addr7,
                                         std::span<const uint8_t> tx,
                                         size_t rx_len) {
      if (tx.empty() || rx_len == 0u) {
        return Outcome::kInvalid;
      }
      return bus_->Arm(Tenant, addr7, tx, rx_len);
    }
    // Reports this tenant's status and enforces its transfer deadline on the
    // caller's clock -- a stalled engine is torn down and rebuilt here, since
    // the interrupt handlers of a stalled engine never run again. The
    // teardown can hold the caller up to ~0.5 ms; it runs only on a faulted
    // bus.
    I2cTransferStatus Poll(uint32_t now_us) {
      return bus_->Poll(Tenant, now_us);
    }
    // The read payload; empty unless the last transfer ended kComplete. Valid
    // until this tenant's next Start*.
    std::span<const uint8_t> Received() const {
      return bus_->Received(Tenant);
    }

   private:
    friend class I2c;
    explicit Port(I2c &bus) : bus_(&bus) {}

    I2c *bus_ = nullptr;
  };

  // The four that are failed transfers. Recoveries and spurious interrupts
  // stay behind: a recovery answers one of these rather than being a fifth,
  // and nothing has asked for either yet.
  I2cFaults GetFaults() const {
    return I2cFaults{.nacks = nacks_,
                     .bus_errors = bus_errors_,
                     .arb_losses = arb_losses_,
                     .timeouts = timeouts_};
  }

  // Called from ISR
  void OnEventIrq();
  void OnErrorIrq();

 private:
  friend class System;
  void Init(const I2cConfig &config, GPIO &gpio);
  template <I2cTenant Tenant>
  Port<Tenant> PortFor() {
    return Port<Tenant>(*this);
  }

  I2c() = default;
  ~I2c() = default;
  I2c(const I2c &) = delete;
  I2c &operator=(const I2c &) = delete;

  enum class Phase : uint8_t {
    kIdle,
    kAddrTx,   // START pending or sent; SB answers with address + W
    kTx,       // feeding tx_buf_
    kRestart,  // repeated START pending; SB answers with address + R
    kAddrRx,   // address + R sent; ADDR prepares the length-specific endgame
    kRx,       // receiving into the active slot
  };

  struct Slot {
    // The release point a tenant synchronizes on: the interrupt side stores
    // it last, Poll and Received load-acquire before touching the buffer.
    std::atomic<I2cTransferStatus> status{I2cTransferStatus::kIdle};
    uint8_t rx_buf[kI2cRxBufSize];
    size_t rx_count = 0;
  };

  static I2C_TypeDef *Hw() {
    if constexpr (Inst == I2cInstance::kI2c1) {
      return I2C1;
    } else {
      return nullptr;
    }
  }
  static constexpr IRQn_Type EventIrqn() { return I2C1_EV_IRQn; }
  static constexpr IRQn_Type ErrorIrqn() { return I2C1_ER_IRQn; }
  static_assert(Inst == I2cInstance::kI2c1, "Unsupported I2c instance");

  Outcome Arm(I2cTenant tenant, uint8_t addr7, std::span<const uint8_t> tx,
              size_t rx_len);
  I2cTransferStatus Poll(I2cTenant tenant, uint32_t now_us);
  std::span<const uint8_t> Received(I2cTenant tenant) const;
  Slot &SlotOf(I2cTenant tenant) { return slots_[std::to_underlying(tenant)]; }
  const Slot &SlotOf(I2cTenant tenant) const {
    return slots_[std::to_underlying(tenant)];
  }
  void ConfigureHw();
  void MaybeRecoverBus();
  void RecoverBus();
  void AbortStuckTransfer();
  void FinishTxPhase();
  void EndTransfer(I2cTransferStatus status);

  bool initialized_ = false;
  I2cConfig cfg_{};
  // Every pin access in the recovery waveform goes through it: the mask this
  // driver holds is an index nowhere but MODER, and GPIO owns that conversion.
  GPIO *gpio_ = nullptr;

  uint8_t tx_buf_[BufSize];
  size_t tx_len_ = 0;
  size_t tx_pos_ = 0;
  size_t rx_len_ = 0;
  size_t rx_pos_ = 0;
  uint8_t addr_ = 0;
  uint32_t started_us_ = 0;
  uint32_t deadline_us_ = 0;
  volatile bool berr_seen_ = false;
  volatile Phase phase_ = Phase::kIdle;
  std::array<Slot, std::to_underlying(I2cTenant::kCount)> slots_{};
  // The slot of the transfer on the wire, or of the last one. Set by Arm
  // before the START that lets the handlers read it.
  Slot *active_ = nullptr;

  // Since-boot bus tallies, interrupt-written. Counted without an accessor
  // -- the Spi1 precedent -- so the record exists from the first transfer
  // and a narrower getter can join when something asks for it.
  volatile uint32_t nacks_ = 0;
  volatile uint32_t bus_errors_ = 0;
  volatile uint32_t arb_losses_ = 0;
  volatile uint32_t timeouts_ = 0;
  volatile uint32_t recoveries_ = 0;
  volatile uint32_t spurious_irqs_ = 0;
};

using I2c1 = I2c<I2cInstance::kI2c1>;
