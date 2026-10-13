// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "i2c.hpp"

#include <atomic>
#include <cstring>

#include "error_code.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "rcc.hpp"
#include "stm32_config.hpp"
#include "stm32f4xx.h"
#include "system.hpp"
#include "time_base.hpp"

namespace {

// One preemption level so EV and ER serialize: ER can never cut into EV
// between the SR1 and SR2 reads of an ADDR clear.
static_assert(irq_priority::kI2c1Event == irq_priority::kI2c1Error,
              "the I2C1 vectors must stay in one preemption tier");

// Masks both vectors (they share a tier) while Poll's abort rewrites
// transfer state the handlers also write.
constexpr uint32_t kI2cMaskPri =
    (irq_priority::kI2c1Event << (8u - __NVIC_PRIO_BITS)) & 0xFFu;

class BasepriGuard {
 public:
  BasepriGuard() : saved_(__get_BASEPRI()) { __set_BASEPRI(kI2cMaskPri); }
  ~BasepriGuard() { __set_BASEPRI(saved_); }
  BasepriGuard(const BasepriGuard &) = delete;
  BasepriGuard &operator=(const BasepriGuard &) = delete;

 private:
  uint32_t saved_;
};

// The bound only exists so a dead bus cannot hang the main loop.
template <typename Pred>
bool SpinFor(uint32_t timeout_us, Pred done) {
  const TimeBase &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();
  while (!done()) {
    if ((time.Micros() - start) > timeout_us) {
      return false;
    }
  }
  return true;
}

// RM0090 requires software to keep OAR1 bit 14 set, own address or not;
// CMSIS names no symbol for it.
constexpr uint32_t kOar1Bit14 = 1u << 14u;

constexpr uint32_t kCr1PendingUs = 25;  // two SCL periods at 100 kHz, rounded
constexpr uint32_t kGracefulStopUs = 30;
constexpr uint32_t kRecoverHalfPeriodUs = 5;  // ~100 kHz recovery clocking
constexpr uint32_t kRecoverStretchUs = 20;

bool SclHigh(GPIO &gpio) {
  return gpio.ReadPin(board::kI2c1Scl.port, board::kI2c1Scl.pin);
}

bool SdaHigh(GPIO &gpio) {
  return gpio.ReadPin(board::kI2c1Sda.port, board::kI2c1Sda.pin);
}

bool LinesIdle(GPIO &gpio) { return SclHigh(gpio) && SdaHigh(gpio); }

}  // namespace

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::Init(const I2cConfig &config, GPIO &gpio) {
  if (initialized_) {
    return;
  }
  initialized_ = true;
  cfg_ = config;
  gpio_ = &gpio;

  if constexpr (Inst == I2cInstance::kI2c1) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;  // readback delay
  }

  // A slave our reset abandoned mid-byte holds SDA until clocked; free it
  // before the peripheral first samples the bus, or BUSY latches from boot.
  MaybeRecoverBus();
  ConfigureHw();

  // BUSY with both lines high is the analog-filter lockup (ES0182); this
  // part cannot disable the filter, and only the manual waveform flushes it.
  if ((Hw()->SR2 & I2C_SR2_BUSY) != 0u && LinesIdle(*gpio_)) {
    RecoverBus();
    ConfigureHw();
  }

  // Still low after recovery is a short or a pinned line. Name it here, or
  // every device behind it only reports itself absent.
  if (!SdaHigh(*gpio_)) {
    Panic(ErrorCode::Stm32::kI2cSdaHeldLow);
  }
  if (!SclHigh(*gpio_)) {
    Panic(ErrorCode::Stm32::kI2cSclHeldLow);
  }

  NVIC_SetPriority(EventIrqn(), irq_priority::kI2c1Event);
  NVIC_EnableIRQ(EventIrqn());
  NVIC_SetPriority(ErrorIrqn(), irq_priority::kI2c1Error);
  NVIC_EnableIRQ(ErrorIrqn());
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::ConfigureHw() {
  I2C_TypeDef *hw = Hw();
  // SWRST wipes the state machine and every register with it -- the only
  // documented way out of a half-shifted byte or a noise-latched flag.
  hw->CR1 = I2C_CR1_SWRST;
  (void)hw->CR1;
  hw->CR1 = 0;

  const uint32_t pclk_hz = Rcc::Apb1Hz();
  const uint32_t freq_mhz = pclk_hz / 1000000u;
  hw->CR2 = freq_mhz & I2C_CR2_FREQ;  // interrupt enables stay off when idle
  if (cfg_.scl_hz > 100000u) {
    // Fast mode, 2:1 duty. Divisors round up so a clock that does not
    // divide exactly lands under the ask, never over it.
    hw->CCR =
        I2C_CCR_FS | ((pclk_hz + (3u * cfg_.scl_hz) - 1u) / (3u * cfg_.scl_hz));
    hw->TRISE = ((freq_mhz * 300u) / 1000u) + 1u;  // 300 ns max SCL rise
  } else {
    hw->CCR = (pclk_hz + (2u * cfg_.scl_hz) - 1u) / (2u * cfg_.scl_hz);
    hw->TRISE = freq_mhz + 1u;  // 1000 ns max SCL rise
  }
  hw->OAR1 = kOar1Bit14;
  hw->CR1 = I2C_CR1_PE;
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::MaybeRecoverBus() {
  if (!SdaHigh(*gpio_)) {
    RecoverBus();
  }
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::RecoverBus() {
  // SetPinMode moves MODER only, so AF4 and kGpioDefault's open-drain pull
  // survive; push-pull would fight a slave holding a line low.
  I2C_TypeDef *hw = Hw();
  hw->CR1 = 0;

  GPIO &gpio = *gpio_;
  const board::BoardPin &scl = board::kI2c1Scl;
  const board::BoardPin &sda = board::kI2c1Sda;
  const TimeBase &time = System::GetInstance().Time();

  // Never drive a claimed-low edge: release both before taking the pins.
  gpio.WritePin(scl.port, scl.pin, true);
  gpio.WritePin(sda.port, sda.pin, true);
  gpio.SetPinMode(scl.port, scl.pin, GPIO_MODE_OUTPUT_OD);
  gpio.SetPinMode(sda.port, sda.pin, GPIO_MODE_OUTPUT_OD);

  // 9 clocks (8 data bits plus ACK) walk a slave out of any bit it died in;
  // the filter-flush caller, with SDA already free, only needs the STOP.
  for (uint8_t pulse = 0; pulse < 9u && !SdaHigh(gpio); ++pulse) {
    gpio.WritePin(scl.port, scl.pin, false);
    time.DelayMicros(kRecoverHalfPeriodUs);
    gpio.WritePin(scl.port, scl.pin, true);
    // Waits out a stretching slave; the bound keeps an SCL shorted low from
    // hanging boot.
    (void)SpinFor(kRecoverStretchUs, [&gpio] { return SclHigh(gpio); });
    time.DelayMicros(kRecoverHalfPeriodUs);
  }

  // A real STOP resets every listener and is ST's analog-filter flush. SDA
  // falls while SCL is low first, or the fall would read as a START.
  gpio.WritePin(scl.port, scl.pin, false);
  time.DelayMicros(kRecoverHalfPeriodUs);
  gpio.WritePin(sda.port, sda.pin, false);
  time.DelayMicros(kRecoverHalfPeriodUs);
  gpio.WritePin(scl.port, scl.pin, true);
  time.DelayMicros(kRecoverHalfPeriodUs);
  gpio.WritePin(sda.port, sda.pin, true);
  time.DelayMicros(kRecoverHalfPeriodUs);

  gpio.SetPinMode(scl.port, scl.pin, GPIO_MODE_AF_OD);
  gpio.SetPinMode(sda.port, sda.pin, GPIO_MODE_AF_OD);
  recoveries_ = recoveries_ + 1;
}

template <I2cInstance Inst, size_t BufSize>
Outcome I2c<Inst, BufSize>::Arm(I2cTenant tenant, uint8_t addr7,
                                std::span<const uint8_t> tx, size_t rx_len) {
  if (!initialized_ || addr7 > 0x7Fu || tx.size() > BufSize ||
      rx_len > kI2cRxBufSize || tenant >= I2cTenant::kCount) {
    return Outcome::kInvalid;
  }
  if (active_ != nullptr && active_->status.load(std::memory_order_acquire) ==
                                I2cTransferStatus::kBusy) {
    return Outcome::kRejected;
  }
  I2C_TypeDef *hw = Hw();
  // The last transfer's STOP may still be draining, and writing CR1 over a
  // pending START/STOP can issue a second one (RM0090's CR1 note).
  if (!SpinFor(kCr1PendingUs, [hw] {
        return (hw->CR1 & (I2C_CR1_START | I2C_CR1_STOP | I2C_CR1_PEC)) == 0u &&
               (hw->SR2 & I2C_SR2_BUSY) == 0u;
      })) {
    return Outcome::kRejected;
  }

  if (!tx.empty()) {
    std::memcpy(tx_buf_, tx.data(), tx.size());
  }
  addr_ = addr7;
  tx_len_ = tx.size();
  tx_pos_ = 0;
  rx_len_ = rx_len;
  rx_pos_ = 0;
  berr_seen_ = false;

  // The configured timeout is only a floor: a full write-read at 100 kHz
  // needs ~29 ms of wire time, and aborting it would fabricate bus faults.
  const uint32_t bit_us = (1000000u + cfg_.scl_hz - 1u) / cfg_.scl_hz;
  const uint32_t wire_us =
      static_cast<uint32_t>(9u * (tx_len_ + rx_len_ + 2u)) * bit_us;
  deadline_us_ = cfg_.transfer_timeout_us;
  if (2u * wire_us > deadline_us_) {
    deadline_us_ = 2u * wire_us;
  }

  hw->SR1 = 0u;  // rc_w0: drops stale error flags; event flags ignore writes
  hw->CR1 &= ~(I2C_CR1_POS | I2C_CR1_ACK);  // the spin above proved CR1 idle
  phase_ = Phase::kAddrTx;
  started_us_ = System::GetInstance().Time().Micros();
  active_ = &SlotOf(tenant);
  active_->status.store(I2cTransferStatus::kBusy, std::memory_order_relaxed);
  hw->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  // Publish every plain store above before the START that lets the ISRs
  // consume them; volatile MMIO alone does not order them.
  std::atomic_signal_fence(std::memory_order_release);
  hw->CR1 |= I2C_CR1_START;
  return Outcome::kOk;
}

template <I2cInstance Inst, size_t BufSize>
I2cTransferStatus I2c<Inst, BufSize>::Poll(I2cTenant tenant, uint32_t now_us) {
  // Only the active slot is ever kBusy, so the deadline is this tenant's.
  Slot &slot = SlotOf(tenant);
  if (slot.status.load(std::memory_order_acquire) ==
          I2cTransferStatus::kBusy &&
      (now_us - started_us_) > deadline_us_) {
    AbortStuckTransfer();
  }
  return slot.status.load(std::memory_order_acquire);
}

template <I2cInstance Inst, size_t BufSize>
std::span<const uint8_t> I2c<Inst, BufSize>::Received(
    I2cTenant tenant) const {
  const Slot &slot = SlotOf(tenant);
  if (slot.status.load(std::memory_order_acquire) !=
      I2cTransferStatus::kComplete) {
    return {};
  }
  return {slot.rx_buf, slot.rx_count};
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::AbortStuckTransfer() {
  I2C_TypeDef *hw = Hw();
  {
    // A handler already pended runs once after the guard drops, sees kIdle,
    // and counts itself spurious.
    BasepriGuard guard;
    hw->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    phase_ = Phase::kIdle;
  }
  timeouts_ = timeouts_ + 1;

  // A slave pinning SDA makes this STOP unformable; the wire check below
  // escalates that to recovery clocking.
  if ((hw->SR2 & I2C_SR2_MSL) != 0u &&
      (hw->CR1 & (I2C_CR1_START | I2C_CR1_STOP)) == 0u) {
    hw->CR1 |= I2C_CR1_STOP;
    (void)SpinFor(kGracefulStopUs,
                  [hw] { return (hw->SR2 & I2C_SR2_BUSY) == 0u; });
  }

  ConfigureHw();
  if (!SdaHigh(*gpio_) ||
      ((hw->SR2 & I2C_SR2_BUSY) != 0u && LinesIdle(*gpio_))) {
    RecoverBus();
    ConfigureHw();
  }

  active_->status.store(
      berr_seen_ ? I2cTransferStatus::kBusError : I2cTransferStatus::kTimeout,
      std::memory_order_release);
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::FinishTxPhase() {
  I2C_TypeDef *hw = Hw();
  if (rx_len_ == 0u) {
    hw->CR1 |= I2C_CR1_STOP;
    EndTransfer(I2cTransferStatus::kComplete);
    return;
  }
  // Restarting off the BTF stretch is latency-proof, and CR1 provably has no
  // START/STOP pending, so the read leg's ACK/POS arm here.
  uint32_t cr1 = hw->CR1 | I2C_CR1_ACK;
  if (rx_len_ == 2u) {
    cr1 |= I2C_CR1_POS;  // re-targets the NACK to the second byte
  }
  hw->CR1 = cr1;
  phase_ = Phase::kRestart;
  hw->CR1 |= I2C_CR1_START;
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::EndTransfer(I2cTransferStatus status) {
  I2C_TypeDef *hw = Hw();
  hw->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
  // POS stays set: the STOP may still be draining and CR1 must not be
  // written over it. The next Arm clears it once CR1 is idle.
  phase_ = Phase::kIdle;
  active_->rx_count = rx_pos_;
  active_->status.store(status, std::memory_order_release);
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::OnEventIrq() {
  I2C_TypeDef *hw = Hw();
  const uint32_t sr1 = hw->SR1;

  if (phase_ == Phase::kIdle) {
    spurious_irqs_ = spurious_irqs_ + 1;
    return;
  }

  if ((sr1 & I2C_SR1_SB) != 0u) {
    // EV5: the SR1 read above plus this DR write clears SB. Returning
    // without the write would re-enter forever.
    if (phase_ == Phase::kRestart) {
      phase_ = Phase::kAddrRx;
      hw->DR = static_cast<uint32_t>(addr_ << 1u) | 1u;
    } else {
      hw->DR = static_cast<uint32_t>(addr_ << 1u);
    }
    return;
  }

  if ((sr1 & I2C_SR1_ADDR) != 0u) {
    if (phase_ != Phase::kAddrRx) {
      // Write direction. SCL is stretched until the SR2 read clears ADDR,
      // so everything up to it is race-free.
      (void)hw->SR2;
      phase_ = Phase::kTx;
      if (tx_len_ == 0u) {
        FinishTxPhase();  // the zero-length probe: START, address, STOP
        return;
      }
      hw->DR = tx_buf_[0];  // TXE is guaranteed under the ADDR stretch
      tx_pos_ = 1;
      if (tx_pos_ < tx_len_) {
        hw->CR2 |= I2C_CR2_ITBUFEN;
      }
      return;
    }
    // Read direction: ACK/POS were armed before the (re)START; what is
    // left is the per-length prep, decided before ADDR's stretch ends.
    if (rx_len_ == 1u) {
      hw->CR1 &= ~I2C_CR1_ACK;
      // The SR2 read starts the lone byte; a preemption outlasting it would
      // leave it NACKed with no STOP programmed and the bus parked.
      __disable_irq();
      (void)hw->SR2;
      hw->CR1 |= I2C_CR1_STOP;
      __enable_irq();
      hw->CR2 |= I2C_CR2_ITBUFEN;  // the byte lands on RXNE; BTF never sets
    } else if (rx_len_ == 2u) {
      hw->CR1 &= ~I2C_CR1_ACK;  // with POS: byte 1 ACKed, byte 2 NACKed
      (void)hw->SR2;            // BTF fires holding both bytes
    } else {
      (void)hw->SR2;  // ACK is set; body reads pace on BTF
    }
    phase_ = Phase::kRx;
    return;
  }

  if (phase_ == Phase::kTx) {
    if ((sr1 & I2C_SR1_TXE) != 0u && tx_pos_ < tx_len_) {
      hw->DR = tx_buf_[tx_pos_];
      ++tx_pos_;
      if (tx_pos_ == tx_len_) {
        // TXE stays set until the STOP, so ITBUFEN would livelock; BTF
        // closes the phase once the last byte is out and ACKed.
        hw->CR2 &= ~I2C_CR2_ITBUFEN;
      }
      return;
    }
    if ((sr1 & I2C_SR1_BTF) != 0u) {
      FinishTxPhase();
    }
    return;
  }

  if (phase_ == Phase::kRx) {
    const size_t remaining = rx_len_ - rx_pos_;
    if (remaining == 1u) {
      // Only the single-byte read gets here; longer reads end in the
      // two-byte unload below.
      if ((sr1 & I2C_SR1_RXNE) != 0u) {
        active_->rx_buf[rx_pos_] = static_cast<uint8_t>(hw->DR);
        ++rx_pos_;
        EndTransfer(I2cTransferStatus::kComplete);
      }
      return;
    }
    if ((sr1 & I2C_SR1_BTF) == 0u) {
      // Only at BTF is SCL stretched; RXNE pacing under latency clears ACK
      // a byte late, ACKing the slave's last byte and wedging the bus.
      return;
    }
    if (remaining == 3u) {
      // The NACK armed here lands on the final byte, which this unload lets
      // clock in; the next BTF holds the last two for an unhurried STOP.
      hw->CR1 &= ~I2C_CR1_ACK;
      active_->rx_buf[rx_pos_] = static_cast<uint8_t>(hw->DR);
      ++rx_pos_;
      return;
    }
    if (remaining == 2u) {
      // Closes the POS pair and the >=3 tail. STOP goes first: unloading DR
      // un-stretches the bus, which must not run with nothing programmed.
      hw->CR1 |= I2C_CR1_STOP;
      active_->rx_buf[rx_pos_] = static_cast<uint8_t>(hw->DR);
      ++rx_pos_;
      active_->rx_buf[rx_pos_] = static_cast<uint8_t>(hw->DR);
      ++rx_pos_;
      EndTransfer(I2cTransferStatus::kComplete);
      return;
    }
    active_->rx_buf[rx_pos_] = static_cast<uint8_t>(hw->DR);
    ++rx_pos_;
    return;
  }
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::OnErrorIrq() {
  I2C_TypeDef *hw = Hw();
  const uint32_t sr1 = hw->SR1;

  if (phase_ == Phase::kIdle) {
    hw->SR1 = 0u;
    spurious_irqs_ = spurious_irqs_ + 1;
    return;
  }

  if ((sr1 & I2C_SR1_AF) != 0u) {
    // The peripheral stops clocking and waits, still master; releasing the
    // bus after a NACK is software's job.
    hw->SR1 = static_cast<uint32_t>(~I2C_SR1_AF);
    nacks_ = nacks_ + 1;
    hw->CR1 |= I2C_CR1_STOP;
    EndTransfer(phase_ == Phase::kTx ? I2cTransferStatus::kNackData
                                     : I2cTransferStatus::kNackAddr);
    return;
  }
  if ((sr1 & I2C_SR1_ARLO) != 0u) {
    // ARLO already dropped to slave mode, so a STOP would hit someone else's
    // frame. On this single-master bus it means a line fault, not traffic.
    hw->SR1 = static_cast<uint32_t>(~I2C_SR1_ARLO);
    arb_losses_ = arb_losses_ + 1;
    EndTransfer(I2cTransferStatus::kArbitrationLost);
    return;
  }
  if ((sr1 & I2C_SR1_BERR) != 0u) {
    // ES0182: BERR is spurious in master mode, so it never aborts alone; a
    // truly broken transfer stalls and the deadline reports kBusError.
    hw->SR1 = static_cast<uint32_t>(~I2C_SR1_BERR);
    bus_errors_ = bus_errors_ + 1;
    berr_seen_ = true;
  }
  if ((sr1 & I2C_SR1_OVR) != 0u) {
    // Unreachable for a stretching master; cleared so it cannot storm.
    hw->SR1 = static_cast<uint32_t>(~I2C_SR1_OVR);
    spurious_irqs_ = spurious_irqs_ + 1;
  }
}

template class I2c<I2cInstance::kI2c1, kI2cBufSize>;

extern "C" {
void I2c1OnEventIrq() { I2c1::GetInstance().OnEventIrq(); }
void I2c1OnErrorIrq() { I2c1::GetInstance().OnErrorIrq(); }
}
