// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "i2c.hpp"

#include <atomic>
#include <cstring>

#include "irq_priority.hpp"
#include "rcc.hpp"
#include "stm32_config.hpp"
#include "stm32f4xx.h"
#include "system.hpp"
#include "time_base.hpp"

namespace {

// EV and ER share one preemption level on purpose: they serialize, so the
// error handler can never cut into the event handler between the SR1 read
// and the SR2 read of an ADDR clear. Raising one above the other would
// reopen that whole class of races.
static_assert(irq_priority::kI2c1Event == irq_priority::kI2c1Error,
              "the I2C1 vectors must stay in one preemption tier");

// Poll's abort rewrites the same transfer state the handlers write; the
// guard shuts both out for the handful of stores that must be atomic
// against them. One constant serves both vectors -- they share a tier.
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

// Bounded flag wait. The waits here are a few SCL periods long; the bound
// only exists so a dead bus cannot hang the main loop.
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

bool SclHigh() {
  return (board::kI2c1Scl.port->IDR & (1u << board::kI2c1Scl.pin)) != 0u;
}

bool SdaHigh() {
  return (board::kI2c1Sda.port->IDR & (1u << board::kI2c1Sda.pin)) != 0u;
}

bool LinesIdle() { return SclHigh() && SdaHigh(); }

void PinMode(GPIO_TypeDef *port, uint16_t pin, uint32_t moder_bits) {
  const uint32_t shift = static_cast<uint32_t>(pin) * 2u;
  port->MODER = (port->MODER & ~(0x3u << shift)) | (moder_bits << shift);
}

constexpr uint32_t kModerOutput = 0x1u;
constexpr uint32_t kModerAf = 0x2u;

}  // namespace

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::Init(const I2cConfig &config) {
  if (initialized_) {
    return;
  }
  initialized_ = true;
  cfg_ = config;

  if constexpr (Inst == I2cInstance::kI2c1) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;  // readback delay
  }

  // A slave abandoned mid-transaction by a reset on our side survives it on
  // theirs and holds SDA until it sees clocks. Free the wire before the
  // peripheral first samples it, or BUSY latches on from boot.
  MaybeRecoverBus();
  ConfigureHw();

  // BUSY latched with both lines reading high is the analog-filter lockup
  // (ES0182): the filter cannot be disabled on this part, and only the
  // manual waveform flushes it -- SWRST alone does not.
  if ((Hw()->SR2 & I2C_SR2_BUSY) != 0u && LinesIdle()) {
    RecoverBus();
    ConfigureHw();
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
  if (!SdaHigh()) {
    RecoverBus();
  }
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::RecoverBus() {
  // Borrow the pins as GPIO (the uart_soft precedent): AFR keeps AF4 and
  // OTYPER/PUPDR keep their open-drain-with-pull programming from
  // kGpioDefault; only MODER moves between AF and plain output. PE goes
  // off first so the peripheral is not driving what we are about to.
  I2C_TypeDef *hw = Hw();
  hw->CR1 = 0;

  GPIO_TypeDef *scl_port = board::kI2c1Scl.port;
  GPIO_TypeDef *sda_port = board::kI2c1Sda.port;
  const uint32_t scl_bit = 1u << board::kI2c1Scl.pin;
  const uint32_t sda_bit = 1u << board::kI2c1Sda.pin;
  const TimeBase &time = System::GetInstance().Time();

  scl_port->BSRR = scl_bit;  // never drive a claimed-low edge: release first
  sda_port->BSRR = sda_bit;
  PinMode(scl_port, board::kI2c1Scl.pin, kModerOutput);
  PinMode(sda_port, board::kI2c1Sda.pin, kModerOutput);

  // Up to 9 clocks walks a slave out of whatever bit of whatever byte it
  // died in: 8 data bits plus its ACK slot. Zero iterations when SDA is
  // already free -- the filter-flush caller only needs the STOP below.
  for (uint8_t pulse = 0; pulse < 9u && !SdaHigh(); ++pulse) {
    scl_port->BSRR = scl_bit << 16u;
    time.DelayMicros(kRecoverHalfPeriodUs);
    scl_port->BSRR = scl_bit;
    // A slave may stretch mid-recovery. SCL shorted low never rises; the
    // bound turns that into a counted failure instead of a boot hang.
    (void)SpinFor(kRecoverStretchUs, [] { return SclHigh(); });
    time.DelayMicros(kRecoverHalfPeriodUs);
  }

  // Close with a real STOP edge -- SDA rising while SCL is high -- so every
  // listener resets its frame state. SDA must fall while SCL is low first,
  // or the fall itself would read as a START. This exact waveform is also
  // ST's flush for the analog-filter lockup.
  scl_port->BSRR = scl_bit << 16u;
  time.DelayMicros(kRecoverHalfPeriodUs);
  sda_port->BSRR = sda_bit << 16u;
  time.DelayMicros(kRecoverHalfPeriodUs);
  scl_port->BSRR = scl_bit;
  time.DelayMicros(kRecoverHalfPeriodUs);
  sda_port->BSRR = sda_bit;
  time.DelayMicros(kRecoverHalfPeriodUs);

  PinMode(scl_port, board::kI2c1Scl.pin, kModerAf);
  PinMode(sda_port, board::kI2c1Sda.pin, kModerAf);
  recoveries_ = recoveries_ + 1;
}

template <I2cInstance Inst, size_t BufSize>
Outcome I2c<Inst, BufSize>::StartWrite(uint8_t addr7,
                                       std::span<const uint8_t> tx) {
  return Arm(addr7, tx, 0);
}

template <I2cInstance Inst, size_t BufSize>
Outcome I2c<Inst, BufSize>::StartWriteRead(uint8_t addr7,
                                           std::span<const uint8_t> tx,
                                           size_t rx_len) {
  if (tx.empty() || rx_len == 0u) {
    return Outcome::kInvalid;
  }
  return Arm(addr7, tx, rx_len);
}

template <I2cInstance Inst, size_t BufSize>
Outcome I2c<Inst, BufSize>::Arm(uint8_t addr7, std::span<const uint8_t> tx,
                                size_t rx_len) {
  if (!initialized_ || addr7 > 0x7Fu || tx.size() > BufSize ||
      rx_len > BufSize) {
    return Outcome::kInvalid;
  }
  if (status_.load(std::memory_order_acquire) == I2cTransferStatus::kBusy) {
    return Outcome::kRejected;
  }
  I2C_TypeDef *hw = Hw();
  // A back-to-back Start can catch the previous transfer's STOP still
  // draining, and CR1 must not be written over a pending START/STOP -- the
  // write can issue a second one (RM0090's CR1 note). Two SCL periods of
  // grace, then backpressure.
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

  // The configured timeout is a floor for short transfers, not a ceiling:
  // a full write-read at 100 kHz needs ~29 ms of wire time, and a knob
  // that aborted it mid-flight would fabricate bus faults.
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
  status_.store(I2cTransferStatus::kBusy, std::memory_order_relaxed);
  hw->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  // Publish every plain store above before the START that lets the ISRs
  // consume them; volatile MMIO alone does not order them.
  std::atomic_signal_fence(std::memory_order_release);
  hw->CR1 |= I2C_CR1_START;
  return Outcome::kOk;
}

template <I2cInstance Inst, size_t BufSize>
I2cTransferStatus I2c<Inst, BufSize>::Poll(uint32_t now_us) {
  if (status_.load(std::memory_order_acquire) == I2cTransferStatus::kBusy &&
      (now_us - started_us_) > deadline_us_) {
    AbortStuckTransfer();
  }
  return status_.load(std::memory_order_acquire);
}

template <I2cInstance Inst, size_t BufSize>
std::span<const uint8_t> I2c<Inst, BufSize>::Received() const {
  if (status_.load(std::memory_order_acquire) != I2cTransferStatus::kComplete) {
    return {};
  }
  return {rx_buf_, rx_pos_};
}

template <I2cInstance Inst, size_t BufSize>
void I2c<Inst, BufSize>::AbortStuckTransfer() {
  I2C_TypeDef *hw = Hw();
  {
    // Shut both handlers out, then take the engine apart. A handler that
    // was already pended runs once after the guard drops, sees kIdle, and
    // counts itself as spurious.
    BasepriGuard guard;
    hw->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    phase_ = Phase::kIdle;
  }
  timeouts_ = timeouts_ + 1;

  // If we still own the bus and nothing else is pending, a STOP releases
  // it cleanly. A slave pinning SDA makes the STOP unformable -- the wire
  // check below escalates that to the recovery clocking.
  if ((hw->SR2 & I2C_SR2_MSL) != 0u &&
      (hw->CR1 & (I2C_CR1_START | I2C_CR1_STOP)) == 0u) {
    hw->CR1 |= I2C_CR1_STOP;
    (void)SpinFor(kGracefulStopUs,
                  [hw] { return (hw->SR2 & I2C_SR2_BUSY) == 0u; });
  }

  ConfigureHw();
  if (!SdaHigh() || ((hw->SR2 & I2C_SR2_BUSY) != 0u && LinesIdle())) {
    RecoverBus();
    ConfigureHw();
  }

  status_.store(
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
  // Repeated START off the BTF stretch: deterministic under any interrupt
  // latency, and the read leg's ACK/POS arm here, where CR1 provably has
  // no START or STOP pending.
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
  // POS is deliberately not cleaned up here: the STOP just issued may still
  // be draining, and CR1 must not be written over it. The next Arm clears
  // it after proving CR1 idle.
  phase_ = Phase::kIdle;
  status_.store(status, std::memory_order_release);
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
      // The SR2 read releases the stretch and the lone byte starts
      // clocking. A preemption outlasting the byte would leave it NACKed
      // with no STOP programmed and the bus parked -- mask the gap shut,
      // it is four instructions.
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
        // Last byte queued. TXE stays asserted from here to the STOP, so
        // leaving ITBUFEN on is a livelock; BTF closes the phase instead,
        // and only fires once the byte is out and acknowledged.
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
        rx_buf_[rx_pos_] = static_cast<uint8_t>(hw->DR);
        ++rx_pos_;
        EndTransfer(I2cTransferStatus::kComplete);
      }
      return;
    }
    if ((sr1 & I2C_SR1_BTF) == 0u) {
      // Multi-byte reads act only at BTF, where SCL is stretched and
      // lateness costs bus time, never data. RXNE-paced reads under
      // interrupt latency clear ACK a byte late, handing the slave an ACK
      // on its true last byte -- a wedged bus and a garbage tail.
      return;
    }
    if (remaining == 3u) {
      // The NACK armed here governs the final byte; unloading one byte
      // lets it clock in. The next BTF then holds the last two under a
      // stretch, where the STOP is programmed with no deadline.
      hw->CR1 &= ~I2C_CR1_ACK;
      rx_buf_[rx_pos_] = static_cast<uint8_t>(hw->DR);
      ++rx_pos_;
      return;
    }
    if (remaining == 2u) {
      // Shared close of the POS pair and the >=3 tail: STOP first --
      // unloading DR first would un-stretch a bus with nothing programmed
      // -- then both held bytes.
      hw->CR1 |= I2C_CR1_STOP;
      rx_buf_[rx_pos_] = static_cast<uint8_t>(hw->DR);
      ++rx_pos_;
      rx_buf_[rx_pos_] = static_cast<uint8_t>(hw->DR);
      ++rx_pos_;
      EndTransfer(I2cTransferStatus::kComplete);
      return;
    }
    rx_buf_[rx_pos_] = static_cast<uint8_t>(hw->DR);  // body: one per BTF
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
    hw->SR1 = ~I2C_SR1_AF;
    nacks_ = nacks_ + 1;
    hw->CR1 |= I2C_CR1_STOP;
    EndTransfer(phase_ == Phase::kTx ? I2cTransferStatus::kNackData
                                     : I2cTransferStatus::kNackAddr);
    return;
  }
  if ((sr1 & I2C_SR1_ARLO) != 0u) {
    // Arbitration loss already put the interface back in slave mode with
    // the lines released; a STOP here would fire on someone else's frame.
    // On this single-master bus it is line-fault evidence, not traffic.
    hw->SR1 = ~I2C_SR1_ARLO;
    arb_losses_ = arb_losses_ + 1;
    EndTransfer(I2cTransferStatus::kArbitrationLost);
    return;
  }
  if ((sr1 & I2C_SR1_BERR) != 0u) {
    // ES0182: BERR asserts spuriously in master mode, so it never aborts a
    // transfer by itself. One a real bus error broke will stall, and the
    // deadline path reports kBusError off the flag noted here.
    hw->SR1 = ~I2C_SR1_BERR;
    bus_errors_ = bus_errors_ + 1;
    berr_seen_ = true;
  }
  if ((sr1 & I2C_SR1_OVR) != 0u) {
    // Unreachable for a stretching master; cleared so it cannot storm.
    hw->SR1 = ~I2C_SR1_OVR;
    spurious_irqs_ = spurious_irqs_ + 1;
  }
}

template class I2c<I2cInstance::kI2c1, kI2cBufSize>;

extern "C" {
void I2c1OnEventIrq() { I2c1::GetInstance().OnEventIrq(); }
void I2c1OnErrorIrq() { I2c1::GetInstance().OnErrorIrq(); }
}
