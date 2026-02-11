#include "icm42688p.hpp"

#include "board.h"
#include "spi.hpp"
#include "system.hpp"
#include <cmath>

using namespace Icm42688pReg;

// ─────────────────────────── CS pin helpers ──────────────────────────────

static inline void CsLow() {
  SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin << 16;
}
static inline void CsHigh() { SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin; }

// ─────────────────────────── Notch filter math ──────────────────────────

static constexpr float kPi = 3.14159265358979323846f;

static inline float Clamp(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// ─────────────────────────── Init ───────────────────────────────────────

void Icm42688p::Init(const Config &cfg) {
  if (initialized_)
    return;

  auto &sys = System::GetInstance();
  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();

  sys.Time().DelayMicros(10000);
  spi.SetPrescaler(static_cast<SpiPrescaler>(cfg.spi_prescaler));

  // Prepare DMA TX buffer: read command + 0xFF fill
  tx_[0] = kBurstStartReg | 0x80;
  for (unsigned i = 1; i < kXferLen; i++)
    tx_[i] = 0xFF;

  SetBank(0);

  // ── WHOAMI check ──────────────────────────────────────────────────────
  bool ok = false;
  for (int i = 0; i < 5; i++) {
    uint8_t who = ReadReg(REG_WHO_AM_I);
    sys.GetFcLink().SendLog("ICM42688P WHOAMI=0x%02X", who);
    if (who == WHO_AM_I_ICM42688P || who == WHO_AM_I_ICM42686P) {
      ok = true;
      break;
    }
    sys.Time().DelayMicros(1000);
  }
  if (!ok) {
    sys.GetFcLink().SendLog("ICM42688P WHOAMI fail");
    return;
  }

  // ── Soft reset ────────────────────────────────────────────────────────
  WriteReg(REG_DEVICE_CONFIG, DEVICE_CONFIG_SOFT_RESET);
  sys.Time().DelayMicros(2000);
  SetBank(0);

  {
    uint8_t who = ReadReg(REG_WHO_AM_I);
    if (who != WHO_AM_I_ICM42688P && who != WHO_AM_I_ICM42686P) {
      sys.GetFcLink().SendLog("ICM42688P WHOAMI fail after reset");
      return;
    }
  }

  // ── INTF_CONFIG1: clock source + disable AFSR ─────────────────────────
  //   CLKSEL = 01 → PLL auto-select (±1% vs ±8% RC-only)
  //   AFSR   = 01 → disabled (bit7=0 clear, bit6=1 set)
  {
    uint8_t v = ReadReg(REG_INTF_CONFIG1);
    v &= ~(INTF_CONFIG1_CLKSEL_CLEAR | INTF_CONFIG1_CLKSEL);
    v |= INTF_CONFIG1_CLKSEL;      // CLKSEL = 01
    v &= ~INTF_CONFIG1_AFSR_CLEAR; // bit7 = 0
    v |= INTF_CONFIG1_AFSR_SET;    // bit6 = 1 → AFSR off
    WriteReg(REG_INTF_CONFIG1, v);
  }

  // ── FSYNC: route GPS PPS → pin 9 ─────────────────────────────────────
  if (cfg.enable_fsync_pin9) {
    SetBank(1);
    {
      uint8_t v = ReadReg(REG_INTF_CONFIG5);
      v &= ~(0x3u << 1); // PIN9_FUNCTION = 00 (FSYNC, default)
      WriteReg(REG_INTF_CONFIG5, v);
    }
    SetBank(0);
  }

  // ── Timestamp + FSYNC config ──────────────────────────────────────────
  if (cfg.enable_tmst_regs || cfg.enable_tmst_fsync) {
    uint8_t tmst = 0;
    if (cfg.enable_tmst_regs)
      tmst |= TMST_CONFIG_TO_REGS_EN | TMST_CONFIG_EN;
    if (cfg.enable_tmst_fsync)
      tmst |= TMST_CONFIG_FSYNC_EN;
    WriteReg(REG_TMST_CONFIG, tmst);
  }

  if (cfg.enable_fsync_pin9) {
    uint8_t fs = (uint8_t)((cfg.fsync_ui_sel & 0x7u) << 4);
    if (cfg.fsync_polarity_falling)
      fs |= 0x01;
    WriteReg(REG_FSYNC_CONFIG, fs);
  }

  // ── Filters (Banks 1 & 2) ────────────────────────────────────────────
  ConfigureFilters(cfg);

  // ── Interrupt configuration ───────────────────────────────────────────

  // INT_CONFIG: Active-High, Push-Pull, Pulsed
  WriteReg(REG_INT_CONFIG,
           INT_CONFIG_INT1_DRIVE_CIRCUIT | INT_CONFIG_INT1_POLARITY);

  // INT_CONFIG1: clear INT_ASYNC_RESET, set 8µs pulse, keep deassertion enabled
  {
    uint8_t v = ReadReg(REG_INT_CONFIG1);
    v &= ~INT_CONFIG1_ASYNC_RESET;    // Required per datasheet
    v |= INT_CONFIG1_TPULSE_DURATION; // 8µs pulse width
    v &= ~INT_CONFIG1_TDEASSERT_DIS;  // Normal deassertion (reliable edge)
    WriteReg(REG_INT_CONFIG1, v);
  }

  // INT_CONFIG0: DRDY clears on sensor register read (our DMA reads data regs)
  {
    uint8_t v = ReadReg(REG_INT_CONFIG0);
    v &= ~(0x3u << 4);
    v |= INT_CONFIG0_DRDY_CLEAR_ON_SENSOR_READ;
    WriteReg(REG_INT_CONFIG0, v);
  }

  // INT_SOURCE0: enable DRDY on INT1
  {
    uint8_t v = ReadReg(REG_INT_SOURCE0);
    v |= INT_SOURCE0_UI_DRDY_INT1_EN;
    WriteReg(REG_INT_SOURCE0, v);
  }

  // ── Sensor power-on ──────────────────────────────────────────────────
  WriteReg(REG_PWR_MGMT0, PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);

  // Gyro needs 30ms to start producing valid data
  sys.Time().DelayMicros(30000);

  // ── ODR / Full-Scale / Filter BW ─────────────────────────────────────
  {
    uint8_t g = (uint8_t)((uint8_t)cfg.gyro_fs << 5) | (uint8_t)cfg.gyro_odr;
    uint8_t a = (uint8_t)((uint8_t)cfg.accel_fs << 5) | (uint8_t)cfg.accel_odr;
    WriteReg(REG_GYRO_CONFIG0, g);
    WriteReg(REG_ACCEL_CONFIG0, a);

    uint8_t bw = (uint8_t)((cfg.accel_ui_filt_bw & 0xF) << 4) |
                 (cfg.gyro_ui_filt_bw & 0xF);
    WriteReg(REG_GYRO_ACCEL_CONFIG0, bw);
  }

  WriteReg(REG_GYRO_CONFIG1, cfg.gyro_cfg1);
  WriteReg(REG_ACCEL_CONFIG1, cfg.accel_cfg1);

  initialized_ = true;

  // ── Enable EXTI for DRDY ─────────────────────────────────────────────
  NVIC_SetPriority(IMU_INT_EXTI_IRQn, 4);
  NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);

  sys.GetFcLink().SendLog("ICM42688P init OK");
}

// ─────────────────────────── Filter configuration ───────────────────────

void Icm42688p::ConfigureFilters(const Config &cfg) {
  SetBank(1);

  // ── Gyro notch filter ─────────────────────────────────────────────────
  {
    uint8_t s2 = ReadReg(REG_GYRO_CONFIG_STATIC2);

    if (cfg.notch_freq_hz > 0.0f) {
      s2 &= ~GYRO_CONFIG_STATIC2_NF_DIS;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);

      float f = Clamp(cfg.notch_freq_hz, 1.0f, 15999.0f);
      float coswz = cosf(2.0f * kPi * f / 32000.0f);

      uint16_t val = 0;
      uint8_t sel = 0;

      if (fabsf(coswz) <= 0.875f) {
        val = (uint16_t)((int32_t)lrintf(coswz * 256.0f) & 0x1FF);
        sel = 0;
      } else {
        sel = 1;
        if (coswz > 0.0f)
          val = (uint16_t)((int32_t)lrintf(8.0f * (1.0f - coswz) * 256.0f) &
                           0x1FF);
        else
          val = (uint16_t)((int32_t)lrintf(-8.0f * (1.0f + coswz) * 256.0f) &
                           0x1FF);
      }

      // Same coefficient for all three axes
      WriteReg(REG_GYRO_CONFIG_STATIC6, (uint8_t)(val & 0xFF));
      WriteReg(REG_GYRO_CONFIG_STATIC7, (uint8_t)(val & 0xFF));
      WriteReg(REG_GYRO_CONFIG_STATIC8, (uint8_t)(val & 0xFF));

      uint8_t s9 = 0;
      if (sel)
        s9 |= (1u << 5) | (1u << 4) | (1u << 3); // COSWZ_SEL for X/Y/Z
      if (val & 0x100)
        s9 |= (1u << 2) | (1u << 1) | (1u << 0); // MSB for X/Y/Z
      WriteReg(REG_GYRO_CONFIG_STATIC9, s9);

      uint8_t s10 = ReadReg(REG_GYRO_CONFIG_STATIC10);
      s10 &= ~(0x7u << 4);
      s10 |= (uint8_t)((cfg.notch_bw_idx & 0x7u) << 4);
      WriteReg(REG_GYRO_CONFIG_STATIC10, s10);
    } else {
      s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
    }
  }

  // ── Gyro anti-alias filter ────────────────────────────────────────────
  {
    uint8_t s2 = ReadReg(REG_GYRO_CONFIG_STATIC2);
    if (cfg.gyro_aaf_dis) {
      s2 |= GYRO_CONFIG_STATIC2_AAF_DIS;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
    } else {
      s2 &= ~GYRO_CONFIG_STATIC2_AAF_DIS;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);

      WriteReg(REG_GYRO_CONFIG_STATIC3, (uint8_t)(cfg.gyro_aaf_delt & 0x3F));
      WriteReg(REG_GYRO_CONFIG_STATIC4,
               (uint8_t)(cfg.gyro_aaf_delt_sqr & 0xFF));

      uint8_t s5 = (uint8_t)((cfg.gyro_aaf_bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.gyro_aaf_delt_sqr >> 8) & 0xF);
      WriteReg(REG_GYRO_CONFIG_STATIC5, s5);
    }
  }

  // ── Accel anti-alias filter (Bank 2) ──────────────────────────────────
  SetBank(2);

  {
    uint8_t s2 = ReadReg(REG_ACCEL_CONFIG_STATIC2);
    if (cfg.accel_aaf_dis) {
      s2 |= ACCEL_CONFIG_STATIC2_AAF_DIS;
      WriteReg(REG_ACCEL_CONFIG_STATIC2, s2);
    } else {
      s2 &= ~ACCEL_CONFIG_STATIC2_AAF_DIS;
      s2 &= ~(0x3Fu << 1);
      s2 |= (uint8_t)((cfg.accel_aaf_delt & 0x3F) << 1);
      WriteReg(REG_ACCEL_CONFIG_STATIC2, s2);

      WriteReg(REG_ACCEL_CONFIG_STATIC3,
               (uint8_t)(cfg.accel_aaf_delt_sqr & 0xFF));

      uint8_t s4 = (uint8_t)((cfg.accel_aaf_bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.accel_aaf_delt_sqr >> 8) & 0xF);
      WriteReg(REG_ACCEL_CONFIG_STATIC4, s4);
    }
  }

  SetBank(0);
}

// ─────────────────────────── ISR path ───────────────────────────────────

void Icm42688p::OnDrdyIrq() {
  if (!initialized_)
    return;

  last_irq_us_ = System::GetInstance().Time().MicrosCorrected();

  if (inflight_) {
    overrun_++;
    return;
  }

  inflight_ = true;
  CsLow();

  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  if (!spi.StartTxRxDma(tx_, rx_, kXferLen, &Icm42688p::SpiDoneThunk, this)) {
    CsHigh();
    inflight_ = false;
    overrun_++;
  }
}

extern "C" void Icm42688pOnDrdyIrq() { Icm42688p::GetInstance().OnDrdyIrq(); }

void Icm42688p::SpiDoneThunk(void *user, bool ok) {
  static_cast<Icm42688p *>(user)->OnSpiDone(ok);
}

void Icm42688p::OnSpiDone(bool ok) {
  CsHigh();

  if (!ok) {
    inflight_ = false;
    overrun_++;
    return;
  }

  // Burst layout after 1-byte address:
  //   [0..5]  Accel X/Y/Z (big-endian int16)
  //   [6..11] Gyro  X/Y/Z (big-endian int16)
  //   [12..13] Temp        (big-endian int16)
  const uint8_t *p = &rx_[1];

  auto s16 = [&](int idx) -> int16_t {
    return (int16_t)((uint16_t)(p[idx] << 8) | p[idx + 1]);
  };

  Sample s{};
  s.timestamp_us = last_irq_us_;
  s.accel[0] = s16(0);
  s.accel[1] = s16(2);
  s.accel[2] = s16(4);
  s.gyro[0] = s16(6);
  s.gyro[1] = s16(8);
  s.gyro[2] = s16(10);
  s.temp_raw = s16(12);
  s.seq = ++seq_;

  if (!sample_buf_.Push(s)) {
    overrun_++;
  }

  inflight_ = false;
}

bool Icm42688p::PopSample(Sample &out) { return sample_buf_.Pop(out); }

// ─────────────────────────── SPI helpers (blocking) ─────────────────────

void Icm42688p::SetBank(uint8_t bank) {
  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  uint8_t tx[2] = {(uint8_t)(REG_BANK_SEL & 0x7F), (uint8_t)(bank & 0x07)};
  CsLow();
  spi.Write(tx, 2);
  CsHigh();
}

void Icm42688p::WriteReg(uint8_t reg, uint8_t val) {
  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  uint8_t tx[2] = {(uint8_t)(reg & 0x7F), val};
  CsLow();
  spi.Write(tx, 2);
  CsHigh();
}

uint8_t Icm42688p::ReadReg(uint8_t reg) {
  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0x00};
  uint8_t rx[2] = {0, 0};
  CsLow();
  spi.TxRx(tx, rx, 2);
  CsHigh();
  return rx[1];
}
