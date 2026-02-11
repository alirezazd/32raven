#include "icm42688p.hpp"

#include "board.h"
#include "icm42688p_reg.hpp"
#include "spi.hpp"
#include "system.hpp"
#include <cmath>

using namespace Icm42688pReg;

static inline void CsLow() {
  SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin << 16;
}
static inline void CsHigh() { SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin; }

static inline float kPi() { return 3.14159265358979323846f; }

static inline float ClampF(float x, float lo, float hi) {
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

void Icm42688p::Init(const Config &cfg) {
  if (initialized_)
    return;

  auto &sys = System::GetInstance();
  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();

  sys.Time().DelayMicros(10000);
  spi.SetPrescaler(static_cast<SpiPrescaler>(cfg.spi_prescaler));

  tx_[0] = (uint8_t)(kBurstStartReg | 0x80);
  for (unsigned i = 1; i < kXferLen; i++)
    tx_[i] = 0xFF;

  SetBank(0);

  bool ok = false;
  for (int i = 0; i < 5; i++) {
    uint8_t who = ReadReg(REG_WHO_AM_I);
    sys.GetFcLink().SendLog("ICM42688P WHOAMI=0x%02X", who);
    if (who == WHO_AM_I_VAL || who == WHO_AM_I_VAL_686) {
      ok = true;
      break;
    }
    sys.Time().DelayMicros(1000);
  }
  if (!ok) {
    sys.GetFcLink().SendLog("ICM42688P WHOAMI mismatch");
    return;
  }

  WriteReg(REG_DEVICE_CONFIG, 0x01);
  sys.Time().DelayMicros(2000);
  SetBank(0);

  {
    uint8_t who = ReadReg(REG_WHO_AM_I);
    if (who != WHO_AM_I_VAL && who != WHO_AM_I_VAL_686) {
      sys.GetFcLink().SendLog("ICM42688P WHOAMI mismatch after reset");
      return;
    }
  }

  if (cfg.enable_fsync_pin9) {
    SetBank(1);
    uint8_t v = ReadReg(REG_INTF_CONFIG5);
    v &= ~(0x3u << 1);
    v |= (0x1u << 1);
    WriteReg(REG_INTF_CONFIG5, v);
    SetBank(0);
  }

  if (cfg.enable_tmst_regs || cfg.enable_tmst_fsync) {
    uint8_t tmst = 0;
    if (cfg.enable_tmst_regs) {
      tmst |= (1u << 4);
      tmst |= (1u << 0);
    }
    if (cfg.enable_tmst_fsync) {
      tmst |= (1u << 1);
    }
    WriteReg(REG_TMST_CONFIG, tmst);
  }

  if (cfg.enable_fsync_pin9) {
    uint8_t fs = 0;
    fs |= (uint8_t)((cfg.fsync_ui_sel & 0x7u) << 4);
    if (cfg.fsync_polarity_falling)
      fs |= 0x01;
    WriteReg(REG_FSYNC_CONFIG, fs);
  }

  ConfigureFilters(cfg);

  // INT_CONFIG (0x14): Physical INT1 pin configuration
  // Bit0=INT1_POLARITY(1=ActiveHigh), Bit1=INT1_DRIVE_CIRCUIT(1=PushPull),
  // Bit2=INT1_MODE(0=Pulsed)
  WriteReg(REG_INT_CONFIG,
           (1u << 1) | (1u << 0)); // Push-Pull, Active High, Pulsed

  {
    uint8_t v = ReadReg(REG_INT_CONFIG1);
    v &= ~(1u << 7); // INT_ASYNC_RESET = 0
    v |= (1u << 6);  // INT_TPULSE_DURATION = 1 (8us)
    v |= (1u << 5);  // INT_TDEASSERT_DISABLE = 1
    WriteReg(REG_INT_CONFIG1, v);
  }

  // INT_CONFIG0 (0x63): UI_DRDY_INT_CLEAR on sensor data read
  // We read data registers in DMA burst (not INT_STATUS), so use
  // CLEAR_ON_DATA_READ.
  {
    uint8_t v = ReadReg(REG_INT_CONFIG0);
    v &= ~(0x3u << 4);
    v |= INT_CONFIG0_CLEAR_ON_SENSOR_REG_READ; // 10b
    WriteReg(REG_INT_CONFIG0, v);
  }

  {
    uint8_t src0 = ReadReg(REG_INT_SOURCE0);
    src0 |= (1u << 3);
    WriteReg(REG_INT_SOURCE0, src0);
  }

  {
    uint8_t pwr = 0;
    pwr |= (0x3u << 2); // GYRO_MODE = LN
    pwr |= (0x3u << 0); // ACCEL_MODE = LN
    WriteReg(REG_PWR_MGMT0, pwr);
  }

  sys.Time().DelayMicros(30000);

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

  // Enable EXTI interrupt for DRDY pin
  NVIC_SetPriority(IMU_INT_EXTI_IRQn, 4);
  NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);

  sys.GetFcLink().SendLog("ICM42688P init OK");
}

void Icm42688p::ConfigureFilters(const Config &cfg) {
  SetBank(1);

  {
    uint8_t s2 = ReadReg(REG_GYRO_CONFIG_STATIC2);

    if (cfg.notch_freq_hz > 0.0f) {
      s2 &= ~(0x01);
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);

      float f = ClampF(cfg.notch_freq_hz, 1.0f, 15999.0f);
      float coswz = cosf(2.0f * kPi() * f / 32000.0f);

      uint16_t val = 0;
      uint8_t sel = 0;

      if (fabsf(coswz) <= 0.875f) {
        int32_t q = (int32_t)lrintf(coswz * 256.0f);
        val = (uint16_t)(q & 0x1FF);
        sel = 0;
      } else {
        sel = 1;
        if (coswz > 0.0f) {
          int32_t q = (int32_t)lrintf(8.0f * (1.0f - coswz) * 256.0f);
          val = (uint16_t)(q & 0x1FF);
        } else {
          int32_t q = (int32_t)lrintf(-8.0f * (1.0f + coswz) * 256.0f);
          val = (uint16_t)(q & 0x1FF);
        }
      }

      WriteReg(REG_GYRO_CONFIG_STATIC6, (uint8_t)(val & 0xFF));
      WriteReg(REG_GYRO_CONFIG_STATIC7, (uint8_t)(val & 0xFF));
      WriteReg(REG_GYRO_CONFIG_STATIC8, (uint8_t)(val & 0xFF));

      uint8_t s9 = 0;
      if (sel)
        s9 |= (1u << 5) | (1u << 4) | (1u << 3);
      if (val & 0x100)
        s9 |= (1u << 2) | (1u << 1) | (1u << 0);
      WriteReg(REG_GYRO_CONFIG_STATIC9, s9);

      uint8_t s10 = ReadReg(REG_GYRO_CONFIG_STATIC10);
      s10 &= ~(0x7u << 4);
      s10 |= (uint8_t)((cfg.notch_bw_idx & 0x7u) << 4);
      WriteReg(REG_GYRO_CONFIG_STATIC10, s10);
    } else {
      s2 |= 0x01;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
    }
  }

  {
    uint8_t s2 = ReadReg(REG_GYRO_CONFIG_STATIC2);
    if (cfg.gyro_aaf_dis) {
      s2 |= 0x02;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
    } else {
      s2 &= ~0x02;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);

      WriteReg(REG_GYRO_CONFIG_STATIC3, (uint8_t)(cfg.gyro_aaf_delt & 0x3F));
      WriteReg(REG_GYRO_CONFIG_STATIC4,
               (uint8_t)(cfg.gyro_aaf_delt_sqr & 0xFF));

      uint8_t s5 = (uint8_t)((cfg.gyro_aaf_bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.gyro_aaf_delt_sqr >> 8) & 0xF);
      WriteReg(REG_GYRO_CONFIG_STATIC5, s5);
    }
  }

  SetBank(2);

  {
    uint8_t s2 = ReadReg(REG_ACCEL_CONFIG_STATIC2);
    if (cfg.accel_aaf_dis) {
      s2 |= 0x01;
      WriteReg(REG_ACCEL_CONFIG_STATIC2, s2);
    } else {
      s2 &= ~0x01;
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

void Icm42688p::OnDrdyIrq() {
  if (!initialized_)
    return;

  last_irq_us_ = System::GetInstance().Time().Micros();

  if (inflight_) {
    overrun_++;
    return;
  }

  inflight_ = true;
  CsLow();

  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  bool started =
      spi.StartTxRxDma(tx_, rx_, kXferLen, &Icm42688p::SpiDoneThunk, this);
  if (!started) {
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

  const uint8_t *p = &rx_[1];

  auto u16 = [&](int idx) -> uint16_t {
    return (uint16_t)((uint16_t)p[idx] << 8) | (uint16_t)p[idx + 1];
  };
  auto s16 = [&](int idx) -> int16_t { return (int16_t)u16(idx); };

  Sample s{};
  s.timestamp_us = last_irq_us_;
  s.accel[0] = s16(0);
  s.accel[1] = s16(2);
  s.accel[2] = s16(4);
  s.gyro[0] = s16(6);
  s.gyro[1] = s16(8);
  s.gyro[2] = s16(10);
  s.seq = ++seq_;

  if (!ring_.Push(s)) {
    overrun_++;
  }

  inflight_ = false;
}

bool Icm42688p::PopSample(Sample &out) { return ring_.Pop(out); }

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
