#include "icm42688p.hpp"
#include "gpio.hpp"
#include "panic.hpp"

#include "board.h"
#include "spi.hpp"
#include "system.hpp"
#include "time_base.hpp"
#include <cmath>
#include <cstdio>

using namespace Icm42688pReg;

static inline float Clamp(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void Icm42688p::Init(GPIO &gpio, Spi1 &spi, const Config &cfg) {
  if (initialized_)
    return;

  fifo_hold_last_data_en_ = cfg.fifo.hold_last;
  fifo_wm_records_ = cfg.fifo.watermark_records;
  if (fifo_wm_records_ == 0 || fifo_wm_records_ > kMaxWatermarkRecords) {
    Panic(ErrorCode::kInvalidFifoWatermarkRecords);
  }

  gpio_ = &gpio;
  spi_ = &spi;

  System::GetInstance().Time().DelayMicros(MILLIS_TO_MICROS(10));
  spi.SetPrescaler(static_cast<SpiPrescaler>(cfg.spi_prescaler));

  CheckWhoAmI();
  SoftReset();
  SetBank(0);
  // Keep gyro/accel OFF while programming non-ODR/FS registers.
  // Matches datasheet register-modification guidance and reference drivers.
  WriteReg(REG_PWR_MGMT0, 0x00u);
  System::GetInstance().Time().DelayMicros(200);

  SetClockSource();
  SetInterfaceConfig(cfg);
  DisableFsync();
  SetInterruptConfig();

  ConfigureFilters(cfg);
  if (cfg.rates.gyro != cfg.rates.accel)
    Panic(ErrorCode::kImuOdrMismatch);
  SetOdrAndFullScale(cfg);

  SetTimestampConfig();
  ClearUserOffsets();
  // Keep temperature sensor disabled while accel/gyro are OFF during FIFO
  // setup.
  WriteReg(REG_PWR_MGMT0, PWR_MGMT0_TEMP_DIS);
  ConfigureFifo();

  // Enable sensors ONCE, last.
  WriteReg(REG_PWR_MGMT0, PWR_MGMT0_TEMP_DIS | PWR_MGMT0_GYRO_MODE_LN |
                              PWR_MGMT0_ACCEL_MODE_LN);

  // Datasheet: no register writes after OFF->ON transition
  System::GetInstance().Time().DelayMicros(200);
  // Optional: allow gyro to settle / become valid
  System::GetInstance().Time().DelayMicros(MILLIS_TO_MICROS(50));

  // EXTI->PR = (1u << 10);

  initialized_ = true;
  spi.EnableIrqs(); // SPI DMA interrupts
  NVIC_SetPriority(IMU_INT_EXTI_IRQn, 3);
  NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);
}

// Filter configuration

void Icm42688p::ConfigureFilters(const Config &cfg) {
  static constexpr float kPi = 3.14159265358979323846f;
  SetBank(1);

  // Gyro notch filter (Bank 1)
  {
    uint8_t s2 = ReadReg(REG_GYRO_CONFIG_STATIC2);

    // ---- Gyro notch filter (datasheet correct) ----
    if (cfg.notch.enabled && cfg.notch.freq_hz > 0.0f) {
      // Datasheet: supported 1kHz..3kHz
      float f_hz = Clamp(cfg.notch.freq_hz, 1000.0f, 3000.0f);

      // Need CLKDIV from Bank 3 reg 0x2A
      SetBank(3);
      uint8_t clkdiv = ReadReg(REG_CLKDIV);
      SetBank(1);

      if (clkdiv == 0) {
        // Invalid, do not enable notch
        s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
        WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
        uint8_t bsel = ReadReg(REG_BANK_SEL);
        System::GetInstance().GetFcLink().SendLog(
            "BANK_SEL for GYRO_CONFIG_STATIC2=%02X", bsel);
      } else {
        float fdrv = 19.2e6f / (float(clkdiv) * 10.0f);
        float coswz = cosf(2.0f * kPi * (f_hz / fdrv));

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

        // Enable notch
        s2 &= ~GYRO_CONFIG_STATIC2_NF_DIS;
        uint8_t bsel = ReadReg(REG_BANK_SEL);
        System::GetInstance().GetFcLink().SendLog(
            "BANK_SEL for GYRO_CONFIG_STATIC2=%02X", bsel);
        WriteReg(REG_GYRO_CONFIG_STATIC2, s2);

        // Same coefficient for all three axes
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
        uint8_t bw = (cfg.notch.bw_idx > 7) ? 7 : cfg.notch.bw_idx;
        s10 |= (uint8_t)(bw << 4);
        WriteReg(REG_GYRO_CONFIG_STATIC10, s10);
      }
    } else {
      s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
      uint8_t bsel = ReadReg(REG_BANK_SEL);
      System::GetInstance().GetFcLink().SendLog(
          "BANK_SEL for GYRO_CONFIG_STATIC2=%02X", bsel);
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
    }
  }

  // Gyro anti-alias filter (Bank 1)
  {
    uint8_t s2 = ReadReg(REG_GYRO_CONFIG_STATIC2);
    if (cfg.gyro_aaf.dis) {
      uint8_t bsel = ReadReg(REG_BANK_SEL);
      System::GetInstance().GetFcLink().SendLog(
          "BANK_SEL for GYRO_CONFIG_STATIC2=%02X", bsel);
      s2 |= GYRO_CONFIG_STATIC2_AAF_DIS;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
    } else {
      s2 &= ~GYRO_CONFIG_STATIC2_AAF_DIS;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);

      WriteReg(REG_GYRO_CONFIG_STATIC3, (uint8_t)(cfg.gyro_aaf.delt & 0x3F));
      WriteReg(REG_GYRO_CONFIG_STATIC4,
               (uint8_t)(cfg.gyro_aaf.delt_sqr & 0xFF));

      uint8_t s5 = (uint8_t)((cfg.gyro_aaf.bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.gyro_aaf.delt_sqr >> 8) & 0xF);
      WriteReg(REG_GYRO_CONFIG_STATIC5, s5);
    }
  }

  // Accel anti-alias filter (Bank 2)
  SetBank(2);

  {
    uint8_t s2 = ReadReg(REG_ACCEL_CONFIG_STATIC2);
    if (cfg.accel_aaf.dis) {
      uint8_t bsel = ReadReg(REG_BANK_SEL);
      System::GetInstance().GetFcLink().SendLog(
          "BANK_SEL for ACCEL_CONFIG_STATIC2=%02X", bsel);
      s2 |= ACCEL_CONFIG_STATIC2_AAF_DIS;
      WriteReg(REG_ACCEL_CONFIG_STATIC2, s2);
    } else {
      s2 &= ~ACCEL_CONFIG_STATIC2_AAF_DIS;
      s2 &= ~(0x3Fu << 1);
      s2 |= (uint8_t)((cfg.accel_aaf.delt & 0x3F) << 1);
      WriteReg(REG_ACCEL_CONFIG_STATIC2, s2);

      WriteReg(REG_ACCEL_CONFIG_STATIC3,
               (uint8_t)(cfg.accel_aaf.delt_sqr & 0xFF));

      uint8_t s4 = (uint8_t)((cfg.accel_aaf.bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.accel_aaf.delt_sqr >> 8) & 0xF);
      WriteReg(REG_ACCEL_CONFIG_STATIC4, s4);
    }
  }

  SetBank(0);
}

// ISR path

void Icm42688p::OnIrq() {
  irq_cnt_.fetch_add(1, std::memory_order_relaxed);
  last_irq_us_ = System::GetInstance().Time().MicrosCorrected();

  if (inflight_.exchange(true, std::memory_order_acq_rel)) {
    overrun_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  fifo_stage_ = FifoDmaStage::kCount;

  // Stage 1: read FIFO_COUNTH + FIFO_COUNTL only.
  // Do not read past COUNTL here, otherwise FIFO_DATA pointer can advance and
  // stage-2 parsing becomes byte-shifted.
  fifo_tx_[0] = static_cast<uint8_t>(REG_FIFO_COUNTH | 0x80u);
  fifo_tx_[1] = 0xFFu;
  fifo_tx_[2] = 0xFFu;

  fifo_xfer_len_ = 3; // 1 cmd + 2 rx bytes

  auto &spi = *spi_;
  CsLow();
  if (!spi.StartTxRxDma(fifo_tx_, fifo_rx_, fifo_xfer_len_,
                        &Icm42688p::SpiDoneThunk, this)) {
    CsHigh();
    inflight_.store(false, std::memory_order_release);
    dma_start_fail_cnt_.fetch_add(1, std::memory_order_relaxed);
    overrun_.fetch_add(1, std::memory_order_relaxed);
  }
}

extern "C" void Icm42688pOnIrq() { Icm42688p::GetInstance().OnIrq(); }

void Icm42688p::SpiDoneThunk(void *user, bool ok) {
  static_cast<Icm42688p *>(user)->OnSpiDone(ok);
}

void Icm42688p::OnSpiDone(bool ok) {
  CsHigh();

  auto finish = [&]() { inflight_.store(false, std::memory_order_release); };

  if (!ok) {
    overrun_.fetch_add(1, std::memory_order_relaxed);
    finish();
    return;
  }

  // Big-endian decode for FIFO_COUNT because FIFO_COUNT_ENDIAN=1.
  auto be_u16 = [](const uint8_t *q) -> uint16_t {
    return static_cast<uint16_t>((static_cast<uint16_t>(q[0]) << 8) |
                                 static_cast<uint16_t>(q[1]));
  };

  auto &spi = *spi_;

  // ------------------------
  // Stage 1: FIFO_COUNT read
  // ------------------------
  if (fifo_stage_ == FifoDmaStage::kCount) {
    const uint8_t *r = &fifo_rx_[1]; // rx[0] is dummy (cmd response)
    last_count_ = be_u16(&r[0]);     // COUNTH, COUNTL (records)

    const uint16_t avail_records = last_count_;
    if (avail_records == 0) {
      fifo_empty_cnt_.fetch_add(1, std::memory_order_relaxed);
      finish();
      return;
    }

    // Drain cap (specialized): read at most fifo_wm_records_ (8) and never
    // exceed buffer.
    uint16_t rd_records = avail_records;
    if (rd_records > fifo_wm_records_)
      rd_records = fifo_wm_records_;
    if (rd_records > kMaxWatermarkRecords)
      rd_records = kMaxWatermarkRecords;

    fifo_read_bytes_ = static_cast<uint16_t>(rd_records * kPacketBytes);
    fifo_xfer_len_ = static_cast<uint16_t>(1u + fifo_read_bytes_);

    // Prepare stage 2: FIFO_DATA burst
    fifo_tx_[0] = static_cast<uint8_t>(REG_FIFO_DATA | 0x80u);
    for (uint16_t i = 1; i < fifo_xfer_len_; i++) {
      fifo_tx_[i] = 0xFFu;
    }

    fifo_stage_ = FifoDmaStage::kData;

    CsLow();
    if (!spi.StartTxRxDma(fifo_tx_, fifo_rx_, fifo_xfer_len_,
                          &Icm42688p::SpiDoneThunk, this)) {
      CsHigh();
      inflight_.store(false, std::memory_order_release);
      dma_start_fail_cnt_.fetch_add(1, std::memory_order_relaxed);
      overrun_.fetch_add(1, std::memory_order_relaxed);
    }
    return;
  }

  // ------------------------
  // Stage 2: FIFO_DATA read
  // ------------------------
  fifo_stage_ = FifoDmaStage::kCount;

  if ((fifo_read_bytes_ == 0) || ((fifo_read_bytes_ % kPacketBytes) != 0)) {
    overrun_.fetch_add(1, std::memory_order_relaxed);
    finish();
    return;
  }

  const uint16_t records =
      static_cast<uint16_t>(fifo_read_bytes_ / kPacketBytes);
  const uint8_t *p = &fifo_rx_[1];

  // Latest-only semantics: parse last record in the drained block
  const uint8_t *rec = p + static_cast<uint32_t>(records - 1u) * kPacketBytes;

  Sample s{};
  if (!ParsePacket3Record(rec, s)) {
    // Parsing failed (bad header, etc.)
    last_bad_header_.store(rec[0], std::memory_order_relaxed);
    parse_fail_cnt_.fetch_add(1, std::memory_order_relaxed);
    overrun_.fetch_add(1, std::memory_order_relaxed);
    finish();
    return;
  }

  s.seq = ++seq_;
  PublishLatest(s);

  finish();
}

void Icm42688p::UpdateTimestampAndSync(uint16_t ts16, uint64_t &out_host_us) {
  // Unwrap 16-bit timestamp (1us ticks) into tmst64_us_
  if (!tmst_inited_) {
    last_tmst16_ = ts16;
    tmst64_us_ = 0;
    tmst_inited_ = true;
  } else {
    const uint16_t dt16 = static_cast<uint16_t>(ts16 - last_tmst16_);
    last_tmst16_ = ts16;
    tmst64_us_ += static_cast<uint64_t>(dt16);
  }

  const int64_t host_us = static_cast<int64_t>(last_irq_us_);
  const int64_t imu_us = static_cast<int64_t>(tmst64_us_);

  // Slow offset servo (keeps timestamps in host domain)
  if (!host_sync_inited_) {
    host_offset_us_ = host_us - imu_us;
    host_sync_inited_ = true;
  } else {
    const int64_t pred_host = imu_us + host_offset_us_;
    const int64_t err = host_us - pred_host;
    host_offset_us_ += (err >> 7);
  }

  out_host_us = static_cast<uint64_t>(imu_us + host_offset_us_);
}

bool Icm42688p::ParsePacket3Record(const uint8_t *rec, Sample &out) {
  if (!rec)
    return false;

  // Header check for Packet3 accel+gyro+temp+timestamp, 16-bit mode.
  // Accept both normal packets (0x68) and ODR-change-tagged variants (0x6C).
  const uint8_t hdr = rec[0];
  if ((hdr & 0xF8u) != 0x68u) {
    return false;
  }

  auto be_u16 = [](const uint8_t *q) -> uint16_t {
    return static_cast<uint16_t>((static_cast<uint16_t>(q[0]) << 8) |
                                 static_cast<uint16_t>(q[1]));
  };
  auto be_s16 = [](const uint8_t *q) -> int16_t {
    return static_cast<int16_t>((static_cast<uint16_t>(q[0]) << 8) |
                                static_cast<uint16_t>(q[1]));
  };

  // Packet3 timestamp field stays at bytes 0x0E..0x0F.
  const uint16_t ts16 = be_u16(&rec[0x0E]);

  uint64_t host_ts_us = 0;
  UpdateTimestampAndSync(ts16, host_ts_us);

  out.timestamp_us = host_ts_us;

  // Packet layout (big-endian)
  out.accel[0] = be_s16(&rec[0x01]);
  out.accel[1] = be_s16(&rec[0x03]);
  out.accel[2] = be_s16(&rec[0x05]);

  out.gyro[0] = be_s16(&rec[0x07]);
  out.gyro[1] = be_s16(&rec[0x09]);
  out.gyro[2] = be_s16(&rec[0x0B]);

  // INTF_CONFIG0.FIFO_HOLD_LAST_DATA_EN=0 inserts invalid code (-32768).
  // Treat any detected invalid code as a hard fault.
  if (!fifo_hold_last_data_en_) {
    static constexpr int16_t kInvalid16 = static_cast<int16_t>(-32768);
    if (out.accel[0] == kInvalid16 || out.accel[1] == kInvalid16 ||
        out.accel[2] == kInvalid16 || out.gyro[0] == kInvalid16 ||
        out.gyro[1] == kInvalid16 || out.gyro[2] == kInvalid16) {
      // Panic(ErrorCode::kImuInvalidSampleDetected);
      return false;
    }
  }

  // Temperature is disabled (PWR_MGMT0.TEMP_DIS=1, FIFO_TEMP_EN=0).
  out.temp_raw = 0;

  return true;
}

void Icm42688p::PublishLatest(const Sample &s_in) {
  const uint8_t kCur = mailbox_idx_.load(std::memory_order_relaxed);
  const uint8_t kNext = (uint8_t)(kCur ^ 1u);

  mailbox_[kNext] = s_in;

  // Publish payload then publish sequence (release).
  mailbox_idx_.store(kNext, std::memory_order_release);
  tick_seq_.store(s_in.seq, std::memory_order_release);
  publish_cnt_.fetch_add(1, std::memory_order_relaxed);
}

bool Icm42688p::WaitAndGetLatest(uint32_t &last_seq, Sample &out) {
  uint32_t s = tick_seq_.load(std::memory_order_acquire);
  if (s == last_seq) {
    return false;
  }

  if (last_seq != 0) {
    const uint32_t kMissed = (uint32_t)(s - last_seq - 1u);
    if (kMissed) {
      drop_cnt_.fetch_add(kMissed, std::memory_order_relaxed);
    }
  }

  for (;;) {
    const uint8_t kIdx = mailbox_idx_.load(std::memory_order_acquire);
    out = mailbox_[kIdx];
    const uint32_t kS2 = tick_seq_.load(std::memory_order_acquire);

    if (kS2 == s && out.seq == s) {
      last_seq = s;
      return true;
    }

    s = kS2;
    if (s == last_seq) {
      return false;
    }
  }
}

void Icm42688p::CheckWhoAmI() {
  auto &time = System::GetInstance().Time();
  const uint32_t kStart = time.Micros();

  SetBank(0);
  while ((uint32_t)(time.Micros() - kStart) < MILLIS_TO_MICROS(1000)) {
    uint8_t who = ReadReg(REG_WHO_AM_I);
    if (who == WHO_AM_I_ICM42688P || who == WHO_AM_I_ICM42686P) {
      return;
    }
    time.DelayMicros(MILLIS_TO_MICROS(10));
  }

  Panic(ErrorCode::kImuWhoAmIFail);
}

void Icm42688p::SoftReset() {
  SetBank(0);

  uint8_t v = ReadReg(REG_DEVICE_CONFIG);
  v |= 0x01u; // SOFT_RESET_CONFIG = 1, preserve SPI_MODE + reserved
  WriteReg(REG_DEVICE_CONFIG, v);

  System::GetInstance().Time().DelayMicros(MILLIS_TO_MICROS(1));
  SetBank(0);
}

void Icm42688p::SetClockSource() {
  SetBank(0);

  uint8_t v = ReadReg(REG_INTF_CONFIG1);
  // INTF_CONFIG1:
  // - AFSR bits[7:6] = 01 (disable adaptive full-scale)
  // - CLKSEL bits[1:0] = 01 (PLL when available)
  // Preserve RTC_MODE and reserved bits.
  v &= static_cast<uint8_t>(~0xC3u);
  v |= 0x41u;
  WriteReg(REG_INTF_CONFIG1, v);
}

void Icm42688p::SetInterfaceConfig(const Config &cfg) {
  SetBank(0);

  uint8_t v = ReadReg(REG_INTF_CONFIG0);

  // Keep reserved bits (3:2), update everything else we control:
  // - bit7 FIFO_HOLD_LAST_DATA_EN (user-configurable)
  // - bit6 FIFO_COUNT_REC = 1 (record mode)
  // - bit5 FIFO_COUNT_ENDIAN = 1 (big-endian count)
  // - bit4 SENSOR_DATA_ENDIAN = 1 (big-endian sensor data)
  // - bits1:0 UI_SIFS_CFG = 3 (disable I2C, keep SPI active)
  uint8_t cfg_bits = 0x73u;
  if (cfg.fifo.hold_last) {
    cfg_bits |= 0x80u;
  }
  v = static_cast<uint8_t>((v & 0x0Cu) | cfg_bits);
  WriteReg(REG_INTF_CONFIG0, v);
}

void Icm42688p::SetInterruptConfig() {
  SetBank(0);
  uint8_t v = ReadReg(REG_INT_CONFIG);
  // Preserve reserved bits 7:6, rewrite bits 5:0
  // INT2: pulsed(0), open-drain(0), active-low(0) => bits [5:3] = 000
  // INT1: pulsed(0), push-pull(1), active-high(1) => bits [2:0] = 0b011
  v = static_cast<uint8_t>((v & 0xC0u) | 0x03u);
  WriteReg(REG_INT_CONFIG, v);
  // ODR >= 4kHz: 8us pulse + deassert disabled.
  // Datasheet note: set INT_ASYNC_RESET=0 for proper INT operation.
  // Preserve reserved bits: bit7 and bits3:0
  // Set bits6:4 = 0b110 (8us pulse, disable deassert duration, async_reset=0)
  v = ReadReg(REG_INT_CONFIG1);
  v = static_cast<uint8_t>((v & 0x8Fu) | 0x60u);
  WriteReg(REG_INT_CONFIG1, v);
}

void Icm42688p::DisableFsync() {
  SetBank(0);

  uint8_t v = ReadReg(REG_FSYNC_CONFIG);

  // Preserve reserved bits: bit7 and bits3:2
  // Clear only: FSYNC_UI_SEL (6:4), FSYNC_UI_FLAG_CLEAR_SEL (1),
  // FSYNC_POLARITY (0)
  v &= static_cast<uint8_t>(~0x73u); // ~0b01110011
  WriteReg(REG_FSYNC_CONFIG, v);
}

void Icm42688p::SetOdrAndFullScale(const Config &cfg) {
  SetBank(0);
  // GYRO_CONFIG0: [7:5]=FS (3b), bit4 reserved, [3:0]=ODR (4b)
  const uint8_t kGyro =
      static_cast<uint8_t>(((static_cast<uint8_t>(cfg.fs.gyro) & 0x07u) << 5) |
                           (static_cast<uint8_t>(cfg.rates.gyro) & 0x0Fu));
  // ACCEL_CONFIG0: [7:5]=FS (only 0..3 valid), bit4 reserved, [3:0]=ODR (4b)
  const uint8_t kAccel =
      static_cast<uint8_t>(((static_cast<uint8_t>(cfg.fs.accel) & 0x03u) << 5) |
                           (static_cast<uint8_t>(cfg.rates.accel) & 0x0Fu));
  WriteReg(REG_GYRO_CONFIG0, kGyro);
  WriteReg(REG_ACCEL_CONFIG0, kAccel);
}

void Icm42688p::SetTimestampConfig() {
  SetBank(0);

  uint8_t v = ReadReg(REG_TMST_CONFIG);

  // Preserve reserved bits 7:5
  // Clear bits 4:1
  v &= 0xE0u;

  // Set TMST_EN = 1
  v |= 0x01u;

  WriteReg(REG_TMST_CONFIG, v);
}

void Icm42688p::ClearUserOffsets() {
  SetBank(0);
  WriteReg(REG_OFFSET_USER0, 0x00u);
  WriteReg(REG_OFFSET_USER1, 0x00u);
  WriteReg(REG_OFFSET_USER2, 0x00u);
  WriteReg(REG_OFFSET_USER3, 0x00u);
  WriteReg(REG_OFFSET_USER4, 0x00u);
  WriteReg(REG_OFFSET_USER5, 0x00u);
  WriteReg(REG_OFFSET_USER6, 0x00u);
  WriteReg(REG_OFFSET_USER7, 0x00u);
  WriteReg(REG_OFFSET_USER8, 0x00u);
}

void Icm42688p::ConfigureFifo() {
  SetBank(0);
  uint8_t v = ReadReg(REG_INT_CONFIG0); // Preserve reserved [7:6], replace
                                        // [5:0]
  v = static_cast<uint8_t>((v & 0xC0u) | 0x0Au);
  WriteReg(REG_INT_CONFIG0, v);
  v = ReadReg(REG_INT_SOURCE0); // Preserve reserved bit7, clear bits6:0
  v = static_cast<uint8_t>((v & 0x80u) | 0x04u); // FIFO_THS -> INT1 only
  WriteReg(REG_INT_SOURCE0, v); // Packet3, 16-bit, timestamp, DMA-safe
  v = ReadReg(REG_FIFO_CONFIG);
  v &= 0x3F; // clear bits 7:6 → FIFO_MODE = 00 (bypass)
  WriteReg(REG_FIFO_CONFIG, v);
  WriteReg(REG_FIFO_CONFIG1,
           0x4B); // FIFO_CONFIG1: resume partial read + accel+gyro+timestamp,
                  // FIFO_TEMP_EN=0, 16-bit (HIRES=0)
  const uint16_t fifo_wm_bytes =
      static_cast<uint16_t>(fifo_wm_records_ * kPacketBytes);
  WriteReg(REG_FIFO_CONFIG2, static_cast<uint8_t>(fifo_wm_bytes & 0xFFu));
  WriteReg(REG_FIFO_CONFIG3,
           static_cast<uint8_t>((fifo_wm_bytes >> 8) & 0x0Fu));
  WriteReg(REG_SIGNAL_PATH_RESET, 0x02); // FIFO_FLUSH
  (void)ReadReg(REG_INT_STATUS);         // clear any pending status bits (R/C)
  WriteReg(REG_FIFO_CONFIG, 0x40);       // FIFO Stream mode
  SetupDmaBuffer();
}

void Icm42688p::SetupDmaBuffer() {
  // Start in count stage. Actual fifo_read_bytes_/fifo_xfer_len_ are computed
  // per IRQ.
  fifo_stage_ = FifoDmaStage::kCount;

  // Optional: prefill TX with 0xFF so we only write the command byte at
  // runtime.
  for (uint16_t i = 0; i < sizeof(fifo_tx_); i++) {
    fifo_tx_[i] = 0xFFu;
  }
  // No fixed command here anymore. OnIrq() writes REG_FIFO_COUNTH|0x80,
  // and OnSpiDone(count) writes REG_FIFO_DATA|0x80.
}
// SPI helpers (blocking)

void Icm42688p::SetBank(uint8_t bank) {
  WriteReg(REG_BANK_SEL, (uint8_t)(bank & 0x07));
  System::GetInstance().Time().DelayMicros(1);
}

void Icm42688p::WriteReg(uint8_t reg, uint8_t val) {
  auto &spi = *spi_;
  uint8_t tx[2] = {(uint8_t)(reg & 0x7F), val};
  CsLow();
  spi.Write(tx, 2);
  CsHigh();
}
uint8_t Icm42688p::ReadReg(uint8_t reg) {
  auto &spi = *spi_;
  uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0x00};
  uint8_t rx[2] = {0, 0};
  CsLow();
  spi.TxRx(tx, rx, 2);
  CsHigh();
  return rx[1];
}
void Icm42688p::CsLow() {
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, false);
}
void Icm42688p::CsHigh() {
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, true);
}
