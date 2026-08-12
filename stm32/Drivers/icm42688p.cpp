// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "icm42688p.hpp"

#include <cmath>
#include <utility>

#include "config_storage.hpp"
#include "error_code.hpp"
#include "gpio.hpp"
#include "irq_priority.hpp"
#include "panic.hpp"
#include "spi.hpp"
#include "stm32_config.hpp"
#include "system.hpp"
#include "time_base.hpp"

using namespace Icm42688pReg;

namespace {

constexpr uint16_t LoadBe16(const uint8_t *p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

constexpr int16_t LoadBe16Signed(const uint8_t *p) {
  return static_cast<int16_t>(LoadBe16(p));
}

}  // namespace

Icm42688p &Icm42688p::GetInstance() {
  static Icm42688p inst;
  return inst;
}

void Icm42688p::Init(GPIO &gpio, Spi2 &spi, EE &ee, const Config &cfg) {
  if (device_id_ != 0u) {
    Panic(ErrorCode::Stm32::kImuReinit);
  }

  ValidateConfig(cfg);

  gpio_ = &gpio;
  spi_ = &spi;
  ee_ = &ee;
  fifo_wm_records_ = cfg.fifo.watermark_records;
  fifo_hold_last_data_en_ = cfg.fifo.hold_last;
  hires_ = cfg.fifo.hires;
  packet_bytes_ =
      hires_ ? Icm42688pReg::kPacket4Bytes : Icm42688pReg::kPacket3Bytes;
  accel_fs_ = cfg.fs.accel;
  gyro_fs_ = cfg.fs.gyro;
  axis_map_ = cfg.axis_map;
  gyro_odr_ = cfg.rates.gyro;
  gyro_odr_hz_ = EffectiveOdrHz(cfg.rates.gyro, cfg.external_clock);
  timestamp_tick_scale_q16_ = TimestampTickScaleQ16(cfg.external_clock);
  timestamp_tick_remainder_q16_ = 0;
  calibration_cfg_ = cfg.calibration;
  recovery_cfg_ = cfg.recovery;
  accel_calibration_ = ConfigStorage::LoadOrInitImuAccelCalibration(ee);

  System::GetInstance().Time().DelayMicros(MillisToMicros(10));
  spi.SetPrescaler(cfg.spi_prescaler);

  CheckWhoAmI();
  device_id_ = 2u | (static_cast<uint32_t>(SpiInstance::kSpi2) << 3) |
               (static_cast<uint32_t>(who_am_i_) << 16);
  SoftReset();
  SetBank(0);
  // Datasheet: keep gyro/accel OFF while programming non-ODR/FS registers.
  WriteReg(Reg::kPwrMgmt0, 0x00u);
  System::GetInstance().Time().DelayMicros(200);

  SetClockSource(cfg);
  SetInterfaceConfig(cfg);
  DisableFsync();
  SetInterruptConfig();

  ConfigureFilters(cfg);
  SetOdrAndFullScale(cfg);

  SetTimestampConfig();
  ClearUserOffsets();
  // Keep temp sensor disabled while accel/gyro OFF during FIFO setup.
  WriteReg(Reg::kPwrMgmt0, PWR_MGMT0_TEMP_DIS);
  ConfigureFifo();

  // Enable all sensors (incl. temp, for FIFO die-temp) once, last.
  WriteReg(Reg::kPwrMgmt0, PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);

  // Datasheet: no register writes for 200us after OFF->ON transition.
  System::GetInstance().Time().DelayMicros(200);
  // Allow gyro to settle / become valid.
  System::GetInstance().Time().DelayMicros(MillisToMicros(50));

  spi.EnableIrqs(irq_priority::kImuSpiDma);
  NVIC_SetPriority(board::kImuInt.exti_irqn, irq_priority::kImuInt);
  NVIC_EnableIRQ(board::kImuInt.exti_irqn);
}

bool Icm42688p::SaveAccelCalibration() {
  if (ee_ == nullptr) {
    return false;
  }
  return ConfigStorage::SaveImuAccelCalibration(*ee_, accel_calibration_);
}

uint32_t Icm42688p::GetDeviceId() const {
  if (device_id_ == 0u) {
    Panic(ErrorCode::Stm32::kImuNotInitialized);
  }

  return device_id_;
}

void Icm42688p::ValidateConfig(const Config &cfg) {
  if (cfg.fifo.watermark_records == 0 ||
      cfg.fifo.watermark_records > kMaxWatermarkRecords) {
    Panic(ErrorCode::Stm32::kInvalidFifoWatermarkRecords);
  }

  // A watermark past the FIFO's capacity in bytes still fits the 12-bit field
  // and is written without complaint; the level is simply never reached, so
  // the interrupt that drives the fast loop never fires.
  const uint32_t watermark_bytes =
      static_cast<uint32_t>(cfg.fifo.watermark_records) *
      (cfg.fifo.hires ? Icm42688pReg::kPacket4Bytes
                      : Icm42688pReg::kPacket3Bytes);
  if (watermark_bytes > Icm42688pReg::kFifoBytes) {
    Panic(ErrorCode::Stm32::kInvalidFifoWatermarkRecords);
  }

  if (cfg.rates.gyro != cfg.rates.accel) {
    Panic(ErrorCode::Stm32::kImuOdrMismatch);
  }

  if (cfg.external_clock.enabled &&
      (cfg.external_clock.frequency_hz < kExternalClockMinHz ||
       cfg.external_clock.frequency_hz > kExternalClockMaxHz)) {
    Panic(ErrorCode::Stm32::kImuInvalidOdr);
  }

  if (Icm42688pReg::OdrHz(cfg.rates.gyro) == 0 ||
      Icm42688pReg::OdrHz(cfg.rates.accel) == 0) {
    Panic(ErrorCode::Stm32::kImuInvalidOdr);
  }

  if (cfg.calibration.gyro_duration_s == 0 ||
      cfg.calibration.gyro_timeout_s == 0) {
    Panic(ErrorCode::Stm32::kImuCalibrationInvalidConfig);
  }

  if (cfg.recovery.overrun_threshold == 0 ||
      cfg.recovery.overrun_window_s == 0) {
    Panic(ErrorCode::Stm32::kImuOverrun);
  }

  // axis_map must be a permutation of {0,1,2}: aliasing/dropping an axis
  // silently misconfigures the estimator with no obvious flight symptom.
  const uint8_t a = cfg.axis_map.x_from;
  const uint8_t b = cfg.axis_map.y_from;
  const uint8_t c = cfg.axis_map.z_from;
  if (a > 2u || b > 2u || c > 2u || a == b || a == c || b == c) {
    Panic(ErrorCode::Stm32::kImuAxisMapInvalid);
  }
}

// Filter configuration

void Icm42688p::ConfigureFilters(const Config &cfg) {
  static constexpr float kPi = 3.14159265358979323846f;
  const auto clampf = [](float x, float lo, float hi) constexpr {
    return (x < lo) ? lo : (x > hi) ? hi : x;
  };
  SetBank(1);

  // Gyro notch filter (Bank 1)
  {
    uint8_t s2 = ReadReg(Reg::kGyroConfigStatic2);

    if (cfg.notch.enabled && cfg.notch.freq_hz > 0.0f) {
      // Datasheet: notch supported 1kHz..3kHz.
      float f_hz = clampf(cfg.notch.freq_hz, 1000.0f, 3000.0f);

      // CLKDIV lives in Bank 3.
      SetBank(3);
      uint8_t clkdiv = ReadReg(Reg::kClkdiv);
      SetBank(1);

      if (clkdiv == 0) {
        // Invalid clock divider: leave notch disabled.
        s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
        WriteReg(Reg::kGyroConfigStatic2, s2);
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

        s2 &= ~GYRO_CONFIG_STATIC2_NF_DIS;
        WriteReg(Reg::kGyroConfigStatic2, s2);

        // Same notch coefficient for all three axes.
        WriteReg(Reg::kGyroConfigStatic6, (uint8_t)(val & 0xFF));
        WriteReg(Reg::kGyroConfigStatic7, (uint8_t)(val & 0xFF));
        WriteReg(Reg::kGyroConfigStatic8, (uint8_t)(val & 0xFF));

        uint8_t s9 = 0;
        if (sel) s9 |= (1u << 5) | (1u << 4) | (1u << 3);
        if (val & 0x100) s9 |= (1u << 2) | (1u << 1) | (1u << 0);
        WriteReg(Reg::kGyroConfigStatic9, s9);

        uint8_t s10 = ReadReg(Reg::kGyroConfigStatic10);
        s10 &= ~(0x7u << 4);
        uint8_t bw = (cfg.notch.bw_idx > 7) ? 7 : cfg.notch.bw_idx;
        s10 |= (uint8_t)(bw << 4);
        WriteReg(Reg::kGyroConfigStatic10, s10);
      }
    } else {
      s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
      WriteReg(Reg::kGyroConfigStatic2, s2);
    }
  }

  // Gyro anti-alias filter (Bank 1)
  {
    uint8_t s2 = ReadReg(Reg::kGyroConfigStatic2);
    if (cfg.gyro_aaf.dis) {
      s2 |= GYRO_CONFIG_STATIC2_AAF_DIS;
      WriteReg(Reg::kGyroConfigStatic2, s2);
    } else {
      s2 &= ~GYRO_CONFIG_STATIC2_AAF_DIS;
      WriteReg(Reg::kGyroConfigStatic2, s2);

      WriteReg(Reg::kGyroConfigStatic3, (uint8_t)(cfg.gyro_aaf.delt & 0x3F));
      WriteReg(Reg::kGyroConfigStatic4,
               (uint8_t)(cfg.gyro_aaf.delt_sqr & 0xFF));

      uint8_t s5 = (uint8_t)((cfg.gyro_aaf.bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.gyro_aaf.delt_sqr >> 8) & 0xF);
      WriteReg(Reg::kGyroConfigStatic5, s5);
    }
  }

  // Accel anti-alias filter (Bank 2)
  SetBank(2);

  {
    uint8_t s2 = ReadReg(Reg::kAccelConfigStatic2);
    if (cfg.accel_aaf.dis) {
      s2 |= ACCEL_CONFIG_STATIC2_AAF_DIS;
      WriteReg(Reg::kAccelConfigStatic2, s2);
    } else {
      s2 &= ~ACCEL_CONFIG_STATIC2_AAF_DIS;
      s2 &= ~(0x3Fu << 1);
      s2 |= (uint8_t)((cfg.accel_aaf.delt & 0x3F) << 1);
      WriteReg(Reg::kAccelConfigStatic2, s2);

      WriteReg(Reg::kAccelConfigStatic3,
               (uint8_t)(cfg.accel_aaf.delt_sqr & 0xFF));

      uint8_t s4 = (uint8_t)((cfg.accel_aaf.bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.accel_aaf.delt_sqr >> 8) & 0xF);
      WriteReg(Reg::kAccelConfigStatic4, s4);
    }
  }

  // UI filter bandwidths (Bank 0).
  SetBank(0);
  WriteReg(Reg::kGyroAccelConfig0,
           static_cast<uint8_t>(((cfg.ui_filter.accel_bw & 0x0Fu) << 4) |
                                (cfg.ui_filter.gyro_bw & 0x0Fu)));
  WriteReg(Reg::kGyroConfig1,
           static_cast<uint8_t>(cfg.ui_filter.gyro_cfg1 & 0xEFu));
  WriteReg(Reg::kAccelConfig1,
           static_cast<uint8_t>(cfg.ui_filter.accel_cfg1 & 0x1Eu));
}

// ISR path

void Icm42688p::OnIrq() {
  irq_cnt_.fetch_add(1, std::memory_order_relaxed);
  last_irq_us_ = System::GetInstance().Time().Micros();

  if (inflight_.exchange(true, std::memory_order_acq_rel)) {
    HandleOverrunFault();
    return;
  }
  const uint16_t transfer_len =
      static_cast<uint16_t>(1u + fifo_wm_records_ * packet_bytes_);
  last_count_ = fifo_wm_records_;

  auto &spi = *spi_;
  CsLow();
  if (!spi.StartTxRxDma(fifo_tx_, fifo_rx_, transfer_len,
                        &Icm42688p::SpiDoneThunk, this)) {
    CsHigh();
    inflight_.store(false, std::memory_order_release);
    dma_start_fail_cnt_.fetch_add(1, std::memory_order_relaxed);
    HandleOverrunFault();
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
    RecoverFromFifoFault();
    HandleOverrunFault();
    finish();
    return;
  }

  if (inject_overrun_fault_for_test_.exchange(false,
                                              std::memory_order_acq_rel)) {
    RecoverFromFifoFault();
    HandleOverrunFault();
    finish();
    return;
  }

  const uint8_t *p = &fifo_rx_[1];

  SampleBatch batch{};
  batch.count = 0;

  for (uint16_t i = 0; i < fifo_wm_records_; ++i) {
    const uint8_t *rec = p + static_cast<uint32_t>(i) * packet_bytes_;
    Sample s{};
    bool ok_rec;
    if (hires_) {
      ok_rec = ParsePacket4Record(rec, s);
    } else {
      ok_rec = ParsePacket3Record(rec, s);
    }
    if (!ok_rec) {
      last_bad_header_.store(rec[0], std::memory_order_relaxed);
      parse_fail_cnt_.fetch_add(1, std::memory_order_relaxed);
      RecoverFromFifoFault();
      HandleOverrunFault();
      finish();
      return;
    }

    s.seq = ++seq_;
    batch.samples[batch.count++] = s;
  }

  PublishLatestBatch(batch);

  finish();
}

void Icm42688p::RecoverFromFifoFault() {
  SetBank(0);
  WriteReg(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(Reg::kIntStatus);

  tmst_inited_ = false;
  host_sync_inited_ = false;
  last_tmst16_ = 0;
  tmst64_us_ = 0;
  timestamp_tick_remainder_q16_ = 0;
  host_offset_us_ = 0;
  last_count_ = 0;
}

void Icm42688p::HandleOverrunFault() {
  overrun_.fetch_add(1, std::memory_order_relaxed);

  const uint64_t now_us = System::GetInstance().Time().Micros();
  const uint64_t window_us = SecondsToMicros(recovery_cfg_.overrun_window_s);

  if (last_overrun_fault_us_ == 0 ||
      (now_us - last_overrun_fault_us_) > window_us) {
    overrun_window_count_ = 1;
  } else {
    overrun_window_count_++;
  }

  last_overrun_fault_us_ = now_us;

  if (overrun_window_count_ >= recovery_cfg_.overrun_threshold) {
    Panic(ErrorCode::Stm32::kImuOverrun);
  }
}

void Icm42688p::InjectOverrunFaultForTest() {
  // One-shot: fault is injected from the SPI completion path so recovery
  // runs in the same context as a real transport fault.
  inject_overrun_fault_for_test_.store(true, std::memory_order_release);
}

Icm42688p::ScaledSample Icm42688p::ScaleSample(const Sample &sample) const {
  // HiRes locks the chip to ±2000 dps + ±16g (datasheet §6.1) with fixed
  // sensitivity; the FS config is ignored, so bypass the FS scale helpers.
  float accel_scale;
  float gyro_scale;
  if (hires_) {
    accel_scale = Icm42688pReg::AccelLsbToMps2_HiRes();
    gyro_scale = Icm42688pReg::GyroLsbToRadS_HiRes();
  } else {
    accel_scale = Icm42688pReg::AccelLsbToMps2(accel_fs_);
    gyro_scale = Icm42688pReg::GyroLsbToRadS(gyro_fs_);
  }

  ScaledSample out{};
  out.timestamp_us = sample.timestamp_us;
  if (hires_) {
    out.temperature_c = Icm42688pReg::FifoTemperatureC16(sample.temp_raw);
  } else {
    out.temperature_c = Icm42688pReg::FifoTemperatureC(
        static_cast<int8_t>(sample.temp_raw & 0xFF));
  }
  out.seq = sample.seq;

  // Chip-frame -> body-NED via per-board axis_map_ (pick + optional sign).
  // Default = identity (chip XYZ = body NED).
  const uint8_t src_x[3] = {axis_map_.x_from, axis_map_.y_from,
                            axis_map_.z_from};
  const bool neg[3] = {axis_map_.x_neg, axis_map_.y_neg, axis_map_.z_neg};
  for (uint8_t body = 0; body < 3u; ++body) {
    const uint8_t chip = src_x[body];
    const float sign = neg[body] ? -1.0f : 1.0f;
    out.accel_mps2[body] =
        sign * static_cast<float>(sample.accel[chip]) * accel_scale;
    out.gyro_rad_s[body] =
        sign * static_cast<float>(sample.gyro[chip]) * gyro_scale;
  }

  return out;
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
    const uint64_t scaled_dt_q16 =
        static_cast<uint64_t>(dt16) * timestamp_tick_scale_q16_ +
        timestamp_tick_remainder_q16_;
    tmst64_us_ += scaled_dt_q16 >> 16;
    timestamp_tick_remainder_q16_ =
        static_cast<uint32_t>(scaled_dt_q16 & (kTimestampScaleQ16 - 1u));
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
  if (!rec) return false;

  // Header check for Packet3 accel+gyro+temp+timestamp, 16-bit mode.
  // Accept both normal packets (0x68) and ODR-change-tagged variants (0x6C).
  const uint8_t hdr = rec[0];
  if ((hdr & 0xF8u) != 0x68u) {
    return false;
  }

  // Packet3 temperature byte is at 0x0D and timestamp stays at 0x0E..0x0F.
  // The byte is two's complement and temp_raw is int16_t to also hold Packet4's
  // 16-bit form, so widening it must sign-extend.
  // NOLINTNEXTLINE(bugprone-signed-char-misuse)
  out.temp_raw = static_cast<int8_t>(rec[0x0D]);
  const uint16_t ts16 = LoadBe16(&rec[0x0E]);

  uint64_t host_ts_us = 0;
  UpdateTimestampAndSync(ts16, host_ts_us);

  out.timestamp_us = host_ts_us;

  // Packet layout (big-endian)
  out.accel[0] = LoadBe16Signed(&rec[0x01]);
  out.accel[1] = LoadBe16Signed(&rec[0x03]);
  out.accel[2] = LoadBe16Signed(&rec[0x05]);

  out.gyro[0] = LoadBe16Signed(&rec[0x07]);
  out.gyro[1] = LoadBe16Signed(&rec[0x09]);
  out.gyro[2] = LoadBe16Signed(&rec[0x0B]);

  // INTF_CONFIG0.FIFO_HOLD_LAST_DATA_EN=0 inserts invalid code (-32768).
  // Treat any detected invalid code as a hard fault.
  if (!fifo_hold_last_data_en_) {
    static constexpr int32_t kInvalid16 = Icm42688pReg::kFifo16InvalidSentinel;
    if (out.accel[0] == kInvalid16 || out.accel[1] == kInvalid16 ||
        out.accel[2] == kInvalid16 || out.gyro[0] == kInvalid16 ||
        out.gyro[1] == kInvalid16 || out.gyro[2] == kInvalid16) {
      Panic(ErrorCode::Stm32::kImuInvalidSampleDetected);
      return false;
    }
  }

  return true;
}

bool Icm42688p::ParsePacket4Record(const uint8_t *rec, Sample &out) {
  if (!rec) return false;

  // Header check: Packet4 = 0x78 with mask 0xF8 (top 5 bits define the
  // variant; bits[1:0] are ODR-change flags, ignored). Also accepts the
  // FSYNC-tagged variant 0x7C — same packet shape, different time-tag.
  const uint8_t hdr = rec[0];
  if ((hdr & Icm42688pReg::kFifoHeaderVariantMask) !=
      Icm42688pReg::kFifoHeaderPacket4) {
    return false;
  }

  // Reassemble split 20-bit signed value: high 16 bits + low 4. Sign extends
  // naturally — int16 `high` carries bit 19, and <<4 in int32 preserves sign.
  auto pack20 = [](int16_t high, uint8_t low4) -> int32_t {
    return (static_cast<int32_t>(high) << 4) |
           static_cast<int32_t>(low4 & 0x0Fu);
  };

  // Packet4 layout (datasheet §6.1, Figure 10 + Packet 4 table):
  //   0x00       FIFO header
  //   0x01-0x02  Accel X [19:12], [11:4]   (high 16 bits, big-endian)
  //   0x03-0x04  Accel Y high 16
  //   0x05-0x06  Accel Z high 16
  //   0x07-0x08  Gyro  X high 16
  //   0x09-0x0A  Gyro  Y high 16
  //   0x0B-0x0C  Gyro  Z high 16
  //   0x0D-0x0E  Temperature [15:8], [7:0]   (16-bit, signed)
  //   0x0F-0x10  Timestamp [15:8], [7:0]
  //   0x11       Accel X [3:0] (upper nibble) | Gyro X [3:0] (lower nibble)
  //   0x12       Accel Y [3:0] | Gyro Y [3:0]
  //   0x13       Accel Z [3:0] | Gyro Z [3:0]
  const int16_t temp16 = LoadBe16Signed(&rec[0x0D]);
  const uint16_t ts16 = LoadBe16(&rec[0x0F]);

  uint64_t host_ts_us = 0;
  UpdateTimestampAndSync(ts16, host_ts_us);
  out.timestamp_us = host_ts_us;
  out.temp_raw = temp16;

  const uint8_t b11 = rec[0x11];
  const uint8_t b12 = rec[0x12];
  const uint8_t b13 = rec[0x13];

  out.accel[0] = pack20(LoadBe16Signed(&rec[0x01]),
                        static_cast<uint8_t>((b11 >> 4) & 0x0Fu));
  out.accel[1] = pack20(LoadBe16Signed(&rec[0x03]),
                        static_cast<uint8_t>((b12 >> 4) & 0x0Fu));
  out.accel[2] = pack20(LoadBe16Signed(&rec[0x05]),
                        static_cast<uint8_t>((b13 >> 4) & 0x0Fu));
  out.gyro[0] =
      pack20(LoadBe16Signed(&rec[0x07]), static_cast<uint8_t>(b11 & 0x0Fu));
  out.gyro[1] =
      pack20(LoadBe16Signed(&rec[0x09]), static_cast<uint8_t>(b12 & 0x0Fu));
  out.gyro[2] =
      pack20(LoadBe16Signed(&rec[0x0B]), static_cast<uint8_t>(b13 & 0x0Fu));

  // Invalid-sample sentinel (datasheet §12.8) — chip writes -524288
  // when FIFO_HOLD_LAST_DATA_EN=0 and no fresh data is available.
  // Mirror of the 16-bit -32768 sentinel handled in ParsePacket3Record.
  if (!fifo_hold_last_data_en_) {
    static constexpr int32_t kInvalid20 = Icm42688pReg::kFifo20InvalidSentinel;
    if (out.accel[0] == kInvalid20 || out.accel[1] == kInvalid20 ||
        out.accel[2] == kInvalid20 || out.gyro[0] == kInvalid20 ||
        out.gyro[1] == kInvalid20 || out.gyro[2] == kInvalid20) {
      Panic(ErrorCode::Stm32::kImuInvalidSampleDetected);
      return false;
    }
  }

  return true;
}

void Icm42688p::PublishLatestBatch(const SampleBatch &batch) {
  published_batch_ = batch;

  // Publish payload then publish sequence (release). ValidateConfig rejects a
  // zero watermark, so the batch always holds at least one sample; the
  // analyzer cannot carry that across the call.
  // NOLINTNEXTLINE(clang-analyzer-security.ArrayBound)
  tick_seq_.store(batch.samples[batch.count - 1u].seq,
                  std::memory_order_release);
  publish_cnt_.fetch_add(1, std::memory_order_relaxed);

  // Defer fast-loop consumption out of IRQ/DMA context.
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

bool Icm42688p::WaitAndGetLatestBatch(uint32_t &last_seq, SampleBatch &out) {
  uint32_t s = tick_seq_.load(std::memory_order_acquire);
  if (s == last_seq) {
    return false;
  }

  for (;;) {
    out = published_batch_;
    const uint32_t s2 = tick_seq_.load(std::memory_order_acquire);

    if (s2 == s && out.count > 0 && out.samples[out.count - 1u].seq == s) {
      if (last_seq != 0) {
        const uint32_t missed = (uint32_t)(out.samples[0].seq - last_seq - 1u);
        if (missed) {
          drop_cnt_.fetch_add(missed, std::memory_order_relaxed);
        }
      }
      last_seq = s;
      return true;
    }

    s = s2;
    if (s == last_seq) {
      return false;
    }
  }
}

Icm42688p::SampleBatch Icm42688p::GetLatestBatch() const {
  SampleBatch out{};
  uint32_t seq = tick_seq_.load(std::memory_order_acquire);
  if (seq == 0) {
    return out;
  }

  for (;;) {
    out = published_batch_;
    const uint32_t seq2 = tick_seq_.load(std::memory_order_acquire);

    if (seq2 == seq && out.count > 0 &&
        out.samples[out.count - 1u].seq == seq) {
      return out;
    }

    if (seq2 == 0) {
      return {};
    }

    seq = seq2;
  }
}

void Icm42688p::CalibrateGyro() {
  const uint32_t duration_us =
      SecondsToMicros(calibration_cfg_.gyro_duration_s);
  const uint32_t timeout_us =
      SecondsToMicros(calibration_cfg_.gyro_timeout_s);
  const uint32_t still_threshold_raw =
      calibration_cfg_.gyro_still_threshold_raw;
  const uint32_t odr_hz = gyro_odr_hz_;
  const uint64_t sample_count_u64 =
      ((uint64_t)duration_us * odr_hz + SecondsToMicros(1) - 1ULL) /
      SecondsToMicros(1);

  const uint32_t sample_count =
      sample_count_u64 == 0 ? 1u : static_cast<uint32_t>(sample_count_u64);

  auto &time = System::GetInstance().Time();
  const uint64_t start_us = time.Micros();
  uint32_t last_seq = LatestSeq();
  int64_t sum[3] = {0, 0, 0};
  int16_t min_g[3] = {0, 0, 0};
  int16_t max_g[3] = {0, 0, 0};
  uint32_t collected = 0;

  while (collected < sample_count) {
    SampleBatch batch{};
    if (!WaitAndGetLatestBatch(last_seq, batch)) {
      if ((uint32_t)(time.Micros() - start_us) >= timeout_us) {
        return;
      }
      continue;
    }

    for (uint8_t i = 0; i < batch.count && collected < sample_count; ++i) {
      const Sample &s = batch.samples[i];
      for (int axis = 0; axis < 3; ++axis) {
        if (collected == 0) {
          min_g[axis] = s.gyro[axis];
          max_g[axis] = s.gyro[axis];
        } else {
          if (s.gyro[axis] < min_g[axis]) {
            min_g[axis] = s.gyro[axis];
          }
          if (s.gyro[axis] > max_g[axis]) {
            max_g[axis] = s.gyro[axis];
          }
        }
        if (static_cast<uint32_t>(max_g[axis] - min_g[axis]) >
            still_threshold_raw) {
          Panic(ErrorCode::Stm32::kImuCalibrationMotionDetected);
        }
        sum[axis] += s.gyro[axis];
      }
      collected++;
    }
  }

  int16_t offset_lsb[3] = {0, 0, 0};
  // HiRes: fixed 131 LSB/dps. Packet3: FS-parameterised (range/32768).
  float dps_per_lsb;
  if (hires_) {
    dps_per_lsb = 1.0f / Icm42688pReg::kHiResGyroLsbPerDps;
  } else {
    dps_per_lsb =
        Icm42688pReg::GyroRangeDps(gyro_fs_) / Icm42688pReg::kFifoDataLsb;
  }
  for (int axis = 0; axis < 3; ++axis) {
    const int32_t bias_raw =
        static_cast<int32_t>(sum[axis] / static_cast<int64_t>(collected));
    const float bias_dps = static_cast<float>(bias_raw) * dps_per_lsb;
    int32_t code = static_cast<int32_t>(lrintf(-bias_dps * 32.0f));
    if (code < -2048) {
      code = -2048;
    } else if (code > 2047) {
      code = 2047;
    }
    offset_lsb[axis] = static_cast<int16_t>(code);
  }

  NVIC_DisableIRQ(board::kImuInt.exti_irqn);
  NVIC_ClearPendingIRQ(board::kImuInt.exti_irqn);
  while (inflight_.load(std::memory_order_acquire)) {
  }

  SetBank(0);
  const uint8_t prev_pwr = ReadReg(Reg::kPwrMgmt0);
  WriteReg(Reg::kPwrMgmt0, 0x00u);
  time.DelayMicros(200);

  WriteGyroUserOffsets(offset_lsb[0], offset_lsb[1], offset_lsb[2]);

  tmst_inited_ = false;
  host_sync_inited_ = false;
  last_tmst16_ = 0;
  tmst64_us_ = 0;
  timestamp_tick_remainder_q16_ = 0;
  host_offset_us_ = 0;

  WriteReg(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(Reg::kIntStatus);
  WriteReg(Reg::kPwrMgmt0, prev_pwr);
  time.DelayMicros(200);
  time.DelayMicros(MillisToMicros(50));
  WriteReg(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(Reg::kIntStatus);
  NVIC_EnableIRQ(board::kImuInt.exti_irqn);
}

void Icm42688p::WriteGyroUserOffsets(int16_t x_offset_lsb, int16_t y_offset_lsb,
                                     int16_t z_offset_lsb) {
  auto pack12 = [](int16_t offset_lsb) -> uint16_t {
    return static_cast<uint16_t>(offset_lsb) & 0x0FFFu;
  };

  const uint16_t x = pack12(x_offset_lsb);
  const uint16_t y = pack12(y_offset_lsb);
  const uint16_t z = pack12(z_offset_lsb);

  SetBank(0);
  WriteReg(Reg::kOffsetUser0, static_cast<uint8_t>(x & 0xFFu));
  WriteReg(Reg::kOffsetUser1, static_cast<uint8_t>(((y >> 8) & 0x0Fu) << 4) |
                                  static_cast<uint8_t>((x >> 8) & 0x0Fu));
  WriteReg(Reg::kOffsetUser2, static_cast<uint8_t>(y & 0xFFu));
  WriteReg(Reg::kOffsetUser3, static_cast<uint8_t>(z & 0xFFu));

  const uint8_t user4 = ReadReg(Reg::kOffsetUser4);
  WriteReg(Reg::kOffsetUser4,
           static_cast<uint8_t>((user4 & 0xF0u) | ((z >> 8) & 0x0Fu)));
}

void Icm42688p::CheckWhoAmI() {
  auto &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();

  SetBank(0);
  while ((uint32_t)(time.Micros() - start) < MillisToMicros(1000)) {
    const uint8_t id = ReadReg(Reg::kWhoAmI);
    if (id == WHO_AM_I_ICM42688P || id == WHO_AM_I_ICM42686P) {
      who_am_i_ = id;
      return;
    }
    time.DelayMicros(MillisToMicros(10));
  }

  Panic(ErrorCode::Stm32::kImuWhoAmIFail);
}

void Icm42688p::SoftReset() {
  SetBank(0);

  uint8_t v = ReadReg(Reg::kDeviceConfig);
  v |= 0x01u;  // SOFT_RESET_CONFIG = 1, preserve SPI_MODE + reserved
  WriteReg(Reg::kDeviceConfig, v);

  System::GetInstance().Time().DelayMicros(MillisToMicros(1));
  SetBank(0);
}

uint32_t Icm42688p::EffectiveOdrHz(
    Icm42688pReg::Odr odr,
    const typename Config::ExternalClock &external_clock) {
  const uint32_t odr_hz = Icm42688pReg::OdrHz(odr);
  if (!external_clock.enabled) {
    return odr_hz;
  }

  return static_cast<uint32_t>(
      (static_cast<uint64_t>(odr_hz) * external_clock.frequency_hz +
       kNominalOdrReferenceHz / 2u) /
      kNominalOdrReferenceHz);
}

uint32_t Icm42688p::TimestampTickScaleQ16(
    const typename Config::ExternalClock &external_clock) {
  if (!external_clock.enabled) {
    return kTimestampScaleQ16;
  }

  return static_cast<uint32_t>(
      ((static_cast<uint64_t>(kNominalTimestampReferenceHz) << 16u) +
       external_clock.frequency_hz / 2u) /
      external_clock.frequency_hz);
}

void Icm42688p::SetClockSource(const Config &cfg) {
  SetBank(1);

  uint8_t pin_cfg = ReadReg(Reg::kIntfConfig5);
  pin_cfg &= static_cast<uint8_t>(~INTF_CONFIG5_PIN9_FUNCTION_MASK);
  pin_cfg |= cfg.external_clock.enabled ? INTF_CONFIG5_PIN9_FUNCTION_CLKIN
                                        : INTF_CONFIG5_PIN9_FUNCTION_FSYNC;
  WriteReg(Reg::kIntfConfig5, pin_cfg);

  SetBank(0);

  uint8_t v = ReadReg(Reg::kIntfConfig1);
  // CLKSEL[1:0]=01 (PLL when available); RTC_MODE bit2 needs pin9 CLKIN for
  // external clocking. Preserve reserved [7:4] and ACCEL_LP_CLK_SEL bit3.
  v &= static_cast<uint8_t>(~(Icm42688pReg::INTF_CONFIG1_CLKSEL_MASK |
                              Icm42688pReg::INTF_CONFIG1_RTC_MODE_MASK));
  v |= Icm42688pReg::INTF_CONFIG1_CLKSEL_PLL;
  if (cfg.external_clock.enabled) {
    v |= Icm42688pReg::INTF_CONFIG1_RTC_MODE_EN;
  }
  WriteReg(Reg::kIntfConfig1, v);
}

void Icm42688p::SetInterfaceConfig(const Config &cfg) {
  SetBank(0);

  uint8_t v = ReadReg(Reg::kIntfConfig0);

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
  WriteReg(Reg::kIntfConfig0, v);
}

void Icm42688p::SetInterruptConfig() {
  SetBank(0);
  uint8_t v = ReadReg(Reg::kIntConfig);
  // Preserve reserved bits 7:6, rewrite bits 5:0
  // INT2: pulsed(0), open-drain(0), active-low(0) => bits [5:3] = 000
  // INT1: pulsed(0), push-pull(1), active-high(1) => bits [2:0] = 0b011
  v = static_cast<uint8_t>((v & 0xC0u) | 0x03u);
  WriteReg(Reg::kIntConfig, v);
  // ODR >= 4kHz: 8us pulse + deassert disabled.
  // Datasheet note: set INT_ASYNC_RESET=0 for proper INT operation.
  // Preserve reserved bits: bit7 and bits3:0
  // Set bits6:4 = 0b110 (8us pulse, disable deassert duration, async_reset=0)
  v = ReadReg(Reg::kIntConfig1);
  v = static_cast<uint8_t>((v & 0x8Fu) | 0x60u);
  WriteReg(Reg::kIntConfig1, v);
}

void Icm42688p::DisableFsync() {
  SetBank(0);

  uint8_t v = ReadReg(Reg::kFsyncConfig);

  // Preserve reserved bits: bit7 and bits3:2
  // Clear only: FSYNC_UI_SEL (6:4), FSYNC_UI_FLAG_CLEAR_SEL (1),
  // FSYNC_POLARITY (0)
  v &= static_cast<uint8_t>(~0x73u);  // ~0b01110011
  WriteReg(Reg::kFsyncConfig, v);
}

void Icm42688p::SetOdrAndFullScale(const Config &cfg) {
  SetBank(0);
  // GYRO_CONFIG0: [7:5]=FS (3b), bit4 reserved, [3:0]=ODR (4b)
  const uint8_t gyro =
      static_cast<uint8_t>(((static_cast<uint8_t>(cfg.fs.gyro) & 0x07u) << 5) |
                           (static_cast<uint8_t>(cfg.rates.gyro) & 0x0Fu));
  // ACCEL_CONFIG0: [7:5]=FS (only 0..3 valid), bit4 reserved, [3:0]=ODR (4b)
  const uint8_t accel =
      static_cast<uint8_t>(((static_cast<uint8_t>(cfg.fs.accel) & 0x03u) << 5) |
                           (static_cast<uint8_t>(cfg.rates.accel) & 0x0Fu));
  WriteReg(Reg::kGyroConfig0, gyro);
  WriteReg(Reg::kAccelConfig0, accel);
}

void Icm42688p::SetTimestampConfig() {
  SetBank(0);

  uint8_t v = ReadReg(Reg::kTmstConfig);

  // Preserve reserved [7:5], clear [4:1], set TMST_EN (bit0).
  v &= 0xE0u;
  v |= 0x01u;

  WriteReg(Reg::kTmstConfig, v);
}

void Icm42688p::ClearUserOffsets() {
  // Rebuilding a Reg from a raw counter is the one path an unlisted address
  // can take to WriteReg; a reordered enum would clear nine wrong registers.
  static_assert(std::to_underlying(Reg::kOffsetUser8) -
                        std::to_underlying(Reg::kOffsetUser0) ==
                    8,
                "user-offset registers must stay contiguous");

  SetBank(0);
  for (uint8_t reg = std::to_underlying(Reg::kOffsetUser0);
       reg <= std::to_underlying(Reg::kOffsetUser8); ++reg) {
    WriteReg(static_cast<Reg>(reg), 0x00u);
  }
}

void Icm42688p::ConfigureFifo() {
  SetBank(0);
  uint8_t v = ReadReg(Reg::kIntConfig0);  // Preserve reserved [7:6], replace
                                          // [5:0]
  v = static_cast<uint8_t>((v & 0xC0u) | 0x0Au);
  WriteReg(Reg::kIntConfig0, v);
  v = ReadReg(Reg::kIntSource0);  // Preserve reserved bit7, clear bits6:0
  v = static_cast<uint8_t>((v & 0x80u) | 0x04u);  // FIFO_THS -> INT1 only
  WriteReg(Reg::kIntSource0, v);  // Packet3, 16-bit, timestamp, DMA-safe
  v = ReadReg(Reg::kFifoConfig);
  v &= 0x3F;  // clear bits 7:6 → FIFO_MODE = 00 (bypass)
  WriteReg(Reg::kFifoConfig, v);
  uint8_t fifo_cfg1 = FIFO_CONFIG1_RESUME_PARTIAL_RD |
                      FIFO_CONFIG1_TMST_FSYNC_EN | FIFO_CONFIG1_TEMP_EN |
                      FIFO_CONFIG1_GYRO_EN | FIFO_CONFIG1_ACCEL_EN;
  if (hires_) {
    fifo_cfg1 |= FIFO_CONFIG1_HIRES_EN;  // switches to Packet4
  }
  WriteReg(Reg::kFifoConfig1, fifo_cfg1);
  const uint16_t fifo_wm_bytes =
      static_cast<uint16_t>(fifo_wm_records_ * packet_bytes_);
  WriteReg(Reg::kFifoConfig2, static_cast<uint8_t>(fifo_wm_bytes & 0xFFu));
  WriteReg(Reg::kFifoConfig3,
           static_cast<uint8_t>((fifo_wm_bytes >> 8) & 0x0Fu));
  WriteReg(Reg::kSignalPathReset, 0x02);  // FIFO_FLUSH
  (void)ReadReg(Reg::kIntStatus);         // clear any pending status bits (R/C)
  WriteReg(Reg::kFifoConfig, 0x40);       // FIFO Stream mode
  SetupDmaBuffer();
}

void Icm42688p::SetupDmaBuffer() {
  // Fixed-burst FIFO reads: read exactly watermark_records packets per IRQ.
  for (size_t i = 0; i < sizeof(fifo_tx_); i++) {
    fifo_tx_[i] = 0xFFu;
  }
  fifo_tx_[0] =
      static_cast<uint8_t>(std::to_underlying(Reg::kFifoData) | 0x80u);
}
// SPI helpers (blocking)

void Icm42688p::SetBank(uint8_t bank) {
  WriteReg(Reg::kBankSel, (uint8_t)(bank & 0x07));
  System::GetInstance().Time().DelayMicros(1);
}

void Icm42688p::WriteReg(Reg reg, uint8_t val) {
  auto &spi = *spi_;
  uint8_t tx[2] = {static_cast<uint8_t>(std::to_underlying(reg) & 0x7F), val};
  CsLow();
  spi.WriteBytes(tx);
  CsHigh();
}
uint8_t Icm42688p::ReadReg(Reg reg) {
  auto &spi = *spi_;
  uint8_t tx[2] = {static_cast<uint8_t>(std::to_underlying(reg) | 0x80), 0x00};
  uint8_t rx[2] = {0, 0};
  CsLow();
  spi.TxRx(tx, rx, 2);
  CsHigh();
  return rx[1];
}
void Icm42688p::CsLow() {
  gpio_->WritePin(board::kSpi2Cs.port, board::kSpi2Cs.pin, false);
}
void Icm42688p::CsHigh() {
  gpio_->WritePin(board::kSpi2Cs.port, board::kSpi2Cs.pin, true);
}

// Explicit instantiation
void Icm42688p::SuspendSampling() { NVIC_DisableIRQ(board::kImuInt.exti_irqn); }

void Icm42688p::ResumeSampling() {
  // The chip never stopped sampling into its own FIFO, so what is in there is
  // a backlog rather than the present. Discarded instead of parsed: a stale
  // burst would be integrated by the AHRS as if it had just happened.
  WriteReg(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(Reg::kIntStatus);
  NVIC_EnableIRQ(board::kImuInt.exti_irqn);
}
