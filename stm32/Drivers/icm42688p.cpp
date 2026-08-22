// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "icm42688p.hpp"

#include <cmath>
#include <utility>

#include "ee_config_storage.hpp"
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

void Icm42688p::Init(GPIO &gpio, Spi2 &spi, EE &ee, const Config &cfg,
                     SharedState &blackboard) {
  if (device_id_ != 0u) {
    Panic(ErrorCode::Stm32::kImuReinit);
  }

  gpio_ = &gpio;

  blackboard_ = &blackboard;
  spi_ = &spi;
  ee_ = &ee;
  fifo_wm_records_ = cfg.fifo.watermark_records;
  fifo_hold_last_data_en_ = cfg.fifo.hold_last;
  last_irq_cnt_ = System::GetInstance().Time().Micros();
  // Seeded from the counter, not from zero: this is the unwrapped host clock,
  // and the log builds its own timeline from the same TIM2 count. Starting at
  // zero would stamp every IMU record a whole boot behind the rest of the file.
  last_irq_us_ = last_irq_cnt_;
  hires_ = cfg.fifo.hires;
  packet_bytes_ =
      hires_ ? Icm42688pReg::kPacket4Bytes : Icm42688pReg::kPacket3Bytes;
  scale_config_ = ScaleConfig{
      .accel_lsb_to_mps2 = hires_ ? Icm42688pReg::AccelLsbToMps2_HiRes()
                                  : Icm42688pReg::AccelLsbToMps2(cfg.fs.accel),
      .gyro_lsb_to_rad_s = hires_ ? Icm42688pReg::GyroLsbToRadS_HiRes()
                                  : Icm42688pReg::GyroLsbToRadS(cfg.fs.gyro),
      .src = {cfg.axis_map.x_from, cfg.axis_map.y_from, cfg.axis_map.z_from},
      .neg = {cfg.axis_map.x_neg, cfg.axis_map.y_neg, cfg.axis_map.z_neg},
  };
  gyro_odr_hz_ = EffectiveOdrHz(cfg.rates.gyro, cfg.external_clock);
  timestamp_tick_scale_q16_ = TimestampTickScaleQ16(cfg.external_clock);
  timestamp_tick_remainder_q16_ = 0;
  accel_calibration_ = EeConfigStorage::LoadOrInitImuAccelCalibration(ee);

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
  // Configured, not running: ResumeSampling arms this once a consumer exists.
}

bool Icm42688p::SaveAccelCalibration() {
  if (ee_ == nullptr) {
    return false;
  }
  return EeConfigStorage::SaveImuAccelCalibration(*ee_, accel_calibration_);
}

uint32_t Icm42688p::GetDeviceId() const {
  if (device_id_ == 0u) {
    Panic(ErrorCode::Stm32::kImuNotInitialized);
  }

  return device_id_;
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
  const uint32_t now_cnt = System::GetInstance().Time().Micros();
  last_irq_us_ += static_cast<uint32_t>(now_cnt - last_irq_cnt_);
  last_irq_cnt_ = now_cnt;

  if (inflight_.exchange(true, std::memory_order_acq_rel)) {
    // These records stay in the FIFO, and a burst reads a fixed count rather
    // than FIFO_COUNT, so the level never returns below the watermark. With
    // FIFO_CONFIG1.FIFO_WM_GT_TH clear the watermark interrupt fires once per
    // upward crossing ("Interrupt only fires once", FIFO_CONFIG2), so without
    // a flush this one missed burst ends the stream for good. The bus belongs
    // to the burst in flight, so defer it.
    resync_pending_.store(true, std::memory_order_release);
    overrun_cnt_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const uint16_t transfer_len =
      static_cast<uint16_t>(1u + (fifo_wm_records_ * packet_bytes_));

  auto &spi = *spi_;
  CsLow();
  if (!spi.StartTxRxDma(fifo_tx_, fifo_rx_, transfer_len,
                        &Icm42688p::SpiDoneThunk)) {
    CsHigh();
    inflight_.store(false, std::memory_order_release);
    dma_start_fail_cnt_.fetch_add(1, std::memory_order_relaxed);
  }
}

extern "C" void Icm42688pOnIrq() { Icm42688p::GetInstance().OnIrq(); }

void Icm42688p::SpiDoneThunk(bool ok) { GetInstance().OnSpiDone(ok); }

void Icm42688p::OnSpiDone(bool ok) {
  CsHigh();

  auto finish = [&]() {
    const uint32_t now_us = System::GetInstance().Time().Micros();
    PublishHealth(now_us);
    PublishTemperature(now_us);
    inflight_.store(false, std::memory_order_release);
  };

  if (!ok) {
    spi_error_cnt_.fetch_add(1, std::memory_order_relaxed);
    FlushAndResync();
    finish();
    return;
  }

  const uint8_t *p = &fifo_rx_[1];

  ImuBurst burst{};
  burst.device_id = device_id_;
  burst.gyro_scale = scale_config_.gyro_lsb_to_rad_s;
  burst.accel_scale = scale_config_.accel_lsb_to_mps2;

  // Outlives the loop so the newest sample's timestamp is still readable once
  // the offset servo below has run.
  Sample sample{};
  for (uint16_t i = 0; i < fifo_wm_records_; ++i) {
    const uint8_t *rec = p + (static_cast<uint32_t>(i) * packet_bytes_);
    const std::optional<Sample> parsed =
        hires_ ? ParsePacket4Record(rec) : ParsePacket3Record(rec);
    if (!parsed) {
      parse_fail_cnt_.fetch_add(1, std::memory_order_relaxed);
      dropped_records_.fetch_add(fifo_wm_records_, std::memory_order_relaxed);
      FlushAndResync();
      finish();
      return;
    }

    sample = parsed.value();

    // Kept raw: the decode happens once per publication, not once per sample.
    last_temp_raw_ = sample.temp_raw;
    have_temp_ = true;

    MapAxes(sample, burst.count, burst);
    ++burst.count;
  }

  // After the loop: tmst64_us_ now holds the newest sample, which is the one
  // whose arrival raised the interrupt last_irq_us_ was stamped from.
  SyncHostOffset();

  burst.timestamp_us = sample.timestamp_us;
  burst.dt_us = static_cast<float>(last_sample_dt_q16_) / 65536.0f;

  PublishBurst(burst);

  if (resync_pending_.exchange(false, std::memory_order_acq_rel)) {
    FlushAndResync();
  }

  finish();
}

void Icm42688p::FlushAndResync() {
  SetBank(0);

  // Records, not bytes -- SetInterfaceConfig sets FIFO_COUNT_REC and big-endian
  // counts. Read on the fault path only; the hot path deliberately transfers a
  // fixed length instead of consulting this.
  const uint32_t hi = ReadReg(Reg::kFifoCountH);
  const uint32_t lo = ReadReg(Reg::kFifoCountL);
  dropped_records_.fetch_add((hi << 8) | lo, std::memory_order_relaxed);

  WriteReg(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(Reg::kIntStatus);

  tmst_inited_ = false;
  host_sync_inited_ = false;
  last_tmst16_ = 0;
  tmst64_us_ = 0;
  timestamp_tick_remainder_q16_ = 0;
  host_offset_us_ = 0;
}

float Icm42688p::ScaleTemperature(int16_t temp_raw) const {
  // Packet4 carries 16 bits, Packet3 only the low 8 -- and temp_raw is int16_t
  // for both, so the narrow form has to be re-signed before it is converted.
  return hires_ ? Icm42688pReg::FifoTemperatureC16(temp_raw)
                : Icm42688pReg::FifoTemperatureC(
                      static_cast<int8_t>(temp_raw & 0xFF));
}

// A signed permutation, so it applies losslessly to the counts and the burst
// leaves this file in body-NED without ever having been scaled.
void Icm42688p::MapAxes(const Sample &sample, uint16_t slot,
                        ImuBurst &out) const {
  const ScaleConfig &map = scale_config_;
  for (uint8_t body = 0; body < 3u; ++body) {
    const int32_t sign = map.neg[body] ? -1 : 1;
    out.gyro[body][slot] = sign * sample.gyro[map.src[body]];
    out.accel[body][slot] = sign * sample.accel[map.src[body]];
  }
}

void Icm42688p::ChipFromBody(const float body[3], float chip[3]) const {
  // body[b] = sign[b] * chip[src[b]], so chip[src[b]] = sign[b] * body[b].
  // Exact only because the map is a signed permutation -- one chip axis per
  // body axis. A general rotation would need a transpose instead.
  const ScaleConfig &map = scale_config_;
  for (uint8_t b = 0; b < 3u; ++b) {
    chip[map.src[b]] = (map.neg[b] ? -1.0f : 1.0f) * body[b];
  }
}

uint64_t Icm42688p::UpdateTimestampAndSync(uint16_t ts16) {
  // Unwrap 16-bit timestamp (1us ticks) into tmst64_us_
  if (!tmst_inited_) {
    last_tmst16_ = ts16;
    tmst64_us_ = 0;
    tmst_inited_ = true;
  } else {
    const uint16_t dt16 = static_cast<uint16_t>(ts16 - last_tmst16_);
    last_tmst16_ = ts16;
    const uint64_t scaled_dt_q16 =
        (static_cast<uint64_t>(dt16) * timestamp_tick_scale_q16_) +
        timestamp_tick_remainder_q16_;
    tmst64_us_ += scaled_dt_q16 >> 16;
    // Kept in Q16 and converted once per burst: the log wants the fraction,
    // and a float per sample would land in the interrupt for nothing.
    last_sample_dt_q16_ = static_cast<uint32_t>(scaled_dt_q16);
    timestamp_tick_remainder_q16_ =
        static_cast<uint32_t>(scaled_dt_q16 & (kTimestampScaleQ16 - 1u));
  }

  const int64_t imu_us = static_cast<int64_t>(tmst64_us_);

  // Seeded here rather than in SyncHostOffset so the first burst is already in
  // the host domain: the servo runs after the parse, and samples stamped with a
  // zero offset would read as if the board had just booted.
  if (!host_sync_inited_) {
    host_offset_us_ = static_cast<int64_t>(last_irq_us_) - imu_us;
    host_sync_inited_ = true;
  }

  return static_cast<uint64_t>(imu_us + host_offset_us_);
}

// One correction per burst, not per record: last_irq_us_ moves once per burst,
// so a per-record call would feed the servo the same measurement once per
// sample -- all but one against a sample older than the interrupt, biasing the
// offset by half a burst and scaling the >> 7 gain by the watermark.
void Icm42688p::SyncHostOffset() {
  if (!host_sync_inited_) {
    return;
  }

  const int64_t host_us = static_cast<int64_t>(last_irq_us_);
  const int64_t imu_us = static_cast<int64_t>(tmst64_us_);
  const int64_t err = host_us - (imu_us + host_offset_us_);
  host_offset_us_ += (err >> 7);
}

std::optional<Icm42688p::Sample> Icm42688p::ParsePacket3Record(
    const uint8_t *rec) {
  if (!rec) return std::nullopt;
  Sample out{};

  // Header check for Packet3 accel+gyro+temp+timestamp, 16-bit mode.
  // Accept both normal packets (0x68) and ODR-change-tagged variants (0x6C).
  const uint8_t hdr = rec[0];
  if ((hdr & 0xF8u) != 0x68u) {
    last_bad_header_.store(hdr, std::memory_order_relaxed);
    return std::nullopt;
  }

  // Packet3 temperature byte is at 0x0D and timestamp stays at 0x0E..0x0F.
  // The byte is two's complement and temp_raw is int16_t to also hold Packet4's
  // 16-bit form, so widening it must sign-extend.
  // NOLINTNEXTLINE(bugprone-signed-char-misuse)
  out.temp_raw = static_cast<int8_t>(rec[0x0D]);
  const uint16_t ts16 = LoadBe16(&rec[0x0E]);

  out.timestamp_us = UpdateTimestampAndSync(ts16);

  // Packet layout (big-endian)
  out.accel[0] = LoadBe16Signed(&rec[0x01]);
  out.accel[1] = LoadBe16Signed(&rec[0x03]);
  out.accel[2] = LoadBe16Signed(&rec[0x05]);

  out.gyro[0] = LoadBe16Signed(&rec[0x07]);
  out.gyro[1] = LoadBe16Signed(&rec[0x09]);
  out.gyro[2] = LoadBe16Signed(&rec[0x0B]);

  // INTF_CONFIG0.FIFO_HOLD_LAST_DATA_EN=0 inserts invalid code (-32768).
  // Rejecting the record is the whole answer here: the caller counts it and
  // resyncs. Whether that is fatal is Sentinel's call, not an ISR's.
  if (!fifo_hold_last_data_en_) {
    static constexpr int32_t kInvalid16 = Icm42688pReg::kFifo16InvalidSentinel;
    if (out.accel[0] == kInvalid16 || out.accel[1] == kInvalid16 ||
        out.accel[2] == kInvalid16 || out.gyro[0] == kInvalid16 ||
        out.gyro[1] == kInvalid16 || out.gyro[2] == kInvalid16) {
      invalid_sample_cnt_.fetch_add(1, std::memory_order_relaxed);
      return std::nullopt;
    }
  }

  return out;
}

std::optional<Icm42688p::Sample> Icm42688p::ParsePacket4Record(
    const uint8_t *rec) {
  if (!rec) return std::nullopt;
  Sample out{};

  // Header check: Packet4 = 0x78 with mask 0xF8 (top 5 bits define the
  // variant; bits[1:0] are ODR-change flags, ignored). Also accepts the
  // FSYNC-tagged variant 0x7C — same packet shape, different time-tag.
  const uint8_t hdr = rec[0];
  if ((hdr & Icm42688pReg::kFifoHeaderVariantMask) !=
      Icm42688pReg::kFifoHeaderPacket4) {
    last_bad_header_.store(hdr, std::memory_order_relaxed);
    return std::nullopt;
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

  out.timestamp_us = UpdateTimestampAndSync(ts16);
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
      invalid_sample_cnt_.fetch_add(1, std::memory_order_relaxed);
      return std::nullopt;
    }
  }

  return out;
}

// Roughly once a second, against a burst rate three orders of magnitude higher:
// the die moves on a ~90 s thermal constant, so anything faster republishes a
// value that has not changed. Also keeps the float divide out of all but one
// interrupt in a thousand.
constexpr uint32_t kTemperaturePeriodUs = 1000000u;

void Icm42688p::PublishTemperature(uint32_t now_us) {
  if (!have_temp_) {
    return;
  }
  if (temp_published_us_ != 0u &&
      (now_us - temp_published_us_) < kTemperaturePeriodUs) {
    return;
  }
  temp_published_us_ = now_us;
  blackboard_->UpdateImuTemperature(ImuTemperature{
      .timestamp_us = now_us,
      .celsius = ScaleTemperature(last_temp_raw_),
  });
}

// Every burst, unconditionally: `timestamp_us` is the stall window Sentinel
// measures, and stm32_config.hpp asserts that window against this rate.
void Icm42688p::PublishHealth(uint32_t now_us) {
  // Summed rather than counted separately: a fifth counter read before these
  // four can report a total smaller than the parts printed beside it.
  const uint32_t overruns = overrun_cnt_.load(std::memory_order_relaxed);
  const uint32_t dma_start_fails =
      dma_start_fail_cnt_.load(std::memory_order_relaxed);
  const uint32_t spi_errors = spi_error_cnt_.load(std::memory_order_relaxed);
  const uint32_t parse_fails = parse_fail_cnt_.load(std::memory_order_relaxed);
  blackboard_->UpdateImuHealth(ImuHealth{
      .timestamp_us = now_us,
      .publish_count = publish_cnt_.load(std::memory_order_relaxed),
      .path_faults = overruns + dma_start_fails + spi_errors + parse_fails,
      .overruns = overruns,
      .dma_start_fails = dma_start_fails,
      .spi_errors = spi_errors,
      .parse_fails = parse_fails,
      .invalid_samples = invalid_sample_cnt_.load(std::memory_order_relaxed),
      .dropped_records = dropped_records_.load(std::memory_order_relaxed),
      .missed_samples = missed_samples_.load(std::memory_order_relaxed),
      .last_bad_header = static_cast<uint8_t>(
          last_bad_header_.load(std::memory_order_relaxed)),
  });
}

void Icm42688p::PublishBurst(const ImuBurst &burst) {
  ImuBurstSlot &slot = blackboard_->ImuBurstMailbox();
  if (slot.fresh) {
    // The consumer is still reading the previous burst, so this one is
    // dropped. Counted in samples to stay comparable with the ODR, which is
    // what kLossPanicPerSec is scaled from. No PendSV either: waking the
    // consumer would only re-offer the burst it already holds.
    missed_samples_.fetch_add(fifo_wm_records_, std::memory_order_relaxed);
    return;
  }

  // Acquire: the write below must not be hoisted above the test that made it
  // safe.
  std::atomic_signal_fence(std::memory_order_acquire);
  // Both fences are load-bearing: without them the compiler is free to move the
  // copy out from between the two `seq` stores that bracket it.
  slot.seq = slot.seq + 1u;
  std::atomic_signal_fence(std::memory_order_release);
  slot.burst = burst;
  std::atomic_signal_fence(std::memory_order_release);
  slot.seq = slot.seq + 1u;
  slot.fresh = true;
  publish_cnt_.fetch_add(1, std::memory_order_relaxed);

  // Defer control-loop consumption out of IRQ/DMA context.
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void Icm42688p::ApplyGyroOffsets(const float bias_body[3]) {
  auto &time = System::GetInstance().Time();

  // The mean is body-NED but OFFSET_USER is per chip axis, so it goes back
  // through the map before any register sees it. Getting this backwards writes
  // a permanent offset to the wrong axis, correcting nothing and spoiling one
  // that was fine.
  float bias_chip[3] = {0.0f, 0.0f, 0.0f};
  ChipFromBody(bias_body, bias_chip);

  int16_t offset_lsb[3] = {0, 0, 0};
  for (int axis = 0; axis < 3; ++axis) {
    // The register is 1/32 dps per code, and the burst already arrives in
    // rad/s, so LSB never enters this path.
    const float bias_dps = bias_chip[axis] / Icm42688pReg::kDegToRad;
    int32_t code = static_cast<int32_t>(lrintf(-bias_dps * 32.0f));
    if (code < -2048) {
      code = -2048;
    } else if (code > 2047) {
      code = 2047;
    }
    offset_lsb[axis] = static_cast<int16_t>(code);
  }

  // Restored, not asserted: sampling is armed only once a consumer is wired.
  const bool was_sampling = NVIC_GetEnableIRQ(board::kImuInt.exti_irqn) != 0u;
  NVIC_DisableIRQ(board::kImuInt.exti_irqn);
  NVIC_ClearPendingIRQ(board::kImuInt.exti_irqn);
  while (inflight_.load(std::memory_order_acquire)) {
  }

  SetBank(0);
  const uint8_t prev_pwr = ReadReg(Reg::kPwrMgmt0);
  WriteReg(Reg::kPwrMgmt0, 0x00u);
  time.DelayMicros(200);

  WriteGyroUserOffsets(offset_lsb[0], offset_lsb[1], offset_lsb[2]);

  // The offsets just written change what the chip reports, so anything already
  // buffered describes the old calibration and the timestamp chain has to
  // restart with it.
  FlushAndResync();
  WriteReg(Reg::kPwrMgmt0, prev_pwr);
  time.DelayMicros(200);
  time.DelayMicros(MillisToMicros(50));
  WriteReg(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(Reg::kIntStatus);
  // No interrupt ran while the offsets were written, and last_irq_us_ is the
  // unwrapped host clock the log stamps against, so that window has to be added
  // rather than dropped. Safe to advance in one step: FlushAndResync above
  // cleared the sync flag, so the next record reseeds the offset instead of
  // servoing across the jump.
  const uint32_t now_cnt = System::GetInstance().Time().Micros();
  last_irq_us_ += static_cast<uint32_t>(now_cnt - last_irq_cnt_);
  last_irq_cnt_ = now_cnt;
  if (was_sampling) {
    NVIC_EnableIRQ(board::kImuInt.exti_irqn);
  }
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
      ((static_cast<uint64_t>(odr_hz) * external_clock.frequency_hz) +
       (kNominalOdrReferenceHz / 2u)) /
      kNominalOdrReferenceHz);
}

uint32_t Icm42688p::TimestampTickScaleQ16(
    const typename Config::ExternalClock &external_clock) {
  if (!external_clock.enabled) {
    return kTimestampScaleQ16;
  }

  return static_cast<uint32_t>(
      ((static_cast<uint64_t>(kNominalTimestampReferenceHz) << 16u) +
       (external_clock.frequency_hz / 2u)) /
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
  // Records, not bytes: SetInterfaceConfig sets FIFO_COUNT_REC, and the
  // watermark is compared against that same count. In bytes this asks for a
  // level past what the FIFO can hold, so FIFO_THS never fires at all.
  WriteReg(Reg::kFifoConfig2, static_cast<uint8_t>(fifo_wm_records_ & 0xFFu));
  WriteReg(Reg::kFifoConfig3,
           static_cast<uint8_t>((fifo_wm_records_ >> 8) & 0x0Fu));
  WriteReg(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(Reg::kIntStatus);    // clear any pending status bits (R/C)
  WriteReg(Reg::kFifoConfig, 0x40);  // FIFO Stream mode
  SetupDmaBuffer();
}

void Icm42688p::SetupDmaBuffer() {
  // MOSI is don't-care for the payload but still clocked out, so the buffer is
  // filled once here rather than per burst; only the leading byte matters.
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

void Icm42688p::SuspendSampling() { NVIC_DisableIRQ(board::kImuInt.exti_irqn); }

void Icm42688p::RestartSampling() {
  // Suspend first: FlushAndResync touches registers the sample interrupt owns.
  // The flush is what breaks the deadlock -- the watermark raises INT1 only on
  // an upward crossing, and a FIFO stuck above it never crosses again.
  SuspendSampling();
  FlushAndResync();
  ResumeSampling();
}

void Icm42688p::ResumeSampling() {
  // The chip never stopped sampling into its own FIFO, so what is in there is
  // a backlog rather than the present. Discarded instead of parsed: a stale
  // burst would be integrated by the AHRS as if it had just happened, and
  // FIFO_THS never re-fires from a FIFO already above the watermark.
  WriteReg(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(Reg::kIntStatus);
  // A burst published while no consumer was wired is the same backlog in
  // software. The IRQ is still masked here, so nothing races the clear.
  blackboard_->ImuBurstMailbox().fresh = false;
  // EXTI latched an edge while sampling was unheard. Its pending bit goes
  // first -- clearing only the NVIC's lets EXTI re-pend at once.
  EXTI->PR = board::kImuInt.pin;
  NVIC_ClearPendingIRQ(board::kImuInt.exti_irqn);
  NVIC_EnableIRQ(board::kImuInt.exti_irqn);
}
