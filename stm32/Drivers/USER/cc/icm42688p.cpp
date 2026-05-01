#include "icm42688p.hpp"

#include <cmath>

#include "board.h"
#include "config_storage.hpp"
#include "gpio.hpp"
#include "panic.hpp"
#include "spi.hpp"
#include "system.hpp"
#include "time_base.hpp"

using namespace Icm42688pReg;

void Icm42688p::Init(GPIO &gpio, Spi2 &spi, EE &ee, const Config &cfg) {
  if (device_id_ != 0u) {
    Panic(ErrorCode::kImuReinit);
  }

  ValidateConfig(cfg);

  gpio_ = &gpio;
  spi_ = &spi;
  ee_ = &ee;
  fifo_wm_records_ = cfg.fifo.watermark_records;
  fifo_hold_last_data_en_ = cfg.fifo.hold_last;
  accel_fs_ = cfg.fs.accel;
  gyro_fs_ = cfg.fs.gyro;
  gyro_odr_ = cfg.rates.gyro;
  gyro_odr_hz_ = EffectiveOdrHz(cfg.rates.gyro, cfg.external_clock);
  timestamp_tick_scale_q16_ = TimestampTickScaleQ16(cfg.external_clock);
  timestamp_tick_remainder_q16_ = 0;
  calibration_cfg_ = cfg.calibration;
  recovery_cfg_ = cfg.recovery;
  accel_calibration_ = ConfigStorage::LoadOrInitImuAccelCalibration(ee);

  System::GetInstance().Time().DelayMicros(MILLIS_TO_MICROS(10));
  spi.SetPrescaler(cfg.spi_prescaler);

  CheckWhoAmI();
  device_id_ = 2u | (static_cast<uint32_t>(SpiInstance::kSpi2) << 3) |
               (static_cast<uint32_t>(who_am_i_) << 16);
  SoftReset();
  SetBank(0);
  // Keep gyro/accel OFF while programming non-ODR/FS registers.
  // Matches datasheet register-modification guidance and reference drivers.
  WriteReg(REG_PWR_MGMT0, 0x00u);
  System::GetInstance().Time().DelayMicros(200);

  SetClockSource(cfg);
  SetInterfaceConfig(cfg);
  DisableFsync();
  SetInterruptConfig();

  ConfigureFilters(cfg);
  SetOdrAndFullScale(cfg);

  SetTimestampConfig();
  ClearUserOffsets();
  // Keep temperature sensor disabled while accel/gyro are OFF during FIFO
  // setup.
  WriteReg(REG_PWR_MGMT0, PWR_MGMT0_TEMP_DIS);
  ConfigureFifo();

  // Enable sensors ONCE, last, including the temperature sensor so FIFO samples
  // carry die temperature for future estimator compensation.
  WriteReg(REG_PWR_MGMT0, PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);

  // Datasheet: no register writes after OFF->ON transition
  System::GetInstance().Time().DelayMicros(200);
  // Optional: allow gyro to settle / become valid
  System::GetInstance().Time().DelayMicros(MILLIS_TO_MICROS(50));

  // EXTI->PR = (1u << 10);

  spi.EnableIrqs();  // SPI DMA interrupts
  NVIC_SetPriority(IMU_INT_EXTI_IRQn, 3);
  NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);
}

bool Icm42688p::SaveAccelCalibration() {
  if (ee_ == nullptr) {
    return false;
  }
  return ConfigStorage::SaveImuAccelCalibration(*ee_, accel_calibration_);
}

uint32_t Icm42688p::GetDeviceId() const {
  if (device_id_ == 0u) {
    Panic(ErrorCode::kImuNotInitialized);
  }

  return device_id_;
}

void Icm42688p::ValidateConfig(const Config &cfg) {
  if (cfg.fifo.watermark_records == 0 ||
      cfg.fifo.watermark_records > kMaxWatermarkRecords) {
    Panic(ErrorCode::kInvalidFifoWatermarkRecords);
  }

  if (cfg.rates.gyro != cfg.rates.accel) {
    Panic(ErrorCode::kImuOdrMismatch);
  }

  if (cfg.external_clock.enabled &&
      (cfg.external_clock.frequency_hz < kExternalClockMinHz ||
       cfg.external_clock.frequency_hz > kExternalClockMaxHz)) {
    Panic(ErrorCode::kImuInvalidOdr);
  }

  if (Icm42688pReg::OdrHz(cfg.rates.gyro) == 0 ||
      Icm42688pReg::OdrHz(cfg.rates.accel) == 0) {
    Panic(ErrorCode::kImuInvalidOdr);
  }

  if (cfg.calibration.gyro_duration_s == 0 ||
      cfg.calibration.gyro_timeout_s == 0) {
    Panic(ErrorCode::kImuCalibrationInvalidConfig);
  }

  if (cfg.recovery.overrun_threshold == 0 ||
      cfg.recovery.overrun_window_s == 0) {
    Panic(ErrorCode::kImuOverrun);
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
    uint8_t s2 = ReadReg(REG_GYRO_CONFIG_STATIC2);

    // ---- Gyro notch filter (datasheet correct) ----
    if (cfg.notch.enabled && cfg.notch.freq_hz > 0.0f) {
      // Datasheet: supported 1kHz..3kHz
      float f_hz = clampf(cfg.notch.freq_hz, 1000.0f, 3000.0f);

      // Need CLKDIV from Bank 3 reg 0x2A
      SetBank(3);
      uint8_t clkdiv = ReadReg(REG_CLKDIV);
      SetBank(1);

      if (clkdiv == 0) {
        // Invalid, do not enable notch
        s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
        WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
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
        WriteReg(REG_GYRO_CONFIG_STATIC2, s2);

        // Same coefficient for all three axes
        WriteReg(REG_GYRO_CONFIG_STATIC6, (uint8_t)(val & 0xFF));
        WriteReg(REG_GYRO_CONFIG_STATIC7, (uint8_t)(val & 0xFF));
        WriteReg(REG_GYRO_CONFIG_STATIC8, (uint8_t)(val & 0xFF));

        uint8_t s9 = 0;
        if (sel) s9 |= (1u << 5) | (1u << 4) | (1u << 3);
        if (val & 0x100) s9 |= (1u << 2) | (1u << 1) | (1u << 0);
        WriteReg(REG_GYRO_CONFIG_STATIC9, s9);

        uint8_t s10 = ReadReg(REG_GYRO_CONFIG_STATIC10);
        s10 &= ~(0x7u << 4);
        uint8_t bw = (cfg.notch.bw_idx > 7) ? 7 : cfg.notch.bw_idx;
        s10 |= (uint8_t)(bw << 4);
        WriteReg(REG_GYRO_CONFIG_STATIC10, s10);
      }
    } else {
      s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
      WriteReg(REG_GYRO_CONFIG_STATIC2, s2);
    }
  }

  // Gyro anti-alias filter (Bank 1)
  {
    uint8_t s2 = ReadReg(REG_GYRO_CONFIG_STATIC2);
    if (cfg.gyro_aaf.dis) {
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
  last_irq_us_ = System::GetInstance().Time().Micros();

  if (inflight_.exchange(true, std::memory_order_acq_rel)) {
    HandleOverrunFault();
    return;
  }
  const uint16_t transfer_len =
      static_cast<uint16_t>(1u + fifo_wm_records_ * kPacketBytes);
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
    const uint8_t *rec = p + static_cast<uint32_t>(i) * kPacketBytes;
    Sample s{};
    if (!ParsePacket3Record(rec, s)) {
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
  WriteReg(REG_SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(REG_INT_STATUS);

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
  const uint64_t window_us =
      static_cast<uint64_t>(SECONDS_TO_MICROS(recovery_cfg_.overrun_window_s));

  if (last_overrun_fault_us_ == 0 ||
      (now_us - last_overrun_fault_us_) > window_us) {
    overrun_window_count_ = 1;
  } else {
    overrun_window_count_++;
  }

  last_overrun_fault_us_ = now_us;

  if (overrun_window_count_ >= recovery_cfg_.overrun_threshold) {
    Panic(ErrorCode::kImuOverrun);
  }
}

void Icm42688p::InjectOverrunFaultForTest() {
  // Arm a one-shot recovery fault. The actual fault is injected from the
  // normal IMU SPI completion path so recovery runs in the same context as a
  // real transport fault.
  inject_overrun_fault_for_test_.store(true, std::memory_order_release);
}

Icm42688p::ScaledSample Icm42688p::ScaleSample(const Sample &sample) const {
  const float accel_scale = Icm42688pReg::AccelLsbToMps2(accel_fs_);
  const float gyro_scale = Icm42688pReg::GyroLsbToRadS(gyro_fs_);

  ScaledSample out{};
  out.timestamp_us = sample.timestamp_us;
  out.temperature_c = Icm42688pReg::FifoTemperatureC(sample.temp_raw);
  out.seq = sample.seq;

  for (uint8_t axis = 0; axis < 3u; ++axis) {
    out.accel_mps2[axis] = static_cast<float>(sample.accel[axis]) * accel_scale;
    out.gyro_rad_s[axis] = static_cast<float>(sample.gyro[axis]) * gyro_scale;
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

  auto be_u16 = [](const uint8_t *q) -> uint16_t {
    return static_cast<uint16_t>((static_cast<uint16_t>(q[0]) << 8) |
                                 static_cast<uint16_t>(q[1]));
  };
  auto be_s16 = [](const uint8_t *q) -> int16_t {
    return static_cast<int16_t>((static_cast<uint16_t>(q[0]) << 8) |
                                static_cast<uint16_t>(q[1]));
  };

  // Packet3 temperature byte is at 0x0D and timestamp stays at 0x0E..0x0F.
  out.temp_raw = static_cast<int8_t>(rec[0x0D]);
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
      Panic(ErrorCode::kImuInvalidSampleDetected);
      return false;
    }
  }

  return true;
}

void Icm42688p::PublishLatestBatch(const SampleBatch &batch) {
  published_batch_ = batch;

  // Publish payload then publish sequence (release).
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
      SECONDS_TO_MICROS(calibration_cfg_.gyro_duration_s);
  const uint32_t timeout_us =
      SECONDS_TO_MICROS(calibration_cfg_.gyro_timeout_s);
  const uint32_t still_threshold_raw =
      calibration_cfg_.gyro_still_threshold_raw;
  const uint32_t odr_hz = gyro_odr_hz_;
  const uint64_t sample_count_u64 =
      ((uint64_t)duration_us * odr_hz + SECONDS_TO_MICROS(1) - 1ULL) /
      SECONDS_TO_MICROS(1);

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
          Panic(ErrorCode::kImuCalibrationMotionDetected);
        }
        sum[axis] += s.gyro[axis];
      }
      collected++;
    }
  }

  int16_t offset_lsb[3] = {0, 0, 0};
  const float dps_per_lsb =
      Icm42688pReg::GyroRangeDps(gyro_fs_) / Icm42688pReg::kFifoDataLsb;
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

  NVIC_DisableIRQ(IMU_INT_EXTI_IRQn);
  NVIC_ClearPendingIRQ(IMU_INT_EXTI_IRQn);
  while (inflight_.load(std::memory_order_acquire)) {
  }

  SetBank(0);
  const uint8_t prev_pwr = ReadReg(REG_PWR_MGMT0);
  WriteReg(REG_PWR_MGMT0, 0x00u);
  time.DelayMicros(200);

  WriteGyroUserOffsets(offset_lsb[0], offset_lsb[1], offset_lsb[2]);

  tmst_inited_ = false;
  host_sync_inited_ = false;
  last_tmst16_ = 0;
  tmst64_us_ = 0;
  timestamp_tick_remainder_q16_ = 0;
  host_offset_us_ = 0;

  WriteReg(REG_SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(REG_INT_STATUS);
  WriteReg(REG_PWR_MGMT0, prev_pwr);
  time.DelayMicros(200);
  time.DelayMicros(MILLIS_TO_MICROS(50));
  WriteReg(REG_SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadReg(REG_INT_STATUS);
  NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);
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
  WriteReg(REG_OFFSET_USER0, static_cast<uint8_t>(x & 0xFFu));
  WriteReg(REG_OFFSET_USER1, static_cast<uint8_t>(((y >> 8) & 0x0Fu) << 4) |
                                 static_cast<uint8_t>((x >> 8) & 0x0Fu));
  WriteReg(REG_OFFSET_USER2, static_cast<uint8_t>(y & 0xFFu));
  WriteReg(REG_OFFSET_USER3, static_cast<uint8_t>(z & 0xFFu));

  const uint8_t user4 = ReadReg(REG_OFFSET_USER4);
  WriteReg(REG_OFFSET_USER4,
           static_cast<uint8_t>((user4 & 0xF0u) | ((z >> 8) & 0x0Fu)));
}

void Icm42688p::CheckWhoAmI() {
  auto &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();

  SetBank(0);
  while ((uint32_t)(time.Micros() - start) < MILLIS_TO_MICROS(1000)) {
    const uint8_t id = ReadReg(REG_WHO_AM_I);
    if (id == WHO_AM_I_ICM42688P || id == WHO_AM_I_ICM42686P) {
      who_am_i_ = id;
      return;
    }
    time.DelayMicros(MILLIS_TO_MICROS(10));
  }

  Panic(ErrorCode::kImuWhoAmIFail);
}

void Icm42688p::SoftReset() {
  SetBank(0);

  uint8_t v = ReadReg(REG_DEVICE_CONFIG);
  v |= 0x01u;  // SOFT_RESET_CONFIG = 1, preserve SPI_MODE + reserved
  WriteReg(REG_DEVICE_CONFIG, v);

  System::GetInstance().Time().DelayMicros(MILLIS_TO_MICROS(1));
  SetBank(0);
}

uint32_t Icm42688p::EffectiveOdrHz(
    Icm42688pReg::Odr odr, const Config::ExternalClock &external_clock) {
  const uint32_t odr_hz = Icm42688pReg::OdrHz(odr);
  if (!external_clock.enabled) {
    return odr_hz;
  }

  return static_cast<uint32_t>(
      (static_cast<uint64_t>(odr_hz) * external_clock.frequency_hz +
       kExternalClockOdrReferenceHz / 2u) /
      kExternalClockOdrReferenceHz);
}

uint32_t Icm42688p::TimestampTickScaleQ16(
    const Config::ExternalClock &external_clock) {
  if (!external_clock.enabled) {
    return kTimestampScaleQ16;
  }

  return static_cast<uint32_t>(
      ((static_cast<uint64_t>(kExternalClockTimestampReferenceHz) << 16u) +
       external_clock.frequency_hz / 2u) /
      external_clock.frequency_hz);
}

void Icm42688p::SetClockSource(const Config &cfg) {
  SetBank(1);

  uint8_t pin_cfg = ReadReg(REG_INTF_CONFIG5);
  pin_cfg &= static_cast<uint8_t>(~INTF_CONFIG5_PIN9_FUNCTION_MASK);
  pin_cfg |= cfg.external_clock.enabled ? INTF_CONFIG5_PIN9_FUNCTION_CLKIN
                                        : INTF_CONFIG5_PIN9_FUNCTION_FSYNC;
  WriteReg(REG_INTF_CONFIG5, pin_cfg);

  SetBank(0);

  uint8_t v = ReadReg(REG_INTF_CONFIG1);
  // INTF_CONFIG1:
  // - AFSR bits[7:6] = 01 (disable adaptive full-scale)
  // - CLKSEL bits[1:0] = 01 (PLL when available)
  // - RTC_MODE bit2 requires the pin 9 CLKIN function when external clocking.
  // Preserve other reserved bits.
  v &=
      static_cast<uint8_t>(~(0xC3u | Icm42688pReg::INTF_CONFIG1_RTC_MODE_MASK));
  v |= static_cast<uint8_t>(0x40u | Icm42688pReg::INTF_CONFIG1_CLKSEL_PLL);
  if (cfg.external_clock.enabled) {
    v |= Icm42688pReg::INTF_CONFIG1_RTC_MODE_EN;
  }
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
  v &= static_cast<uint8_t>(~0x73u);  // ~0b01110011
  WriteReg(REG_FSYNC_CONFIG, v);
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
  WriteReg(REG_GYRO_CONFIG0, gyro);
  WriteReg(REG_ACCEL_CONFIG0, accel);
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
  for (uint8_t reg = REG_OFFSET_USER0; reg <= REG_OFFSET_USER8; ++reg) {
    WriteReg(reg, 0x00u);
  }
}

void Icm42688p::ConfigureFifo() {
  SetBank(0);
  uint8_t v = ReadReg(REG_INT_CONFIG0);  // Preserve reserved [7:6], replace
                                         // [5:0]
  v = static_cast<uint8_t>((v & 0xC0u) | 0x0Au);
  WriteReg(REG_INT_CONFIG0, v);
  v = ReadReg(REG_INT_SOURCE0);  // Preserve reserved bit7, clear bits6:0
  v = static_cast<uint8_t>((v & 0x80u) | 0x04u);  // FIFO_THS -> INT1 only
  WriteReg(REG_INT_SOURCE0, v);  // Packet3, 16-bit, timestamp, DMA-safe
  v = ReadReg(REG_FIFO_CONFIG);
  v &= 0x3F;  // clear bits 7:6 → FIFO_MODE = 00 (bypass)
  WriteReg(REG_FIFO_CONFIG, v);
  WriteReg(
      REG_FIFO_CONFIG1,
      static_cast<uint8_t>(FIFO_CONFIG1_RESUME_PARTIAL_RD |
                           FIFO_CONFIG1_TMST_FSYNC_EN | FIFO_CONFIG1_TEMP_EN |
                           FIFO_CONFIG1_GYRO_EN | FIFO_CONFIG1_ACCEL_EN));
  const uint16_t fifo_wm_bytes =
      static_cast<uint16_t>(fifo_wm_records_ * kPacketBytes);
  WriteReg(REG_FIFO_CONFIG2, static_cast<uint8_t>(fifo_wm_bytes & 0xFFu));
  WriteReg(REG_FIFO_CONFIG3,
           static_cast<uint8_t>((fifo_wm_bytes >> 8) & 0x0Fu));
  WriteReg(REG_SIGNAL_PATH_RESET, 0x02);  // FIFO_FLUSH
  (void)ReadReg(REG_INT_STATUS);          // clear any pending status bits (R/C)
  WriteReg(REG_FIFO_CONFIG, 0x40);        // FIFO Stream mode
  SetupDmaBuffer();
}

void Icm42688p::SetupDmaBuffer() {
  // Fixed-burst FIFO reads: read exactly watermark_records packets per IRQ.
  for (uint16_t i = 0; i < sizeof(fifo_tx_); i++) {
    fifo_tx_[i] = 0xFFu;
  }
  fifo_tx_[0] = static_cast<uint8_t>(REG_FIFO_DATA | 0x80u);
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
  gpio_->WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, false);
}
void Icm42688p::CsHigh() {
  gpio_->WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, true);
}
