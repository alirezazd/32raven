// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "icm42688p.hpp"

#include <cmath>
#include <utility>

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

static_assert(Icm42688p::WatermarkFitsFifo(kIcm42688pConfig),
              "IMU FIFO watermark exceeds what the chip holds at this packet "
              "size -- lower STM32_CONTROL_LOOP_HZ or the gyro ODR");
static_assert(Icm42688p::AxisMapIsPermutation(kIcm42688pConfig.axis_map),
              "IMU axis map is not a permutation of {0,1,2}");
static_assert(Icm42688p::AxisMapIsRotation(kIcm42688pConfig.axis_map),
              "IMU axis map is a mirror, not a rotation -- one sign is wrong");

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

void Icm42688p::Init(GPIO &gpio, Spi2 &spi, const Config &cfg,
                     SharedState &blackboard) {
  if (device_id_ != 0u) {
    Panic(ErrorCode::Stm32::kImuReinit);
  }

  gpio_ = &gpio;

  blackboard_ = &blackboard;
  spi_ = &spi;

  fifo_wm_records_ = cfg.fifo.watermark_records;
  fifo_hold_last_data_en_ = cfg.fifo.hold_last;
  last_irq_cnt_ = System::GetInstance().Time().Micros();
  // Seeded from the TIM2 count, not zero: the log's timeline uses the same
  // count, and zero would stamp IMU records a whole boot behind the rest.
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

  System::GetInstance().Time().DelayMicros(MillisToMicros(10));
  spi.SetPrescaler(cfg.spi_prescaler);

  CheckWhoAmI();
  device_id_ = 2u | (static_cast<uint32_t>(SpiInstance::kSpi2) << 3) |
               (static_cast<uint32_t>(who_am_i_) << 16);
  SoftReset();
  SetBank(0);
  // Datasheet: keep gyro/accel OFF while programming non-ODR/FS registers.
  WriteRegister(Reg::kPwrMgmt0, 0x00u);
  System::GetInstance().Time().DelayMicros(200);

  SetClockSource(cfg);
  SetInterfaceConfig(cfg);
  DisableFsync();
  SetInterruptConfig();

  ConfigureFilters(cfg);
  SetOdrAndFullScale(cfg);

  SetTimestampConfig();
  WriteRegister(Reg::kPwrMgmt0, PWR_MGMT0_TEMP_DIS);
  ConfigureFifo();

  // Last, with temp on too: the FIFO records carry die temperature.
  WriteRegister(Reg::kPwrMgmt0,
                PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);

  // Datasheet: no register writes for 200us after OFF->ON transition.
  System::GetInstance().Time().DelayMicros(200);
  // Allow gyro to settle / become valid.
  System::GetInstance().Time().DelayMicros(MillisToMicros(50));

  spi.EnableIrqs(irq_priority::kImuSpiDma);
  NVIC_SetPriority(board::kImuInt.exti_irqn, irq_priority::kImuInt);
  // Configured, not running: ResumeSampling arms this once a consumer exists.
}

uint32_t Icm42688p::GetDeviceId() const {
  if (device_id_ == 0u) {
    Panic(ErrorCode::Stm32::kImuNotInitialized);
  }

  return device_id_;
}

void Icm42688p::ConfigureFilters(const Config &cfg) {
  static constexpr float kPi = 3.14159265358979323846f;
  const auto clampf = [](float x, float lo, float hi) constexpr {
    return (x < lo) ? lo : (x > hi) ? hi : x;
  };
  SetBank(1);

  {
    uint8_t s2 = ReadRegister(Reg::kGyroConfigStatic2);

    if (cfg.notch.enabled && cfg.notch.freq_hz > 0.0f) {
      // Datasheet: notch supported 1kHz..3kHz.
      float f_hz = clampf(cfg.notch.freq_hz, 1000.0f, 3000.0f);

      SetBank(3);
      uint8_t clkdiv = ReadRegister(Reg::kClkdiv);
      SetBank(1);

      if (clkdiv == 0) {
        s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
        WriteRegister(Reg::kGyroConfigStatic2, s2);
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
        WriteRegister(Reg::kGyroConfigStatic2, s2);

        WriteRegister(Reg::kGyroConfigStatic6, (uint8_t)(val & 0xFF));
        WriteRegister(Reg::kGyroConfigStatic7, (uint8_t)(val & 0xFF));
        WriteRegister(Reg::kGyroConfigStatic8, (uint8_t)(val & 0xFF));

        // Bits 7:6 are reserved and factory-trimmed.
        uint8_t s9 =
            static_cast<uint8_t>(ReadRegister(Reg::kGyroConfigStatic9) & 0xC0u);
        if (sel) s9 |= (1u << 5) | (1u << 4) | (1u << 3);
        if (val & 0x100) s9 |= (1u << 2) | (1u << 1) | (1u << 0);
        WriteRegister(Reg::kGyroConfigStatic9, s9);

        uint8_t s10 = ReadRegister(Reg::kGyroConfigStatic10);
        s10 &= ~(0x7u << 4);
        uint8_t bw = (cfg.notch.bw_idx > 7) ? 7 : cfg.notch.bw_idx;
        s10 |= (uint8_t)(bw << 4);
        WriteRegister(Reg::kGyroConfigStatic10, s10);
      }
    } else {
      s2 |= GYRO_CONFIG_STATIC2_NF_DIS;
      WriteRegister(Reg::kGyroConfigStatic2, s2);
    }
  }

  {
    uint8_t s2 = ReadRegister(Reg::kGyroConfigStatic2);
    if (cfg.gyro_aaf.dis) {
      s2 |= GYRO_CONFIG_STATIC2_AAF_DIS;
      WriteRegister(Reg::kGyroConfigStatic2, s2);
    } else {
      s2 &= ~GYRO_CONFIG_STATIC2_AAF_DIS;
      WriteRegister(Reg::kGyroConfigStatic2, s2);

      WriteRegister(Reg::kGyroConfigStatic3,
                    (uint8_t)(cfg.gyro_aaf.delt & 0x3F));
      WriteRegister(Reg::kGyroConfigStatic4,
                    (uint8_t)(cfg.gyro_aaf.delt_sqr & 0xFF));

      uint8_t s5 = (uint8_t)((cfg.gyro_aaf.bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.gyro_aaf.delt_sqr >> 8) & 0xF);
      WriteRegister(Reg::kGyroConfigStatic5, s5);
    }
  }

  SetBank(2);

  {
    uint8_t s2 = ReadRegister(Reg::kAccelConfigStatic2);
    if (cfg.accel_aaf.dis) {
      s2 |= ACCEL_CONFIG_STATIC2_AAF_DIS;
      WriteRegister(Reg::kAccelConfigStatic2, s2);
    } else {
      s2 &= ~ACCEL_CONFIG_STATIC2_AAF_DIS;
      s2 &= ~(0x3Fu << 1);
      s2 |= (uint8_t)((cfg.accel_aaf.delt & 0x3F) << 1);
      WriteRegister(Reg::kAccelConfigStatic2, s2);

      WriteRegister(Reg::kAccelConfigStatic3,
                    (uint8_t)(cfg.accel_aaf.delt_sqr & 0xFF));

      uint8_t s4 = (uint8_t)((cfg.accel_aaf.bitshift & 0xF) << 4) |
                   (uint8_t)((cfg.accel_aaf.delt_sqr >> 8) & 0xF);
      WriteRegister(Reg::kAccelConfigStatic4, s4);
    }
  }

  SetBank(0);
  WriteRegister(Reg::kGyroAccelConfig0,
                static_cast<uint8_t>(((cfg.ui_filter.accel_bw & 0x0Fu) << 4) |
                                     (cfg.ui_filter.gyro_bw & 0x0Fu)));
}

void Icm42688p::OnIrq() {
  const uint32_t now_cnt = System::GetInstance().Time().Micros();
  last_irq_us_ += static_cast<uint32_t>(now_cnt - last_irq_cnt_);
  last_irq_cnt_ = now_cnt;

  if (inflight_.exchange(true, std::memory_order_acq_rel)) {
    // Unread, these keep the FIFO from dropping back below the watermark, and
    // FIFO_THS fires only on an upward crossing: the in-flight burst flushes.
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
    FlushAndResync();
    finish();
    return;
  }

  const uint8_t *p = &fifo_rx_[1];

  ImuBurst burst{};
  burst.device_id = device_id_;
  burst.gyro_scale = scale_config_.gyro_lsb_to_rad_s;
  burst.accel_scale = scale_config_.accel_lsb_to_mps2;

  // Outlives the loop: the newest sample's timestamp is read after the servo.
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

  // Records, big-endian, per SetInterfaceConfig. Read on the fault path only:
  // the hot path deliberately transfers a fixed length instead.
  const uint32_t hi = ReadRegister(Reg::kFifoCountH);
  const uint32_t lo = ReadRegister(Reg::kFifoCountL);
  dropped_records_.fetch_add((hi << 8) | lo, std::memory_order_relaxed);

  WriteRegister(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadRegister(Reg::kIntStatus);

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
// leaves this file in the body frame without ever having been scaled.
void Icm42688p::MapAxes(const Sample &sample, uint16_t slot,
                        ImuBurst &out) const {
  const ScaleConfig &map = scale_config_;
  for (uint8_t body = 0; body < 3u; ++body) {
    const int32_t sign = map.neg[body] ? -1 : 1;
    out.gyro[body][slot] = sign * sample.gyro[map.src[body]];
    out.accel[body][slot] = sign * sample.accel[map.src[body]];
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

  // Seeded here, not in SyncHostOffset: the servo runs after the parse, and
  // a zero offset would stamp the first burst as if the board just booted.
  if (!host_sync_inited_) {
    host_offset_us_ = static_cast<int64_t>(last_irq_us_) - imu_us;
    host_sync_inited_ = true;
  }

  return static_cast<uint64_t>(imu_us + host_offset_us_);
}

// Once per burst: last_irq_us_ moves once per burst, so a per-record call
// biases the offset by half a burst and scales the >> 7 gain by the watermark.
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

  // Packet3 header 0x68, or 0x6C when FSYNC-tagged; ODR-change bits ignored.
  const uint8_t hdr = rec[0];
  if ((hdr & 0xF8u) != 0x68u) {
    return std::nullopt;
  }

  // The byte is two's complement, so widening it to int16_t must sign-extend.
  // NOLINTNEXTLINE(bugprone-signed-char-misuse)
  out.temp_raw = static_cast<int8_t>(rec[0x0D]);
  const uint16_t ts16 = LoadBe16(&rec[0x0E]);

  out.timestamp_us = UpdateTimestampAndSync(ts16);

  out.accel[0] = LoadBe16Signed(&rec[0x01]);
  out.accel[1] = LoadBe16Signed(&rec[0x03]);
  out.accel[2] = LoadBe16Signed(&rec[0x05]);

  out.gyro[0] = LoadBe16Signed(&rec[0x07]);
  out.gyro[1] = LoadBe16Signed(&rec[0x09]);
  out.gyro[2] = LoadBe16Signed(&rec[0x0B]);

  // Rejecting is the whole answer: the caller counts and resyncs, and whether
  // that is fatal is Sentinel's call, not an ISR's.
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

  // Also 0x7C when FSYNC-tagged (same shape); ODR-change bits ignored.
  const uint8_t hdr = rec[0];
  if ((hdr & Icm42688pReg::kFifoHeaderVariantMask) !=
      Icm42688pReg::kFifoHeaderPacket4) {
    return std::nullopt;
  }

  // Sign-extends for free: int16 `high` carries bit 19 into the int32 shift.
  auto pack20 = [](int16_t high, uint8_t low4) -> int32_t {
    return (static_cast<int32_t>(high) << 4) |
           static_cast<int32_t>(low4 & 0x0Fu);
  };

  // Packet4 layout: datasheet §6.1, Figure 10.
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

  // Datasheet §12.8: the sentinel marks a slot with no fresh data.
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

// The die's thermal constant is ~90 s, so faster only republishes an unchanged
// value, and the float divide stays out of all but one burst in a thousand.
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
  // Summed rather than counted separately: a standalone total, read before
  // the parts, can come out smaller than the parts printed beside it.
  const uint32_t overruns = overrun_cnt_.load(std::memory_order_relaxed);
  const SpiFaults bus = Spi2::GetInstance().GetFaults();
  const uint32_t dma_start_fails = bus.start_refused;
  const uint32_t spi_errors = bus.dma_errors;
  const uint32_t parse_fails = parse_fail_cnt_.load(std::memory_order_relaxed);
  blackboard_->UpdateImuHealth(ImuHealth{
      .timestamp_us = now_us,
      .publish_count = publish_cnt_.load(std::memory_order_relaxed),
      .path_faults =
          overruns + dma_start_fails + spi_errors + bus.timeouts + parse_fails,
      .overruns = overruns,
      .parse_fails = parse_fails,
      .invalid_samples = invalid_sample_cnt_.load(std::memory_order_relaxed),
      .dropped_records = dropped_records_.load(std::memory_order_relaxed),
      .missed_samples = missed_samples_.load(std::memory_order_relaxed),
  });
}

void Icm42688p::PublishBurst(const ImuBurst &burst) {
  ImuBurstSlot &slot = blackboard_->ImuBurstMailbox();
  if (slot.fresh) {
    // The consumer still holds the previous burst, so this one is dropped and
    // counted in samples, Sentinel's unit; a PendSV would re-offer the old one.
    missed_samples_.fetch_add(fifo_wm_records_, std::memory_order_relaxed);
    return;
  }

  // Keeps the writes below from being hoisted above the `fresh` test.
  std::atomic_signal_fence(std::memory_order_acquire);
  // Both fences pin the copy between the two `seq` stores that bracket it.
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

void Icm42688p::CheckWhoAmI() {
  auto &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();

  SetBank(0);
  while ((uint32_t)(time.Micros() - start) < MillisToMicros(1000)) {
    const uint8_t id = ReadRegister(Reg::kWhoAmI);
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

  uint8_t v = ReadRegister(Reg::kDeviceConfig);
  v |= 0x01u;  // SOFT_RESET_CONFIG = 1, preserve SPI_MODE + reserved
  WriteRegister(Reg::kDeviceConfig, v);

  // One tick over: DelayMicros can end up to a tick short of its count.
  System::GetInstance().Time().DelayMicros(MillisToMicros(1) + 1u);
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

  uint8_t pin_cfg = ReadRegister(Reg::kIntfConfig5);
  pin_cfg &= static_cast<uint8_t>(~INTF_CONFIG5_PIN9_FUNCTION_MASK);
  pin_cfg |= cfg.external_clock.enabled ? INTF_CONFIG5_PIN9_FUNCTION_CLKIN
                                        : INTF_CONFIG5_PIN9_FUNCTION_FSYNC;
  WriteRegister(Reg::kIntfConfig5, pin_cfg);

  SetBank(0);

  uint8_t v = ReadRegister(Reg::kIntfConfig1);
  // RTC_MODE needs pin 9 as CLKIN; keep reserved [7:4] and ACCEL_LP_CLK_SEL.
  v &= static_cast<uint8_t>(~(Icm42688pReg::INTF_CONFIG1_CLKSEL_MASK |
                              Icm42688pReg::INTF_CONFIG1_RTC_MODE_MASK));
  v |= Icm42688pReg::INTF_CONFIG1_CLKSEL_PLL;
  if (cfg.external_clock.enabled) {
    v |= Icm42688pReg::INTF_CONFIG1_RTC_MODE_EN;
  }
  WriteRegister(Reg::kIntfConfig1, v);
}

void Icm42688p::SetInterfaceConfig(const Config &cfg) {
  SetBank(0);

  uint8_t v = ReadRegister(Reg::kIntfConfig0);

  // 0x73: FIFO_COUNT_REC, big-endian count + data, UI_SIFS_CFG=11 (I2C off).
  // 0x80 is FIFO_HOLD_LAST_DATA_EN; reserved bits 3:2 are preserved.
  uint8_t cfg_bits = 0x73u;
  if (cfg.fifo.hold_last) {
    cfg_bits |= 0x80u;
  }
  v = static_cast<uint8_t>((v & 0x0Cu) | cfg_bits);
  WriteRegister(Reg::kIntfConfig0, v);
}

void Icm42688p::SetInterruptConfig() {
  SetBank(0);
  uint8_t v = ReadRegister(Reg::kIntConfig);
  // Keep reserved 7:6; 0x03: INT1 pulsed, push-pull, active-high; INT2 all 0.
  v = static_cast<uint8_t>((v & 0xC0u) | 0x03u);
  WriteRegister(Reg::kIntConfig, v);
  // 0x60: 8 us pulse and no deassert, as ODR >= 4 kHz requires; ASYNC_RESET=0,
  // which the datasheet needs for proper INT operation. Keep reserved 7, 3:0.
  v = ReadRegister(Reg::kIntConfig1);
  v = static_cast<uint8_t>((v & 0x8Fu) | 0x60u);
  WriteRegister(Reg::kIntConfig1, v);
}

void Icm42688p::DisableFsync() {
  SetBank(0);

  uint8_t v = ReadRegister(Reg::kFsyncConfig);

  // Clear FSYNC_UI_SEL (6:4), FSYNC_UI_FLAG_CLEAR_SEL (1) and FSYNC_POLARITY
  // (0); bits 7 and 3:2 are reserved.
  v &= static_cast<uint8_t>(~0x73u);
  WriteRegister(Reg::kFsyncConfig, v);
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
  WriteRegister(Reg::kGyroConfig0, gyro);
  WriteRegister(Reg::kAccelConfig0, accel);
}

void Icm42688p::SetTimestampConfig() {
  SetBank(0);

  uint8_t v = ReadRegister(Reg::kTmstConfig);

  // Preserve reserved [7:5], clear [4:1], set TMST_EN (bit0).
  v &= 0xE0u;
  v |= 0x01u;

  WriteRegister(Reg::kTmstConfig, v);
}

void Icm42688p::ConfigureFifo() {
  SetBank(0);
  // Preserve reserved [7:6], replace [5:0].
  uint8_t v = ReadRegister(Reg::kIntConfig0);
  v = static_cast<uint8_t>((v & 0xC0u) | 0x0Au);
  WriteRegister(Reg::kIntConfig0, v);
  v = ReadRegister(Reg::kIntSource0);  // Preserve reserved bit7, clear bits6:0
  v = static_cast<uint8_t>((v & 0x80u) | 0x04u);  // FIFO_THS -> INT1 only
  WriteRegister(Reg::kIntSource0, v);
  v = ReadRegister(Reg::kFifoConfig);
  v &= 0x3F;  // clear bits 7:6 → FIFO_MODE = 00 (bypass)
  WriteRegister(Reg::kFifoConfig, v);
  uint8_t fifo_cfg1 = FIFO_CONFIG1_RESUME_PARTIAL_RD |
                      FIFO_CONFIG1_TMST_FSYNC_EN | FIFO_CONFIG1_TEMP_EN |
                      FIFO_CONFIG1_GYRO_EN | FIFO_CONFIG1_ACCEL_EN;
  if (hires_) {
    fifo_cfg1 |= FIFO_CONFIG1_HIRES_EN;  // switches to Packet4
  }
  WriteRegister(Reg::kFifoConfig1, fifo_cfg1);
  // Records, not bytes: FIFO_COUNT_REC makes the watermark a record count, and
  // in bytes it asks for more than the FIFO holds, so FIFO_THS never fires.
  WriteRegister(Reg::kFifoConfig2,
                static_cast<uint8_t>(fifo_wm_records_ & 0xFFu));
  WriteRegister(Reg::kFifoConfig3,
                static_cast<uint8_t>((fifo_wm_records_ >> 8) & 0x0Fu));
  WriteRegister(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadRegister(Reg::kIntStatus);    // clear any pending status bits (R/C)
  WriteRegister(Reg::kFifoConfig, 0x40);  // FIFO Stream mode
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

void Icm42688p::SetBank(uint8_t bank) {
  WriteRegister(Reg::kBankSel, (uint8_t)(bank & 0x07));
  System::GetInstance().Time().DelayMicros(1);
}

void Icm42688p::WriteRegister(Reg target, uint8_t value) {
  auto &spi = *spi_;
  uint8_t tx[2] = {static_cast<uint8_t>(std::to_underlying(target) & 0x7F),
                   value};
  CsLow();
  // Discarded: no failure path to thread it through, the bus counts it anyway,
  // and CheckWhoAmI rejects a part dead enough to matter before these run.
  (void)spi.WriteBytes(tx);
  CsHigh();
}
uint8_t Icm42688p::ReadRegister(Reg source) {
  auto &spi = *spi_;
  uint8_t tx[2] = {static_cast<uint8_t>(std::to_underlying(source) | 0x80),
                   0x00};
  uint8_t rx[2] = {0, 0};
  CsLow();
  // Discarded for the same reason as WriteRegister.
  (void)spi.TxRx(tx, rx, 2);
  CsHigh();
  return rx[1];
}
void Icm42688p::CsLow() {
  gpio_->WritePin(board::kSpi2Cs.port, board::kSpi2Cs.pin, false);
}
void Icm42688p::CsHigh() {
  gpio_->WritePin(board::kSpi2Cs.port, board::kSpi2Cs.pin, true);
}

void Icm42688p::SuspendSampling() {
  NVIC_DisableIRQ(board::kImuInt.exti_irqn);
  // A burst already started finishes on the DMA interrupt, which stays
  // enabled. Until it has, the bus is the burst's, not the caller's.
  while (inflight_.load(std::memory_order_acquire)) {
  }
}

void Icm42688p::RestartSampling() {
  // Suspend first: the sample IRQ owns the registers the flush touches. A FIFO
  // stuck above the watermark never raises INT1 again; the flush frees it.
  SuspendSampling();
  FlushAndResync();
  ResumeSampling();
}

void Icm42688p::ResumeSampling() {
  // Flushed, not parsed: the AHRS would take the backlog for the present, and
  // FIFO_THS never re-fires from a FIFO already above the watermark.
  WriteRegister(Reg::kSignalPathReset, SIGNAL_PATH_RESET_FIFO_FLUSH);
  (void)ReadRegister(Reg::kIntStatus);
  // A burst published while no consumer was wired is the same backlog in
  // software. The IRQ is still masked here, so nothing races the clear.
  blackboard_->ImuBurstMailbox().fresh = false;
  // EXTI latched an edge while sampling was unheard. Its pending bit goes
  // first -- clearing only the NVIC's lets EXTI re-pend at once.
  EXTI->PR = board::kImuInt.pin;
  NVIC_ClearPendingIRQ(board::kImuInt.exti_irqn);
  NVIC_EnableIRQ(board::kImuInt.exti_irqn);
}
