// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <atomic>
#include <cstdint>
#include <optional>

#include "ee.hpp"
#include "icm42688p_reg.hpp"
#include "shared_state.hpp"
#include "spi.hpp"

class GPIO;

class Icm42688p {
 public:
  struct Config {
    SpiPrescaler spi_prescaler;

    struct ExternalClock {
      bool enabled;
      uint32_t frequency_hz;
    } external_clock;

    struct Rates {
      Icm42688pReg::Odr gyro;
      Icm42688pReg::Odr accel;
    } rates;

    struct FullScale {
      Icm42688pReg::GyroFs gyro;
      Icm42688pReg::AccelFs accel;
    } fs;

    struct UiFilter {
      uint8_t gyro_bw;
      uint8_t accel_bw;
      uint8_t gyro_cfg1;
      uint8_t accel_cfg1;
    } ui_filter;

    struct Notch {
      float freq_hz;
      uint8_t bw_idx;
      bool enabled;
    } notch;

    struct Aaf {
      bool dis;
      uint8_t delt;
      uint16_t delt_sqr;
      uint8_t bitshift;
    } gyro_aaf, accel_aaf;

    struct Fifo {
      uint16_t watermark_records;
      bool hold_last;
      bool hires;
    } fifo;

    // Per-board chip-frame → body-NED axis remap, applied inside this driver
    // and nowhere else, so no downstream code re-flips axes.
    // Each component picks a chip axis (0=X, 1=Y, 2=Z) and optional sign.
    // Identity default: chip +X/+Y/+Z = body +X(N)/+Y(E)/+Z(D).
    struct AxisMap {
      uint8_t x_from = 0;  // chip-frame axis index feeding body +X
      bool x_neg = false;
      uint8_t y_from = 1;
      bool y_neg = false;
      uint8_t z_from = 2;
      bool z_neg = false;
    } axis_map{};
  };

  // Checked where the config is defined rather than at Init: every field is a
  // build-time constant, so a bad one fails the build instead of the aircraft.
  static consteval bool WatermarkFitsFifo(const Config &cfg) {
    const uint32_t capacity = Icm42688pReg::kFifoBytes /
                              (cfg.fifo.hires ? Icm42688pReg::kPacket4Bytes
                                              : Icm42688pReg::kPacket3Bytes);
    return cfg.fifo.watermark_records != 0u &&
           cfg.fifo.watermark_records <= capacity;
  }

  // Aliasing or dropping an axis misconfigures the estimator with no obvious
  // flight symptom, and a non-permutation is not invertible by ChipFromBody.
  static consteval bool AxisMapIsPermutation(const Config::AxisMap &map) {
    return map.x_from <= 2u && map.y_from <= 2u && map.z_from <= 2u &&
           map.x_from != map.y_from && map.x_from != map.z_from &&
           map.y_from != map.z_from;
  }

  static Icm42688p &GetInstance();

  void OnIrq();
  void SuspendSampling();
  void ResumeSampling();
  // Suspend, discard whatever the chip buffered, resume. Sentinel's lever on
  // a stalled sample path.
  void RestartSampling();
  // bias_body is body-NED; OFFSET_USER is per chip axis. Blocking, and stops
  // the sample path for ~50 ms: slow loop only, never while armed.
  void ApplyGyroOffsets(const float bias_body[3]);

  uint32_t GetDeviceId() const;
  bool IsInitialized() const { return device_id_ != 0u; }

 private:
  friend class System;
  void Init(GPIO &gpio, Spi2 &spi, const Config &cfg, SharedState &blackboard);

  Icm42688p() = default;
  ~Icm42688p() = default;
  Icm42688p(const Icm42688p &) = delete;
  Icm42688p &operator=(const Icm42688p &) = delete;

  void WriteReg(Icm42688pReg::Reg reg, uint8_t val);
  uint8_t ReadReg(Icm42688pReg::Reg reg);
  void SetBank(uint8_t bank);
  void CsLow();
  void CsHigh();

  void CheckWhoAmI();
  void SoftReset();
  void SetClockSource(const Config &cfg);
  void SetInterfaceConfig(const Config &cfg);
  void DisableFsync();        // FSYNC_UI_SEL = 0
  void SetInterruptConfig();  // INT_CONFIG=0x03, INT_CONFIG1=0x60

  void ConfigureFilters(const Config &cfg);
  void SetOdrAndFullScale(const Config &cfg);

  void SetTimestampConfig();
  void ClearUserOffsets();
  void WriteGyroUserOffsets(int16_t x_offset_lsb, int16_t y_offset_lsb,
                            int16_t z_offset_lsb);
  void ConfigureFifo();
  void SetupDmaBuffer();
  void FlushAndResync();

  static void SpiDoneThunk(bool ok);
  void OnSpiDone(bool ok);
  void PublishBurst(const ImuBurst &burst);
  void PublishHealth(uint32_t now_us);
  // Rate-limited inside; called on every burst so the cadence lives with the
  // reason for it rather than with the caller.
  void PublishTemperature(uint32_t now_us);
  // One decoded FIFO record, still in the chip's frame. Never leaves this file:
  // MapAxes is the sole frame boundary, and a chip-frame count is unusable to
  // anything that does not already know the part.
  struct Sample {
    uint64_t timestamp_us;
    int32_t accel[3];
    int32_t gyro[3];
    int16_t temp_raw;
  };

  std::optional<Sample> ParsePacket3Record(const uint8_t *rec);
  std::optional<Sample> ParsePacket4Record(const uint8_t *rec);

  // Writes one sample into the burst's axis-major arrays. Out-param rather than
  // a returned struct because the destination is structure-of-arrays: a
  // per-sample return would only be transposed at the call site.
  void MapAxes(const Sample &sample, uint16_t slot, ImuBurst &out) const;

  // Frame half of MapAxes, inverted. CalibrateGyro measures in body-NED but
  // OFFSET_USER is per chip axis, so the mean has to come back through the map
  // before it is written -- and that write is permanent. Units are not
  // inverted: the offset register takes dps, so nothing returns to LSB.
  void ChipFromBody(const float body[3], float chip[3]) const;
  // Packet4 carries 16 temperature bits and Packet3 only 8, so the decode is
  // here rather than at either call site.
  float ScaleTemperature(int16_t temp_raw) const;
  // Per record: the sensor stamps each sample, so each needs unwrapping.
  uint64_t UpdateTimestampAndSync(uint16_t ts16);
  // Per burst: INT1 fires when the newest sample lands, so last_irq_us_
  // and tmst64_us_ describe the same instant only once the loop is done.
  void SyncHostOffset();
  static uint32_t EffectiveOdrHz(
      Icm42688pReg::Odr odr,
      const typename Config::ExternalClock &external_clock);
  static uint32_t TimestampTickScaleQ16(
      const typename Config::ExternalClock &external_clock);

  static constexpr uint32_t kExternalClockMinHz = 31000u;
  static constexpr uint32_t kExternalClockMaxHz = 50000u;
  // Properties of the part, not of the board: the frequencies the datasheet
  // quotes its ODRs and its timestamp resolution against. CLKIN scales both.
  static constexpr uint32_t kNominalOdrReferenceHz = 32000u;
  static constexpr uint32_t kNominalTimestampReferenceHz = 32768u;
  static constexpr uint32_t kTimestampScaleQ16 = 1u << 16;

  // Buffer sized for worst-case Packet4 (20-byte record); Packet3 fills
  // only the first 16 bytes. Static so SPI DMA buffers never re-allocate
  // when the FIFO mode flips.
  static constexpr uint16_t kMaxPacketBytes = Icm42688pReg::kPacket4Bytes;
  static constexpr uint16_t kMaxReadBytes = kImuMaxSamples * kMaxPacketBytes;

  // Set from Config::Fifo::hires at Init, alongside the FS selections.
  bool hires_{false};
  uint16_t packet_bytes_{0};

  uint16_t fifo_wm_records_{0};
  uint8_t fifo_tx_[1 + kMaxReadBytes]{};
  uint8_t fifo_rx_[1 + kMaxReadBytes]{};

  std::atomic<bool> inflight_{false};
  // Set when an interrupt arrived mid-burst and its records went undrained.
  // Consumed by OnSpiDone, which is where the bus is free again.
  std::atomic<bool> resync_pending_{false};
  std::atomic<uint32_t> overrun_cnt_{0};
  // Records the chip held that a flush threw away, read from FIFO_COUNT at the
  // moment of the flush rather than assumed from the watermark.
  std::atomic<uint32_t> dropped_records_{0};
  std::atomic<uint32_t> publish_cnt_{0};
  std::atomic<uint32_t> parse_fail_cnt_{0};
  std::atomic<uint32_t> invalid_sample_cnt_{0};
  // Accumulated from 32-bit deltas, exactly as tmst64_us_ is built from the
  // sensor's 16-bit ticks. A single widened read would carry TIM2's
  // 71.6-minute wrap into the sync servo as a 33-second backward step.
  volatile uint64_t last_irq_us_{0};
  uint32_t last_irq_cnt_{0};

  uint16_t last_tmst16_{0};
  uint64_t tmst64_us_{0};
  uint32_t last_sample_dt_q16_{0};
  uint32_t timestamp_tick_scale_q16_{0};
  uint32_t timestamp_tick_remainder_q16_{0};
  bool tmst_inited_{false};

  int64_t host_offset_us_{0};
  bool host_sync_inited_{false};
  bool fifo_hold_last_data_en_{false};
  // Resolved once at Init rather than branched on per sample.
  struct ScaleConfig {
    float accel_lsb_to_mps2;
    float gyro_lsb_to_rad_s;
    // The axis map unpacked, so the two users index it directly instead of
    // rebuilding these arrays per call. Also keeps this whole object
    // zero-valued until Init, which is what holds it out of .data.
    uint8_t src[3];
    bool neg[3];
  };
  ScaleConfig scale_config_{};
  uint32_t gyro_odr_hz_{0};
  uint8_t who_am_i_{0};
  uint32_t device_id_{0};

  GPIO *gpio_{nullptr};
  SharedState *blackboard_{nullptr};
  Spi2 *spi_{nullptr};

  std::atomic<uint32_t> missed_samples_{0};

  // Touched only from the sample interrupt, so no handshake: the publication
  // itself is the handoff, and a burst that lands mid-publish just moves the
  // next one's reading.
  int16_t last_temp_raw_ = 0;
  uint32_t temp_published_us_ = 0;
  // A cold start would otherwise publish the zero code, which decodes to a
  // plausible 25 degC rather than to something a reader can reject.
  bool have_temp_ = false;
};
