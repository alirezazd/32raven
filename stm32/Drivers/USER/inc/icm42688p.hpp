#pragma once

#include "icm42688p_reg.hpp"
#include "ring_buffer.hpp"

#include <cstdint>

class Icm42688p {
public:
  struct Config {
    uint8_t spi_prescaler;

    // ODR / Full-Scale
    Icm42688pReg::Odr gyro_odr;
    Icm42688pReg::Odr accel_odr;
    Icm42688pReg::GyroFs gyro_fs;
    Icm42688pReg::AccelFs accel_fs;

    // UI filter bandwidth index (GYRO_ACCEL_CONFIG0)
    // 0=ODR/2, 1=ODR/4, 2=ODR/8, 3=ODR/16 ...
    uint8_t gyro_ui_filt_bw;
    uint8_t accel_ui_filt_bw;

    // UI filter order (GYRO_CONFIG1 / ACCEL_CONFIG1 raw values)
    uint8_t gyro_cfg1;
    uint8_t accel_cfg1;

    // Gyro notch filter (Bank 1)
    float notch_freq_hz;  // 0 to disable
    uint8_t notch_bw_idx; // 0..7

    // Gyro anti-alias filter (Bank 1)
    bool gyro_aaf_dis;
    uint8_t gyro_aaf_delt;
    uint16_t gyro_aaf_delt_sqr;
    uint8_t gyro_aaf_bitshift;

    // Accel anti-alias filter (Bank 2)
    bool accel_aaf_dis;
    uint8_t accel_aaf_delt;
    uint16_t accel_aaf_delt_sqr;
    uint8_t accel_aaf_bitshift;

    // FSYNC (GPS PPS on pin 9)
    bool enable_fsync_pin9;
    bool enable_tmst_regs;
    bool enable_tmst_fsync;
    uint8_t fsync_ui_sel; // 0x4 = route to TMST only
    bool fsync_polarity_falling;
  };

  struct Sample {
    uint64_t timestamp_us; // GPS-corrected microseconds (monotonic)
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp_raw;
    uint32_t seq;
  };

  static Icm42688p &GetInstance() {
    static Icm42688p inst;
    return inst;
  }

  void Init(const Config &cfg);

  // Called from EXTI ISR — timestamps and starts DMA read
  void OnDrdyIrq();

  // Called from main loop — pops oldest sample
  bool PopSample(Sample &out);

  uint32_t OverrunCount() const { return overrun_; }
  bool IsInitialized() const { return initialized_; }

private:
  Icm42688p() = default;
  ~Icm42688p() = default;
  Icm42688p(const Icm42688p &) = delete;
  Icm42688p &operator=(const Icm42688p &) = delete;

  // SPI helpers (blocking — init only)
  void WriteReg(uint8_t reg, uint8_t val);
  uint8_t ReadReg(uint8_t reg);
  void SetBank(uint8_t bank);

  void ConfigureFilters(const Config &cfg);

  // DMA completion path
  static void SpiDoneThunk(void *user, bool ok);
  void OnSpiDone(bool ok);

  bool initialized_ = false;

  // DMA burst: 1 addr byte + 14 data bytes
  //   Accel XYZ (6) + Gyro XYZ (6) + Temp (2) = 14
  static constexpr uint8_t kBurstStartReg = Icm42688pReg::REG_ACCEL_DATA_X1;
  static constexpr uint8_t kPayloadLen = 14;
  static constexpr uint8_t kXferLen = 1 + kPayloadLen;

  uint8_t tx_[kXferLen] = {};
  uint8_t rx_[kXferLen] = {};

  volatile bool inflight_ = false;
  volatile uint32_t overrun_ = 0;
  volatile uint64_t last_irq_us_ = 0;

  // SPSC ring buffer: ISR pushes, main loop pops
  RingBuffer<Sample, 64> sample_buf_;
  uint32_t seq_ = 0;
};
