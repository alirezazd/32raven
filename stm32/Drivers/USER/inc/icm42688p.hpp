#pragma once

#include "icm42688p_reg.hpp"
#include "ring_buffer.hpp"
#include <cstdint>

class Icm42688p {
public:
  struct Config {
    uint8_t spi_prescaler;

    // ODR / FS
    Icm42688pReg::Odr gyro_odr;
    Icm42688pReg::Odr accel_odr;
    Icm42688pReg::GyroFs gyro_fs;
    Icm42688pReg::AccelFs accel_fs;

    // UI Filter Bandwidth (Bank 0)
    // GYRO_ACCEL_CONFIG0: [7:4] Accel BW, [3:0] Gyro BW
    // 0=ODR/2, 1=ODR/4...
    uint8_t gyro_ui_filt_bw;
    uint8_t accel_ui_filt_bw;

    // UI Filter Order (Bank 0)
    // GYRO_CONFIG1 / ACCEL_CONFIG1
    uint8_t gyro_cfg1;  // GYRO_CONFIG1 (order, etc)
    uint8_t accel_cfg1; // ACCEL_CONFIG1 (order, etc)

    // Gyro Notch Filter (Bank 1)
    float notch_freq_hz;  // e.g. 0 to disable, or 1000.0f
    uint8_t notch_bw_idx; // 0..7 (Register 0x13[6:4])

    // Gyro AAF (Bank 1)
    bool gyro_aaf_dis;          // True to disable
    uint8_t gyro_aaf_delt;      // Register 0x0C[5:0]
    uint16_t gyro_aaf_delt_sqr; // Register 0x0D + 0x0E[3:0] (12 bits)
    uint8_t gyro_aaf_bitshift;  // Register 0x0E[7:4]

    // Accel AAF (Bank 2)
    bool accel_aaf_dis;          // True to disable
    uint8_t accel_aaf_delt;      // Register 0x03[6:1]
    uint16_t accel_aaf_delt_sqr; // Register 0x04 + 0x05[3:0] (12 bits)
    uint8_t accel_aaf_bitshift;  // Register 0x05[7:4]

    // Feature Flags
    bool enable_fsync_pin9;
    bool enable_tmst_regs;
    bool enable_tmst_fsync;
    uint8_t fsync_ui_sel;
    bool fsync_polarity_falling;
  };

  struct Sample {
    uint64_t timestamp_us; // GPS-corrected microseconds from Micros()
    int16_t accel[3];
    int16_t gyro[3];
    uint32_t seq;
  };

  static Icm42688p &GetInstance() {
    static Icm42688p inst;
    return inst;
  }

  void Init(const Config &cfg);

  // Called from EXTI ISR (DRDY)
  void OnDrdyIrq();

  // Called from main loop
  bool PopSample(Sample &out);

  uint32_t OverrunCount() const { return overrun_; }
  bool IsInitialized() const { return initialized_; }

private:
  Icm42688p() = default;
  ~Icm42688p() = default;
  Icm42688p(const Icm42688p &) = delete;
  Icm42688p &operator=(const Icm42688p &) = delete;

  // SPI helpers (blocking, used only during init)
  void WriteReg(uint8_t reg, uint8_t val);
  uint8_t ReadReg(uint8_t reg);
  void SetBank(uint8_t bank);

  // Configuration helpers
  void ConfigureFilters(const Config &cfg);

  // DMA completion
  static void SpiDoneThunk(void *user, bool ok);
  void OnSpiDone(bool ok);

private:
  bool initialized_ = false;

  // DMA transaction buffers
  static constexpr uint8_t kBurstStartReg = Icm42688pReg::REG_ACCEL_DATA_X1;
  // Payload: 6 Accel + 6 Gyro + 2 Temp + 2 Timestamp + ?
  // Ref code read 20 bytes?
  // Let's stick to: Accel(6) + Gyro(6) + Temp(2) + Tmst(2) = 16 bytes.

  static constexpr uint8_t kPayloadLen = 14; // Accel(6) + Gyro(6) + Temp(2)
  static constexpr uint8_t kXferLen = 1 + kPayloadLen;

  uint8_t tx_[kXferLen] = {};
  uint8_t rx_[kXferLen] = {};

  volatile bool inflight_ = false;
  volatile uint32_t overrun_ = 0;

  // last captured IRQ timestamp (GPS-corrected us)
  volatile uint64_t last_irq_us_ = 0;

  // SPSC ring buffer: ISR pushes, main loop pops
  RingBuffer<Sample, 16> ring_;
  uint32_t seq_ = 0;
};
