#pragma once

#include <atomic>
#include <cstdint>

#include "ee.hpp"
#include "ee_schema.hpp"
#include "icm42688p_reg.hpp"
#include "spi.hpp"
#include "stm32_limits.hpp"

class GPIO;

class Icm42688p {
 public:
  static constexpr uint16_t kMaxWatermarkRecords =
      stm32_limits::kIcm42688pMaxWatermarkRecords;

  struct Config {
    SpiPrescaler spi_prescaler;

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
    } fifo;

    struct Calibration {
      uint32_t gyro_duration_s;
      uint32_t gyro_timeout_s;
      uint32_t gyro_still_threshold_raw;
    } calibration;

    struct Recovery {
      uint32_t overrun_threshold;
      uint32_t overrun_window_s;
      uint32_t fault_led_period_ms;
    } recovery;
  };

  struct Sample {
    uint64_t timestamp_us;
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp_raw;
    uint32_t seq;
  };

  struct SampleBatch {
    uint8_t count;
    Sample samples[kMaxWatermarkRecords];
  };

  // DEBUG_DIAG_BEGIN (remove block when debug campaign is complete)
  struct DebugDiag {
    uint8_t hdr;
    uint8_t rec[16];
    int16_t be_accel[3];
    int16_t le_accel[3];
    int16_t sh1_accel[3];
    int16_t be_gyro[3];
    int16_t le_gyro[3];
    int16_t sh1_gyro[3];
  };
  // DEBUG_DIAG_END

  static Icm42688p &GetInstance() {
    static Icm42688p inst;
    return inst;
  }

  void Init(GPIO &gpio, Spi1 &spi, EE &ee, const Config &cfg);
  void OnIrq();
  bool WaitAndGetLatestBatch(uint32_t &last_seq, SampleBatch &out);
  SampleBatch GetLatestBatch() const;
  void CalibrateGyro();
  void InjectOverrunFaultForTest();
  const ee_schema::ImuAccelCalibration &GetAccelCalibration() const {
    return accel_calibration_;
  }
  ee_schema::ImuAccelCalibration &GetAccelCalibration() {
    return accel_calibration_;
  }
  bool SaveAccelCalibration();

  uint32_t ImuPathOverrun() const {
    return overrun_.load(std::memory_order_relaxed);
  }
  uint32_t MainMissedTicks() const {
    return drop_cnt_.load(std::memory_order_relaxed);
  }
  uint32_t IrqCount() const { return irq_cnt_.load(std::memory_order_relaxed); }
  uint32_t PublishCount() const {
    return publish_cnt_.load(std::memory_order_relaxed);
  }
  uint32_t ParseFailCount() const {
    return parse_fail_cnt_.load(std::memory_order_relaxed);
  }
  uint32_t DmaStartFailCount() const {
    return dma_start_fail_cnt_.load(std::memory_order_relaxed);
  }
  uint32_t LastBadHeader() const {
    return last_bad_header_.load(std::memory_order_relaxed);
  }
  uint32_t LastFifoCount() const { return (uint32_t)last_count_; }
  uint32_t LatestSeq() const {
    return tick_seq_.load(std::memory_order_relaxed);
  }
  bool IsInitialized() const { return initialized_; }

 private:
  Icm42688p() = default;
  ~Icm42688p() = default;
  Icm42688p(const Icm42688p &) = delete;
  Icm42688p &operator=(const Icm42688p &) = delete;

  void WriteReg(uint8_t reg, uint8_t val);
  uint8_t ReadReg(uint8_t reg);
  void SetBank(uint8_t bank);
  void CsLow();
  void CsHigh();

  void CheckWhoAmI();
  void ValidateConfig(const Config &cfg);
  void SoftReset();
  void SetClockSource();  // INTF_CONFIG1: AFSR off + CLKSEL=PLL
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
  void RecoverFromFifoFault();
  void HandleOverrunFault();

  static void SpiDoneThunk(void *user, bool ok);
  void OnSpiDone(bool ok);
  void PublishLatestBatch(const SampleBatch &batch);
  bool ParsePacket3Record(const uint8_t *rec, Sample &out);
  void UpdateTimestampAndSync(uint16_t ts16, uint64_t &out_host_us);
  // Packet3 record stride remains 16 bytes in FIFO stream layout.
  static constexpr uint16_t kPacketBytes = 16;
  static constexpr uint16_t kMaxReadBytes = kMaxWatermarkRecords * kPacketBytes;

  uint16_t fifo_wm_records_{0};
  uint8_t fifo_tx_[1 + kMaxReadBytes]{};
  uint8_t fifo_rx_[1 + kMaxReadBytes]{};

  std::atomic<bool> inflight_{false};
  std::atomic<uint32_t> overrun_{0};
  std::atomic<uint32_t> irq_cnt_{0};
  std::atomic<uint32_t> publish_cnt_{0};
  std::atomic<uint32_t> parse_fail_cnt_{0};
  std::atomic<uint32_t> dma_start_fail_cnt_{0};
  std::atomic<uint32_t> last_bad_header_{0};
  volatile uint64_t last_irq_us_{0};
  volatile uint16_t last_count_{0};

  uint16_t last_tmst16_{0};
  uint64_t tmst64_us_{0};
  bool tmst_inited_{false};

  int64_t host_offset_us_{0};
  bool host_sync_inited_{false};
  bool fifo_hold_last_data_en_{false};
  Icm42688pReg::GyroFs gyro_fs_{};
  Icm42688pReg::Odr gyro_odr_{};
  Config::Calibration calibration_cfg_{};
  Config::Recovery recovery_cfg_{};
  uint64_t last_overrun_fault_us_{0};
  uint32_t overrun_window_count_{0};
  std::atomic<bool> inject_overrun_fault_for_test_{false};

  bool initialized_{false};
  GPIO *gpio_{nullptr};
  Spi1 *spi_{nullptr};
  EE *ee_{nullptr};
  ee_schema::ImuAccelCalibration accel_calibration_{};

  alignas(8) SampleBatch published_batch_{};
  std::atomic<uint32_t> tick_seq_{0};
  std::atomic<uint32_t> drop_cnt_{0};
  uint32_t seq_{0};
};
