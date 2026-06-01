#pragma once

// NOLINTBEGIN

#include <cstdint>

namespace Icm42688pReg {

// ─────────────────────────── Bank 0 registers ───────────────────────────

constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
constexpr uint8_t REG_BANK_SEL = 0x76;
constexpr uint8_t REG_DRIVE_CONFIG = 0x13;
constexpr uint8_t REG_FIFO_CONFIG = 0x16;
constexpr uint8_t REG_FIFO_CONFIG1 = 0x5F;
constexpr uint8_t REG_FIFO_CONFIG2 = 0x60;
constexpr uint8_t REG_FIFO_CONFIG3 = 0x61;
constexpr uint8_t REG_FIFO_COUNTH = 0x2E;
constexpr uint8_t REG_FIFO_COUNTL = 0x2F;
constexpr uint8_t REG_FIFO_DATA = 0x30;
constexpr uint8_t REG_TEMP_DATA1 = 0x1D;
constexpr uint8_t REG_TEMP_DATA0 = 0x1E;
constexpr uint8_t REG_ACCEL_DATA_X1 = 0x1F;
constexpr uint8_t REG_ACCEL_DATA_X0 = 0x20;
constexpr uint8_t REG_ACCEL_DATA_Y1 = 0x21;
constexpr uint8_t REG_ACCEL_DATA_Y0 = 0x22;
constexpr uint8_t REG_ACCEL_DATA_Z1 = 0x23;
constexpr uint8_t REG_ACCEL_DATA_Z0 = 0x24;
constexpr uint8_t REG_GYRO_DATA_X1 = 0x25;
constexpr uint8_t REG_GYRO_DATA_X0 = 0x26;
constexpr uint8_t REG_GYRO_DATA_Y1 = 0x27;
constexpr uint8_t REG_GYRO_DATA_Y0 = 0x28;
constexpr uint8_t REG_GYRO_DATA_Z1 = 0x29;
constexpr uint8_t REG_GYRO_DATA_Z0 = 0x2A;
constexpr uint8_t REG_INT_CONFIG = 0x14;
constexpr uint8_t REG_INT_STATUS = 0x2D;
constexpr uint8_t REG_INT_STATUS2 = 0x37;
constexpr uint8_t REG_INT_STATUS3 = 0x38;
constexpr uint8_t REG_SIGNAL_PATH_RESET = 0x4B;
constexpr uint8_t REG_INTF_CONFIG0 = 0x4C;
constexpr uint8_t REG_INTF_CONFIG1 = 0x4D;
constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
constexpr uint8_t REG_GYRO_CONFIG1 = 0x51;
constexpr uint8_t REG_GYRO_ACCEL_CONFIG0 = 0x52;
constexpr uint8_t REG_ACCEL_CONFIG1 = 0x53;
constexpr uint8_t REG_TMST_CONFIG = 0x54;
constexpr uint8_t REG_FSYNC_CONFIG = 0x62;

constexpr uint8_t REG_INT_CONFIG1 = 0x64;
constexpr uint8_t REG_INT_SOURCE0 = 0x65;
constexpr uint8_t REG_INT_SOURCE1 = 0x66;
constexpr uint8_t REG_INT_SOURCE3 = 0x68;
constexpr uint8_t REG_INT_SOURCE4 = 0x69;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_OFFSET_USER0 = 0x77;
constexpr uint8_t REG_OFFSET_USER1 = 0x78;
constexpr uint8_t REG_OFFSET_USER2 = 0x79;
constexpr uint8_t REG_OFFSET_USER3 = 0x7A;
constexpr uint8_t REG_OFFSET_USER4 = 0x7B;
constexpr uint8_t REG_OFFSET_USER5 = 0x7C;
constexpr uint8_t REG_OFFSET_USER6 = 0x7D;
constexpr uint8_t REG_OFFSET_USER7 = 0x7E;
constexpr uint8_t REG_OFFSET_USER8 = 0x7F;

// ─────────────────────────── Bank 1 registers ───────────────────────────

constexpr uint8_t REG_GYRO_CONFIG_STATIC2 = 0x0B;
constexpr uint8_t REG_GYRO_CONFIG_STATIC3 = 0x0C;
constexpr uint8_t REG_GYRO_CONFIG_STATIC4 = 0x0D;
constexpr uint8_t REG_GYRO_CONFIG_STATIC5 = 0x0E;
constexpr uint8_t REG_GYRO_CONFIG_STATIC6 = 0x0F;
constexpr uint8_t REG_GYRO_CONFIG_STATIC7 = 0x10;
constexpr uint8_t REG_GYRO_CONFIG_STATIC8 = 0x11;
constexpr uint8_t REG_GYRO_CONFIG_STATIC9 = 0x12;
constexpr uint8_t REG_GYRO_CONFIG_STATIC10 = 0x13;
constexpr uint8_t REG_INTF_CONFIG5 = 0x7B;

// ─────────────────────────── Bank 2 registers ───────────────────────────

constexpr uint8_t REG_ACCEL_CONFIG_STATIC2 = 0x03;
constexpr uint8_t REG_ACCEL_CONFIG_STATIC3 = 0x04;
constexpr uint8_t REG_ACCEL_CONFIG_STATIC4 = 0x05;

// ─────────────────────────── Bit definitions ────────────────────────────

// DEVICE_CONFIG (0x11)
constexpr uint8_t DEVICE_CONFIG_SOFT_RESET = 1u << 0;

// INT_CONFIG (0x14)
constexpr uint8_t INT_CONFIG_INT1_MODE = 1u << 2;  // 0=Pulsed, 1=Latched
constexpr uint8_t INT_CONFIG_INT1_DRIVE_CIRCUIT =
    1u << 1;  // 0=OpenDrain, 1=PushPull
constexpr uint8_t INT_CONFIG_INT1_POLARITY = 1u
                                             << 0;  // 0=ActiveLow, 1=ActiveHigh

// FIFO_CONFIG (0x16)
constexpr uint8_t FIFO_CONFIG_MODE_BYPASS = 0x0u << 6;
constexpr uint8_t FIFO_CONFIG_MODE_STREAM = 0x1u << 6;
constexpr uint8_t FIFO_CONFIG_MODE_STOP_ON_FULL = 0x2u << 6;

// INT_STATUS (0x2D)
constexpr uint8_t INT_STATUS_DATA_RDY = 1u << 3;
constexpr uint8_t INT_STATUS_FIFO_THS_INT = 1u << 2;
constexpr uint8_t INT_STATUS_FIFO_FULL_INT = 1u << 1;

// INTF_CONFIG0 (0x4C)
constexpr uint8_t INTF_CONFIG0_FIFO_COUNT_REC_REC = 1u << 6;   // Record mode
constexpr uint8_t INTF_CONFIG0_FIFO_COUNT_REC_BYTE = 0u << 6;  // Byte mode
constexpr uint8_t INTF_CONFIG0_FIFO_COUNT_ENDIAN_LITTLE = 0u << 5;
constexpr uint8_t INTF_CONFIG0_FIFO_COUNT_ENDIAN_BIG = 1u << 5;
constexpr uint8_t INTF_CONFIG0_SENSOR_DATA_ENDIAN_LITTLE = 0u << 4;
constexpr uint8_t INTF_CONFIG0_SENSOR_DATA_ENDIAN_BIG = 1u << 4;

constexpr uint8_t INTF_CONFIG0_UI_SIFS_CFG_MASK = 0x03u;         // bits [1:0]
constexpr uint8_t INTF_CONFIG0_UI_SIFS_CFG_DISABLE_SPI = 0x02u;  // 10b
constexpr uint8_t INTF_CONFIG0_UI_SIFS_CFG_DISABLE_I2C = 0x03u;  // 11b

constexpr uint8_t INTF_CONFIG1_RTC_MODE_MASK = 0x04u;
constexpr uint8_t INTF_CONFIG1_RTC_MODE_EN = 0x04u;
constexpr uint8_t INTF_CONFIG1_RTC_MODE_DIS = 0x00u;

constexpr uint8_t INTF_CONFIG1_CLKSEL_MASK = 0x03u;
constexpr uint8_t INTF_CONFIG1_CLKSEL_RC = 0x00u;
constexpr uint8_t INTF_CONFIG1_CLKSEL_PLL = 0x01u;
constexpr uint8_t INTF_CONFIG1_CLKSEL_DIS = 0x03u;

constexpr uint8_t INTF_CONFIG1_ACCEL_LP_CLK_SEL = 1u << 3;

// PWR_MGMT0 (0x4E)
constexpr uint8_t PWR_MGMT0_TEMP_DIS = 1u << 5;  // 1=temperature sensor off
constexpr uint8_t PWR_MGMT0_GYRO_MODE_LN = 0x3u << 2;   // Low Noise
constexpr uint8_t PWR_MGMT0_ACCEL_MODE_LN = 0x3u << 0;  // Low Noise

// TMST_CONFIG (0x54)
constexpr uint8_t TMST_CONFIG_TO_REGS_EN = 1u
                                           << 4;  // Timestamp readable via regs
constexpr uint8_t TMST_CONFIG_RES = 1u << 3;      // 0=1µs, 1=16µs/RTC
constexpr uint8_t TMST_CONFIG_DELTA_EN = 1u << 2;  // Delta timestamp
constexpr uint8_t TMST_CONFIG_FSYNC_EN = 1u << 1;  // FSYNC timestamp latch
constexpr uint8_t TMST_CONFIG_EN = 1u << 0;        // Master enable

// FIFO_CONFIG1 bit definitions (datasheet-correct)
constexpr uint8_t FIFO_CONFIG1_ACCEL_EN = 1u << 0;
constexpr uint8_t FIFO_CONFIG1_GYRO_EN = 1u << 1;
constexpr uint8_t FIFO_CONFIG1_TEMP_EN = 1u << 2;
constexpr uint8_t FIFO_CONFIG1_TMST_FSYNC_EN = 1u << 3;
constexpr uint8_t FIFO_CONFIG1_HIRES_EN = 1u << 4;
constexpr uint8_t FIFO_CONFIG1_WM_GT_TH = 1u << 5;
constexpr uint8_t FIFO_CONFIG1_RESUME_PARTIAL_RD = 1u << 6;

// FIFO header byte field definitions (datasheet §6.2). The 1-byte header
// at offset 0 of every FIFO packet encodes packet variant + ODR-change
// flags. The driver gates parsing on the variant detected here.
constexpr uint8_t FIFO_HEADER_MSG = 1u << 7;    // 1 = no sensor data
constexpr uint8_t FIFO_HEADER_ACCEL = 1u << 6;  // accel payload present
constexpr uint8_t FIFO_HEADER_GYRO = 1u << 5;   // gyro payload present
constexpr uint8_t FIFO_HEADER_20 = 1u << 4;     // 20-bit ext present (Packet4)
constexpr uint8_t FIFO_HEADER_TIMESTAMP_MASK = 0x0Cu;  // bits[3:2]
constexpr uint8_t FIFO_HEADER_TIMESTAMP_ODR = 0x08u;   // 10b → ODR timestamp
constexpr uint8_t FIFO_HEADER_TIMESTAMP_FSYNC =
    0x0Cu;  // 11b → first ODR after FSYNC
constexpr uint8_t FIFO_HEADER_ODR_DIFF_ACCEL = 1u << 1;  // accel ODR changed
constexpr uint8_t FIFO_HEADER_ODR_DIFF_GYRO = 1u << 0;   // gyro ODR changed

// Mask + nominal header values for Packet3 (16-byte) and Packet4 (20-byte
// HiRes). Masks the top 5 bits; ODR-change flags (bits[1:0]) are ignored.
//   Packet3: MSG=0, ACCEL=1, GYRO=1, 20=0, TS=ODR → 0x68
//   Packet4: MSG=0, ACCEL=1, GYRO=1, 20=1, TS=ODR → 0x78
constexpr uint8_t kFifoHeaderVariantMask = 0xF8u;
constexpr uint8_t kFifoHeaderPacket3 = 0x68u;
constexpr uint8_t kFifoHeaderPacket4 = 0x78u;

// FSYNC_CONFIG (0x62)
constexpr uint8_t FSYNC_CONFIG_UI_SEL_MASK = 0x70;  // bits [6:4]
constexpr uint8_t FSYNC_CONFIG_UI_SEL_SHIFT = 4;
constexpr uint8_t FSYNC_CONFIG_POLARITY = 1u << 0;
constexpr uint8_t FSYNC_UI_SEL_DISABLE = 0x0;  // 000 = no tagging

// INT_CONFIG0 (0x63) — UI_DRDY_INT_CLEAR bits [5:4]
constexpr uint8_t REG_INT_CONFIG0 = 0x63;
constexpr uint8_t INT_CONFIG0_DRDY_CLEAR_ON_STATUS_READ = 0x0u << 4;
constexpr uint8_t INT_CONFIG0_DRDY_CLEAR_ON_SENSOR_READ = 0x2u << 4;
constexpr uint8_t INT_CONFIG0_DRDY_CLEAR_ON_BOTH = 0x3u << 4;
constexpr uint8_t INT_CONFIG0_FIFO_THS_CLEAR_ON_STATUS_READ = 0x0u << 2;
constexpr uint8_t INT_CONFIG0_FIFO_THS_CLEAR_ON_FIFO_READ = 0x2u << 2;
constexpr uint8_t INT_CONFIG0_FIFO_THS_CLEAR_ON_BOTH = 0x3u << 2;
constexpr uint8_t INT_CONFIG0_FIFO_FULL_CLEAR_ON_STATUS_READ = 0x0u << 0;
constexpr uint8_t INT_CONFIG0_FIFO_FULL_CLEAR_ON_FIFO_READ = 0x2u << 0;
constexpr uint8_t INT_CONFIG0_FIFO_FULL_CLEAR_ON_BOTH = 0x3u << 0;

// INT_CONFIG1 (0x64)
constexpr uint8_t INT_CONFIG1_TPULSE_DURATION =
    1u << 6;  // 0=100µs (default), 1=8µs
constexpr uint8_t INT_CONFIG1_TDEASSERT_DIS = 1u << 5;  // Disable deassertion
constexpr uint8_t INT_CONFIG1_ASYNC_RESET =
    1u << 4;  // 1=Reset (default), 0=Enable proper INT pin function

// INT_SOURCE0 (0x65)
constexpr uint8_t INT_SOURCE0_UI_FSYNC_INT1_EN = 1u << 6;
constexpr uint8_t INT_SOURCE0_PLL_RDY_INT1_EN = 1u << 5;
constexpr uint8_t INT_SOURCE0_RESET_DONE_INT1_EN = 1u << 4;
constexpr uint8_t INT_SOURCE0_UI_DRDY_INT1_EN = 1u << 3;
constexpr uint8_t INT_SOURCE0_FIFO_THS_INT1_EN = 1u << 2;
constexpr uint8_t INT_SOURCE0_FIFO_FULL_INT1_EN = 1u << 1;

// GYRO_CONFIG_STATIC2 (Bank 1, 0x0B)
constexpr uint8_t GYRO_CONFIG_STATIC2_AAF_DIS = 1u << 1;
constexpr uint8_t GYRO_CONFIG_STATIC2_NF_DIS = 1u << 0;

// INTF_CONFIG5 (Bank 1, 0x7B) — PIN9_FUNCTION bits [2:1]
//   00 = FSYNC (default), 10 = CLKIN
constexpr uint8_t INTF_CONFIG5_PIN9_FUNCTION_MASK = 0x06u;
constexpr uint8_t INTF_CONFIG5_PIN9_FUNCTION_FSYNC = 0x00u;
constexpr uint8_t INTF_CONFIG5_PIN9_FUNCTION_CLKIN = 0x04u;

// SIGNAL_PATH_RESET (0x4B)
constexpr uint8_t SIGNAL_PATH_RESET_DMP_MEM_RESET_EN = 1u << 5;
constexpr uint8_t SIGNAL_PATH_RESET_ABORT_AND_RESET = 1u << 3;
constexpr uint8_t SIGNAL_PATH_RESET_TMST_STROBE = 1u << 2;
constexpr uint8_t SIGNAL_PATH_RESET_FIFO_FLUSH = 1u << 1;

// ACCEL_CONFIG_STATIC2 (Bank 2, 0x03)
constexpr uint8_t ACCEL_CONFIG_STATIC2_AAF_DIS = 1u << 0;

// CLKDIV (Bank 3, 0x2A)
constexpr uint8_t REG_CLKDIV = 0x2A;

// ─────────────────────────── WHO_AM_I values ────────────────────────────

constexpr uint8_t WHO_AM_I_ICM42688P = 0x47;
constexpr uint8_t WHO_AM_I_ICM42686P = 0x44;

// ─────────────────────────── Enumerations ───────────────────────────────

enum class Odr : uint8_t {
  k32kHz = 0x01,
  k16kHz = 0x02,
  k8kHz = 0x03,
  k4kHz = 0x04,
  k2kHz = 0x05,
  k1kHz = 0x06,
  k200Hz = 0x07,
  k100Hz = 0x08,
  k50Hz = 0x09,
  k25Hz = 0x0A,
  k12_5Hz = 0x0B,
  k500Hz = 0x0F,
};

enum class GyroFs : uint8_t {
  k2000dps = 0x00,
  k1000dps = 0x01,
  k500dps = 0x02,
  k250dps = 0x03,
  k125dps = 0x04,
  k62_5dps = 0x05,
  k31_25dps = 0x06,
  k15_625dps = 0x07,
};

enum class AccelFs : uint8_t {
  k16g = 0x00,
  k8g = 0x01,
  k4g = 0x02,
  k2g = 0x03,
};

constexpr float kStandardGravityMps2 = 9.80665f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kFifoDataLsb = 32768.0f;

static constexpr float GyroRangeDps(GyroFs fs) {
  switch (fs) {
    case GyroFs::k2000dps:
      return 2000.0f;
    case GyroFs::k1000dps:
      return 1000.0f;
    case GyroFs::k500dps:
      return 500.0f;
    case GyroFs::k250dps:
      return 250.0f;
    case GyroFs::k125dps:
      return 125.0f;
    case GyroFs::k62_5dps:
      return 62.5f;
    case GyroFs::k31_25dps:
      return 31.25f;
    case GyroFs::k15_625dps:
      return 15.625f;
    default:
      return 2000.0f;
  }
}

static constexpr float AccelRangeG(AccelFs fs) {
  switch (fs) {
    case AccelFs::k16g:
      return 16.0f;
    case AccelFs::k8g:
      return 8.0f;
    case AccelFs::k4g:
      return 4.0f;
    case AccelFs::k2g:
      return 2.0f;
    default:
      return 16.0f;
  }
}

// Datasheet conversion for the 16-bit FIFO accel samples.
static constexpr float AccelLsbToMps2(AccelFs fs) {
  return (AccelRangeG(fs) / kFifoDataLsb) * kStandardGravityMps2;
}

// Datasheet conversion for the 16-bit FIFO gyro samples.
static constexpr float GyroLsbToRadS(GyroFs fs) {
  return (GyroRangeDps(fs) / kFifoDataLsb) * kDegToRad;
}

// Datasheet conversion for the 8-bit FIFO temperature sample.
static constexpr float FifoTemperatureC(int8_t fifo_temp_data) {
  return (static_cast<float>(fifo_temp_data) / 2.07f) + 25.0f;
}

// ── FIFO sample ranges (datasheet §12.8) ───────────────────────────────
// With FIFO_HOLD_LAST_DATA_EN=0, the chip inserts an invalid-sample
// sentinel when no fresh data is available; the firmware Panics on
// detection (Stm32::kImuInvalidSampleDetected). Valid samples are
// constrained to the ranges below — the encoder side (SIL chip slave /
// future Packet4 writer) saturates to these bounds, never the raw
// int16/int20 limits.
//
// In 20-bit mode the chip enforces LSB constraints:
//   gyro  — LSB[0]   = 0  (values are EVEN, gap = 2)
//   accel — LSB[1:0] = 0  (values are multiples of 4, gap = 4)
constexpr int16_t kFifo16ValidMin = -32767;
constexpr int16_t kFifo16ValidMax = +32767;
constexpr int16_t kFifo16InvalidSentinel = -32768;

constexpr int32_t kFifo20GyroValidMin = -524256;
constexpr int32_t kFifo20GyroValidMax = +524286;
constexpr int32_t kFifo20AccelValidMin = -524256;
constexpr int32_t kFifo20AccelValidMax = +524284;
constexpr int32_t kFifo20InvalidSentinel = -524288;

// ── 20-bit HiRes (Packet4) constants ───────────────────────────────────
// HiRes mode forces ±2000 dps + ±16g (datasheet §6.1 — other FSR
// settings are ignored when FIFO_HIRES_EN=1). The "actual data" widths
// are 19-bit gyro (LSB[0]=0) and 18-bit accel (LSBs[1:0]=0); the FIFO
// emits the full 20-bit signed value with those LSBs forced low.
//
// Sensitivity factors are referenced to the raw 20-bit value: divide
// the int32 FIFO sample by the constants below to get physical units.
//   "the corresponding sensitivity scale factor values are 131 LSB/dps
//    for gyroscope and 8192 LSB/g for accelerometer." (datasheet §6.1)
//
// AAF + Notch require LN power mode (datasheet §12.4 — "Use of Notch
// Filter and Anti-Alias Filter is supported only for Low Noise (LN)
// mode operation"). The current PWR_MGMT0 = GYRO_MODE_LN|ACCEL_MODE_LN
// in firmware satisfies this; do not drop to LP while AAF/Notch are
// enabled.
constexpr float kHiResGyroLsbPerDps = 131.0f;
constexpr float kHiResAccelLsbPerG = 8192.0f;

// HiRes accel / gyro LSB → physical conversions. Take the 20-bit signed
// raw int (stored in an int32_t) and return SI units. These do NOT take
// the FS enum because HiRes locks the chip to ±2000 dps + ±16g.
inline constexpr float AccelLsbToMps2_HiRes() {
  return (1.0f / kHiResAccelLsbPerG) * kStandardGravityMps2;
}
inline constexpr float GyroLsbToRadS_HiRes() {
  return (1.0f / kHiResGyroLsbPerDps) * kDegToRad;
}

// 16-bit FIFO temperature decode for Packet4 (the field widens from
// 8 bits in Packet3 to 16 bits in Packet4; offset stays at 25 °C, scale
// factor is 64× finer — matching TDK / InvenSense reference firmware).
inline constexpr float FifoTemperatureC16(int16_t fifo_temp_data) {
  return (static_cast<float>(fifo_temp_data) / 132.48f) + 25.0f;
}

// ── FIFO packet sizes (datasheet §6.1, Figure 10) ──────────────────────
// Driver allocates DMA + parses by these. Packet3 / Packet4 are the
// only variants the firmware uses; Packet1/2 (accel-only / gyro-only)
// stay defined here for completeness but aren't on the driver path.
constexpr uint16_t kPacket1Bytes = 8;
constexpr uint16_t kPacket2Bytes = 8;
constexpr uint16_t kPacket3Bytes = 16;
constexpr uint16_t kPacket4Bytes = 20;

static constexpr uint32_t OdrHz(Odr odr) {
  switch (odr) {
    case Odr::k32kHz:
      return 32000;
    case Odr::k16kHz:
      return 16000;
    case Odr::k8kHz:
      return 8000;
    case Odr::k4kHz:
      return 4000;
    case Odr::k2kHz:
      return 2000;
    case Odr::k1kHz:
      return 1000;
    case Odr::k500Hz:
      return 500;
    case Odr::k200Hz:
      return 200;
    case Odr::k100Hz:
      return 100;
    case Odr::k50Hz:
      return 50;
    case Odr::k25Hz:
      return 25;
    case Odr::k12_5Hz:
      return 13;
    default:
      return 0;
  }
}

}  // namespace Icm42688pReg

// NOLINTEND
