#pragma once

#include <cstdint>

namespace Icm42688pReg {

// ─────────────────────────── Bank 0 registers ───────────────────────────

constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
constexpr uint8_t REG_DRIVE_CONFIG = 0x13;
constexpr uint8_t REG_INT_CONFIG = 0x14;
constexpr uint8_t REG_TEMP_DATA1 = 0x1D;
constexpr uint8_t REG_TEMP_DATA0 = 0x1E;
constexpr uint8_t REG_ACCEL_DATA_X1 = 0x1F;
constexpr uint8_t REG_INT_STATUS = 0x2D;
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
constexpr uint8_t REG_INT_CONFIG0 = 0x63;
constexpr uint8_t REG_INT_CONFIG1 = 0x64;
constexpr uint8_t REG_INT_SOURCE0 = 0x65;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_BANK_SEL = 0x76;

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
constexpr uint8_t INT_CONFIG_INT1_MODE = 1u << 2; // 0=Pulsed, 1=Latched
constexpr uint8_t INT_CONFIG_INT1_DRIVE_CIRCUIT =
    1u << 1; // 0=OpenDrain, 1=PushPull
constexpr uint8_t INT_CONFIG_INT1_POLARITY = 1u
                                             << 0; // 0=ActiveLow, 1=ActiveHigh

// INT_STATUS (0x2D)
constexpr uint8_t INT_STATUS_DATA_RDY = 1u << 3;

// INTF_CONFIG1 (0x4D)
//   Bits [7:6] AFSR: 10=ON (default), 01=OFF
//   Bit  [2]   RTC_MODE
//   Bits [1:0] CLKSEL: 00=RC only, 01=PLL auto-select
constexpr uint8_t INTF_CONFIG1_AFSR_CLEAR =
    1u << 7; // Set to enable AFSR (default)
constexpr uint8_t INTF_CONFIG1_AFSR_SET = 1u << 6; // Set to disable AFSR
constexpr uint8_t INTF_CONFIG1_RTC_MODE = 1u << 2;
constexpr uint8_t INTF_CONFIG1_CLKSEL = 1u << 0;
constexpr uint8_t INTF_CONFIG1_CLKSEL_CLEAR = 1u << 1;

// PWR_MGMT0 (0x4E)
constexpr uint8_t PWR_MGMT0_GYRO_MODE_LN = 0x3u << 2;  // Low Noise
constexpr uint8_t PWR_MGMT0_ACCEL_MODE_LN = 0x3u << 0; // Low Noise

// TMST_CONFIG (0x54)
constexpr uint8_t TMST_CONFIG_TO_REGS_EN = 1u
                                           << 4;  // Timestamp readable via regs
constexpr uint8_t TMST_CONFIG_RES = 1u << 3;      // 0=1µs, 1=16µs/RTC
constexpr uint8_t TMST_CONFIG_DELTA_EN = 1u << 2; // Delta timestamp
constexpr uint8_t TMST_CONFIG_FSYNC_EN = 1u << 1; // FSYNC timestamp latch
constexpr uint8_t TMST_CONFIG_EN = 1u << 0;       // Master enable

// FSYNC_CONFIG (0x62)
//   Bits [6:4] UI_SEL: which sensor register gets tagged
//     0x4 = route FSYNC to TMST (no data tagging)
//   Bit  [0]   POLARITY: 0=rising, 1=falling

// INT_CONFIG0 (0x63) — UI_DRDY_INT_CLEAR bits [5:4]
constexpr uint8_t INT_CONFIG0_DRDY_CLEAR_ON_STATUS_READ = 0x0u << 4;
constexpr uint8_t INT_CONFIG0_DRDY_CLEAR_ON_SENSOR_READ = 0x2u << 4;
constexpr uint8_t INT_CONFIG0_DRDY_CLEAR_ON_BOTH = 0x3u << 4;

// INT_CONFIG1 (0x64)
constexpr uint8_t INT_CONFIG1_TPULSE_DURATION = 1u << 6; // 0=100ns, 1=8µs
constexpr uint8_t INT_CONFIG1_TDEASSERT_DIS = 1u << 5;   // Disable deassertion
constexpr uint8_t INT_CONFIG1_ASYNC_RESET = 1u
                                            << 4; // Must clear for proper INT

// INT_SOURCE0 (0x65)
constexpr uint8_t INT_SOURCE0_UI_FSYNC_INT1_EN = 1u << 6;
constexpr uint8_t INT_SOURCE0_PLL_RDY_INT1_EN = 1u << 5;
constexpr uint8_t INT_SOURCE0_RESET_DONE_INT1_EN = 1u << 4;
constexpr uint8_t INT_SOURCE0_UI_DRDY_INT1_EN = 1u << 3;

// GYRO_CONFIG_STATIC2 (Bank 1, 0x0B)
constexpr uint8_t GYRO_CONFIG_STATIC2_AAF_DIS = 1u << 1;
constexpr uint8_t GYRO_CONFIG_STATIC2_NF_DIS = 1u << 0;

// INTF_CONFIG5 (Bank 1, 0x7B) — PIN9_FUNCTION bits [2:1]
//   00 = FSYNC (default), 10 = CLKIN
constexpr uint8_t INTF_CONFIG5_PIN9_CLKIN = 1u << 2;

// ACCEL_CONFIG_STATIC2 (Bank 2, 0x03)
constexpr uint8_t ACCEL_CONFIG_STATIC2_AAF_DIS = 1u << 0;

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

} // namespace Icm42688pReg
