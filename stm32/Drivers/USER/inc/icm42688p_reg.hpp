#pragma once

#include <cstdint>

namespace Icm42688pReg {

// Bank 0
constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
constexpr uint8_t REG_DRIVE_CONFIG = 0x13;
constexpr uint8_t REG_INT_CONFIG = 0x14;
constexpr uint8_t REG_FIFO_CONFIG = 0x16;
constexpr uint8_t REG_TEMP_DATA1 = 0x1D;
constexpr uint8_t REG_TEMP_DATA0 = 0x1E;
constexpr uint8_t REG_ACCEL_DATA_X1 = 0x1F;
constexpr uint8_t REG_INT_STATUS = 0x2D;
constexpr uint8_t REG_FIFO_COUNTH = 0x2E;
constexpr uint8_t REG_FIFO_COUNTL = 0x2F;
constexpr uint8_t REG_FIFO_DATA = 0x30;
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
constexpr uint8_t REG_APEX_CONFIG0 = 0x56;
constexpr uint8_t REG_SMD_CONFIG = 0x57;
constexpr uint8_t REG_FIFO_CONFIG1 = 0x5F;
constexpr uint8_t REG_FIFO_CONFIG2 = 0x60;
constexpr uint8_t REG_FIFO_CONFIG3 = 0x61;
constexpr uint8_t REG_FSYNC_CONFIG = 0x62;
constexpr uint8_t REG_INT_CONFIG0 = 0x63;
constexpr uint8_t REG_INT_CONFIG1 = 0x64;
constexpr uint8_t REG_INT_SOURCE0 = 0x65;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_BANK_SEL = 0x76;

// INT_CONFIG0 Bits (UI_DRDY_INT_CLEAR is bits[5:4])
constexpr uint8_t INT_CONFIG0_CLEAR_ON_STATUS_READ = 0x0u << 4; // 00
constexpr uint8_t INT_CONFIG0_CLEAR_ON_STATUS_READ_01 =
    0x1u << 4; // 01 (still status)
constexpr uint8_t INT_CONFIG0_CLEAR_ON_SENSOR_REG_READ = 0x2u << 4; // 10
constexpr uint8_t INT_CONFIG0_CLEAR_ON_BOTH = 0x3u << 4;            // 11

// Bank 1
constexpr uint8_t REG_GYRO_CONFIG_STATIC2 = 0x0B;
constexpr uint8_t REG_GYRO_CONFIG_STATIC3 = 0x0C;
constexpr uint8_t REG_GYRO_CONFIG_STATIC4 = 0x0D;
constexpr uint8_t REG_GYRO_CONFIG_STATIC5 = 0x0E;
constexpr uint8_t REG_GYRO_CONFIG_STATIC6 = 0x0F; // Gyro X NF COSWZ
constexpr uint8_t REG_GYRO_CONFIG_STATIC7 = 0x10; // Gyro Y NF COSWZ
constexpr uint8_t REG_GYRO_CONFIG_STATIC8 = 0x11; // Gyro Z NF COSWZ
constexpr uint8_t REG_GYRO_CONFIG_STATIC9 =
    0x12; // Gyro NF Bits (COSWZ_SEL etc)
constexpr uint8_t REG_GYRO_CONFIG_STATIC10 = 0x13; // Gyro NF BW
constexpr uint8_t REG_XG_ST_DATA = 0x5F;
constexpr uint8_t REG_YG_ST_DATA = 0x60;
constexpr uint8_t REG_ZG_ST_DATA = 0x61;
constexpr uint8_t REG_TMSTVAL0 = 0x62;
constexpr uint8_t REG_TMSTVAL1 = 0x63;
constexpr uint8_t REG_TMSTVAL2 = 0x64;
constexpr uint8_t REG_INTF_CONFIG4 = 0x7A;
constexpr uint8_t REG_INTF_CONFIG5 = 0x7B;
constexpr uint8_t REG_INTF_CONFIG6 = 0x7C;

// Bank 2
constexpr uint8_t REG_ACCEL_CONFIG_STATIC2 = 0x03;
constexpr uint8_t REG_ACCEL_CONFIG_STATIC3 = 0x04;
constexpr uint8_t REG_ACCEL_CONFIG_STATIC4 = 0x05;
constexpr uint8_t REG_XA_ST_DATA = 0x3B;
constexpr uint8_t REG_YA_ST_DATA = 0x3C;
constexpr uint8_t REG_ZA_ST_DATA = 0x3D;

// Values / Bits

// WHO_AM_I
constexpr uint8_t WHO_AM_I_VAL = 0x47;
constexpr uint8_t WHO_AM_I_VAL_686 = 0x44;

// ODR
enum class Odr : uint8_t {
  k32kHz = 0x01,
  k16kHz = 0x02,
  k8kHz = 0x03,
  k4kHz = 0x04,
  k2kHz = 0x05,
  k1kHz = 0x06,
  k200Hz = 0x07,
  k50Hz = 0x08,
  k12_5Hz = 0x09,
  k6_25Hz = 0x0A,
  k3_125Hz = 0x0B,
  k1_5625Hz = 0x0C,
  k500Hz = 0x0F,
};

// Full Scale
enum class GyroFs : uint8_t {
  k2000dps = 0x00,
  k1000dps = 0x01,
  k500dps = 0x02,
  k250dps = 0x03,
  k125dps = 0x04,
  k15_625dps = 0x05,
  k31_25dps = 0x06,
  k62_5dps = 0x07,
};

enum class AccelFs : uint8_t {
  k16g = 0x00,
  k8g = 0x01,
  k4g = 0x02,
  k2g = 0x03,
};

} // namespace Icm42688pReg
