/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file InvenSense_ICM20948_registers.hpp
 *
 * Invensense ICM-20948 registers.
 *
 */

// NOLINTBEGIN(readability-identifier-naming)

#pragma once

#include <cstddef>
#include <cstdint>

namespace InvenSense_ICM20948 {
// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace Register {

enum class BANK_0 : uint8_t {
  WHO_AM_I = 0x00,

  USER_CTRL = 0x03,

  PWR_MGMT_1 = 0x06,

  INT_PIN_CFG = 0x0F,
  INT_ENABLE_1 = 0x11,
  INT_STATUS = 0x19, // Restored for interrupt clearing

  I2C_MST_STATUS = 0x17,

  ACCEL_XOUT_H = 0x2D,

  TEMP_OUT_H = 0x39,
  TEMP_OUT_L = 0x3A,
  EXT_SLV_SENS_DATA_00 = 0x3B,

  FIFO_EN_2 = 0x67,
  FIFO_RST = 0x68,
  FIFO_MODE = 0x69,
  FIFO_COUNTH = 0x70,
  FIFO_COUNTL = 0x71,
  FIFO_R_W = 0x72,

  FIFO_CFG = 0x76,

  REG_BANK_SEL = 0x7F,
};

enum class BANK_2 : uint8_t {
  GYRO_SMPLRT_DIV = 0x00,
  GYRO_CONFIG_1 = 0x01,

  ACCEL_SMPLRT_DIV_2 = 0x11,

  ACCEL_CONFIG = 0x14,

  REG_BANK_SEL = 0x7F,
};

}; // namespace Register

//---------------- BANK0 Register bits
// USER_CTRL
enum USER_CTRL_BIT : uint8_t {
  DMP_EN = Bit7,
  FIFO_EN = Bit6,
  I2C_IF_DIS = Bit4, // Reset I2C Slave module and put the serial interface in
                     // SPI mode only
  DMP_RST = Bit3,    // Reset DMP module.
  SRAM_RST = Bit2,   // Reset SRAM module.
  // I2C_MST_RST removed
};

// PWR_MGMT_1
enum PWR_MGMT_1_BIT : uint8_t {
  DEVICE_RESET = Bit7,
  SLEEP = Bit6,

  TEMP_DIS = Bit3,
  CLKSEL_2 = Bit2,
  CLKSEL_1 = Bit1,
  CLKSEL_0 = Bit0,
};

// INT_PIN_CFG
enum INT_PIN_CFG_BIT : uint8_t {
  INT1_ACTL = Bit7,
  // BYPASS_EN removed (We use SPI, not I2C Passthrough)
};

// INT_ENABLE_1
enum INT_ENABLE_1_BIT : uint8_t {
  RAW_DATA_0_RDY_EN = Bit0,
};

// FIFO_EN_2
enum FIFO_EN_2_BIT : uint8_t {
  ACCEL_FIFO_EN = Bit4,
  GYRO_Z_FIFO_EN = Bit3,
  GYRO_Y_FIFO_EN = Bit2,
  GYRO_X_FIFO_EN = Bit1,
  TEMP_FIFO_EN = Bit0,
};

// FIFO_RST
enum FIFO_RST_BIT : uint8_t {
  FIFO_RESET = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

// FIFO_MODE
enum FIFO_MODE_BIT : uint8_t {
  Snapshot = Bit0,
};

// FIFO_CFG
enum FIFO_CFG_BIT : uint8_t {
  FIFO_CFG = Bit0,
};

// REG_BANK_SEL
enum REG_BANK_SEL_BIT : uint8_t {
  USER_BANK_0 = 0,           // 0: Select USER BANK 0.
  USER_BANK_1 = Bit4,        // 1: Select USER BANK 1.
  USER_BANK_2 = Bit5,        // 2: Select USER BANK 2.
  USER_BANK_3 = Bit5 | Bit4, // 3: Select USER BANK 3.
};

//---------------- BANK2 Register bits
// GYRO_CONFIG_1
enum GYRO_CONFIG_1_BIT : uint8_t {
  // 5:3 GYRO_DLPFCFG[2:0]
  GYRO_DLPFCFG = Bit5 | Bit4 | Bit3, // 7 -

  // 2:1 GYRO_FS_SEL[1:0]
  GYRO_FS_SEL_250_DPS = 0,            // 0b00 = ±250 dps
  GYRO_FS_SEL_500_DPS = Bit1,         // 0b01 = ±500 dps
  GYRO_FS_SEL_1000_DPS = Bit2,        // 0b10 = ±1000 dps
  GYRO_FS_SEL_2000_DPS = Bit2 | Bit1, // 0b11 = ±2000 dps

  GYRO_FCHOICE = Bit0, // 0 – Bypass gyro DLPF
};

// ACCEL_CONFIG
enum ACCEL_CONFIG_BIT : uint8_t {
  // 5:3 ACCEL_DLPFCFG[2:0]
  ACCEL_DLPFCFG = Bit5 | Bit4 | Bit3, // 7 -

  // 2:1 ACCEL_FS_SEL[1:0]
  ACCEL_FS_SEL_2G = 0,            // 0b00: ±2g
  ACCEL_FS_SEL_4G = Bit1,         // 0b01: ±4g
  ACCEL_FS_SEL_8G = Bit2,         // 0b10: ±8g
  ACCEL_FS_SEL_16G = Bit2 | Bit1, // 0b11: ±16g

  ACCEL_FCHOICE = Bit0, // 0: Bypass accel DLPF
};

namespace FIFO {
static constexpr size_t SIZE = 512;

// FIFO_DATA layout when FIFO_EN has ACCEL_FIFO_EN and GYRO_{Z, Y, X}_FIFO_EN
// set
struct DATA {
  uint8_t ACCEL_XOUT_H;
  uint8_t ACCEL_XOUT_L;
  uint8_t ACCEL_YOUT_H;
  uint8_t ACCEL_YOUT_L;
  uint8_t ACCEL_ZOUT_H;
  uint8_t ACCEL_ZOUT_L;
  uint8_t GYRO_XOUT_H;
  uint8_t GYRO_XOUT_L;
  uint8_t GYRO_YOUT_H;
  uint8_t GYRO_YOUT_L;
  uint8_t GYRO_ZOUT_H;
  uint8_t GYRO_ZOUT_L;
};
} // namespace FIFO
} // namespace InvenSense_ICM20948

// NOLINTEND(readability-identifier-naming)
