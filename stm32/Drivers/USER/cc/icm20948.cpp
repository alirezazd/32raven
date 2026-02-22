#include "icm20948.hpp"
#include "gpio.hpp"

#include "system.hpp"
#include <cstring> // for memcpy

using namespace InvenSense_ICM20948;

void Icm20948::SelectRegisterBank(uint8_t bank) {
  // if (bank != last_bank_) {
  WriteRegRaw(static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL), bank << 4);
  last_bank_ = bank;
  // }
}

void Icm20948::Init(GPIO &gpio, const Config &config) {
  if (initialized_)
    return;

  gpio_ = &gpio;
  config_ = config;

  // Power-up delay
  System::GetInstance().Time().DelayMicros(20000);

  // Force bank selection (Sensor might be in random bank after soft reboot)
  last_bank_ = 0xFF;

  // Force Bank 0
  SelectRegisterBank(0); // Uses WriteRegRaw

  // Device reset
  WriteReg(0, static_cast<uint8_t>(Register::BANK_0::PWR_MGMT_1),
           PWR_MGMT_1_BIT::DEVICE_RESET, false);

  // Poll until reset bit clears (timeout 100ms)
  {
    const uint64_t kStart = System::GetInstance().Time().Micros();
    while (true) {
      uint8_t pwr =
          ReadReg(0, static_cast<uint8_t>(Register::BANK_0::PWR_MGMT_1));
      if ((pwr & PWR_MGMT_1_BIT::DEVICE_RESET) == 0)
        break;
      if (System::GetInstance().Time().Micros() - kStart > 100000) {
        ErrorHandler();
        return;
      }
    }
  }

  // Wake up and set Clock Source
  {
    const uint64_t kStart = System::GetInstance().Time().Micros();
    while (true) {
      SelectRegisterBank(0);

      // Force: CLKSEL=1 (Auto), SLEEP=0
      WriteRegRaw(static_cast<uint8_t>(Register::BANK_0::PWR_MGMT_1),
                  PWR_MGMT_1_BIT::CLKSEL_0);
      System::GetInstance().Time().DelayMicros(1000);

      uint8_t pwr =
          ReadRegRaw(static_cast<uint8_t>(Register::BANK_0::PWR_MGMT_1));

      // Accept only if SLEEP cleared and CLKSEL == 1
      const bool kSleepCleared = ((pwr & PWR_MGMT_1_BIT::SLEEP) == 0);
      const bool kClkselOk = ((pwr & 0x07) == PWR_MGMT_1_BIT::CLKSEL_0);

      if (kSleepCleared && kClkselOk) {
        break;
      }

      if (System::GetInstance().Time().Micros() - kStart > 100000) {
        ErrorHandler();
      }
    }
  }

  // SPI-only mode (disable primary I2C)
  WriteReg(0, static_cast<uint8_t>(Register::BANK_0::USER_CTRL),
           USER_CTRL_BIT::I2C_IF_DIS, true, USER_CTRL_BIT::I2C_IF_DIS);

  // Interrupt pin behavior (INT_ANYRD_2CLEAR)
  WriteReg(0, static_cast<uint8_t>(Register::BANK_0::INT_PIN_CFG), 0x10, true);

  // Enable DRDY interrupt
  WriteReg(0, static_cast<uint8_t>(Register::BANK_0::INT_ENABLE_1),
           INT_ENABLE_1_BIT::RAW_DATA_0_RDY_EN, true);

  // Switch to Bank 2 and configure accel/gyro
  ConfigureAccel();
  ConfigureGyro();

  // Configure SPI Speed (Driver side)
  auto &spi = Spi1::GetInstance();
  spi.SetPrescaler(static_cast<SpiPrescaler>(config_.spi_prescaler));

  // Initialize DMA TX buffer to 0xFF (Idle MOSI)
  memset(dma_tx_buf_, 0xFF, kDmaBufSize);

  // Back to Bank 0
  SelectRegisterBank(0);

  // Read INT_STATUS (no verify)
  ReadReg(0, static_cast<uint8_t>(Register::BANK_0::INT_STATUS));

  initialized_ = true;

  /* EXTI interrupt init */
  NVIC_SetPriority(EXTI15_10_IRQn, 2);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Icm20948::ConfigureAccel() {
  // Accel Config
  WriteReg(2, static_cast<uint8_t>(Register::BANK_2::ACCEL_CONFIG),
           config_.accel.range | config_.accel.dlpf_config, true);

  // Accel Sample Rate Divider (SMPLRT_DIV_2)
  WriteReg(2, static_cast<uint8_t>(Register::BANK_2::ACCEL_SMPLRT_DIV_2),
           config_.accel.sample_rate_div, true);
}

void Icm20948::ConfigureGyro() {
  // Gyro Config
  WriteReg(2, static_cast<uint8_t>(Register::BANK_2::GYRO_CONFIG_1),
           config_.gyro.range | config_.gyro.dlpf_config, true);

  // Gyro Sample Rate Divider
  WriteReg(2, static_cast<uint8_t>(Register::BANK_2::GYRO_SMPLRT_DIV),
           config_.gyro.sample_rate_div, true);
}

extern "C" void Icm20948OnDrdyIrq(uint32_t timestamp) {
  Icm20948::GetInstance().OnDrdyIrq(timestamp);
}

void Icm20948::OnDrdyIrq(uint32_t timestamp) {
  last_drdy_time_ = timestamp;

  auto &spi = Spi1::GetInstance();
  if (spi.Busy()) {
    missed_drdy_count_++;
    return;
  }

  // Ensure we operate in Bank 0 for data reading.
  // Although Init() and Configure*() methods restore Bank 0,
  // we could enforce it here if robustness issues arise.

  // Prepare TX buffer for Burst Read
  // Address: ACCEL_XOUT_H (0x2D in Bank 0) | READ_BIT (0x80)
  // Burst Read Map (22 Bytes) + 1 Byte Header:
  // - Accel (6)
  // - Gyro (6)
  // - Temp (2)
  // - Mag (8) [EXT_SLV_SENS_DATA_00 - even if unused, must read for
  // INT_ANYRD_2CLEAR]

  static constexpr uint8_t kReadBit = 0x80;

  dma_tx_buf_[0] =
      static_cast<uint8_t>(Register::BANK_0::ACCEL_XOUT_H) | kReadBit;
  // Fill rest with 0? Spi driver handles it?
  // Existing code: `dma_tx_buf_[0] = ...;` then StartTxRxDma.
  // We need to set DMA buffer size correctly.
  static constexpr size_t kBurstSize = 22 + 1; // 1 byte cmd + 22 bytes data

  // Ensure kDmaBufSize is large enough (header says 32, so 23 is fine).

  // CS Low
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, false);

  // Start DMA
  bool started = spi.StartTxRxDma(dma_tx_buf_, dma_rx_buf_, kBurstSize,
                                  OnDmaComplete, this);
  if (!started) {
    gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, true);
    missed_drdy_count_++;
    return;
  }
}

void Icm20948::OnDmaComplete(void *user, bool ok) {
  Icm20948 *self = static_cast<Icm20948 *>(user);
  // CS High
  self->gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, true);

  if (ok) {
    // TODO: Parse data
    // Data in self->dma_rx_buf_[1] ...
  }
}

void Icm20948::WriteRegRaw(uint8_t reg, uint8_t val) {
  auto &spi = Spi1::GetInstance();
  uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};

  // CS Low
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, false);
  spi.Write(tx, 2);
  // CS High
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, true);
}

uint8_t Icm20948::ReadRegRaw(uint8_t reg) {
  auto &spi = Spi1::GetInstance();
  uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), 0x00};
  uint8_t rx[2] = {0, 0};

  // CS Low
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, false);
  spi.TxRx(tx, rx, 2);
  // CS High
  gpio_->WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, true);

  return rx[1];
}

void Icm20948::WriteReg(uint8_t bank, uint8_t reg, uint8_t val, bool verify,
                        uint8_t verify_mask) {
  uint64_t start = System::GetInstance().Time().Micros();

  while (true) {
    SelectRegisterBank(bank);
    WriteRegRaw(reg, val);

    if (!verify)
      return;

    // Digest time
    System::GetInstance().Time().DelayMicros(10);

    // Verify
    SelectRegisterBank(
        bank); // Ensuring bank wasn't switched by interrupt or other context
    uint8_t got = ReadRegRaw(reg);
    if ((got & verify_mask) == (val & verify_mask)) {
      break;
    }

    // Timeout check
    if (System::GetInstance().Time().Micros() - start > 100000) {
      ErrorHandler();
    }
  }
}

uint8_t Icm20948::ReadReg(uint8_t bank, uint8_t reg) {
  SelectRegisterBank(bank);
  return ReadRegRaw(reg);
}
