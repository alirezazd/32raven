#include "icm20948.hpp"
#include "spi.hpp"
#include "system.hpp"
#include <cstdio>

namespace {
constexpr uint8_t kRegWhoAmI = 0x00;
constexpr uint8_t kRegPwrMgmt1 = 0x06;
constexpr uint8_t kRegUserCtrl = 0x03;
constexpr uint8_t kWhoAmIRes = 0xEA;
} // namespace

void Icm20948::Init() {
  if (initialized_)
    return;

  /* EXTI interrupt init */
  NVIC_SetPriority(EXTI15_10_IRQn, 4);
  NVIC_EnableIRQ(EXTI15_10_IRQn);

  // Power-up delay
  HAL_Delay(20);

  // 1. Wake up
  WriteReg(kRegPwrMgmt1, 0x01); // Auto Clock, Clear Sleep
  HAL_Delay(1);

  // 2. Disable I2C
  WriteReg(kRegUserCtrl, 0x10); // I2C_IF_DIS
  HAL_Delay(1);

  // 3. Configure Interrupts
  // INT_PIN_CFG (0x0F) = 0x10 (INT_ANYRD_2CLEAR)
  // This ensures the interrupt status is cleared when we read the data
  // registers (via DMA).
  WriteReg(0x0F, 0x10);

  // INT_ENABLE_1 (0x11) = 0x01 (RAW_DATA_0_RDY_EN)
  // Note: 0x10 is INT_ENABLE (I2C_MST, DMP, WOM), 0x11 is INT_ENABLE_1 (Raw
  // Data Ready)
  WriteReg(0x11, 0x01);
  HAL_Delay(1);

  // 3. Check WhoAmI
  uint8_t who_am_i = ReadReg(kRegWhoAmI);
  if (who_am_i != kWhoAmIRes) {
    printf("ICM20948 Init Failed: WhoAmI=0x%02X expected 0x%02X\n", who_am_i,
           kWhoAmIRes);
    return;
  }

  // 4. Speed up SPI to ~2.6MHz
  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  spi.SetPrescaler(SpiPrescaler::kDiv32);

  initialized_ = true;
  printf("ICM20948 Initialized. WhoAmI=0x%02X\n", who_am_i);
}

extern "C" void Icm20948OnDrdyIrq(uint32_t timestamp) {
  Icm20948::GetInstance().OnDrdyIrq(timestamp);
}

void Icm20948::OnDrdyIrq(uint32_t timestamp) {
  last_drdy_time_ = timestamp;

  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  if (spi.Busy()) {
    missed_drdy_count_++;
    return;
  }

  // Prepare TX buffer for Burst Read
  // Address: ACCEL_XOUT_H (0x2D) | READ_BIT (0x80)
  static constexpr uint8_t kRegAccelXOutH = 0x2D;
  static constexpr uint8_t kReadBit = 0x80;
  dma_tx_buf_[0] = kRegAccelXOutH | kReadBit;

  // CS Low
  SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin << 16;

  // Start DMA
  spi.StartTxRxDma(dma_tx_buf_, dma_rx_buf_, kDmaBufSize, OnDmaComplete, this);
}

void Icm20948::OnDmaComplete(void *user, bool ok) {
  // CS High
  SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin;

  // Process data if ok
  if (ok) {
    // TODO: Parse dma_rx_buf_
  }
}

void Icm20948::WriteReg(uint8_t reg, uint8_t val) {
  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};

  // CS Low
  SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin << 16;
  spi.Write(tx, 2);
  // CS High
  SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin;
}

uint8_t Icm20948::ReadReg(uint8_t reg) {
  auto &spi = Spi<SpiInstance::kSpi1>::GetInstance();
  uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), 0x00};
  uint8_t rx[2] = {0, 0};

  // CS Low
  SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin << 16;
  spi.TxRx(tx, rx, 2);
  // CS High
  SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin;

  return rx[1];
}
