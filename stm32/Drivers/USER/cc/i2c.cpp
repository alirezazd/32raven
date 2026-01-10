#include "I2C.hpp"
#include "i2c.hpp"
#include "system.hpp"

// Define Global Legacy Handles for ISR/Extern compatibility
extern "C" {
I2C_HandleTypeDef hi2c1;
}

template <I2CInstance Inst> I2C<Inst>::I2C() {}

template <I2CInstance Inst> I2C<Inst>::~I2C() {}

template <I2CInstance Inst> I2C_HandleTypeDef *I2C<Inst>::GetHandle() {
  if (Inst == I2CInstance::kI2C1)
    return &hi2c1;
  ErrorHandler();
  return nullptr;
}

template <I2CInstance Inst> void I2C<Inst>::Init(const Config &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  I2C_HandleTypeDef *p_handle = GetHandle();

  if constexpr (Inst == I2CInstance::kI2C1) {
    p_handle->Instance = I2C1;
  }

  // Common configuration
  p_handle->Init.ClockSpeed = config.clockSpeed;
  p_handle->Init.DutyCycle = config.dutyCycle;
  p_handle->Init.OwnAddress1 = config.ownAddress1;
  p_handle->Init.AddressingMode = config.addressingMode;
  p_handle->Init.DualAddressMode = config.dualAddressMode;
  p_handle->Init.OwnAddress2 = config.ownAddress2;
  p_handle->Init.GeneralCallMode = config.generalCallMode;
  p_handle->Init.NoStretchMode = config.noStretchMode;

  if (HAL_I2C_Init(p_handle) != HAL_OK) {
    ErrorHandler();
  }
}

// Explicit Instantiation
template class I2C<I2CInstance::kI2C1>;
