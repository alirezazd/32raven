#ifndef USER_DRIVERS_I2C_HPP
#define USER_DRIVERS_I2C_HPP

#include "stm32f4xx_hal.h"

enum class I2CInstance { I2C_1, I2C_3 };

// Common Config for all I2C instances
struct I2CConfig {
  uint32_t clockSpeed;
  uint32_t dutyCycle;
  uint32_t ownAddress1;
  uint32_t addressingMode;
  uint32_t dualAddressMode;
  uint32_t ownAddress2;
  uint32_t generalCallMode;
  uint32_t noStretchMode;
};

template <I2CInstance Inst> class I2C {
public:
  using Config = I2CConfig;

  static I2C &getInstance() {
    static_assert(
        Inst == I2CInstance::I2C_1 || Inst == I2CInstance::I2C_3,
        "Unsupported I2C Instance. Only I2C1 and I2C3 are supported.");
    static I2C instance;
    return instance;
  }

private:
  friend class System;
  static void init(const Config &config) { getInstance()._init(config); }

  // static void init(const Config &config) { getInstance()._init(config); }

private:
  I2C();
  ~I2C();

  // Prevent copying
  I2C(const I2C &) = delete;
  I2C &operator=(const I2C &) = delete;

  void _init(const Config &config);
  I2C_HandleTypeDef *_getHandle();
  bool initialized_ = false;
};

#endif // USER_DRIVERS_I2C_HPP
