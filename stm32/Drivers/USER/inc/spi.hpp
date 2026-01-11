#ifndef USER_SPI_HPP
#define USER_SPI_HPP

#include "stm32f4xx_hal.h"

class Spi {
public:
  static Spi &GetInstance() {
    static Spi instance;
    return instance;
  }

private:
  friend class System;
  void Init();

  Spi() = default;
};

#endif // USER_SPI_HPP
