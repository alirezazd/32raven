#pragma once
#include "programmer.hpp"
#include "system.hpp"
#include "uart.hpp"
#include <cstdint>

class EP2 {
public:
  static EP2 &GetInstance() {
    static EP2 instance;
    return instance;
  }

  struct Config {
    int ep2_tx_gpio = 20;
    int ep2_rx_gpio = 21;
    uint32_t baudrate = 115200;
  };

private:
  friend class System;
  void Init(const Config &cfg, Uart *uart);
  void GpioInit(const Config &cfg);
  Uart *uart_ = nullptr;
  bool initialized_ = false;
};