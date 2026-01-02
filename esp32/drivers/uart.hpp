#pragma once
#include <cstddef>
#include <cstdint>

class Uart {
public:
  static Uart &GetInstance() {
    static Uart instance;
    return instance;
  }

  struct Config {
    int uart_num = 1; // UART1 for STM32 link
    int tx_gpio = 4;  // ESP TX -> STM32 PA10
    int rx_gpio = 5;  // ESP RX <- STM32 PA9
    uint32_t baud_rate = 115200;
    // 0 none, 1 even, 2 odd
    int parity = 1;
    int rx_buf = 2048;
    int tx_buf = 2048;
  };

  int Write(const uint8_t *data, size_t size);
  int Read(uint8_t *data, size_t size, uint32_t timeout_ms = 0);
  void Flush();
  bool SetBaudRate(uint32_t baud_rate);

private:
  friend class System;
  void Init(const Config &cfg);
  Config cfg_{};
  bool initialized_ = false;

  Uart() = default;
  ~Uart() = default;
  Uart(const Uart &) = delete;
  Uart &operator=(const Uart &) = delete;
};
