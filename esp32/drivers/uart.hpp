#pragma once
#include "hal/gpio_types.h"
#include <cstddef>
#include <cstdint>

class Uart {
public:
  enum class Id : uint8_t { kStm32, kEp2, kCount };
  struct Config {
    int uart_num = 1; // ESP32 UART peripheral number
    gpio_num_t tx_gpio = GPIO_NUM_4;
    gpio_num_t rx_gpio = GPIO_NUM_5;

    uint32_t baud_rate = 115200;

    enum class Parity { kNone = 0, kEven = 1, kOdd = 2 };
    Parity parity = Parity::kEven;

    int rx_buf = 2048;
    int tx_buf = 2048;
  };

  // New type alias added as per instruction
  using ErrorHandler = void (*)(const char *msg);

  int Write(const uint8_t *data, size_t size);
  int Read(uint8_t *data, size_t size, uint32_t timeout_ms = 0);
  void Flush();
  bool DrainTx(uint32_t timeout_ms);
  bool SetBaudRate(uint32_t baud_rate);

  // Init method signature updated as per instruction
  void Init(const Config &cfg, ErrorHandler error_handler = nullptr);

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
