#pragma once

#include <cstddef>
#include <cstdint>

#include "hal/gpio_types.h"

enum class UartInstance : uint8_t { kFcLink, kRcRx };

enum class UartParity : uint8_t {
  kNone = 0,
  kEven = 1,
  kOdd = 2,
};

struct UartConfig {
  static constexpr int kMinBufferSize = 256;

  struct Pins {
    gpio_num_t tx_gpio = GPIO_NUM_NC;
    gpio_num_t rx_gpio = GPIO_NUM_NC;
  };

  struct Line {
    uint32_t baud_rate = 0;
    UartParity parity = UartParity::kNone;
  };

  struct Buffers {
    int rx_bytes = 0;
    int tx_bytes = 0;
  };

  Pins pins{};
  Line line{};
  Buffers buffers{};
};

template <UartInstance Inst>
class Uart {
 public:
  static Uart &GetInstance() {
    static Uart instance;
    return instance;
  }
  int Write(const uint8_t *data, size_t size);
  int Read(uint8_t *data, size_t size, uint32_t timeout_ms = 0);
  size_t BufferedRxBytes() const;
  void Flush();
  void DrainTx(uint32_t timeout_ms);
  void SetBaudRate(uint32_t baud_rate);
  const UartConfig &GetConfig() const { return cfg_; }

 private:
  friend class System;
  void Init(const UartConfig &cfg);
  UartConfig cfg_{};
  Uart() = default;
  ~Uart() = default;
  Uart(const Uart &) = delete;
  Uart &operator=(const Uart &) = delete;
};

using UartFcLink = Uart<UartInstance::kFcLink>;
using UartRcRx = Uart<UartInstance::kRcRx>;
