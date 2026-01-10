#pragma once

#include "board.h"
#include <cstddef>
#include <cstdint>

enum class UartInstance { kUart1, kUart2 };

struct UartConfig {
  uint32_t baudRate;
  uint32_t wordLength;
  uint32_t stopBits;
  uint32_t parity;
  uint32_t mode;
  uint32_t hwFlowCtl;
  uint32_t overSampling;
};

template <UartInstance Inst> class Uart {
public:
  static Uart &GetInstance() {
    static Uart instance;
    return instance;
  }

  void Init(const UartConfig &config);
  void Send(const char *str);
  void Send(const uint8_t *data, size_t len);

private:
  Uart() = default;
  ~Uart() = default;

  UART_HandleTypeDef *GetHandle();
  bool initialized_ = false;
};
