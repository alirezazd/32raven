#include "uart.hpp"
#include <cstring>

extern "C" {
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
}

template <UartInstance Inst> UART_HandleTypeDef *Uart<Inst>::GetHandle() {
  if constexpr (Inst == UartInstance::kUart1) {
    return &huart1;
  } else if constexpr (Inst == UartInstance::kUart2) {
    return &huart2;
  } else {
    return nullptr; // Should be unreachable with valid Inst
  }
}

template <UartInstance Inst> void Uart<Inst>::Init(const UartConfig &config) {
  if (initialized_) {
    // ErrorHandler();
    return;
  }
  initialized_ = true;

  UART_HandleTypeDef *handle = GetHandle();
  if (!handle) {
    ErrorHandler();
    return;
  }

  if constexpr (Inst == UartInstance::kUart1) {
    handle->Instance = USART1;
  } else if constexpr (Inst == UartInstance::kUart2) {
    handle->Instance = USART2;
  }

  handle->Init.BaudRate = config.baudRate;
  handle->Init.WordLength = config.wordLength;
  handle->Init.StopBits = config.stopBits;
  handle->Init.Parity = config.parity;
  handle->Init.Mode = config.mode;
  handle->Init.HwFlowCtl = config.hwFlowCtl;
  handle->Init.OverSampling = config.overSampling;

  if (HAL_UART_Init(handle) != HAL_OK) {
    ErrorHandler();
  }
}

template <UartInstance Inst> void Uart<Inst>::Send(const char *str) {
  Send((const uint8_t *)str, strlen(str));
}

template <UartInstance Inst>
void Uart<Inst>::Send(const uint8_t *data, size_t len) {
  UART_HandleTypeDef *handle = GetHandle();
  HAL_UART_Transmit(handle, (uint8_t *)data, (uint16_t)len, 100);
}

// Explicit Instantiations
template class Uart<UartInstance::kUart1>;
template class Uart<UartInstance::kUart2>;
