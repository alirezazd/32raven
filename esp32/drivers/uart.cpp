#include "uart.hpp"

#include "panic.hpp"

extern "C" {
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

namespace {

template <UartInstance Inst>
constexpr uart_port_t ToPort() {
  if constexpr (Inst == UartInstance::kFcLink) {
    return UART_NUM_0;
  } else if constexpr (Inst == UartInstance::kRcRx) {
    return UART_NUM_1;
  } else {
    static_assert(Inst == UartInstance::kFcLink || Inst == UartInstance::kRcRx,
                  "Invalid Uart instance");
  }
}

uart_parity_t ToParity(UartParity parity) {
  if (parity == UartParity::kOdd) {
    return UART_PARITY_ODD;
  }
  if (parity == UartParity::kEven) {
    return UART_PARITY_EVEN;
  }
  return UART_PARITY_DISABLE;
}

}  // namespace

template <UartInstance Inst>
void Uart<Inst>::Init(const UartConfig &cfg) {
  if (cfg.pins.tx_gpio == GPIO_NUM_NC || cfg.pins.rx_gpio == GPIO_NUM_NC ||
      cfg.line.baud_rate == 0 ||
      cfg.buffers.rx_bytes < UartConfig::kMinBufferSize ||
      cfg.buffers.tx_bytes < UartConfig::kMinBufferSize) {
    Panic(ErrorCode::kUartParamConfigFailed);
  }

  cfg_ = cfg;
  constexpr uart_port_t port = ToPort<Inst>();

  uart_config_t ucfg{};
  ucfg.baud_rate = static_cast<int>(cfg_.line.baud_rate);
  ucfg.data_bits = UART_DATA_8_BITS;
  ucfg.parity = ToParity(cfg_.line.parity);
  ucfg.stop_bits = UART_STOP_BITS_1;
  ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  ucfg.rx_flow_ctrl_thresh = 0;

  if (uart_param_config(port, &ucfg) != ESP_OK) {
    Panic(ErrorCode::kUartParamConfigFailed);
  }
  if (uart_set_pin(port, cfg_.pins.tx_gpio, cfg_.pins.rx_gpio, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE) != ESP_OK) {
    Panic(ErrorCode::kUartSetPinFailed);
  }
  if (uart_driver_install(port, cfg_.buffers.rx_bytes, cfg_.buffers.tx_bytes,
                          0, nullptr, 0) !=
      ESP_OK) {
    Panic(ErrorCode::kUartDriverInstallFailed);
  }

  (void)uart_flush_input(port);
}

template <UartInstance Inst>
int Uart<Inst>::Write(const uint8_t *data, size_t size) {
  if (size == 0) {
    return 0;
  }
  if (!data) {
    Panic(ErrorCode::kUartInvalidArg);
  }

  constexpr uart_port_t port = ToPort<Inst>();
  const int num_bytes_written =
      uart_write_bytes(port, (const char *)data, (uint32_t)size);
  if (num_bytes_written < 0) {
    Panic(ErrorCode::kUartOperationFailed);
  }
  return num_bytes_written;
}

template <UartInstance Inst>
int Uart<Inst>::Read(uint8_t *data, size_t size, uint32_t timeout_ms) {
  if (size == 0) {
    return 0;
  }
  if (!data) {
    Panic(ErrorCode::kUartInvalidArg);
  }

  constexpr uart_port_t port = ToPort<Inst>();
  TickType_t timeout_ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
  const int num_bytes_read =
      uart_read_bytes(port, data, (uint32_t)size, timeout_ticks);
  if (num_bytes_read < 0) {
    Panic(ErrorCode::kUartOperationFailed);
  }
  return num_bytes_read;
}

template <UartInstance Inst>
size_t Uart<Inst>::BufferedRxBytes() const {
  constexpr uart_port_t port = ToPort<Inst>();
  size_t buffered_bytes = 0;
  if (uart_get_buffered_data_len(port, &buffered_bytes) != ESP_OK) {
    Panic(ErrorCode::kUartOperationFailed);
  }
  return buffered_bytes;
}

template <UartInstance Inst>
void Uart<Inst>::Flush() {
  if (uart_flush_input(ToPort<Inst>()) != ESP_OK) {
    Panic(ErrorCode::kUartOperationFailed);
  }
}

template <UartInstance Inst>
void Uart<Inst>::DrainTx(uint32_t timeout_ms) {
  constexpr uart_port_t port = ToPort<Inst>();
  if (uart_wait_tx_done(port, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) {
    Panic(ErrorCode::kUartOperationFailed);
  }
}

template <UartInstance Inst>
void Uart<Inst>::SetBaudRate(uint32_t baud_rate) {
  if (cfg_.line.baud_rate == 0) {
    Panic(ErrorCode::kUartNotInitialized);
  }
  if (baud_rate == 0) {
    Panic(ErrorCode::kUartInvalidArg);
  }

  cfg_.line.baud_rate = baud_rate;
  constexpr uart_port_t port = ToPort<Inst>();

  uart_config_t ucfg{};
  ucfg.baud_rate = static_cast<int>(cfg_.line.baud_rate);
  ucfg.data_bits = UART_DATA_8_BITS;
  ucfg.parity = ToParity(cfg_.line.parity);
  ucfg.stop_bits = UART_STOP_BITS_1;
  ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  ucfg.rx_flow_ctrl_thresh = 0;

  if (uart_param_config(port, &ucfg) != ESP_OK) {
    Panic(ErrorCode::kUartOperationFailed);
  }
}

template class Uart<UartInstance::kFcLink>;
template class Uart<UartInstance::kRcRx>;
