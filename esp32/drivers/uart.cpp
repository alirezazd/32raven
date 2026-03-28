#include "uart.hpp"

#include "panic.hpp"

extern "C" {
#include "driver/uart.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

namespace {

template <UartInstance Inst> constexpr uart_port_t ToPort() {
  if constexpr (Inst == UartInstance::kFcLink) {
    return UART_NUM_0;
  } else if constexpr (Inst == UartInstance::kEp2) {
    return UART_NUM_1;
  } else {
    static_assert(Inst == UartInstance::kFcLink || Inst == UartInstance::kEp2,
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

} // namespace

template <UartInstance Inst> void Uart<Inst>::Init(const UartConfig &cfg) {
  if (initialized_) {
    Panic(ErrorCode::kUartReinit);
  }

  cfg_ = cfg;
  constexpr uart_port_t port = ToPort<Inst>();

  uart_config_t ucfg{};
  ucfg.baud_rate = static_cast<int>(cfg_.baud_rate);
  ucfg.data_bits = UART_DATA_8_BITS;
  ucfg.parity = ToParity(cfg_.parity);
  ucfg.stop_bits = UART_STOP_BITS_1;
  ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  ucfg.rx_flow_ctrl_thresh = 0;

  if (uart_param_config(port, &ucfg) != ESP_OK) {
    Panic(ErrorCode::kUartParamConfigFailed);
  }
  if (uart_set_pin(port, cfg_.tx_gpio, cfg_.rx_gpio, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE) != ESP_OK) {
    Panic(ErrorCode::kUartSetPinFailed);
  }
  if (uart_driver_install(port, cfg_.rx_buf, cfg_.tx_buf, 0, nullptr, 0) !=
      ESP_OK) {
    Panic(ErrorCode::kUartDriverInstallFailed);
  }

  (void)uart_flush_input(port);
  initialized_ = true;
}

template <UartInstance Inst>
int Uart<Inst>::Write(const uint8_t *data, size_t size) {
  if (!initialized_) {
    return -1;
  }
  if (!data || size == 0) {
    return 0;
  }

  constexpr uart_port_t port = ToPort<Inst>();
  const int num_bytes_written =
      uart_write_bytes(port, (const char *)data, (uint32_t)size);
  return (num_bytes_written < 0) ? -1 : num_bytes_written;
}

template <UartInstance Inst>
int Uart<Inst>::Read(uint8_t *data, size_t size, uint32_t timeout_ms) {
  if (!initialized_) {
    return -1;
  }
  if (!data || size == 0) {
    return 0;
  }

  constexpr uart_port_t port = ToPort<Inst>();
  TickType_t timeout_ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
  const int num_bytes_read =
      uart_read_bytes(port, data, (uint32_t)size, timeout_ticks);
  return (num_bytes_read < 0) ? -1 : num_bytes_read;
}

template <UartInstance Inst> void Uart<Inst>::Flush() {
  if (!initialized_) {
    return;
  }

  constexpr uart_port_t port = ToPort<Inst>();
  (void)uart_flush_input(port);
}

template <UartInstance Inst> bool Uart<Inst>::DrainTx(uint32_t timeout_ms) {
  if (!initialized_) {
    return false;
  }

  constexpr uart_port_t port = ToPort<Inst>();
  return uart_wait_tx_done(port, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

template <UartInstance Inst> bool Uart<Inst>::SetBaudRate(uint32_t baud_rate) {
  if (!initialized_ || baud_rate == 0) {
    return false;
  }

  cfg_.baud_rate = baud_rate;
  constexpr uart_port_t port = ToPort<Inst>();

  uart_config_t ucfg{};
  ucfg.baud_rate = static_cast<int>(cfg_.baud_rate);
  ucfg.data_bits = UART_DATA_8_BITS;
  ucfg.parity = ToParity(cfg_.parity);
  ucfg.stop_bits = UART_STOP_BITS_1;
  ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  ucfg.rx_flow_ctrl_thresh = 0;

  return uart_param_config(port, &ucfg) == ESP_OK;
}

template class Uart<UartInstance::kFcLink>;
template class Uart<UartInstance::kEp2>;
