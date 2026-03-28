#include "uart.hpp"
#include "panic.hpp"

extern "C" {
#include "driver/uart.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

static uart_port_t ToPort(uint8_t n) {
  switch (n) {
  case 0:
    return UART_NUM_0;
  case 1:
    return UART_NUM_1;
#if SOC_UART_NUM > 2
  case 2:
    return UART_NUM_2;
#endif
  default:
    return UART_NUM_MAX;
  }
}

static uart_parity_t ToParity(Uart::Config::Parity p) {
  if (p == Uart::Config::Parity::kOdd)
    return UART_PARITY_ODD;
  if (p == Uart::Config::Parity::kEven)
    return UART_PARITY_EVEN;
  return UART_PARITY_DISABLE;
}

void Uart::Init(const Config &cfg) {
  if (initialized_) {
    Panic(ErrorCode::kUartReinit);
  }
  cfg_ = cfg;
  const uart_port_t port = ToPort(cfg_.uart_num);

  if (port == UART_NUM_MAX) {
    Panic(ErrorCode::kUartInvalidNumber);
  }

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

  // If you ever re-init, call uart_driver_delete(port) first.
  if (uart_driver_install(port, cfg_.rx_buf, cfg_.tx_buf, 0, nullptr, 0) !=
      ESP_OK) {
    Panic(ErrorCode::kUartDriverInstallFailed);
  }

  (void)uart_flush_input(port);
  initialized_ = true;
}

int Uart::Write(const uint8_t *data, size_t size) {
  if (!initialized_)
    return -1;
  if (!data || size == 0)
    return 0;
  const uart_port_t port = ToPort(cfg_.uart_num);

  const int num_bytes_written =
      uart_write_bytes(port, (const char *)data, (uint32_t)size);
  return (num_bytes_written < 0) ? -1 : num_bytes_written;
}

int Uart::Read(uint8_t *data, size_t size, uint32_t timeout_ms) {
  if (!initialized_)
    return -1;
  if (!data || size == 0)
    return 0;
  const uart_port_t port = ToPort(cfg_.uart_num);

  TickType_t to = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
  const int num_bytes_read = uart_read_bytes(port, data, (uint32_t)size, to);
  return (num_bytes_read < 0) ? -1 : num_bytes_read;
}

void Uart::Flush() {
  if (!initialized_)
    return;
  const uart_port_t port = ToPort(cfg_.uart_num);
  (void)uart_flush_input(port);
}

bool Uart::SetBaudRate(uint32_t baud_rate) {
  if (!initialized_ || baud_rate == 0)
    return false;
  cfg_.baud_rate = baud_rate;

  const uart_port_t port = ToPort(cfg_.uart_num);

  uart_config_t ucfg{};
  ucfg.baud_rate = static_cast<int>(cfg_.baud_rate);
  ucfg.data_bits = UART_DATA_8_BITS;
  ucfg.parity = ToParity(cfg_.parity);
  ucfg.stop_bits = UART_STOP_BITS_1;
  ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  ucfg.rx_flow_ctrl_thresh = 0;

  return uart_param_config(port, &ucfg) == ESP_OK;
}

bool Uart::DrainTx(uint32_t timeout_ms) {
  if (!initialized_)
    return false;
  const uart_port_t port = ToPort(cfg_.uart_num);
  return uart_wait_tx_done(port, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}
