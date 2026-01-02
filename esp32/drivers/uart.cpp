#include "uart.hpp"

extern "C" {
#include "driver/uart.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

static uart_port_t ToPort(int n) {
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
    return UART_NUM_1;
  }
}

static uart_parity_t ToParity(int p) {
  // 0 none, 1 even, 2 odd
  if (p == 2)
    return UART_PARITY_ODD;
  if (p == 1)
    return UART_PARITY_EVEN;
  return UART_PARITY_DISABLE;
}

void Uart::Init(const Config &cfg) {
  cfg_ = cfg;

  const uart_port_t kPort = ToPort(cfg_.uart_num);

  // Install UART driver (TX/RX ring buffers)
  // Keep it simple: assume init is called once. If you re-init, add
  // uart_driver_delete(port).
  (void)uart_driver_install(kPort, cfg_.rx_buf, cfg_.tx_buf, 0, nullptr, 0);

  uart_config_t ucfg{};
  ucfg.baud_rate = (int)cfg_.baud_rate;
  ucfg.data_bits = UART_DATA_8_BITS;
  ucfg.parity = ToParity(cfg_.parity);
  ucfg.stop_bits = UART_STOP_BITS_1;
  ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  ucfg.rx_flow_ctrl_thresh = 0;

  (void)uart_param_config(kPort, &ucfg);

  // Route UART to chosen GPIOs
  (void)uart_set_pin(kPort, cfg_.tx_gpio, cfg_.rx_gpio, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);

  (void)uart_flush_input(kPort);
  initialized_ = true;
}

int Uart::Write(const uint8_t *data, size_t size) {
  if (!initialized_ || !data || size == 0)
    return 0;
  const uart_port_t kPort = ToPort(cfg_.uart_num);

  const int kNumBytesWritten =
      uart_write_bytes(kPort, (const char *)data, (uint32_t)size);
  return (kNumBytesWritten < 0) ? -1 : kNumBytesWritten;
}

int Uart::Read(uint8_t *data, size_t size, uint32_t timeout_ms) {
  if (!initialized_ || !data || size == 0)
    return 0;
  const uart_port_t kPort = ToPort(cfg_.uart_num);

  TickType_t to = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
  const int kNumBytesRead = uart_read_bytes(kPort, data, (uint32_t)size, to);
  return (kNumBytesRead < 0) ? -1 : kNumBytesRead;
}

void Uart::Flush() {
  if (!initialized_)
    return;
  const uart_port_t kPort = ToPort(cfg_.uart_num);
  (void)uart_flush_input(kPort);
}

bool Uart::SetBaudRate(uint32_t baud_rate) {
  if (!initialized_ || baud_rate == 0)
    return false;
  cfg_.baud_rate = baud_rate;

  const uart_port_t kPort = ToPort(cfg_.uart_num);

  uart_config_t ucfg{};
  ucfg.baud_rate = (int)cfg_.baud_rate;
  ucfg.data_bits = UART_DATA_8_BITS;
  ucfg.parity = ToParity(cfg_.parity);
  ucfg.stop_bits = UART_STOP_BITS_1;
  ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  ucfg.rx_flow_ctrl_thresh = 0;

  return uart_param_config(kPort, &ucfg) == ESP_OK;
}
