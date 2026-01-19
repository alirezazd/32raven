#include "ep2_receiver.hpp"

static constexpr char kTag[] = "ep2";

void EP2::Init(const Config &cfg, Uart *uart) {
  uart_ = uart;
  GpioInit(cfg);
  initialized_ = true;
}

void EP2::GpioInit(const Config &cfg) {
  const int kRxPin = cfg.ep2_rx_gpio;
  const int kTxPin = cfg.ep2_tx_gpio;
  gpio_config_t rx_io{};
  gpio_config_t tx_io{};
  rx_io.pin_bit_mask = (1ULL << kRxPin);
  tx_io.pin_bit_mask = (1ULL << kTxPin);
  rx_io.mode = GPIO_MODE_INPUT;
  tx_io.mode = GPIO_MODE_OUTPUT;
  rx_io.pull_up_en = GPIO_PULLUP_DISABLE;
  rx_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  rx_io.intr_type = GPIO_INTR_DISABLE;
  tx_io.pull_up_en = GPIO_PULLUP_DISABLE;
  tx_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  tx_io.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&rx_io);
  gpio_config(&tx_io);
}
