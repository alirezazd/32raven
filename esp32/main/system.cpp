#include "system.hpp"

#include "button.hpp"
#include "led.hpp"
#include "programmer.hpp"
#include "tcp_server.hpp"
#include "uart.hpp"
#include "wifi.hpp"

extern "C" {
#include "esp_log.h"
}

static constexpr const char *kTag = "system";

void System::Init() {
  if (initialized_)
    return;

  LED::GetInstance().Init(LED::Config{});
  ESP_LOGI(kTag, "LED driver initialized");
  Button::GetInstance().Init(Button::Config{});
  ESP_LOGI(kTag, "Button driver initialized");
  WifiController::GetInstance().Init(WifiController::Config{});
  ESP_LOGI(kTag, "Wifi driver initialized");
  TcpServer::GetInstance().Init(TcpServer::Config{});
  ESP_LOGI(kTag, "TCP Server initialized");
  Uart::GetInstance().Init(Uart::Config{});
  ESP_LOGI(kTag, "Uart driver initialized");
  Programmer::GetInstance().Init(Programmer::Config{}, &Uart::GetInstance());
  ESP_LOGI(kTag, "Programmer initialized");

  // Optional but strongly recommended
  // catch missing driver init early
  // assert(LED::GetInstance().initialized());
  // assert(Button::GetInstance().initialized());

  initialized_ = true;
}

LED &System::Led() { return LED::GetInstance(); }

::Button &System::Button() { return ::Button::GetInstance(); }

WifiController &System::Wifi() { return WifiController::GetInstance(); }

::TcpServer &System::Tcp() { return ::TcpServer::GetInstance(); }

::Uart &System::Uart() { return ::Uart::GetInstance(); }

::Programmer &System::Programmer() { return ::Programmer::GetInstance(); }
