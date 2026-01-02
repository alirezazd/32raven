#include "system.hpp"

#include "button.hpp"
#include "http_server.hpp"
#include "led.hpp"
#include "wifi.hpp"

extern "C" {
#include "esp_log.h"
}

static constexpr const char *kTag = "system";

void System::_init() {
  if (initialized_)
    return;

  LED::GetInstance().Init(LED::Config{});
  ESP_LOGI(kTag, "LED driver initialized");
  Button::GetInstance().Init(Button::Config{});
  ESP_LOGI(kTag, "Button driver initialized");
  WifiController::GetInstance().Init(WifiController::Config{});
  ESP_LOGI(kTag, "Wifi driver initialized");
  HttpServer::GetInstance().Init(HttpServer::Config{});
  ESP_LOGI(kTag, "HTTP Server initialized");
  Uart::GetInstance().Init(Uart::Config{});
  ESP_LOGI(kTag, "Uart driver initialized");

  // Optional but strongly recommended
  // catch missing driver init early
  // assert(LED::GetInstance().initialized());
  // assert(Button::GetInstance().initialized());

  initialized_ = true;
}

LED &System::led() { return LED::GetInstance(); }

Button &System::button() { return Button::GetInstance(); }

WifiController &System::wifi() { return WifiController::GetInstance(); }

HttpServer &System::http() { return HttpServer::GetInstance(); }

Uart &System::uart() { return Uart::GetInstance(); }
