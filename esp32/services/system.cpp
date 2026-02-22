#include "system.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "user_config.hpp"

#include "button.hpp"
#include "led.hpp"
#include "panic.hpp"
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

  ErrorCode err;

  err = LED::GetInstance().Init(LED::Config{});
  if (err != ErrorCode::kOk)
    Panic(err);
  ESP_LOGI(kTag, "LED driver initialized");

  err = Button::GetInstance().Init(kButtonConfig);
  if (err != ErrorCode::kOk)
    Panic(err);
  ESP_LOGI(kTag, "Button driver initialized");

  err = WifiController::GetInstance().Init(WifiController::Config{});
  if (err != ErrorCode::kOk)
    Panic(err);
  ESP_LOGI(kTag, "Wifi driver initialized");

  err = TcpServer::GetInstance().Init(TcpServer::Config{});
  if (err != ErrorCode::kOk)
    Panic(err);
  ESP_LOGI(kTag, "TCP Server initialized");

  err = uarts_[(int)Uart::Id::kStm32].Init(kStm32UartConfig);
  if (err != ErrorCode::kOk)
    Panic(err);
  ESP_LOGI(kTag, "STM32 Uart initialized");

  err = uarts_[(int)Uart::Id::kEp2].Init(kEp2UartConfig);
  if (err != ErrorCode::kOk)
    Panic(err);
  ESP_LOGI(kTag, "EP2 Uart initialized");

  auto &stm32_uart = Uart(::Uart::Id::kStm32);
  auto &ep2_uart = Uart(::Uart::Id::kEp2);
  err = Programmer::GetInstance().Init(Programmer::Config{}, &stm32_uart);
  if (err != ErrorCode::kOk)
    Panic(err);
  ESP_LOGI(kTag, "Programmer initialized");

  Mavlink::GetInstance().Init(kMavlinkConfig, &ep2_uart);
  ESP_LOGI(kTag, "Mavlink service initialized");

  FcLink::GetInstance().Init(FcLink::Config{}, &stm32_uart);
  ESP_LOGI(kTag, "FcLink service initialized");

  CommandHandler::GetInstance().Init(CommandHandler::Config{});
  ESP_LOGI(kTag, "CommandHandler service initialized");

  initialized_ = true;
}

void System::StopNetwork() {
  WifiController::GetInstance().Stop();
  TcpServer::GetInstance().Stop();
}

void System::StartNetwork() {
  WifiController::GetInstance().StartAp();
  TcpServer::GetInstance().Start();
}

::LED &System::Led() { return LED::GetInstance(); }

::Button &System::Button() { return ::Button::GetInstance(); }

WifiController &System::Wifi() { return WifiController::GetInstance(); }

::TcpServer &System::Tcp() { return ::TcpServer::GetInstance(); }

::Mavlink &System::Mavlink() { return ::Mavlink::GetInstance(); }

::FcLink &System::FcLink() { return ::FcLink::GetInstance(); }

::CommandHandler &System::CommandHandler() {
  return ::CommandHandler::GetInstance();
}

::Uart &System::Uart(::Uart::Id id) {
  if ((int)id < (int)::Uart::Id::kCount) {
    return uarts_[(int)id];
  }
  return uarts_[0]; // fallback
}

::Uart &System::Uart() { return Uart(::Uart::Id::kStm32); }

::Programmer &System::Programmer() { return ::Programmer::GetInstance(); }
