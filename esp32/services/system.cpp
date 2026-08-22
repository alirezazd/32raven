// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "system.hpp"

#include <array>
#include <cstddef>

#include "error_code.hpp"
#include "esp32_config.hpp"
#include "panic.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/task.h"
}
static constexpr const char *kTag = "system";

namespace {

// Boot order, and the only place it is written down.
constexpr std::array<System::Component,
                     static_cast<std::size_t>(System::Component::kCount)>
    kInitOrder{
        System::Component::kLed,
        System::Component::kBuzzer,
        System::Component::kTonePlayer,
        System::Component::kButton,
        System::Component::kDisplayI2c,
        System::Component::kDisplayPanel,
        System::Component::kUi,
        System::Component::kWifi,
        System::Component::kTcpServer,
        System::Component::kUdpServer,
        System::Component::kUsbCdcServer,
        System::Component::kTelemUart,
        System::Component::kFcLinkUart,
        System::Component::kProgrammer,
        System::Component::kFcLink,
        System::Component::kMavlink,
        System::Component::kCommandHandler,
    };

// -Werror=switch already ties Component to InitComponent's switch; this ties it
// to the boot order, so an enumerator nobody brings up fails the build.
consteval bool InitOrderCoversEveryComponent() {
  std::array<bool, static_cast<std::size_t>(System::Component::kCount)> seen{};
  for (const System::Component c : kInitOrder) {
    const auto i = static_cast<std::size_t>(c);
    if (i >= seen.size() || seen[i]) {
      return false;
    }
    seen[i] = true;
  }
  for (const bool s : seen) {
    if (!s) {
      return false;
    }
  }
  return true;
}
static_assert(InitOrderCoversEveryComponent());

}  // namespace

System &System::GetInstance() {
  static System instance;
  return instance;
}

void System::Init() {
  if (initialized_) {
    Panic(ErrorCode::Esp32::kSystemReinit);
  }
  initialized_ = true;
  main_task_handle_ = xTaskGetCurrentTaskHandle();
  for (const Component c : kInitOrder) {
    InitComponent(c);
  }
}

void System::InitComponent(Component c) {
  switch (c) {
    case Component::kLed:
      Led().Init(kLedConfig);
      ESP_LOGI(kTag, "LED driver initialized");
      break;
    case Component::kBuzzer:
      Buzzer().Init(kBuzzerConfig);
      ESP_LOGI(kTag, "Buzzer driver initialized");
      break;
    case Component::kTonePlayer:
      TonePlayer().Init(kTonePlayerConfig, &Buzzer());
      ESP_LOGI(kTag, "TonePlayer service initialized");
      break;
    case Component::kButton:
      Button().Init(kButtonConfig);
      ESP_LOGI(kTag, "Button driver initialized");
      break;
    case Component::kDisplayI2c:
      if (kSsd1306PanelConfig.enabled) {
        DisplayI2c().Init(kDisplayI2cConfig);
        ESP_LOGI(kTag, "Display I2C initialized");
      }
      break;
    case Component::kDisplayPanel:
      DisplayPanel().Init(kSsd1306PanelConfig, &DisplayI2c());
      if (kSsd1306PanelConfig.enabled) {
        ESP_LOGI(kTag, "SSD1306 panel initialized");
      }
      break;
    case Component::kUi:
      if (kSsd1306PanelConfig.enabled) {
        Ui().Init(kUiConfig, &DisplayPanel(), Wifi(), Programmer(), Mavlink());
        ESP_LOGI(kTag, "UI initialized");
      }
      break;
    case Component::kWifi:
      Wifi().Init(kWifiConfig);
      ESP_LOGI(kTag, "Wifi driver initialized");
      break;
    case Component::kTcpServer:
      Tcp().Init(kTcpServerConfig);
      ESP_LOGI(kTag, "TCP Server initialized");
      break;
    case Component::kUdpServer:
      Udp().Init(kUdpServerConfig, Wifi());
      ESP_LOGI(kTag, "UDP Server initialized");
      break;
    case Component::kUsbCdcServer:
      UsbCdc().Init(kUsbCdcServerConfig);
      ESP_LOGI(kTag, "USB CDC server initialized");
      break;
    case Component::kTelemUart:
      TelemUart().Init(kTelemUartConfig);
      ESP_LOGI(kTag, "Telem UART initialized");
      break;
    case Component::kFcLinkUart:
      FcLinkUart().Init(kFcLinkUartConfig);
      ESP_LOGI(kTag, "FcLink Uart initialized");
      break;
    case Component::kProgrammer:
      Programmer().Init(kProgrammerConfig, &FcLinkUart());
      ESP_LOGI(kTag, "Programmer initialized");
      break;
    case Component::kFcLink:
      FcLink().Init(kFcLinkConfig, &FcLinkUart());
      ESP_LOGI(kTag, "FcLink service initialized");
      break;
    case Component::kMavlink:
      Mavlink().Init(kMavlinkConfig, &Telem(), FcLink());
      ESP_LOGI(kTag, "Mavlink service initialized");
      break;
    case Component::kCommandHandler:
      CommandHandler().Init(::CommandHandler::Config{});
      ESP_LOGI(kTag, "CommandHandler service initialized");
      break;
    case Component::kCount:
      break;
  }
}

void System::StopNetwork() {
  Udp().Stop();
  Tcp().Stop();
  Wifi().Stop();
}

// Best effort: both servers log their own failure and leave Running() false,
// and every caller here is a last resort with nothing to fall back to. Callers
// that need to know ask Wifi().IsOn() / Tcp().Running() instead, which also
// covers StartAp() -- an esp_err_t from here could only ever describe the two
// sockets.
void System::StartNetwork() {
  Wifi().StartAp();
  Tcp().Start();
  Udp().Start();
}

void System::Halt() {
  if (main_task_handle_ != nullptr &&
      main_task_handle_ != xTaskGetCurrentTaskHandle()) {
    vTaskSuspend(main_task_handle_);
  }
}
