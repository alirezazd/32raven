#include "system.hpp"

#include "esp32_config.hpp"
#include "panic.hpp"

extern "C" {
#include "esp_log.h"
}
static constexpr const char *kTag = "system";

void System::Init() {
  InitComponent(Component::kLed);
  InitComponent(Component::kBuzzer);
  InitComponent(Component::kTonePlayer);
  InitComponent(Component::kButton);
  InitComponent(Component::kDisplayI2c);
  InitComponent(Component::kDisplayPanel);
  InitComponent(Component::kUi);
  InitComponent(Component::kWifi);
  InitComponent(Component::kTcpServer);
  InitComponent(Component::kUdpServer);
  InitComponent(Component::kFcLinkUart);
  InitComponent(Component::kRcRxUart);
  InitComponent(Component::kProgrammer);
  InitComponent(Component::kMavlink);
  InitComponent(Component::kFcLink);
  InitComponent(Component::kCommandHandler);
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
        Ui().Init(kUiConfig, &DisplayPanel());
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
      Udp().Init(kUdpServerConfig);
      ESP_LOGI(kTag, "UDP Server initialized");
      break;
    case Component::kFcLinkUart:
      FcLinkUart().Init(kFcLinkUartConfig);
      ESP_LOGI(kTag, "FcLink Uart initialized");
      break;
    case Component::kRcRxUart:
      RcRxUart().Init(kRcRxUartConfig);
      ESP_LOGI(kTag, "RcRx Uart initialized");
      break;
    case Component::kProgrammer:
      Programmer().Init(kProgrammerConfig, &FcLinkUart());
      ESP_LOGI(kTag, "Programmer initialized");
      break;
    case Component::kMavlink:
      Mavlink().Init(kMavlinkConfig, &RcRxUart(), &Udp());
      ESP_LOGI(kTag, "Mavlink service initialized");
      break;
    case Component::kFcLink:
      FcLink().Init(kFcLinkConfig, &FcLinkUart());
      ESP_LOGI(kTag, "FcLink service initialized");
      break;
    case Component::kCommandHandler:
      CommandHandler().Init(::CommandHandler::Config{});
      ESP_LOGI(kTag, "CommandHandler service initialized");
      break;
  }
}

void System::StopNetwork() {
  Udp().Stop();
  Tcp().Stop();
  Wifi().Stop();
}

void System::StartNetwork() {
  Wifi().StartAp();
  Tcp().Start();
  Udp().Start();
}
