#include "system.hpp"
#include "esp32_config.hpp"
#include "panic.hpp"

extern "C" {
#include "esp_log.h"
}
static constexpr const char *kTag = "system";

void System::Init() {
  if (initialized_) {
    Panic(ErrorCode::kSystemReinit);
  }

  initialized_ = true;

  for (int i = 0; i < static_cast<int>(Component::kCount); ++i) {
    InitComponent(static_cast<Component>(i));
  }

  (void)TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kConfirm);
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
    TonePlayer().Init(::TonePlayer::Config{}, &Buzzer());
    ESP_LOGI(kTag, "TonePlayer service initialized");
    break;
  case Component::kButton:
    Button().Init(kButtonConfig);
    ESP_LOGI(kTag, "Button driver initialized");
    break;
  case Component::kWifi:
    Wifi().Init(kWifiConfig);
    ESP_LOGI(kTag, "Wifi driver initialized");
    break;
  case Component::kTcpServer:
    Tcp().Init(::TcpServer::Config{});
    ESP_LOGI(kTag, "TCP Server initialized");
    break;
  case Component::kFcLinkUart:
    FcLinkUart().Init(kFcLinkUartConfig);
    ESP_LOGI(kTag, "FcLink Uart initialized");
    break;
  case Component::kEp2Uart:
    Ep2Uart().Init(kEp2UartConfig);
    ESP_LOGI(kTag, "EP2 Uart initialized");
    break;
  case Component::kProgrammer:
    Programmer().Init(kProgrammerConfig, &FcLinkUart());
    ESP_LOGI(kTag, "Programmer initialized");
    break;
  case Component::kMavlink:
    Mavlink().Init(kMavlinkConfig, &Ep2Uart());
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
  case Component::kCount:
    break;
  }
}

void System::StopNetwork() {
  Wifi().Stop();
  Tcp().Stop();
}

void System::StartNetwork() {
  Wifi().StartAp();
  Tcp().Start();
}
