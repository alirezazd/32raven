// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "states.hpp"

#include <cstdio>

#include "common_config.hpp"
#include "ctx.hpp"
#include "error_code.hpp"
#include "esp32_config.hpp"
#include "panic.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
#include "timebase.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "ESP32-SM";

// Mavlink().Poll stays at the call sites: ESC config drains the link but runs
// with the telemetry link off.
static void DrainFcLink(AppContext &ctx) {
  ctx.sys->FcLink().Poll();
  while (auto packet = ctx.sys->FcLink().PopPacket()) {
    ctx.sys->CommandHandler().Dispatch(ctx, *packet);
  }
}

// Navigation model. Two menus, one gesture each:
//   short press — cycle within the current menu
//     normal: Serving -> MavlinkWifi -> MavlinkUsb -> Serving
//     config: Dfu -> EscConfig -> Dfu
//   hold        — swap menus, from anywhere
//     normal -> Dfu (config entry point), config -> Serving
// Program is transient and sits outside the cycle; a hold aborts it back to
// Dfu rather than leaving the config menu mid-flash.

// Serving State
void ServingState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Serving");
  ctx.sys->Ui().SetAppState(Ui::AppState::kServing);
  // Telem UART is the always-on default link — a SiK radio (or any
  // transparent serial peer) starts seeing heartbeats the moment it's
  // plugged in, with no user action required.
  ctx.sys->Mavlink().SetTransport(&ctx.sys->Telem());
  ctx.sys->Mavlink().SetTelemetryLink(true);
  // The STM32 keeps streaming FC link packets while DFU/program modes are not
  // polling them, so resume in a fresh resync state instead of parsing stale
  // buffered bytes as fatal corruption.
  ctx.sys->FcLink().ResetRxState();
  ctx.sys->StopNetwork();
  ctx.sys->Led().SetPattern(LED::Pattern::kBreathe, 3000);
  ctx.sys->TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kConfirm);
}

void ServingState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (button.ConsumePress() && ctx.sys->Ui().NotifyUserActivity()) {
    ctx.sm->ReqTransition(*ctx.mavlink_wifi_state);
    return;
  }
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }
  DrainFcLink(ctx);
  ctx.sys->Mavlink().Poll(now);
}

// MavlinkWifi State
void MavlinkWifiState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering MavlinkWifi");
  ctx.sys->Ui().SetAppState(Ui::AppState::kMavlinkWifi);
  ctx.sys->Mavlink().SetTransport(&ctx.sys->Udp());
  ctx.sys->Mavlink().SetTelemetryLink(true);
  ctx.sys->Tcp().Stop();
  ctx.sys->Wifi().StartAp();
  ctx.sys->Udp().Start();
}

void MavlinkWifiState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (button.ConsumePress() && ctx.sys->Ui().NotifyUserActivity()) {
    ESP_LOGI(kTag, "MavlinkWifi -> MavlinkUsb (press)");
    ctx.sm->ReqTransition(*ctx.mavlink_usb_state);
    return;
  }
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "MavlinkWifi -> Dfu (long press)");
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }
  DrainFcLink(ctx);
  ctx.sys->Mavlink().Poll(now);
}

// MavlinkUsb State
void MavlinkUsbState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering MavlinkUsb");
  // For now the UI re-uses the WiFi screen — the widget reads the active
  // transport from the Mavlink service to render the right text.
  ctx.sys->Ui().SetAppState(Ui::AppState::kMavlinkUsb);
  ctx.sys->Mavlink().SetTransport(&ctx.sys->UsbCdc());
  ctx.sys->Mavlink().SetTelemetryLink(true);
  // No AP / UDP socket needed for USB CDC; tear them down so an already
  // associated WiFi peer doesn't keep consuming radio.
  ctx.sys->StopNetwork();
}

void MavlinkUsbState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (button.ConsumePress() && ctx.sys->Ui().NotifyUserActivity()) {
    ESP_LOGI(kTag, "MavlinkUsb -> Serving (press)");
    ctx.sm->ReqTransition(*ctx.serving_state);
    return;
  }
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "MavlinkUsb -> Dfu (long press)");
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }
  DrainFcLink(ctx);
  ctx.sys->Mavlink().Poll(now);
}

// Dfu State
void DfuState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Dfu");
  ctx.sys->Ui().SetAppState(Ui::AppState::kDfu);
  ctx.sys->Mavlink().SetTelemetryLink(false);
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 400);
  ctx.sys->StartNetwork();
  ctx.sys->Tcp().DisableBridge();
}

void DfuState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (button.ConsumePress() && ctx.sys->Ui().NotifyUserActivity()) {
    ESP_LOGI(kTag, "DFU -> EscConfig (press)");
    ctx.sm->ReqTransition(*ctx.esc_config_state);
    return;
  }
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "DFU -> Serving (long press)");
    ctx.sm->ReqTransition(*ctx.serving_state);
    return;
  }

  ctx.sys->Tcp().Poll(now);

  while (auto ev = ctx.sys->Tcp().PopEvent()) {
    if (ctx.sys->CommandHandler().Dispatch(ctx, *ev) ==
        CommandHandler::DfuTcpAction::kEnterProgram) {
      ctx.sm->ReqTransition(*ctx.program_state);
      return;
    }
  }
}

// Program State
void ProgramState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Program mode");
  ctx.sys->Ui().SetAppState(Ui::AppState::kProgram);
  // Treat programming start like user activity so the progress UI is visible.
  ctx.sys->Ui().NotifyUserActivity();
  ctx.sys->Mavlink().SetTelemetryLink(false);
  ctx.sys->Programmer().Start(ctx.sys->Tcp().GetStatus().total);
  ctx.sys->Led().Off();
  last_activity_ = ctx.sys->Timebase().NowMs();
  last_written_ = ctx.sys->Programmer().Written();
}

void ProgramState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (button.ConsumePress()) {
    ctx.sys->Ui().NotifyUserActivity();
  }
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "Program -> Dfu (long press)");
    ctx.sys->Programmer().Abort(now);
    ctx.sys->Tcp().StopDownload();
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }

  ctx.sys->Tcp().Poll(now);
  ctx.sys->Programmer().Poll(now);

  auto &tcp = ctx.sys->Tcp();
  auto &prog = ctx.sys->Programmer();

  if (prog.Error()) {
    const uint32_t programmer_error = prog.LastErrorCode();
    tcp.StopDownload();
    prog.Abort(now);
    ESP_LOGE(kTag, "Prog Error -> Panic");
    Panic(programmer_error);
  }

  if (prog.Done()) {
    ESP_LOGI(kTag, "Prog Done -> Transitioning to Dfu");
    TcpServer::Status st{};
    st.rx = prog.Written();
    st.total = prog.Total();
    st.state = 1;
    tcp.StopDownload();
    tcp.SetStatus(st);

    ctx.sys->Programmer().Boot();
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }

  while (auto ev = tcp.PopEvent()) {
    if (ev->id == TcpServer::EventId::kAbort ||
        ev->id == TcpServer::EventId::kCtrlDown ||
        ev->id == TcpServer::EventId::kDataDown) {
      ESP_LOGE(kTag, "ProgramState Event: %d -> Abort", (int)ev->id);
      tcp.StopDownload();
      prog.Abort(now);
      if (ev->id == TcpServer::EventId::kAbort) {
        ctx.sm->ReqTransition(*ctx.serving_state);
      } else {
        Panic(ErrorCode::Esp32::kTcpServerError);
      }
      return;
    }
  }

  if (prog.IsVerifying()) {
    ctx.sys->Led().Toggle();
    return;
  }

  TcpServer::Status st = tcp.GetStatus();
  st.rx = prog.Written();
  tcp.SetStatus(st);

  size_t free = prog.Free();
  if (free > 0) {
    uint8_t buf[512];
    size_t n = tcp.ReadDownload(buf, (free < sizeof(buf)) ? free : sizeof(buf));
    if (n > 0) {
      prog.PushBytes(buf, n, now);
      ctx.sys->Led().Toggle();
      last_activity_ = now;
    }
  }

  uint32_t current_written = prog.Written();
  if (current_written != last_written_) {
    last_activity_ = now;
    last_written_ = current_written;
  }

  if (!prog.IsVerifying() && !prog.Done()) {
    if ((now - last_activity_) > 3000) {
      ESP_LOGE(kTag, "Programmer timed out -> Panic");
      tcp.StopDownload();
      prog.Abort(now);
      Panic(ErrorCode::Esp32::kProgrammerTimedOut);
    }
  }
}

// EscConfig State
void EscConfigState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering EscConfig");
  ctx.sys->Ui().SetAppState(Ui::AppState::kEscConfig);
  ctx.sys->Mavlink().SetTelemetryLink(false);
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 800);
  ctx.sys->StopNetwork();
  ctx.sys->FcLink().ResetRxState();
  last_lease_ms_ = 0;  // re-entry renews at once, not one interval late
}

void EscConfigState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (button.ConsumePress() && ctx.sys->Ui().NotifyUserActivity()) {
    ESP_LOGI(kTag, "EscConfig -> Dfu (press)");
    ctx.sys->FcLink().SendPacket(message::MsgId::kSetEscConfigMode,
                                 message::SetEscConfigModeMsg{.enabled = 0u});
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "EscConfig -> Serving (long press)");
    ctx.sys->FcLink().SendPacket(message::MsgId::kSetEscConfigMode,
                                 message::SetEscConfigModeMsg{.enabled = 0u});
    ctx.sm->ReqTransition(*ctx.serving_state);
    return;
  }

  // Renewed, not latched: whatever stops the renewals closes the STM32's gate.
  if (last_lease_ms_ == 0 ||
      (now - last_lease_ms_) >= common_config::kFcLinkExchangeIntervalMs) {
    last_lease_ms_ = now;
    ctx.sys->FcLink().SendPacket(message::MsgId::kSetEscConfigMode,
                                 message::SetEscConfigModeMsg{.enabled = 1u});
  }

  DrainFcLink(ctx);
}
