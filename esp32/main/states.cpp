#include "states.hpp"

#include <cstdio>

#include "ctx.hpp"
#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
#include "timebase.hpp"

extern "C" {
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "ESP32-SM";

namespace {

bool ConsumeScreenAwarePress(AppContext &ctx, Button &button) {
  if (!button.ConsumePress()) {
    return false;
  }

  const bool screen_on = ctx.sys->Ui().IsScreenOn();
  ctx.sys->Ui().NotifyUserActivity();
  return screen_on;
}

bool HandleLongLongReboot(Button &button) {
  if (!button.ConsumeLongLongPress()) {
    return false;
  }

  ESP_LOGW(kTag, "LONG-LONG press -> reboot");
  esp_restart();
  return true;
}

void DrainMavlinkCommandEvents(AppContext &ctx) {
  Mavlink::CommandLongEvent ev{};
  while (ctx.sys->Mavlink().PopCommandLongEvent(ev)) {
    ctx.sys->CommandHandler().Dispatch(ctx, ev);
  }
}

}  // namespace

// Serving State
void ServingState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Serving");
  ctx.sys->Ui().SetAppState(Ui::AppState::kServing);
  ctx.sys->Mavlink().SetPrimaryLinkEnabled(false);
  ctx.sys->StopNetwork();
  ctx.sys->Led().SetPattern(LED::Pattern::kBreathe, 3000);
  ctx.sys->FcLink().PerformHandshake();
  ctx.sys->TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kConfirm);
}

void ServingState::OnStep(AppContext &ctx, SmTick now) {
  (void)now;
  auto &button = ctx.sys->Button();
  button.Poll();

  if (ConsumeScreenAwarePress(ctx, button)) {
    ctx.sm->ReqTransition(*ctx.mavlink_wifi_state);
    return;
  }
  if (HandleLongLongReboot(button)) {
    return;
  }
  if (button.ConsumeLongPress()) {
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }

  ctx.sys->Mavlink().Poll();
  ctx.sys->Mavlink().OfferRcChannels(ctx.sys->Mavlink().GetRcState(),
                                     ctx.sys->Timebase().NowMs());
  DrainMavlinkCommandEvents(ctx);
  ctx.sys->FcLink().ForwardRcState(
      ctx.sys->Mavlink()
          .GetRcState());  // FcLink will regulate forwarding rate internally
  ctx.sys->FcLink().Poll();
  DrainMavlinkCommandEvents(ctx);
  while (auto packet = ctx.sys->FcLink().PopPacket()) {
    ctx.sys->CommandHandler().Dispatch(ctx, *packet);
  }
}

// MavlinkWifi State
void MavlinkWifiState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering MavlinkWifi");
  ctx.sys->Ui().SetAppState(Ui::AppState::kMavlinkWifi);
  ctx.sys->Mavlink().SetPrimaryLinkEnabled(true);
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 800);
  ctx.sys->Tcp().Stop();
  ctx.sys->Wifi().StartAp();
  ctx.sys->Udp().Start();
}

void MavlinkWifiState::OnStep(AppContext &ctx, SmTick now) {
  (void)now;
  auto &button = ctx.sys->Button();
  button.Poll();

  if (ConsumeScreenAwarePress(ctx, button)) {
    ESP_LOGI(kTag, "MavlinkWifi -> Serving (press)");
    ctx.sm->ReqTransition(*ctx.serving_state);
    return;
  }
  if (HandleLongLongReboot(button)) {
    return;
  }
  if (button.ConsumeLongPress()) {
    ESP_LOGI(kTag, "MavlinkWifi -> Dfu (long press)");
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }

  ctx.sys->Mavlink().Poll();
  ctx.sys->Mavlink().OfferRcChannels(ctx.sys->Mavlink().GetRcState(),
                                     ctx.sys->Timebase().NowMs());
  DrainMavlinkCommandEvents(ctx);
  ctx.sys->FcLink().Poll();
  while (auto packet = ctx.sys->FcLink().PopPacket()) {
    ctx.sys->CommandHandler().Dispatch(ctx, *packet);
  }
  DrainMavlinkCommandEvents(ctx);
}

// Dfu State
void DfuState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Dfu");
  ctx.sys->Ui().SetAppState(Ui::AppState::kDfu);
  ctx.sys->Mavlink().SetPrimaryLinkEnabled(false);
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 400);
  ctx.sys->StartNetwork();
  ctx.sys->Tcp().DisableBridge();
}

void DfuState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (ConsumeScreenAwarePress(ctx, button)) {
    ESP_LOGI(kTag, "DFU -> Serving (press)");
    ctx.sm->ReqTransition(*ctx.serving_state);
    return;
  }
  if (HandleLongLongReboot(button)) {
    return;
  }
  // Poll MAVLink and FcLink for Monitoring
  ctx.sys->Mavlink().Poll();
  DrainMavlinkCommandEvents(ctx);
  ctx.sys->FcLink().Poll();
  while (auto packet = ctx.sys->FcLink().PopPacket()) {
    ctx.sys->CommandHandler().Dispatch(ctx, *packet);
  }

  ctx.sys->Tcp().Poll(now);

  while (auto ev = ctx.sys->Tcp().PopEvent()) {
    CommandHandler::TransitionResult res =
        ctx.sys->CommandHandler().Dispatch(ctx, *ev);
    switch (res) {
      case CommandHandler::TransitionResult::kTransitionToProgram:
        ctx.sm->ReqTransition(*ctx.program_state);
        return;
      case CommandHandler::TransitionResult::kTransitionToServing:
        ctx.sm->ReqTransition(*ctx.serving_state);
        return;
      case CommandHandler::TransitionResult::kNone:
        break;
      case CommandHandler::TransitionResult::kUnknown:
      default:
        ESP_LOGE(kTag, "Unknown TCP Result: %d -> Panic", (int)res);
        Panic(ErrorCode::kTcpServerError);
    }
  }
}

// Program State
void ProgramState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Program mode");
  ctx.sys->Ui().SetAppState(Ui::AppState::kProgram);
  ctx.sys->Mavlink().SetPrimaryLinkEnabled(false);
  ctx.sys->Programmer().Start(ctx.sys->Tcp().GetStatus().total);
  ctx.sys->Led().Off();
  last_activity_ = ctx.sys->Timebase().NowMs();
  last_written_ = ctx.sys->Programmer().Written();
}

void ProgramState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();

  (void)ConsumeScreenAwarePress(ctx, button);
  if (HandleLongLongReboot(button)) {
    return;
  }
  if (button.ConsumeLongPress()) {
    ESP_LOGI(kTag, "Program -> Dfu (long press)");
    ctx.sys->Programmer().Abort(now);
    ctx.sys->Tcp().StopDownload();
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }

  ctx.sys->Tcp().Poll(now);
  ctx.sys->Programmer().Poll(now);
  DrainMavlinkCommandEvents(ctx);

  auto &tcp = ctx.sys->Tcp();
  auto &prog = ctx.sys->Programmer();

  if (prog.Error()) {
    ESP_LOGE(kTag, "Prog Error -> Panic");
    Panic(prog.LastErrorCode());
  }

  if (prog.Done()) {
    ESP_LOGI(kTag, "Prog Done -> Transitioning to Dfu");
    // Report success status before correct transition
    TcpServer::Status st = tcp.GetStatus();
    st.state = 1;  // Done
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
      prog.Abort(now);
      if (ev->id == TcpServer::EventId::kAbort) {
        ctx.sm->ReqTransition(*ctx.serving_state);
      } else {
        Panic(ErrorCode::kTcpServerError);
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
      prog.Abort(now);
      Panic(ErrorCode::kProgrammerTimedOut);
    }
  }
}
