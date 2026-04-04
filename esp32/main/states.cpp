#include "states.hpp"

#include <cstdio>

#include "ctx.hpp"
#include "main_ui_widget.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
#include "timebase.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "ESP32-SM";

// Serving State
void ServingState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Serving");
  MainUiWidget::GetInstance().SetStatus("Serving");
  ctx.sys->StopNetwork();
  in_flight_ = false;
  ctx.sys->Led().SetPattern(LED::Pattern::kBreathe, 3000);
  if (!ctx.sys->FcLink().PerformHandshake()) {
    ctx.sys->TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }
  ctx.sys->TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kConfirm);
}

void ServingState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();
  if (button.IsPressed()) {
    ctx.sys->Display().NotifyUserActivity();
  }

  ctx.sys->Mavlink().Poll();
  ctx.sys->FcLink().ForwardRcState(
      ctx.sys->Mavlink()
          .GetRcState());  // FcLink will regulate forwarding rate internally
  ctx.sys->FcLink().Poll();
  while (auto packet = ctx.sys->FcLink().PopPacket()) {
    ctx.sys->CommandHandler().Dispatch(ctx, *packet);
  }
  if (!in_flight_) {
    if (button.ConsumeLongPress()) {
      ESP_LOGI(kTag, "Serving -> DFU (long press)");
      ctx.sm->ReqTransition(*ctx.dfu_state);
      return;
    }
  }
  (void)button.ConsumePress();
  (void)button.ConsumeRelease();
}

// Dfu State
void DfuState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Dfu");
  MainUiWidget::GetInstance().SetStatus("DFU");
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 400);
  ctx.sys->StartNetwork();
  ctx.sys->Tcp().DisableBridge();
}

void DfuState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();
  if (button.IsPressed()) {
    ctx.sys->Display().NotifyUserActivity();
  }

  if (button.ConsumeLongPress()) {
    ESP_LOGI(kTag, "DFU -> Serving (long press)");
    ctx.sm->ReqTransition(*ctx.serving_state);
    return;
  }

  // Poll MAVLink and FcLink for Monitoring
  ctx.sys->Mavlink().Poll();
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
        ESP_LOGE(kTag, "Unknown TCP Result: %d -> HardError", (int)res);
        ctx.sm->ReqTransition(*ctx.hard_error_state);
        break;
    }
  }
  (void)button.ConsumePress();
  (void)button.ConsumeRelease();
}

// Program State
void ProgramState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Program mode");
  MainUiWidget::GetInstance().SetStatus("Programming");
  ctx.sys->Programmer().Start(ctx.sys->Tcp().GetStatus().total);
  ctx.sys->Led().Off();
  last_activity_ = ctx.sys->Timebase().NowMs();
  last_written_ = ctx.sys->Programmer().Written();
}

void ProgramState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();
  if (button.IsPressed()) {
    ctx.sys->Display().NotifyUserActivity();
  }
  (void)button.ConsumePress();
  (void)button.ConsumeRelease();
  (void)button.ConsumeLongPress();

  ctx.sys->Tcp().Poll(now);
  ctx.sys->Programmer().Poll(now);

  auto &tcp = ctx.sys->Tcp();
  auto &prog = ctx.sys->Programmer();

  if (prog.Error()) {
    ESP_LOGE(kTag, "Prog Error -> HardError");
    ctx.sm->ReqTransition(*ctx.hard_error_state);
    return;
  }

  if (prog.Done()) {
    ESP_LOGI(kTag, "Prog Done -> Transitioning to Dfu");
    // Report success status before correct transition
    TcpServer::Status st = tcp.GetStatus();
    st.state = 1;  // Done
    tcp.SetStatus(st);

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
        ctx.sm->ReqTransition(*ctx.hard_error_state);
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
      ESP_LOGE(kTag, "Programmer timed out");
      prog.Abort(now);
      ctx.sm->ReqTransition(*ctx.hard_error_state);
      return;
    }
  }
}

void ProgramState::OnExit(AppContext &ctx) {
  ESP_LOGI(kTag, "exit Program");
  ctx.sys->Programmer().Boot();
}

// Hard Error State
void HardErrorState::OnEnter(AppContext &ctx) {
  ESP_LOGE(kTag, "ENTERING HARD ERROR STATE");
  MainUiWidget::GetInstance().SetStatus("Hard error");
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 100);
  ctx.sys->StartNetwork();
  ctx.sys->Tcp().Start();
}

void HardErrorState::OnStep(AppContext &ctx, SmTick now) {
  auto &button = ctx.sys->Button();
  button.Poll();
  if (button.IsPressed()) {
    ctx.sys->Display().NotifyUserActivity();
  }
  (void)button.ConsumePress();
  (void)button.ConsumeRelease();
  (void)button.ConsumeLongPress();

  // Poll system services to keep them alive (e.g. flushing TCP buffers)
  ctx.sys->Tcp().Poll(now);

  // Send Error Message periodically to any connected client
  static uint32_t last_log = 0;
  if (now - last_log > 1000) {
    last_log = now;
    const char *msg =
        "CRITICAL ERROR: System Halted. Physical Reset Required.\n";
    ctx.sys->Tcp().SendData((const uint8_t *)msg, strlen(msg));
  }

  // Consume and ignore all TCP events to prevent state transitions
  while (ctx.sys->Tcp().PopEvent()) {
    __asm__ __volatile__("nop");
  }
}
