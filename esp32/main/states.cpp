#include "states.hpp"
#include "ctx.hpp"
#include "message.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
#include <cstdio>
#include <cstring>

extern "C" {
#include "esp_log.h"
#include "esp_system.h"        // for esp_restart
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "states";

// Serving State
void ServingState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ESP_LOGI(kTag, "entering Serving");
  ctx.sys->StopNetwork();
  in_flight_ = false;
  ctx.sys->Led().SetPattern(LED::Pattern::kBreathe, 3000);

  if (!ctx.sys->FcLink().PerformHandshake()) {
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }
}

void ServingState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Mavlink().Poll(now);
  ctx.sys->FcLink().Poll(now);

  message::Packet pkt;
  while (ctx.sys->FcLink().GetPacket(pkt)) {
    if (!ctx.sys->CommandHandler().Dispatch(ctx, pkt)) {
      ESP_LOGW(kTag, "Ignored unknown packet ID: %d", pkt.header.id);
    }
  }

  mavlink_message_t msg;
  while (ctx.sys->Mavlink().GetMessage(msg)) {
    ctx.sys->CommandHandler().Dispatch(ctx, msg);
  }

  if (!in_flight_) {
    ctx.sys->Button().Poll(now);
    if (ctx.sys->Button().ConsumeLongPress()) {
      ESP_LOGI(kTag, "Serving -> DFU (long press)");
      ctx.sm->ReqTransition(*ctx.dfu_state);
      return;
    }
  }
}

void ServingState::OnExit(AppContext &, SmTick) {
  ESP_LOGI(kTag, "exiting Serving");
}

// Dfu State
void DfuState::OnEnter(AppContext &ctx, SmTick now) {
  ESP_LOGI(kTag, "entering Dfu");
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 400);
  ctx.sys->StartNetwork();
  ctx.sys->Tcp().DisableBridge();
}

void DfuState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Button().Poll(now);

  if (ctx.sys->Button().ConsumeLongPress()) {
    ESP_LOGI(kTag, "DFU -> Serving (long press)");
    ctx.sm->ReqTransition(*ctx.serving_state);
    return;
  }

  // Poll MAVLink and FcLink for Monitoring
  ctx.sys->Mavlink().Poll(now);
  ctx.sys->FcLink().Poll(now);

  mavlink_message_t msg;
  while (ctx.sys->Mavlink().GetMessage(msg)) {
    ctx.sys->CommandHandler().Dispatch(ctx, msg);
  }

  message::Packet pkt;
  while (ctx.sys->FcLink().GetPacket(pkt)) {
    if (!ctx.sys->CommandHandler().Dispatch(ctx, pkt)) {
      // Just ignore unknown packets in monitor mode
    }
  }

  ctx.sys->Tcp().Poll(now);

  TcpServer::Event ev;
  while (ctx.sys->Tcp().PopEvent(ev)) {
    CommandHandler::Result res = ctx.sys->CommandHandler().Dispatch(ctx, ev);
    switch (res) {
    case CommandHandler::Result::kTransitionToProgram:
      ctx.sm->ReqTransition(*ctx.program_state);
      return;
    case CommandHandler::Result::kTransitionToServing:
      ctx.sm->ReqTransition(*ctx.serving_state);
      return;
    case CommandHandler::Result::kNone:
    case CommandHandler::Result::kHandled:
      break;
    default:
      ESP_LOGW(kTag, "Unhandled TCP Result: %d", (int)res);
      break;
    }
  }
}

void DfuState::OnExit(AppContext &ctx, SmTick) { ESP_LOGI(kTag, "exit Dfu"); }

// Program State
void ProgramState::OnEnter(AppContext &ctx, SmTick now) {
  ESP_LOGI(kTag, "entering Program mode");
  ctx.sys->Programmer().Start(ctx.sys->Tcp().GetStatus().total, now);
  ctx.sys->Led().Off();
  last_activity_ = now;
  last_written_ = ctx.sys->Programmer().Written();
}

void ProgramState::OnStep(AppContext &ctx, SmTick now) {
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
    ESP_LOGI(kTag, "Prog Done -> Rebooting System");
    esp_restart();
    return;
  }

  TcpServer::Event ev;
  while (tcp.PopEvent(ev)) {
    if (ev.id == TcpServer::EventId::kAbort ||
        ev.id == TcpServer::EventId::kCtrlDown ||
        ev.id == TcpServer::EventId::kDataDown) {
      ESP_LOGE(kTag, "ProgramState Event: %d -> Abort", (int)ev.id);
      prog.Abort(now);
      if (ev.id == TcpServer::EventId::kAbort) {
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

void ProgramState::OnExit(AppContext &ctx, SmTick) {
  ESP_LOGI(kTag, "exit Program");
  ctx.sys->Programmer().Boot();
}

// Hard Error State
void HardErrorState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ESP_LOGE(kTag, "ENTERING HARD ERROR STATE");
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 100);
  ctx.sys->StartNetwork();
  ctx.sys->Tcp().Start();
}

void HardErrorState::OnStep(AppContext &ctx, SmTick now) {
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
  TcpServer::Event ev;
  while (ctx.sys->Tcp().PopEvent(ev)) {
    __asm__ __volatile__("nop");
  }
}

void HardErrorState::OnExit(AppContext &ctx, SmTick) {
  ctx.sm->ReqTransition(*ctx.hard_error_state); // Should never exit
}