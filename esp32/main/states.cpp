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

// =========================================================================
// Serving State (Default Runtime)
// =========================================================================

#include "dispatcher.hpp"

// --- Handlers ---

static void OnPong(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  (void)pkt;
  // Pong is usually handled by handshake logic, but if valid here, good.
  ESP_LOGI(kTag, "CMD: Pong");
}

static void OnPing(AppContext &ctx, const message::Packet &pkt) {
  (void)pkt;
  ESP_LOGI(kTag, "CMD: Ping -> Sending Pong");
  // Send Pong back
  message::Packet tx_pkt;
  tx_pkt.header.id = (uint8_t)message::MsgId::kPong;
  tx_pkt.header.len = 0;
  ctx.sys->FlightController().SendPacket(tx_pkt);
}

static void OnRcChannels(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  // Log RC Channels (maybe rate limited?)
  // Payload: [TargetSys, TargetComp, Ch1..18]
  // Verify length
  if (pkt.header.len < 2 + 18 * 2)
    return;

  // Just log for now to prove it works
  // ESP_LOGI(kTag, "CMD: RC Channels Received");
}

// --- Dispatch Table ---
static const Epistole::Dispatcher<AppContext>::Entry kServingHandlers[] = {
    {message::MsgId::kPong, OnPong},
    {message::MsgId::kPing, OnPing},
    {message::MsgId::kRcChannels, OnRcChannels},
};

static const Epistole::Dispatcher<AppContext>
    kServingDispatcher(kServingHandlers,
                       sizeof(kServingHandlers) / sizeof(kServingHandlers[0]));

void ServingState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ESP_LOGI(kTag, "entering Serving");
  // Ensure WiFi/TCP are off when entering Serving state
  ctx.sys->Tcp().Stop();
  ctx.sys->Wifi().Stop();

  in_flight_ = false; // Default to not in flight upon entry

  ctx.sys->Led().SetPattern(LED::Pattern::kBreathe, 3000);

  // --- Handshake Logic ---
  if (!ctx.sys->FlightController().PerformHandshake()) {
    // Fail - go to DFU
    ctx.sm->ReqTransition(*ctx.dfu_state);
    return;
  }
}

void ServingState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Mavlink().Poll(now);
  ctx.sys->FlightController().Poll(now);
  message::Packet pkt;
  if (ctx.sys->FlightController().GetPacket(pkt)) {
    if (!kServingDispatcher.Dispatch(ctx, pkt)) {
      // Unknown command
    }
  }

  ctx.sys->Button().Poll(now);

  if (ctx.sys->Button().ConsumeLongPress()) {
    if (!in_flight_) {
      ESP_LOGI(kTag, "Serving -> DFU (long press)");
      ctx.sm->ReqTransition(*ctx.dfu_state);
      return;
    } else {
      ESP_LOGW(kTag, "DFU Request Ignored (In Flight)");
    }
  }
}

void ServingState::OnExit(AppContext &, SmTick) {
  ESP_LOGI(kTag, "exiting Serving");
}

// =========================================================================
// Dfu State (Formerly ListenState)
// =========================================================================

void DfuState::OnEnter(AppContext &ctx, SmTick now) {
  ESP_LOGI(kTag, "entering Dfu");

  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 400);

  // Turn on WiFi and TCP
  ctx.sys->Wifi().StartAp();
  ctx.sys->Tcp().Start();
  ctx.sys->Tcp().DisableBridge();
}

void DfuState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Button().Poll(now);

  // Exit on Button Long Press
  if (ctx.sys->Button().ConsumeLongPress()) {
    ESP_LOGI(kTag, "DFU -> Serving (long press)");
    ctx.sm->ReqTransition(*ctx.serving_state);
    return;
  }

  ctx.sys->Tcp().Poll(now);

  // --- Process DFU Events (Inlined) ---
  TcpServer::Event ev;
  while (ctx.sys->Tcp().PopEvent(ev)) {
    switch (ev.id) {
    case TcpServer::EventId::kBegin: {
      ctx.sys->Tcp().StartDownload(ev.begin.size);

      ESP_LOGI(kTag, "BEGIN size=%u crc=%u target=%d", (unsigned)ev.begin.size,
               (unsigned)ev.begin.crc, (int)ev.begin.target);
      // Send handshake ACK
      ctx.sys->Tcp().SendCtrlLine("OK");
      ctx.sys->Programmer().SetTarget(ev.begin.target);
      ctx.sm->ReqTransition(*ctx.program_state);
      return;
    }
    case TcpServer::EventId::kAbort: {
      ctx.sys->Tcp().StopDownload();
      ESP_LOGI(kTag, "ABORT");
      ctx.sm->ReqTransition(*ctx.serving_state);
      return;
    }
    case TcpServer::EventId::kReset: {
      ctx.sys->Tcp().DisableBridge();
      ESP_LOGW(kTag, "RESET requested. Rebooting...");
      ctx.sys->Programmer().Boot();
      esp_restart();
      return;
    }
    case TcpServer::EventId::kBridge: {
      ctx.sys->Tcp().EnableBridge();
      ESP_LOGI(kTag, "Bridge Enabled (Monitor Mode) -> BridgeState");
      ctx.sm->ReqTransition(*ctx.bridge_state);
      return;
    }
    default:
      break;
    }
  }
}

void DfuState::OnExit(AppContext &ctx, SmTick) {
  ESP_LOGI(kTag, "exit Dfu (WiFi remains on)");
}

// =========================================================================
// Bridge State
// =========================================================================

void BridgeState::OnEnter(AppContext &ctx, SmTick now) {
  ESP_LOGI(kTag, "entering Bridge");
  line_idx_ = 0;
  last_print_ = now;
  // No handler, we poll via OnStep
  ctx.sys->Mavlink().SetHandler(nullptr);
}

void BridgeState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Tcp().Poll(now);
  ctx.sys->Mavlink().Poll(now);

  // Check for events (Abort, Reset)
  TcpServer::Event ev;
  while (ctx.sys->Tcp().PopEvent(ev)) {
    if (ev.id == TcpServer::EventId::kAbort ||
        ev.id == TcpServer::EventId::kCtrlDown) {
      ctx.sys->Tcp().StopDownload();
      ESP_LOGI(kTag, "Bridge -> Dfu (ABORT/Disconnect)");
      ctx.sm->ReqTransition(*ctx.dfu_state);
      return;
    } else if (ev.id == TcpServer::EventId::kReset) {
      ctx.sys->Tcp().DisableBridge();
      ESP_LOGW(kTag, "RESET requested. Rebooting...");
      ctx.sys->Programmer().Boot();
      esp_restart();
      return;
    }
  }

  // Transparent Bridge Logic (TCP <-> STM32)
  uint8_t buf[128];
  int n_tcp = ctx.sys->Tcp().ReadDownload(buf, sizeof(buf));
  if (n_tcp > 0) {
    // Forward TCP -> STM32
    ctx.sys->Uart().Write(buf, (size_t)n_tcp);
  }

  // Forward STM32 -> TCP
  int n_uart = ctx.sys->Uart().Read(buf, sizeof(buf));
  if (n_uart > 0) {
    ctx.sys->Tcp().SendData(buf, (size_t)n_uart);

    // Legacy command check (ABORT via UART)
    for (int i = 0; i < n_uart; ++i) {
      char c = (char)buf[i];
      if (c == '\n' || c == '\r') {
        line_buf_[line_idx_] = '\0';
        if (line_idx_ > 0 && strcasecmp(line_buf_, "ABORT") == 0) {
          ESP_LOGI(kTag, "Bridge -> Dfu (UART ABORT)");
          ctx.sm->ReqTransition(*ctx.dfu_state);
          return;
        }
        line_idx_ = 0;
      } else if (line_idx_ < sizeof(line_buf_) - 1) {
        line_buf_[line_idx_++] = c;
      } else {
        line_idx_ = 0;
      }
    }
  }

  // Periodic Print (1s)
  if (now - last_print_ >= 1000) {
    last_print_ = now;

    // Get RC
    const auto &rc = ctx.sys->Mavlink().GetRc();

    char msg[512];
    int len = std::snprintf(
        msg, sizeof(msg),
        "RC: A=%d E=%d T=%d R=%d A1=%d A2=%d A3=%d A4=%d A5=%d A6=%d A7=%d "
        "A8=%d A9=%d A10=%d A11=%d A12=%d\n",
        rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3],
        rc.channels[4], rc.channels[5], rc.channels[6], rc.channels[7],
        rc.channels[8], rc.channels[9], rc.channels[10], rc.channels[11],
        rc.channels[12], rc.channels[13], rc.channels[14], rc.channels[15]);

    if (len > 0) {
      ctx.sys->Tcp().SendData((uint8_t *)msg, (size_t)len);
    }
  }
}

void BridgeState::OnExit(AppContext &ctx, SmTick) {
  ESP_LOGI(kTag, "exit Bridge");
  ctx.sys->Mavlink().SetHandler(nullptr);
}

// =========================================================================
// Program State
// =========================================================================

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
    // Ensure cleanup if needed, but restart cleans all.
    esp_restart();
    return;
  }

  // Check for events
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

  // Update status
  TcpServer::Status st = tcp.GetStatus();
  st.rx = prog.Written();
  tcp.SetStatus(st);

  // Transfer data
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

  // Check for progress in writing to reset timeout
  uint32_t current_written = prog.Written();
  if (current_written != last_written_) {
    last_activity_ = now;
    last_written_ = current_written;
  }

  // Timeout
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

// =========================================================================
// Hard Error State
// =========================================================================

void HardErrorState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ESP_LOGE(kTag, "ENTERING HARD ERROR STATE (LOCKED)");
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 100);
}

void HardErrorState::OnStep(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
  // Locked. No exit logic.
}

void HardErrorState::OnExit(AppContext &, SmTick) {}