#include "states.hpp"
#include "ctx.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
extern "C" {
#include "esp_log.h"
#include "esp_system.h" // for esp_restart
}

static constexpr const char *kTag = "states";

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ESP_LOGI(kTag, "entering Idle");
  ctx.sys->Led().SetPattern(LED::Pattern::kBreathe, 3000);
  line_idx_ = 0;
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {

  ctx.sys->Button().Poll(now);

  if (ctx.sys->Button().ConsumeLongPress()) {
    ESP_LOGI(kTag, "Idle -> Listen (long press)");
    ctx.listen_state->SetMode(ListenState::ListenMode::kHard);
    ctx.sm->ReqTransition(*ctx.listen_state);
    return;
  }

  // Check UART for "BEGIN" command
  uint8_t rx[16];
  int n = ctx.sys->Uart().Read(rx, sizeof(rx));
  for (int i = 0; i < n; ++i) {
    char c = (char)rx[i];
    if (c == '\n' || c == '\r') {
      line_buf_[line_idx_] = '\0';
      if (line_idx_ > 0) {
        if (strcmp(line_buf_, "BEGIN") == 0) {
          ESP_LOGI(kTag, "Idle -> Listen (UART BEGIN)");
          ctx.listen_state->SetMode(ListenState::ListenMode::kSoft);
          ctx.sm->ReqTransition(*ctx.listen_state);
          return;
        }
      }
      line_idx_ = 0;
    } else {
      if (line_idx_ < sizeof(line_buf_) - 1) {
        line_buf_[line_idx_++] = c;
      } else {
        // overflow, reset
        line_idx_ = 0;
      }
    }
  }
}

void IdleState::OnExit(AppContext &, SmTick) { ESP_LOGI(kTag, "exiting Idle"); }

void ListenState::OnEnter(AppContext &ctx, SmTick now) {
  ESP_LOGI(kTag, "entering Listen");

  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 400);

  ctx.sys->Wifi().StartAp();
  ctx.sys->Tcp().Start();
  ctx.sys->Tcp().SetUploadEnabled(false);
  last_active_ = now;
}

// --- Helper Functions ---

static bool RunBridge(AppContext &ctx) {
  bool activity = false;
  // Transparent Bridge: Uart <-> Tcp
  // 1. Tcp RX -> Uart TX
  // Ensure we can receive bytes from client (buffered in Upload ringbuffer)
  if (!ctx.sys->Tcp().UploadEnabled()) {
    ctx.sys->Tcp().SetUploadEnabled(true);
  }

  uint8_t buf[128];
  // Read from TCP buffer (filled by client data)
  int n_tcp = ctx.sys->Tcp().ReadUpload(buf, sizeof(buf));
  if (n_tcp > 0) {
    ctx.sys->Uart().Write(buf, (size_t)n_tcp);
  }

  // 2. Uart RX -> Tcp TX
  // Read from UART (STM32 output)
  int n_uart = ctx.sys->Uart().Read(buf, sizeof(buf));
  if (n_uart > 0) {
    ctx.sys->Tcp().SendData(buf, (size_t)n_uart);
    activity = true;
  }
  return activity;
}

// Returns true if state transition occurred
static bool CheckForEvents(AppContext &ctx) {
  TcpServer::Event ev;
  while (ctx.sys->Tcp().PopEvent(ev)) {
    switch (ev.id) {
    case TcpServer::EventId::kBegin: {
      ctx.sys->Tcp().SetUploadEnabled(true);

      TcpServer::Status st = ctx.sys->Tcp().GetStatus();
      st.total = ev.begin.size;
      st.rx = 0;
      ctx.sys->Tcp().SetStatus(st);

      ESP_LOGI(kTag, "BEGIN size=%u crc=%u target=%d", (unsigned)ev.begin.size,
               (unsigned)ev.begin.crc, (int)ev.begin.target);

      ctx.sys->Programmer().SetTarget(ev.begin.target);
      ctx.sm->ReqTransition(*ctx.program_state);
      return true;
    }
    case TcpServer::EventId::kAbort: {
      ctx.sys->Tcp().SetUploadEnabled(false);

      TcpServer::Status st = ctx.sys->Tcp().GetStatus();
      st.total = 0;
      st.rx = 0;
      ctx.sys->Tcp().SetStatus(st);

      ESP_LOGI(kTag, "ABORT");
      ctx.sm->ReqTransition(*ctx.idle_state);
      return true;
    }
    case TcpServer::EventId::kReset: {
      ctx.sys->Tcp().SetUploadEnabled(false);

      ESP_LOGW(kTag, "RESET requested. Resetting STM32...");
      // 1. Reset STM32 to App
      ctx.sys->Programmer().Boot();

      // 2. Short delay to allow pulse to happen?
      // Programmer::Boot() is blocking (sleeps during pulse), so valid.

      ESP_LOGW(kTag, "Rebooting ESP32...");
      // 3. Reset ESP32
      esp_restart();
      return true; // Unreachable
    }
    default:
      break;
    }
  }
  return false;
}

void ListenState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Button().Poll(now);

  // Hard Mode: Exit on Button Long Press
  if (mode_ == ListenMode::kHard) {
    if (ctx.sys->Button().ConsumeLongPress()) {
      ESP_LOGI(kTag, "Listen(Hard) -> Idle (long press)");
      ctx.sm->ReqTransition(*ctx.idle_state);
      return;
    }
  }

  ctx.sys->Tcp().Poll(now);

  if (CheckForEvents(ctx)) {
    return;
  }

  bool activity = RunBridge(ctx);
  if (activity) {
    last_active_ = now;
  }

  // Soft Mode: Exit on Timeout
  if (mode_ == ListenMode::kSoft) {
    if ((now - last_active_) > 100) {
      ESP_LOGI(kTag, "Listen(Soft) -> Idle (timeout)");
      ctx.sm->ReqTransition(*ctx.idle_state);
      return;
    }
  }
}

void ListenState::OnExit(AppContext &ctx, SmTick) {
  ESP_LOGI(kTag, "exit Listen");
}

void ProgramState::OnEnter(AppContext &ctx, SmTick now) {
  ESP_LOGI(kTag, "entering Program mode");

  ctx.sys->Programmer().Start(ctx.sys->Tcp().GetStatus().total, now);
  // Manual control for activity
  ctx.sys->Led().Off();
}

void ProgramState::OnStep(AppContext &ctx, SmTick now) {
  // Wanted to use DMA but realized it's not worth it
  ctx.sys->Tcp().Poll(now);
  ctx.sys->Programmer().Poll(now);

  auto &tcp = ctx.sys->Tcp();
  auto &prog = ctx.sys->Programmer();

  if (prog.Error()) {
    ESP_LOGE(kTag, "Prog Error -> ErrorState");
    ctx.sm->ReqTransition(*ctx.error_state);
    return;
  }

  if (prog.Done()) {
    ESP_LOGI(kTag, "Prog Done -> Listen");
    ctx.sm->ReqTransition(*ctx.listen_state);
    return;
  }

  // Toggle LED during verification
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
    size_t n = tcp.ReadUpload(buf, (free < sizeof(buf)) ? free : sizeof(buf));
    if (n > 0) {
      prog.PushBytes(buf, n, now);
      ctx.sys->Led().Toggle();
    }
  }
}

void ProgramState::OnExit(AppContext &ctx, SmTick) {
  ESP_LOGI(kTag, "exit Program");
  ctx.sys->Tcp().Stop();
  ctx.sys->Wifi().Stop();
  ctx.sys->Programmer().Boot();
}

void ErrorState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ESP_LOGE(kTag, "ENTERING ERROR STATE");
  ctx.sys->Led().SetPattern(LED::Pattern::kDoubleBlink, 300);
}

void ErrorState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Button().Poll(now);
  // Optional: Exit error on button press?
  if (ctx.sys->Button().ConsumePress() ||
      ctx.sys->Button().ConsumeLongPress()) {
    ESP_LOGI(kTag, "Error ACK -> Idle");
    ctx.sm->ReqTransition(*ctx.idle_state);
  }
}

void ErrorState::OnExit(AppContext &, SmTick) { ESP_LOGI(kTag, "exit Error"); }