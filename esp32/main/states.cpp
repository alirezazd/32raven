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
    ctx.sm->ReqTransition(*ctx.listen_state);
    return;
  }

  // Check UART for any activity
  uint8_t rx[1];
  int n = ctx.sys->Uart().Read(rx, sizeof(rx));
  if (n > 0) {
    ESP_LOGI(kTag, "Idle -> Listen (UART Activity)");
    ctx.sm->ReqTransition(*ctx.listen_state);
    return;
  }
}

void IdleState::OnExit(AppContext &, SmTick) { ESP_LOGI(kTag, "exiting Idle"); }

void ListenState::OnEnter(AppContext &ctx, SmTick now) {
  ESP_LOGI(kTag, "entering Listen");

  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 400);

  ctx.sys->Wifi().StartAp();
  ctx.sys->Tcp().Start();
  ctx.sys->Tcp().SetUploadEnabled(false);
  line_idx_ = 0;
}

// --- Helper Functions ---

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

  // Exit on Button Long Press
  if (ctx.sys->Button().ConsumeLongPress()) {
    ESP_LOGI(kTag, "Listen -> Idle (long press)");
    ctx.sm->ReqTransition(*ctx.idle_state);
    return;
  }

  ctx.sys->Tcp().Poll(now);

  if (CheckForEvents(ctx)) {
    return;
  }

  // Transparent Bridge: Uart <-> Tcp
  // 1. Tcp RX -> Uart TX
  if (!ctx.sys->Tcp().UploadEnabled()) {
    ctx.sys->Tcp().SetUploadEnabled(true);
  }

  uint8_t buf[128];
  int n_tcp = ctx.sys->Tcp().ReadUpload(buf, sizeof(buf));
  if (n_tcp > 0) {
    ctx.sys->Uart().Write(buf, (size_t)n_tcp);
  }

  // 2. Uart RX -> Tcp TX + Spy for "ABort"
  int n_uart = ctx.sys->Uart().Read(buf, sizeof(buf));
  if (n_uart > 0) {
    ctx.sys->Tcp().SendData(buf, (size_t)n_uart);

    for (int i = 0; i < n_uart; ++i) {
      char c = (char)buf[i];
      if (c == '\n' || c == '\r') {
        line_buf_[line_idx_] = '\0';
        if (line_idx_ > 0) {
          if (strcmp(line_buf_, "ABort") == 0) {
            ESP_LOGI(kTag, "Listen -> Idle (UART ABort)");
            ctx.sm->ReqTransition(*ctx.idle_state);
            return;
          }
        }
        line_idx_ = 0;
      } else {
        if (line_idx_ < sizeof(line_buf_) - 1) {
          line_buf_[line_idx_++] = c;
        } else {
          // overflow
          line_idx_ = 0;
        }
      }
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