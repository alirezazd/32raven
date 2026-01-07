#include "states.hpp"
#include "ctx.hpp"
#include "logv2.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
extern "C" {
#include "esp_system.h" // for esp_restart
}

static constexpr const char *kTag = "states";

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  LOGI(kTag, "entering Idle");
  ctx.sys->Led().SetPattern(LED::Pattern::kBreathe, 3000);
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {

  ctx.sys->Button().Poll(now);

  if (ctx.sys->Button().ConsumeLongPress()) {
    LOGI(kTag, "Idle -> Listen (long press)");
    ctx.sm->ReqTransition(*ctx.listen_state);
  }
}

void IdleState::OnExit(AppContext &, SmTick) { LOGI(kTag, "exiting Idle"); }

void ListenState::OnEnter(AppContext &ctx, SmTick now) {
  LOGI(kTag, "entering Listen");

  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, 400);

  ctx.sys->Wifi().StartAp();
  ctx.sys->Tcp().Start();
  ctx.sys->Tcp().SetUploadEnabled(false);
}

void ListenState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Button().Poll(now);
  if (ctx.sys->Button().ConsumeLongPress()) {
    LOGI(kTag, "Listen -> Idle (long press)");
    ctx.sm->ReqTransition(*ctx.idle_state);
    return;
  }

  ctx.sys->Tcp().Poll(now);
  TcpServer::Event ev;
  while (ctx.sys->Tcp().PopEvent(ev)) {
    switch (ev.id) {
    case TcpServer::EventId::kBegin: {
      ctx.sys->Tcp().SetUploadEnabled(true);

      TcpServer::Status st = ctx.sys->Tcp().GetStatus();
      st.total = ev.begin.size;
      st.rx = 0;
      ctx.sys->Tcp().SetStatus(st);

      LOGI(kTag, "BEGIN size=%u crc=%u", (unsigned)ev.begin.size,
           (unsigned)ev.begin.crc);

      ctx.sm->ReqTransition(*ctx.program_state);
      break;
    }
    case TcpServer::EventId::kAbort: {
      ctx.sys->Tcp().SetUploadEnabled(false);

      TcpServer::Status st = ctx.sys->Tcp().GetStatus();
      st.total = 0;
      st.rx = 0;
      ctx.sys->Tcp().SetStatus(st);

      LOGI(kTag, "ABORT");
      ctx.sm->ReqTransition(*ctx.idle_state);
      break;
    }
    case TcpServer::EventId::kReset: {
      ctx.sys->Tcp().SetUploadEnabled(false);

      LOGW(kTag, "RESET requested. Resetting STM32...");
      // 1. Reset STM32 to App
      ctx.sys->Programmer().Boot();

      // 2. Short delay to allow pulse to happen?
      // Programmer::Boot() is blocking (sleeps during pulse), so valid.

      LOGW(kTag, "Rebooting ESP32...");
      // 3. Reset ESP32
      esp_restart();
      break;
    }
    default:
      break;
    }
  }

  if (ctx.sys->Tcp().UploadEnabled()) {
    uint8_t buf[512];
    for (int i = 0; i < 8; ++i) {
      int n = ctx.sys->Tcp().ReadUpload(buf, sizeof(buf));
      if (n <= 0)
        break;

      TcpServer::Status st = ctx.sys->Tcp().GetStatus();
      st.rx += (uint32_t)n;
      ctx.sys->Tcp().SetStatus(st);
    }
  }
}

void ListenState::OnExit(AppContext &ctx, SmTick) { LOGI(kTag, "exit Listen"); }

void ProgramState::OnEnter(AppContext &ctx, SmTick now) {
  LOGI(kTag, "entering Program mode");

  ctx.sys->Programmer().Start(ctx.sys->Tcp().GetStatus().total, now);
  // Manual control for activity
  ctx.sys->Led().Off();
}

// TODO: Use DMA
// TODO: Handle LED
// TODO: Verification
void ProgramState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Tcp().Poll(now);
  ctx.sys->Programmer().Poll(now);

  auto &tcp = ctx.sys->Tcp();
  auto &prog = ctx.sys->Programmer();

  if (prog.Error()) {
    LOGE(kTag, "Prog Error -> ErrorState");
    ctx.sm->ReqTransition(*ctx.error_state);
    return;
  }

  if (prog.Done()) {
    LOGI(kTag, "Prog Done -> Idle");
    ctx.sm->ReqTransition(*ctx.idle_state);
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
  LOGI(kTag, "exit Program");
  ctx.sys->Tcp().Stop();
  ctx.sys->Wifi().Stop();
  ctx.sys->Programmer().Boot();
}

void ErrorState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  LOGE(kTag, "ENTERING ERROR STATE");
  ctx.sys->Led().SetPattern(LED::Pattern::kDoubleBlink, 300);
}

void ErrorState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Button().Poll(now);
  // Optional: Exit error on button press?
  if (ctx.sys->Button().ConsumePress() ||
      ctx.sys->Button().ConsumeLongPress()) {
    LOGI(kTag, "Error ACK -> Idle");
    ctx.sm->ReqTransition(*ctx.idle_state);
  }
}

void ErrorState::OnExit(AppContext &, SmTick) { LOGI(kTag, "exit Error"); }