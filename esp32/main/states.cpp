#include "states.hpp"
#include "ctx.hpp"
#include "logv2.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
#include "timebase.hpp"

static constexpr const char *kTag = "states";

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  LOGI(kTag, "entering Idle");
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

  next_toggle_ = now + period_ms_;

  ctx.sys->Led().On(); // optional immediate feedback

  ctx.sys->Wifi().StartAp();
  ctx.sys->Tcp().Start();
  ctx.sys->Tcp().SetUploadEnabled(false);
}

void ListenState::OnStep(AppContext &ctx, SmTick now) {

  ctx.sys->Tcp().Poll(now);
  ctx.sys->Button().Poll(now);

  if (ctx.sys->Button().ConsumeLongPress()) {
    LOGI(kTag, "Listen -> Idle (long press)");
    ctx.sm->ReqTransition(*ctx.idle_state);
    return;
  }

  if (TimeReached(now, next_toggle_)) {
    LOGV(kTag, "toggle led");
    ctx.sys->Led().Toggle();
    next_toggle_ = now + period_ms_;
  }

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
      break;
    }
    case TcpServer::EventId::kReset: {
      ctx.sys->Tcp().SetUploadEnabled(false);
      // TODO: Implement actual system reset logic here (esp_restart)
      LOGI(kTag, "RESET");
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

    if (ctx.sys->Tcp().UploadOverflowed()) {
      LOGI(kTag, "upload overflow");
      ctx.sys->Tcp().ClearUploadOverflow();
      ctx.sys->Tcp().SetUploadEnabled(false);
    }
  }
}

void ListenState::OnExit(AppContext &ctx, SmTick) {
  LOGI(kTag, "exit Listen");

  // ctx.sys->Http().Stop(); // Keep running for ProgramState
  // ctx.sys->Led().Off();   // Keep LED on for feedback
}

void ProgramState::OnEnter(AppContext &ctx, SmTick now) {
  LOGI(kTag, "entering Program mode");

  ctx.sys->Programmer().Start(ctx.sys->Tcp().GetStatus().total, now);
}

void ProgramState::OnStep(AppContext &ctx, SmTick now) {
  ctx.sys->Tcp().Poll(now);

  // flow control: only read if programmer has space
  size_t free = ctx.sys->Programmer().Free();
  if (free > 0) {
    uint8_t buf[512];
    size_t to_read = (free < sizeof(buf)) ? free : sizeof(buf);
    int n = ctx.sys->Tcp().ReadUpload(buf, to_read);
    if (n > 0) {
      ctx.sys->Programmer().PushBytes(buf, n, now);
    } else {
      ctx.sys->Programmer().Poll(now);
    }
  } else {
    ctx.sys->Programmer().Poll(now);
  }

  // Update status for the client
  TcpServer::Status st = ctx.sys->Tcp().GetStatus();
  st.rx = ctx.sys->Programmer().Written();
  ctx.sys->Tcp().SetStatus(st);

  // Check if done
  if (ctx.sys->Programmer().Done()) {
    // Client checks status, so we should stay until reset or manual disconnect.
  }
}

void ProgramState::OnExit(AppContext &ctx, SmTick) {
  LOGI(kTag, "exit Program");
  ctx.sys->Tcp().Stop();
  ctx.sys->Wifi().Stop();
  ctx.sys->Led().Off();
}