#include "states.hpp"
#include "ctx.hpp"
#include "http_server.hpp"
#include "logv2.hpp"
#include "system.hpp"
#include "timebase.hpp"

static constexpr const char *kTag = "states";

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  LOGI(kTag, "entering Idle");

  ctx.sys->Http().Stop();
  ctx.sys->Led().Off();
  ctx.sys->Wifi().Stop();
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
  ctx.sys->Http().Start();
  ctx.sys->Http().SetUploadEnabled(false);
}

void ListenState::OnStep(AppContext &ctx, SmTick now) {

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

  HttpServer::Event ev;
  while (ctx.sys->Http().PopEvent(ev)) {
    switch (ev.id) {
    case HttpServer::EventId::kBegin: {
      ctx.sys->Http().SetUploadEnabled(true);

      HttpServer::Status st = ctx.sys->Http().GetStatus();
      st.total = ev.begin.size;
      st.rx = 0;
      ctx.sys->Http().SetStatus(st);

      LOGI(kTag, "BEGIN size=%u crc=%u", (unsigned)ev.begin.size,
           (unsigned)ev.begin.crc);

      ctx.sm->ReqTransition(*ctx.program_state);
      break;
    }
    case HttpServer::EventId::kAbort: {
      ctx.sys->Http().SetUploadEnabled(false);

      HttpServer::Status st = ctx.sys->Http().GetStatus();
      st.total = 0;
      st.rx = 0;
      ctx.sys->Http().SetStatus(st);

      LOGI(kTag, "ABORT");
      break;
    }
    case HttpServer::EventId::kReset: {
      ctx.sys->Http().SetUploadEnabled(false);
      // TODO: Implement actual system reset logic here (esp_restart)
      LOGI(kTag, "RESET");
      break;
    }
    default:
      break;
    }
  }

  if (ctx.sys->Http().UploadEnabled()) {
    uint8_t buf[512];
    for (int i = 0; i < 8; ++i) {
      int n = ctx.sys->Http().ReadUpload(buf, sizeof(buf));
      if (n <= 0)
        break;

      HttpServer::Status st = ctx.sys->Http().GetStatus();
      st.rx += (uint32_t)n;
      ctx.sys->Http().SetStatus(st);
    }

    if (ctx.sys->Http().UploadOverflowed()) {
      LOGI(kTag, "upload overflow");
      ctx.sys->Http().ClearUploadOverflow();
      ctx.sys->Http().SetUploadEnabled(false);
    }
  }
}

void ListenState::OnExit(AppContext &ctx, SmTick) {
  LOGI(kTag, "exit Listen");

  ctx.sys->Http().Stop();

  ctx.sys->Led().Off();
}

void ProgramState::OnEnter(AppContext &ctx, SmTick now) {
  LOGI(kTag, "entering Program mode");

  ctx.sys->Programmer().Start(ctx.sys->Http().GetStatus().total, now);
}

void ProgramState::OnStep(AppContext &ctx, SmTick now) {
  /*
  uint8_t buf[512];
  int n = ctx.sys->Http().ReadUpload(buf, sizeof(buf));
  if (n <= 0)
    return;

  ctx.sys->Programmer().PushBytes(buf, n, now);
  */
  (void)ctx;
  (void)now;
}

void ProgramState::OnExit(AppContext &ctx, SmTick) {
  LOGI(kTag, "exit Program");
}