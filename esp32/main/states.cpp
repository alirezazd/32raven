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

  ctx.sys->http().Stop();
  ctx.sys->led().Off();
  ctx.sys->wifi().Stop();
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {

  ctx.sys->button().Poll(now);

  if (ctx.sys->button().ConsumeLongPress()) {
    LOGI(kTag, "Idle -> Listen (long press)");
    ctx.sm->ReqTransition(*ctx.listen_state);
  }
}

void IdleState::OnExit(AppContext &, SmTick) { LOGI(kTag, "exiting Idle"); }

void ListenState::OnEnter(AppContext &ctx, SmTick now) {
  LOGI(kTag, "entering Listen");

  next_toggle_ = now + period_ms_;

  ctx.sys->led().On(); // optional immediate feedback

  ctx.sys->wifi().StartAp();
  ctx.sys->http().Start();
  ctx.sys->http().SetUploadEnabled(false);
}

void ListenState::OnStep(AppContext &ctx, SmTick now) {

  ctx.sys->button().Poll(now);

  if (ctx.sys->button().ConsumeLongPress()) {
    LOGI(kTag, "Listen -> Idle (long press)");
    ctx.sm->ReqTransition(*ctx.idle_state);
    return;
  }

  if (TimeReached(now, next_toggle_)) {
    LOGV(kTag, "toggle led");
    ctx.sys->led().Toggle();
    next_toggle_ = now + period_ms_;
  }

  HttpServer::Event ev;
  while (ctx.sys->http().PopEvent(ev)) {
    switch (ev.id) {
    case HttpServer::EventId::kBegin: {
      ctx.sys->http().SetUploadEnabled(true);

      HttpServer::Status st = ctx.sys->http().GetStatus();
      st.total = ev.begin.size;
      st.rx = 0;
      ctx.sys->http().SetStatus(st);

      LOGI(kTag, "BEGIN size=%u crc=%u", (unsigned)ev.begin.size,
           (unsigned)ev.begin.crc);
      break;
    }
    case HttpServer::EventId::kAbort: {
      ctx.sys->http().SetUploadEnabled(false);

      HttpServer::Status st = ctx.sys->http().GetStatus();
      st.total = 0;
      st.rx = 0;
      ctx.sys->http().SetStatus(st);

      LOGI(kTag, "ABORT");
      break;
    }
    case HttpServer::EventId::kReset: {
      ctx.sys->http().SetUploadEnabled(false);
      // TODO: Implement actual system reset logic here (esp_restart)
      LOGI(kTag, "RESET");
      break;
    }
    default:
      break;
    }
  }

  if (ctx.sys->http().UploadEnabled()) {
    uint8_t buf[512];
    for (int i = 0; i < 8; ++i) {
      int n = ctx.sys->http().ReadUpload(buf, sizeof(buf));
      if (n <= 0)
        break;

      HttpServer::Status st = ctx.sys->http().GetStatus();
      st.rx += (uint32_t)n;
      ctx.sys->http().SetStatus(st);
    }

    if (ctx.sys->http().UploadOverflowed()) {
      LOGI(kTag, "upload overflow");
      ctx.sys->http().ClearUploadOverflow();
      ctx.sys->http().SetUploadEnabled(false);
    }
  }
}

void ListenState::OnExit(AppContext &ctx, SmTick) {
  LOGI(kTag, "exit Listen");

  ctx.sys->http().Stop();

  ctx.sys->led().Off();
}

void ProgramState::OnEnter(AppContext &ctx, SmTick now) {
  LOGI(kTag, "entering Program mode");

  ctx.sys->led().On(); // optional immediate feedback
}
