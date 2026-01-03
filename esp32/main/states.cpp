#include "states.hpp"
#include "ctx.hpp"
#include "http_server.hpp"
#include "logv2.hpp"
#include "system.hpp"
#include "timebase.hpp"

static constexpr const char *kTag = "states";

void IdleState::on_enter(AppContext &ctx, sm_tick_t now) {
  (void)now;
  LOGI(kTag, "enter Idle");

  if (!ctx.sys)
    return;
  ctx.sys->http().Stop();
  ctx.sys->led().Off();
  ctx.sys->wifi().Stop();
}

void IdleState::on_step(AppContext &ctx, sm_tick_t now) {
  if (!ctx.sys || !ctx.sm || !ctx.listen)
    return;

  ctx.sys->button().Poll(now);

  if (ctx.sys->button().ConsumeLongPress()) {
    LOGI(kTag, "Idle -> Listen (long press)");
    ctx.sm->request_transition(*ctx.listen);
  }
}

void IdleState::on_exit(AppContext &, sm_tick_t) { LOGI(kTag, "exit Idle"); }

void ListenState::on_enter(AppContext &ctx, sm_tick_t now) {
  LOGI(kTag, "enter Listen");

  next_toggle_ = now + period_ms_;

  if (!ctx.sys)
    return;
  ctx.sys->led().On(); // optional immediate feedback

  ctx.sys->wifi().StartAp();
  ctx.sys->http().Start();
  ctx.sys->http().SetUploadEnabled(false);
}

void ListenState::on_step(AppContext &ctx, sm_tick_t now) {
  if (!ctx.sys || !ctx.sm || !ctx.idle)
    return;

  ctx.sys->button().Poll(now);

  if (ctx.sys->button().ConsumeLongPress()) {
    LOGI(kTag, "Listen -> Idle (long press)");
    ctx.sm->request_transition(*ctx.idle);
    return;
  }

  if (TimeReached(now, next_toggle_)) {
    LOGV(kTag, "toggle led");
    ctx.sys->led().Toggle();
    next_toggle_ = now + period_ms_;
  }

  HttpServer::Event ev;
  while (ctx.sys->http().PopEvent(ev)) {
    if (ev.id == HttpServer::EventId::kBegin) {
      ctx.sys->http().SetUploadEnabled(true);

      HttpServer::Status st = ctx.sys->http().GetStatus();
      st.total = ev.begin.size;
      st.rx = 0;
      ctx.sys->http().SetStatus(st);

      LOGI(kTag, "BEGIN size=%u crc=%u", (unsigned)ev.begin.size,
           (unsigned)ev.begin.crc);
    } else if (ev.id == HttpServer::EventId::kAbort) {
      ctx.sys->http().SetUploadEnabled(false);

      HttpServer::Status st = ctx.sys->http().GetStatus();
      st.total = 0;
      st.rx = 0;
      ctx.sys->http().SetStatus(st);

      LOGI(kTag, "ABORT");
    } else if (ev.id == HttpServer::EventId::kReset) {
      ctx.sys->http().SetUploadEnabled(false);
      LOGI(kTag, "RESET");
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

void ListenState::on_exit(AppContext &ctx, sm_tick_t) {
  LOGI(kTag, "exit Listen");

  if (!ctx.sys)
    return;
  ctx.sys->http().Stop();

  ctx.sys->led().Off();
}
