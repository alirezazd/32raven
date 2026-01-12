#include "m9n_service.hpp"

namespace {
constexpr uint8_t SYNC1 = 0xB5;
constexpr uint8_t SYNC2 = 0x62;
constexpr uint8_t ID_NAV_PVT = 0x07;
} // namespace

// Base State
struct BaseState : public IState<ParserContext> {
  const char *Name() const override { return "Base"; }
  void OnEnter(ParserContext &, SmTick) override {}
  void OnStep(ParserContext &, SmTick) override {}
  void OnExit(ParserContext &, SmTick) override {}
  void OnEvent(ParserContext &, const SmEvent &, SmTick) override {}
};

// State Definitions
struct Sync1State : public BaseState {
  const char *Name() const override { return "Sync1"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

struct Sync2State : public BaseState {
  const char *Name() const override { return "Sync2"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

struct ClassState : public BaseState {
  const char *Name() const override { return "Class"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

struct IdState : public BaseState {
  const char *Name() const override { return "Id"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

struct LengthLState : public BaseState {
  const char *Name() const override { return "LengthL"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

struct LengthHState : public BaseState {
  const char *Name() const override { return "LengthH"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

struct PayloadState : public BaseState {
  const char *Name() const override { return "Payload"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

struct CkAState : public BaseState {
  const char *Name() const override { return "CkA"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

struct CkBState : public BaseState {
  const char *Name() const override { return "CkB"; }
  void OnEvent(ParserContext &ctx, const SmEvent &, SmTick) override;
};

// State Instances
static Sync1State s_sync1;
static Sync2State s_sync2;
static ClassState s_cls;
static IdState s_id;
static LengthLState s_len_l;
static LengthHState s_len_h;
static PayloadState s_payload;
static CkAState s_ck_a;
static CkBState s_ck_b;

// State Implementations

void Sync1State::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  if (ctx.current_byte == SYNC1) {
    ctx.sm->ReqTransition(s_sync2);
  }
}

void Sync2State::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  if (ctx.current_byte == SYNC2) {
    ctx.sm->ReqTransition(s_cls);
  } else if (ctx.current_byte == SYNC1) {
    ctx.sm->ReqTransition(s_sync2);
  } else {
    ctx.sm->ReqTransition(s_sync1);
  }
}

void ClassState::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  ctx.cls = ctx.current_byte;
  ctx.ck_a_calc = 0;
  ctx.ck_b_calc = 0;
  ctx.ck_a_calc = (ctx.ck_a_calc + ctx.cls) & 0xFF;
  ctx.ck_b_calc = (ctx.ck_b_calc + ctx.ck_a_calc) & 0xFF;

  ctx.sm->ReqTransition(s_id);
}

void IdState::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  ctx.id = ctx.current_byte;
  ctx.ck_a_calc = (ctx.ck_a_calc + ctx.id) & 0xFF;
  ctx.ck_b_calc = (ctx.ck_b_calc + ctx.ck_a_calc) & 0xFF;

  ctx.sm->ReqTransition(s_len_l);
}

void LengthLState::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  ctx.len = ctx.current_byte;
  ctx.ck_a_calc = (ctx.ck_a_calc + ctx.current_byte) & 0xFF;
  ctx.ck_b_calc = (ctx.ck_b_calc + ctx.ck_a_calc) & 0xFF;

  ctx.sm->ReqTransition(s_len_h);
}

void LengthHState::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  ctx.len |= (static_cast<uint16_t>(ctx.current_byte) << 8);
  ctx.ck_a_calc = (ctx.ck_a_calc + ctx.current_byte) & 0xFF;
  ctx.ck_b_calc = (ctx.ck_b_calc + ctx.ck_a_calc) & 0xFF;

  ctx.payload_idx = 0;

  if (ctx.len > ParserContext::MAX_PAYLOAD_SIZE) {
    ctx.sm->ReqTransition(s_sync1);
  } else if (ctx.len == 0) {
    ctx.sm->ReqTransition(s_ck_a);
  } else {
    ctx.sm->ReqTransition(s_payload);
  }
}

void PayloadState::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  if (ctx.payload_idx < ctx.len) {
    ctx.payload_buf[ctx.payload_idx] = ctx.current_byte;
    ctx.ck_a_calc = (ctx.ck_a_calc + ctx.current_byte) & 0xFF;
    ctx.ck_b_calc = (ctx.ck_b_calc + ctx.ck_a_calc) & 0xFF;
    ctx.payload_idx++;
  }

  if (ctx.payload_idx >= ctx.len) {
    ctx.sm->ReqTransition(s_ck_a);
  }
}

void CkAState::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  ctx.ck_a = ctx.current_byte;
  if (ctx.ck_a == ctx.ck_a_calc) {
    ctx.sm->ReqTransition(s_ck_b);
  } else {
    ctx.sm->ReqTransition(s_sync1);
  }
}

void CkBState::OnEvent(ParserContext &ctx, const SmEvent &, SmTick) {
  ctx.ck_b = ctx.current_byte;
  if (ctx.ck_b == ctx.ck_b_calc) {
    if (ctx.id == ID_NAV_PVT && ctx.len == sizeof(PVTData)) {
      std::memcpy(&ctx.pvt_out, ctx.payload_buf, sizeof(PVTData));
      ctx.new_data_out = true;
    }
  }
  ctx.sm->ReqTransition(s_sync1);
}

// M9NService Implementation

M9NService::M9NService() : ctx_(pvt_data_, new_data_), sm_(ctx_) {
  ctx_.sm = &sm_;
  sm_.Start(s_sync1, 0);
}

void M9NService::ProcessByte(uint8_t byte) {
  ctx_.current_byte = byte; // Set input

  // Post event with id=1 (Byte Received)
  SmEvent ev;
  ev.id = 1;
  sm_.PostEvent(ev, 0);

  // Process transition if referenced
  sm_.Step(0);
}
