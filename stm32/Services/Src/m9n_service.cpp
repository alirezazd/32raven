#include "m9n_service.hpp"
#include "system.hpp"
#include "uart.hpp"

namespace {
constexpr uint8_t SYNC1 = 0xB5;
constexpr uint8_t SYNC2 = 0x62;
constexpr uint8_t CLS_NAV = 0x01;
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
  void OnStep(ParserContext &ctx, SmTick) override;
};

struct Sync2State : public BaseState {
  const char *Name() const override { return "Sync2"; }
  void OnStep(ParserContext &ctx, SmTick) override;
};

struct ClassState : public BaseState {
  const char *Name() const override { return "Class"; }
  void OnStep(ParserContext &ctx, SmTick) override;
};

struct IdState : public BaseState {
  const char *Name() const override { return "Id"; }
  void OnStep(ParserContext &ctx, SmTick) override;
};

struct LengthLState : public BaseState {
  const char *Name() const override { return "LengthL"; }
  void OnStep(ParserContext &ctx, SmTick) override;
};

struct LengthHState : public BaseState {
  const char *Name() const override { return "LengthH"; }
  void OnStep(ParserContext &ctx, SmTick) override;
};

struct PayloadState : public BaseState {
  const char *Name() const override { return "Payload"; }
  void OnStep(ParserContext &ctx, SmTick) override;
};

struct CkAState : public BaseState {
  const char *Name() const override { return "CkA"; }
  void OnStep(ParserContext &ctx, SmTick) override;
};

struct CkBState : public BaseState {
  const char *Name() const override { return "CkB"; }
  void OnStep(ParserContext &ctx, SmTick) override;
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

void Sync1State::OnStep(ParserContext &ctx, SmTick) {
  if (ctx.current_byte == SYNC1) {
    ctx.sm->ReqTransition(s_sync2);
  }
}

void Sync2State::OnStep(ParserContext &ctx, SmTick) {
  if (ctx.current_byte == SYNC2) {
    ctx.sm->ReqTransition(s_cls);
  } else if (ctx.current_byte == SYNC1) {
    ctx.sm->ReqTransition(s_sync2);
  } else {
    ctx.sm->ReqTransition(s_sync1);
  }
}

void ClassState::OnStep(ParserContext &ctx, SmTick) {
  ctx.cls = ctx.current_byte;
  ctx.ck_a_calc = 0;
  ctx.ck_b_calc = 0;
  ctx.ck_a_calc += ctx.cls;
  ctx.ck_b_calc += ctx.ck_a_calc;

  ctx.sm->ReqTransition(s_id);
}

void IdState::OnStep(ParserContext &ctx, SmTick) {
  ctx.id = ctx.current_byte;
  ctx.ck_a_calc += ctx.id;
  ctx.ck_b_calc += ctx.ck_a_calc;

  ctx.sm->ReqTransition(s_len_l);
}

void LengthLState::OnStep(ParserContext &ctx, SmTick) {
  ctx.len = ctx.current_byte;
  ctx.ck_a_calc += ctx.current_byte;
  ctx.ck_b_calc += ctx.ck_a_calc;

  ctx.sm->ReqTransition(s_len_h);
}

void LengthHState::OnStep(ParserContext &ctx, SmTick) {
  ctx.len |= (static_cast<uint16_t>(ctx.current_byte) << 8);
  ctx.ck_a_calc += ctx.current_byte;
  ctx.ck_b_calc += ctx.ck_a_calc;

  ctx.payload_idx = 0;

  if (ctx.len > ParserContext::MAX_PAYLOAD_SIZE) {
    ctx.oversize_len_count++;
    ctx.sm->ReqTransition(s_sync1);
  } else if (ctx.len == 0) {
    ctx.sm->ReqTransition(s_ck_a);
  } else {
    ctx.sm->ReqTransition(s_payload);
  }
}

void PayloadState::OnStep(ParserContext &ctx, SmTick) {
  if (ctx.payload_idx < ctx.len) {
    ctx.payload_buf[ctx.payload_idx] = ctx.current_byte;
    ctx.ck_a_calc += ctx.current_byte;
    ctx.ck_b_calc += ctx.ck_a_calc;
    ctx.payload_idx++;
  }

  if (ctx.payload_idx >= ctx.len) {
    ctx.sm->ReqTransition(s_ck_a);
  }
}

void CkAState::OnStep(ParserContext &ctx, SmTick) {
  ctx.ck_a = ctx.current_byte;
  if (ctx.ck_a == ctx.ck_a_calc) {
    ctx.sm->ReqTransition(s_ck_b);
  } else {
    ctx.checksum_fail_count++;
    ctx.sm->ReqTransition(s_sync1);
  }
}

void CkBState::OnStep(ParserContext &ctx, SmTick) {
  ctx.ck_b = ctx.current_byte;
  if (ctx.ck_b == ctx.ck_b_calc) {
    if (ctx.cls == CLS_NAV && ctx.id == ID_NAV_PVT &&
        ctx.len == sizeof(PVTData)) {
      std::memcpy(&ctx.pvt_out, ctx.payload_buf, sizeof(PVTData));
      ctx.new_data_out = true;
      ctx.frame_ok_count++;
      // Use driver-level timestamp (IDLE IRQ time) for precision
      ctx.timestamp_out =
          Uart<UartInstance::kUart2>::GetInstance().GetLastRxTime();
    }
  } else {
    ctx.checksum_fail_count++;
  }
  ctx.sm->ReqTransition(s_sync1);
}

// M9NService Implementation

M9NService::M9NService()
    : ctx_(pvt_data_, new_data_, last_frame_time_), sm_(ctx_) {
  ctx_.sm = &sm_;
  sm_.Start(s_sync1, 0);
}

void M9NService::ProcessByte(uint8_t byte) {
  ctx_.current_byte = byte; // Set input

  // Optimization: Call OnStep directly via StateMachine::Step framework
  // Bypasses PostEvent queue overhead.
  sm_.Step(0);
}
