#include "m10_service.hpp"

#include "m10_reg.hpp"
#include "system.hpp"
#include "uart.hpp"

struct M10BaseState : public IState<M10ParserContext> {
  const char *Name() const override { return "Base"; }
  void OnStep(M10ParserContext &, SmTick) override {}
};

struct M10Sync1State : public M10BaseState {
  const char *Name() const override { return "Sync1"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

struct M10Sync2State : public M10BaseState {
  const char *Name() const override { return "Sync2"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

struct M10ClassState : public M10BaseState {
  const char *Name() const override { return "Class"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

struct M10IdState : public M10BaseState {
  const char *Name() const override { return "Id"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

struct M10LengthLState : public M10BaseState {
  const char *Name() const override { return "LengthL"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

struct M10LengthHState : public M10BaseState {
  const char *Name() const override { return "LengthH"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

struct M10PayloadState : public M10BaseState {
  const char *Name() const override { return "Payload"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

struct M10CkAState : public M10BaseState {
  const char *Name() const override { return "CkA"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

struct M10CkBState : public M10BaseState {
  const char *Name() const override { return "CkB"; }
  void OnStep(M10ParserContext &ctx, SmTick) override;
};

static M10Sync1State s_sync1;
static M10Sync2State s_sync2;
static M10ClassState s_cls;
static M10IdState s_id;
static M10LengthLState s_len_l;
static M10LengthHState s_len_h;
static M10PayloadState s_payload;
static M10CkAState s_ck_a;
static M10CkBState s_ck_b;

void M10Sync1State::OnStep(M10ParserContext &ctx, SmTick) {
  if (ctx.current_byte == UBX::kSync1) {
    ctx.sm->ReqTransition(s_sync2);
  }
}

void M10Sync2State::OnStep(M10ParserContext &ctx, SmTick) {
  if (ctx.current_byte == UBX::kSync2) {
    ctx.sm->ReqTransition(s_cls);
  } else if (ctx.current_byte == UBX::kSync1) {
    ctx.sm->ReqTransition(s_sync2);
  } else {
    ctx.sm->ReqTransition(s_sync1);
  }
}

void M10ClassState::OnStep(M10ParserContext &ctx, SmTick) {
  ctx.cls = ctx.current_byte;
  ctx.ck_a_calc = 0;
  ctx.ck_b_calc = 0;
  ctx.ck_a_calc += ctx.cls;
  ctx.ck_b_calc += ctx.ck_a_calc;

  ctx.sm->ReqTransition(s_id);
}

void M10IdState::OnStep(M10ParserContext &ctx, SmTick) {
  ctx.id = ctx.current_byte;
  ctx.ck_a_calc += ctx.id;
  ctx.ck_b_calc += ctx.ck_a_calc;

  ctx.sm->ReqTransition(s_len_l);
}

void M10LengthLState::OnStep(M10ParserContext &ctx, SmTick) {
  ctx.len = ctx.current_byte;
  ctx.ck_a_calc += ctx.current_byte;
  ctx.ck_b_calc += ctx.ck_a_calc;

  ctx.sm->ReqTransition(s_len_h);
}

void M10LengthHState::OnStep(M10ParserContext &ctx, SmTick) {
  ctx.len |= (static_cast<uint16_t>(ctx.current_byte) << 8);
  ctx.ck_a_calc += ctx.current_byte;
  ctx.ck_b_calc += ctx.ck_a_calc;

  ctx.payload_idx = 0;

  if (ctx.len > M10ParserContext::kMaxPayloadSize) {
    ctx.oversize_len_count++;
    ctx.sm->ReqTransition(s_sync1);
  } else if (ctx.len == 0) {
    ctx.sm->ReqTransition(s_ck_a);
  } else {
    ctx.sm->ReqTransition(s_payload);
  }
}

void M10PayloadState::OnStep(M10ParserContext &ctx, SmTick) {
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

void M10CkAState::OnStep(M10ParserContext &ctx, SmTick) {
  ctx.ck_a = ctx.current_byte;
  if (ctx.ck_a == ctx.ck_a_calc) {
    ctx.sm->ReqTransition(s_ck_b);
  } else {
    ctx.checksum_fail_count++;
    ctx.sm->ReqTransition(s_sync1);
  }
}

void M10CkBState::OnStep(M10ParserContext &ctx, SmTick) {
  ctx.ck_b = ctx.current_byte;

  if (ctx.ck_b != ctx.ck_b_calc) {
    ctx.checksum_fail_count++;
    ctx.sm->ReqTransition(s_sync1);
    return;
  }

  if (ctx.cls == UBX::kClsNav && ctx.id == UBX::kIdNavPvt &&
      ctx.len == sizeof(M10PVTData)) {
    std::memcpy(&ctx.pvt_out, ctx.payload_buf, sizeof(M10PVTData));
    ctx.epoch_ready = true;

    ctx.pvt_itow_ms = ctx.pvt_out.iTOW;
    ctx.pvt_rx_us = Uart<UartInstance::kUart2>::GetInstance().GetLastRxTime();

    ctx.frame_ok_count++;
  } else if (ctx.cls == UBX::kClsNav && ctx.id == UBX::kIdNavDop &&
             ctx.len == sizeof(M10DOPData)) {
    std::memcpy(&ctx.dop_out, ctx.payload_buf, sizeof(M10DOPData));
    ctx.dop_ready = true;
    ctx.frame_ok_count++;
  } else if (ctx.cls == UBX::kClsNav && ctx.id == UBX::kIdNavCov &&
             ctx.len == sizeof(M10COVData)) {
    std::memcpy(&ctx.cov_out, ctx.payload_buf, sizeof(M10COVData));
    ctx.cov_ready = true;
    ctx.frame_ok_count++;
  } else if (ctx.cls == UBX::kClsNav && ctx.id == UBX::kIdNavEoe &&
             ctx.len == 4) {
    uint32_t eoe_itow_ms = (uint32_t)ctx.payload_buf[0] |
                           ((uint32_t)ctx.payload_buf[1] << 8) |
                           ((uint32_t)ctx.payload_buf[2] << 16) |
                           ((uint32_t)ctx.payload_buf[3] << 24);

    uint64_t now_us = Uart<UartInstance::kUart2>::GetInstance().GetLastRxTime();
    constexpr uint64_t kMaxAgeUs = 150000;

    if (ctx.epoch_ready && eoe_itow_ms == ctx.pvt_itow_ms &&
        (uint64_t)(now_us - ctx.pvt_rx_us) < kMaxAgeUs) {
      ctx.new_data_out = true;
    }

    ctx.epoch_ready = false;
    ctx.dop_ready = false;
    ctx.cov_ready = false;

    ctx.frame_ok_count++;
  } else {
    ctx.frame_ok_count++;
  }

  ctx.sm->ReqTransition(s_sync1);
}

M10Service::M10Service()
    : ctx_(pvt_data_, dop_data_, cov_data_, new_data_), sm_(ctx_) {
  ctx_.sm = &sm_;
  sm_.Start(s_sync1);
}

void M10Service::ProcessByte(uint8_t byte) {
  ctx_.current_byte = byte;
  sm_.Step(0);
}
