#pragma once
#include "ctx.hpp"
#include "state_machine.hpp"
#include "system.hpp"

struct IdleState;

struct IFastTickState {
  virtual ~IFastTickState() = default;
  virtual void OnFastTick(AppContext &ctx,
                          const Icm42688p::SampleBatch &batch) = 0;
};

struct IdleState : public IState<AppContext>, public IFastTickState {
  const char *Name() const override { return "Idle"; }
  void OnEnter(AppContext &ctx) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnFastTick(AppContext &ctx,
                  const Icm42688p::SampleBatch &batch) override;
  void StepSlow(AppContext &ctx, SmTick now);

 private:
  uint64_t last_imu_send_us_ = 0;
  uint32_t last_status_send_us_ = 0;
  uint32_t slow_loop_counter_ = 0;

  // Diagnostic metrics (shared between OnFastTick and StepSlow)
  uint32_t max_send_us_ = 0;
  uint32_t max_seq_gap_ = 0;
  uint32_t max_raw_dt_us_ = 0;
  uint32_t prev_imu_seq_ = 0;
};
