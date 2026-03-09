#include "states.hpp"
#include "system.hpp"
#include "user_config.hpp"
#include <cstdint>

int main(void) {
  System::GetInstance().Init(kSystemDefault);

  AppContext ctx;
  ctx.sys = &System::GetInstance();

  IdleState idle_state;
  ctx.idle = &idle_state;

  StateMachine<AppContext> sm(ctx);
  ctx.sm = &sm;

  sm.Start(idle_state, ctx.sys->Time().Micros());

  uint32_t last_imu_seq = 0;

  while (1) {
    // Run control step ONLY when a new IMU batch is ready.
    Icm42688p::SampleBatch batch;
    if (Icm42688p::GetInstance().WaitAndGetLatestBatch(last_imu_seq, batch)) {
      if (ctx.fast_tick_state && batch.count > 0) {
        ctx.fast_tick_state->OnFastTick(ctx, batch);
      }
      continue; // keep fast loop highest priority
    }

    // Cooperative state step (best-effort).
    ctx.sm->Step(static_cast<SmTick>(ctx.sys->Time().Micros()));
    __WFI();
  }
}
