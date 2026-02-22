#include "states.hpp"
#include "system.hpp"
#include "user_config.hpp"
#include <cstdint>

int main(void) {
  System::GetInstance().Init(kSystemDefault);

  AppContext ctx;
  ctx.sys = &System::GetInstance();

  IdleState idle_state;
  NotIdleState not_idle_state;
  ctx.idle = &idle_state;
  ctx.not_idle = &not_idle_state;

  StateMachine<AppContext> sm(ctx);
  ctx.sm = &sm;

  sm.Start(idle_state, ctx.sys->Time().MicrosCorrected());

  uint32_t last_imu_seq = 0;

  while (1) {
    __WFI();
    // Run control step ONLY when a new IMU sample is ready
    Icm42688p::Sample raw;
    if (Icm42688p::GetInstance().WaitAndGetLatest(last_imu_seq, raw)) {
      ctx.sm->OnFastTick(raw);
    }
    // Cooperative state step (consumes TIM5 slow-task ticks)
    ctx.sm->Step(static_cast<SmTick>(ctx.sys->Time().MicrosCorrected()));
  }
}
