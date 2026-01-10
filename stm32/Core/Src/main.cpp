#include "states.hpp"
#include "system.hpp"
#include "user_config.hpp"
#include <cstdint>

extern "C" volatile uint32_t g_dma5_irq_seen;

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

  sm.Start(idle_state, MICROS() / 1000);

  while (1) {
    uint32_t now = MICROS() / 1000;
    sm.Step(now);
  }
}