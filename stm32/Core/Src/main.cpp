#include "ctx.hpp"
#include "states.hpp"
#include "stm32_config.hpp"
#include "system.hpp"

namespace {

AppContext app{};
StateMachine<AppContext> sm(app);
IdleState idle;

}  // namespace

extern "C" void ExpressMain(void) {
  if (!app.fast_tick_state) {
    return;  // ISR can fire before main() assigns the fast-tick state
  }
  const Icm42688p::SampleBatch batch =
      Icm42688p::GetInstance().GetLatestBatch();
  app.fast_tick_state->OnFastTick(app, batch);
}

int main(void) {
  System::GetInstance().Init(kSystemDefault);
  app.sm = &sm;
  sm.Start(idle);
  while (1) {
    app.sm->Step(app.sys->Time().Micros());
    app.sys->Wdg().Kick();  // main-loop liveness; wedged loop -> reset
    __WFI();
  }
}
