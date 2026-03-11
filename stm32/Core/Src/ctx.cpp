#include "ctx.hpp"
#include "states.hpp"
#include "system.hpp"

AppContext::AppContext() {
  sys = &System::GetInstance();

  static IdleState idle_state;
  idle = &idle_state;
}
