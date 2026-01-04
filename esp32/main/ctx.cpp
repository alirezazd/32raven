#include "ctx.hpp"
#include "states.hpp"
#include "system.hpp"

// The implementation file is the ONLY place where
// we bridge the circular dependency.
AppContext::AppContext() {
  // 1) Link System (Singleton)
  sys = &System::getInstance();

  // 2) Allocate States
  // Using static ensures they live forever and are allocated once.
  static IdleState s_idle;
  static ListenState s_listen;

  idle_state = &s_idle;
  listen_state = &s_listen;
}
