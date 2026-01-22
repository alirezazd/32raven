#include "ctx.hpp"
#include "states.hpp"
#include "system.hpp"

// The implementation file is the ONLY place where
// we bridge the circular dependency.
AppContext::AppContext() {
  // 1) Link System (Singleton)
  sys = &System::GetInstance();

  // 2) Allocate States
  // Using static ensures they live forever and are allocated once.
  static ServingState serving;
  static DfuState dfu;
  static BridgeState bridge;
  static ProgramState prog;
  static HardErrorState err;

  serving_state = &serving;
  dfu_state = &dfu;
  bridge_state = &bridge;
  program_state = &prog;
  hard_error_state = &err;
}
