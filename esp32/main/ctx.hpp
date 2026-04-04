#pragma once

// Forward declarations
struct ServingState;
struct DfuState;
struct ProgramState;
struct HardErrorState;

class System;

template <typename Context>
class StateMachine;

struct AppContext {
  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  ServingState *serving_state = nullptr;
  DfuState *dfu_state = nullptr;
  ProgramState *program_state = nullptr;
  HardErrorState *hard_error_state = nullptr;
};
