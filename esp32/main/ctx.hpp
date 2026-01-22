#pragma once

// Forward declarations
struct ServingState;
struct DfuState;
struct BridgeState;
struct ProgramState;
struct HardErrorState;

class System;

template <typename Context> class StateMachine;

struct AppContext {
  AppContext(); // Logic moved to cpp

  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;

  ServingState *serving_state = nullptr;
  DfuState *dfu_state = nullptr;
  BridgeState *bridge_state = nullptr;
  ProgramState *program_state = nullptr;
  HardErrorState *hard_error_state = nullptr;
};