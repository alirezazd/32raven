// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

struct IdleState;
struct ArmedState;
struct EscConfigState;
struct IFastTickState;

class System;

template <typename Context>
class StateMachine;

struct AppContext {
  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  IFastTickState *fast_tick_state = nullptr;
  IdleState *idle_state = nullptr;
  ArmedState *armed_state = nullptr;
  EscConfigState *esc_config_state = nullptr;
};
