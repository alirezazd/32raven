// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

class System;

template <typename Context>
class StateMachine;
struct IFastTickState;

struct AppContext {
  AppContext();

  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  IFastTickState *fast_tick_state = nullptr;
};
