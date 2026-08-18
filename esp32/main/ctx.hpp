// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "timebase.hpp"

// Forward declarations
struct ServingState;
struct MavlinkWifiState;
struct MavlinkUsbState;
struct DfuState;
struct ProgramState;
struct EscConfigState;
struct WifiLogState;
struct UsbLogState;
struct LogPullState;

class System;

template <typename Context>
class StateMachine;

struct AppContext {
  System *sys = nullptr;
  StateMachine<AppContext> *sm = nullptr;
  TimeMs now_ms = 0;
  ServingState *serving_state = nullptr;
  MavlinkWifiState *mavlink_wifi_state = nullptr;
  MavlinkUsbState *mavlink_usb_state = nullptr;
  DfuState *dfu_state = nullptr;
  ProgramState *program_state = nullptr;
  EscConfigState *esc_config_state = nullptr;
  WifiLogState *wifi_log_state = nullptr;
  UsbLogState *usb_log_state = nullptr;
  LogPullState *log_pull_state = nullptr;
};
