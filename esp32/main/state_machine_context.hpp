// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "host_link.hpp"
#include "state_machine.hpp"
#include "states.hpp"
#include "timebase.hpp"

struct StateMachineContext {
  StateMachine<StateMachineContext> *sm = nullptr;
  TimeMs now_ms = 0;
  ServingState serving_state;
  MavlinkWifiState mavlink_wifi_state;
  MavlinkUsbState mavlink_usb_state;
  ServiceState service_state;
  ProgramState program_state;
  EscConfigState esc_config_state;
  WifiLogState wifi_log_state;
  UsbLogState usb_log_state;
  LogPullState log_pull_state;
  // The link whose BEGIN armed the current transfer; Program answers only it.
  HostLink *host_link = nullptr;
};
