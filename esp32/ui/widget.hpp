// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "display_renderer.hpp"
#include "mavlink.hpp"
#include "timebase.hpp"
#include "wifi.hpp"

class Ui;

struct IWidget;

struct WidgetContext {
  Ui *ui = nullptr;
  DisplayRenderer *renderer = nullptr;
  const WifiController *wifi = nullptr;
  const Mavlink *mavlink = nullptr;

  void LoadWidget(IWidget *widget) const;
};

struct IWidget {
  virtual ~IWidget() = default;
  virtual const char *Name() const = 0;
  virtual void OnEnter(WidgetContext &ctx) { (void)ctx; }
  virtual void OnStep(WidgetContext &ctx, TimeMs now) = 0;
};
