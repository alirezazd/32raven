#pragma once

#include "ui.hpp"

class RcChannelsWidget : public IWidget {
 public:
  const char *Name() const override { return "rc_channels"; }

  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  void Render(WidgetContext &ctx) const;

  static constexpr size_t kChannelCount = 4;
};
