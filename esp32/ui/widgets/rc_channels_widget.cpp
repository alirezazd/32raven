#include "rc_channels_widget.hpp"

#include <algorithm>
#include <array>
#include <cstdio>

#include "system.hpp"

namespace {

constexpr DisplayTextStyle kTextStyle{
    .scale = 1,
};
constexpr char kSampleLine[] = "ch1: 2000";
constexpr int16_t kTextInsetX = 1;
constexpr int16_t kLineSpacing = 1;

}  // namespace

void RcChannelsWidget::OnEnter(WidgetContext &ctx) {
  Render(ctx);
}

void RcChannelsWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  (void)now;
  Render(ctx);
}

void RcChannelsWidget::Render(WidgetContext &ctx) const {
  if (ctx.display == nullptr || ctx.renderer == nullptr) {
    return;
  }

  DisplayManager &display = *ctx.display;
  DisplayRenderer &renderer = *ctx.renderer;
  const DisplayTextBounds sample_bounds =
      renderer.MeasureText(kSampleLine, kTextStyle);
  if (sample_bounds.width == 0 || sample_bounds.height == 0) {
    renderer.Clear();
    return;
  }

  const int16_t line_height = static_cast<int16_t>(sample_bounds.height);
  const int16_t line_step = static_cast<int16_t>(line_height + kLineSpacing);
  const int16_t total_height = static_cast<int16_t>(
      (kChannelCount * line_height) + ((kChannelCount - 1) * kLineSpacing));
  int16_t line_top = std::max<int16_t>(
      0, static_cast<int16_t>(display.Height() - total_height) / 2);

  const RcState &rc_state = Sys().Mavlink().GetRcState();
  std::array<uint16_t, kChannelCount> values{};
  for (size_t index = 0; index < kChannelCount; ++index) {
    values[index] = rc_state.channels[index];
  }

  renderer.Clear();

  for (size_t index = 0; index < kChannelCount; ++index) {
    char line[16];
    std::snprintf(line, sizeof(line), "ch%u: %u",
                  static_cast<unsigned>(index + 1),
                  static_cast<unsigned>(values[index]));
    const int16_t cursor_y = static_cast<int16_t>(line_top - sample_bounds.y);
    renderer.DrawText(line, kTextInsetX, cursor_y, kTextStyle);
    line_top = static_cast<int16_t>(line_top + line_step);
  }
}
