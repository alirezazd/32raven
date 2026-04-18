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
constexpr char kStatusSampleLine[] = "LQ:100 STALE";
constexpr int16_t kTextInsetX = 1;
constexpr int16_t kLineSpacing = 1;
constexpr int16_t kStatusGap = 3;

}  // namespace

void RcChannelsWidget::OnEnter(WidgetContext &ctx) {
  Render(ctx);
}

void RcChannelsWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  (void)now;
  Render(ctx);
}

void RcChannelsWidget::Render(WidgetContext &ctx) const {
  if (ctx.ui == nullptr || ctx.renderer == nullptr) {
    return;
  }

  Ui &ui = *ctx.ui;
  DisplayRenderer &renderer = *ctx.renderer;
  const DisplayTextBounds sample_bounds =
      renderer.MeasureText(kSampleLine, kTextStyle);
  const DisplayTextBounds status_bounds =
      renderer.MeasureText(kStatusSampleLine, kTextStyle);
  if (sample_bounds.width == 0 || sample_bounds.height == 0 ||
      status_bounds.width == 0 || status_bounds.height == 0) {
    renderer.Clear();
    return;
  }

  const auto snapshot =
      Sys().Mavlink().GetRcChannelsSnapshot(Sys().Timebase().NowMs());

  const int16_t line_height = static_cast<int16_t>(sample_bounds.height);
  const int16_t line_step = static_cast<int16_t>(line_height + kLineSpacing);
  const int16_t status_height = static_cast<int16_t>(status_bounds.height);
  const int16_t total_height = static_cast<int16_t>(
      (kChannelCount * line_height) + ((kChannelCount - 1) * kLineSpacing) +
      kStatusGap + status_height);
  int16_t line_top = std::max<int16_t>(
      0, static_cast<int16_t>(ui.Height() - total_height) / 2);

  renderer.Clear();

  for (size_t index = 0; index < kChannelCount; ++index) {
    const uint16_t value =
        snapshot.have_data ? snapshot.msg.channels[index] : 0u;
    char line[16];
    std::snprintf(line, sizeof(line), "ch%u: %u",
                  static_cast<unsigned>(index + 1u),
                  static_cast<unsigned>(value));
    const int16_t cursor_y = static_cast<int16_t>(line_top - sample_bounds.y);
    renderer.DrawText(line, kTextInsetX, cursor_y, kTextStyle);
    line_top = static_cast<int16_t>(line_top + line_step);
  }

  line_top = static_cast<int16_t>(line_top + kStatusGap);
  char status[20];
  if (snapshot.have_data) {
    std::snprintf(status, sizeof(status), "LQ:%3u %s",
                  static_cast<unsigned>(snapshot.msg.rssi),
                  snapshot.live ? "LIVE" : "STALE");
  } else {
    std::snprintf(status, sizeof(status), "LQ: -- STALE");
  }
  renderer.DrawText(status, kTextInsetX,
                    static_cast<int16_t>(line_top - status_bounds.y),
                    kTextStyle);
}
