#include "boot_widget.hpp"

#include "bitmap/boot_logo.hpp"
#include "system.hpp"
#include "timebase.hpp"

namespace {

size_t CenterOffset(size_t content_size, size_t bounds_size) {
  if (content_size >= bounds_size) {
    return 0;
  }
  return (bounds_size - content_size) / 2;
}

void DrawCenteredBitmap(WidgetContext &ctx, const uint8_t *bitmap_data) {
  if (ctx.renderer == nullptr || bitmap_data == nullptr) {
    return;
  }

  const size_t offset_x =
      CenterOffset(boot_logo::kVisibleWidth, ctx.renderer->Width());
  const size_t offset_y =
      CenterOffset(boot_logo::kVisibleHeight, ctx.renderer->Height());

  ctx.renderer->Clear();
  ctx.renderer->DrawBitmap(bitmap_data, boot_logo::kVisibleWidth,
                           boot_logo::kVisibleHeight, offset_x, offset_y);
}

}  // namespace

void BootWidget::OnEnter(WidgetContext &ctx) {
  DrawCenteredBitmap(ctx, boot_logo::kBitmapData.data());
  deadline_ms_ = TimeAfter(Sys().Timebase().NowMs(), timeout_ms_);
  mode_ = Mode::kShowing;
}

void BootWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  switch (mode_) {
    case Mode::kShowing:
      if (!TimeReached(now, deadline_ms_)) {
        return;
      }
      if (ctx.ui != nullptr) {
        ctx.ui->StartMosaicTransition(now);
      }
      mode_ = Mode::kTransitioning;
      return;

    case Mode::kTransitioning:
      if (ctx.ui != nullptr && ctx.ui->IsTransitionActive()) {
        return;
      }
      ctx.LoadWidget(next_widget_);
      mode_ = Mode::kDone;
      return;

    case Mode::kDone:
    default:
      return;
  }
}
