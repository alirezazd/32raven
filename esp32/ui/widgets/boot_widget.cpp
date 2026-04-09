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

void BootWidget::ShowingState::OnEnter(WidgetContext &ctx) {
  DrawCenteredBitmap(ctx, boot_logo::kBitmapData.data());
  widget_.deadline_ms_ =
      TimeAfter(Sys().Timebase().NowMs(), widget_.timeout_ms_);
}

void BootWidget::ShowingState::OnStep(WidgetContext &ctx, SmTick now) {
  if (ctx.sm == nullptr) {
    return;
  }
  if (!TimeReached(now, widget_.deadline_ms_)) {
    return;
  }

  ctx.sm->ReqTransition(widget_.transitioning_state_);
}

void BootWidget::TransitioningState::OnEnter(WidgetContext &ctx) {
  if (ctx.ui == nullptr) {
    return;
  }

  ctx.ui->StartMosaicTransition(Sys().Timebase().NowMs());
}

void BootWidget::TransitioningState::OnStep(WidgetContext &ctx, SmTick now) {
  (void)now;
  if (ctx.sm == nullptr) {
    return;
  }
  if (ctx.ui == nullptr || !ctx.ui->IsTransitionActive()) {
    ctx.sm->ReqTransition(widget_.done_state_);
  }
}

void BootWidget::DoneState::OnEnter(WidgetContext &ctx) {
  ctx.LoadWidget(widget_.next_widget_);
}

void BootWidget::DoneState::OnStep(WidgetContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}

IState<WidgetContext> &BootWidget::InitialState() { return showing_state_; }
