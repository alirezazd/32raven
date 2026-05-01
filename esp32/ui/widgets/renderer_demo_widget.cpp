#include "renderer_demo_widget.hpp"

#include <algorithm>

#include "system.hpp"
#include "timebase.hpp"

namespace {

constexpr DisplayTextStyle kTitleStyle{
    .scale = 1,
    .font = DisplayTextStyle::Font::kCompact,
};
constexpr int16_t kTitleInsetX = 1;
constexpr int16_t kTitleTopY = 0;
constexpr int16_t kShapeTopY = 10;

void DrawTitle(DisplayRenderer &renderer, const char *title) {
  const DisplayTextBounds bounds = renderer.MeasureText(title, kTitleStyle);
  const int16_t baseline_y = static_cast<int16_t>(kTitleTopY - bounds.y);
  renderer.DrawText(title, kTitleInsetX, baseline_y, kTitleStyle);
}

}  // namespace

void RendererDemoWidget::OnEnter(WidgetContext &ctx) {
  page_ = Page::kLines;
  Render(ctx);
  next_page_ms_ = TimeAfter(Sys().Timebase().NowMs(), kPageDurationMs);
}

void RendererDemoWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  if (ctx.renderer == nullptr) {
    return;
  }

  if (!TimeReached(now, next_page_ms_)) {
    return;
  }

  const uint8_t next_index =
      (static_cast<uint8_t>(page_) + 1u) % static_cast<uint8_t>(Page::kCount);
  page_ = static_cast<Page>(next_index);
  if (page_ == Page::kLines) {
    Sys().TonePlayer().PlayBuiltin(TonePlayer::BuiltinTone::kConfirm);
  }

  Render(ctx);
  next_page_ms_ = TimeAfter(now, kPageDurationMs);
}

void RendererDemoWidget::Render(WidgetContext &ctx) const {
  if (ctx.renderer == nullptr) {
    return;
  }

  DisplayRenderer &renderer = *ctx.renderer;
  renderer.Clear();
  DrawTitle(renderer, PageTitle(page_));
  RenderPage(renderer, page_);
}

void RendererDemoWidget::RenderPage(DisplayRenderer &renderer,
                                    Page page) const {
  switch (page) {
    case Page::kLines:
      renderer.DrawLine(4, 34, 67, 12);
      renderer.DrawFastHLine(8, 18, 54);
      renderer.DrawFastVLine(36, kShapeTopY + 1, 24);
      break;

    case Page::kRects:
      renderer.DrawRect(5, 13, 24, 16);
      renderer.FillRect(40, 15, 20, 12);
      renderer.DrawRect(37, 12, 26, 18);
      break;

    case Page::kCircles:
      renderer.DrawCircle(19, 24, 9);
      renderer.FillCircle(52, 24, 7);
      renderer.DrawCircle(52, 24, 10);
      break;

    case Page::kEllipses:
      renderer.DrawEllipse(19, 24, 12, 8);
      renderer.FillEllipse(52, 24, 9, 6);
      renderer.DrawEllipse(52, 24, 12, 8);
      break;

    case Page::kTriangles:
      renderer.DrawTriangle(6, 31, 18, 12, 30, 31);
      renderer.FillTriangle(41, 31, 53, 12, 65, 31);
      renderer.DrawTriangle(38, 31, 53, 10, 68, 31);
      break;

    case Page::kRoundRects:
      renderer.DrawRoundRect(4, 12, 26, 18, 4);
      renderer.FillRoundRect(39, 14, 22, 14, 4);
      renderer.DrawRoundRect(36, 11, 28, 20, 5);
      break;

    case Page::kCount:
    default:
      break;
  }
}

const char *RendererDemoWidget::PageTitle(Page page) {
  switch (page) {
    case Page::kLines:
      return "lines";
    case Page::kRects:
      return "rects";
    case Page::kCircles:
      return "circles";
    case Page::kEllipses:
      return "ellipses";
    case Page::kTriangles:
      return "triangles";
    case Page::kRoundRects:
      return "roundrects";
    case Page::kCount:
    default:
      return "";
  }
}
