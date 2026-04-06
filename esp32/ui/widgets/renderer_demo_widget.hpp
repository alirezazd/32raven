#pragma once

#include "ui.hpp"

class RendererDemoWidget : public IWidget {
 public:
  const char *Name() const override { return "renderer_demo"; }

  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  enum class Page : uint8_t {
    kLines = 0,
    kRects,
    kCircles,
    kEllipses,
    kTriangles,
    kRoundRects,
    kCount,
  };

  void Render(WidgetContext &ctx) const;
  void RenderPage(DisplayRenderer &renderer, Page page) const;
  static const char *PageTitle(Page page);

  static constexpr TimeMs kPageDurationMs = 2000;

  Page page_ = Page::kLines;
  TimeMs next_page_ms_ = 0;
};
