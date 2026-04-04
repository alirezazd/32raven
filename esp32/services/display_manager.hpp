#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "display_renderer.hpp"
#include "ssd1306_panel.hpp"
#include "state_machine.hpp"
#include "timebase.hpp"

class DisplayManager;
class DisplayCanvas;
struct IWidget;

class DisplayCanvas : public RenderCanvas {
 public:
  struct DirtyRange {
    bool dirty = false;
    size_t x_begin = 0;
    size_t x_end = 0;  // Exclusive.
  };

  static constexpr size_t kWidth = Ssd1306Panel::kWidth;
  static constexpr size_t kHeight = Ssd1306Panel::kHeight;
  static constexpr size_t kPageCount = Ssd1306Panel::kPageCount;
  static constexpr size_t kBufferSize = Ssd1306Panel::kFramebufferSize;

  void Clear();
  void Fill(bool on);
  size_t Width() const override { return kWidth; }
  size_t Height() const override { return kHeight; }
  bool SetPixel(size_t x, size_t y, bool on) override;
  bool DrawPackedBitmap(const uint8_t *bitmap_data, size_t width, size_t height,
                        size_t offset_x, size_t offset_y);
  bool HasDirtyRanges() const;
  const DirtyRange &GetDirtyRange(size_t page) const { return dirty_ranges_[page]; }
  const uint8_t *PageData(size_t page) const { return buffer_.data() + (page * kWidth); }
  void ClearDirtyRanges();
  const uint8_t *Data() const { return buffer_.data(); }
  size_t Size() const { return buffer_.size(); }

 private:
  void MarkDirtyRange(size_t page, size_t x_begin, size_t x_end);

  std::array<uint8_t, kBufferSize> buffer_{};
  std::array<DirtyRange, kPageCount> dirty_ranges_{};
};

struct WidgetContext {
  DisplayManager *display = nullptr;
  DisplayRenderer *renderer = nullptr;
  StateMachine<WidgetContext> *sm = nullptr;

  void LoadWidget(IWidget *widget) const;
};

struct IWidget {
  virtual ~IWidget() = default;
  virtual const char *Name() const = 0;
  virtual void OnEnter(WidgetContext &ctx) { (void)ctx; }
  virtual void OnStep(WidgetContext &ctx, TimeMs now) = 0;
  virtual void OnExit(WidgetContext &ctx) { (void)ctx; }
};

class StateMachineWidget : public IWidget {
 public:
  void OnEnter(WidgetContext &ctx) override final;
  void OnStep(WidgetContext &ctx, TimeMs now) override final;
  void OnExit(WidgetContext &ctx) override;

 protected:
  virtual IState<WidgetContext> &InitialState() = 0;
  WidgetContext &Context() { return ctx_; }

 private:
  WidgetContext ctx_{};
  StateMachine<WidgetContext> sm_{ctx_};
};

class DisplayManager {
 public:
  struct Config {
    uint8_t fps_cap = 30;
    uint8_t boot_logo_timeout_s = 0;
    uint8_t ui_timeout_s = 30;
  };

  static constexpr size_t kWidth = DisplayCanvas::kWidth;
  static constexpr size_t kHeight = DisplayCanvas::kHeight;
  static DisplayManager &GetInstance() {
    static DisplayManager instance;
    return instance;
  }

  void Init(const Config &cfg, Ssd1306Panel *panel);
  void LoadWidget(IWidget *widget);

  DisplayRenderer &Renderer() { return renderer_; }
  const DisplayRenderer &Renderer() const { return renderer_; }
  void NotifyUserActivity();
  void SetFadeOut(uint8_t interval);
  void DisableFadeOut();
  void DisplayOn();
  void DisplayOff();

  TimeMs GetFrameIntervalMs() const;
  size_t Width() const { return kWidth; }
  size_t Height() const { return kHeight; }
  const DisplayCanvas &Canvas() const { return canvas_; }

 private:
  friend class System;

  DisplayManager();
  static void TaskEntry(void *param);
  void Task();
  bool ApplyPendingWidget(WidgetContext &ctx);
  void FlushIfDirty();
  void UpdatePowerState(TimeMs now);

  Config cfg_{};
  Ssd1306Panel *panel_ = nullptr;
  DisplayCanvas canvas_{};
  DisplayRenderer renderer_;
  IWidget *current_widget_ = nullptr;
  IWidget *pending_widget_ = nullptr;
  void *task_handle_ = nullptr;  // TaskHandle_t
  TimeMs last_user_activity_ms_ = 0;
  TimeMs inactivity_fade_start_ms_ = 0;
  bool display_on_ = true;
  bool inactivity_fade_active_ = false;

  ~DisplayManager() = default;
  DisplayManager(const DisplayManager &) = delete;
  DisplayManager &operator=(const DisplayManager &) = delete;
};
