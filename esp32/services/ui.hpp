#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "display_renderer.hpp"
#include "error_code.hpp"
#include "ssd1306_panel.hpp"
#include "timebase.hpp"

class Ui;
class DisplayCanvas;
struct IWidget;
class BootWidget;
class ErrorWidget;
class MainUiWidget;

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
  const DirtyRange &GetDirtyRange(size_t page) const {
    return dirty_ranges_[page];
  }
  const uint8_t *PageData(size_t page) const {
    return buffer_.data() + (page * kWidth);
  }
  void ClearDirtyRanges();
  const uint8_t *Data() const { return buffer_.data(); }
  size_t Size() const { return buffer_.size(); }

 private:
  void MarkDirtyRange(size_t page, size_t x_begin, size_t x_end);

  std::array<uint8_t, kBufferSize> buffer_{};
  std::array<DirtyRange, kPageCount> dirty_ranges_{};
};

struct WidgetContext {
  Ui *ui = nullptr;
  DisplayRenderer *renderer = nullptr;

  void LoadWidget(IWidget *widget) const;
};

struct IWidget {
  virtual ~IWidget() = default;
  virtual const char *Name() const = 0;
  virtual void OnEnter(WidgetContext &ctx) { (void)ctx; }
  virtual void OnStep(WidgetContext &ctx, TimeMs now) = 0;
};

class Ui {
 public:
  enum class TransitionEffect : uint8_t {
    kSlide,
    kMosaic,
  };

  enum class SlideDirection : uint8_t {
    kLeft,
    kRight,
  };

  enum class AppState : uint8_t {
    kBooting,
    kServing,
    kDfu,
    kMavlinkWifi,
    kProgram,
    kHardError,
  };

  enum class MainScreen : uint8_t {
    kBooting,
    kServing,
    kDfuDisconnected,
    kDfuIdleConnected,
    kMavlinkWifiDisconnected,
    kMavlinkWifiConnected,
    kProgramming,
    kVerifying,
  };

  struct Config {
    uint8_t fps_cap = 30;
    uint8_t boot_logo_timeout_s = 0;
    uint8_t ui_timeout_s = 30;
    uint8_t transition_speed_x = 2;
  };

  static constexpr size_t kWidth = DisplayCanvas::kWidth;
  static constexpr size_t kHeight = DisplayCanvas::kHeight;
  static constexpr TimeMs kDefaultMosaicTransitionDurationMs = 900;
  static constexpr uint8_t kDefaultMosaicBlockSizePx = 8;
  static Ui &GetInstance() {
    static Ui instance;
    return instance;
  }

  void Init(const Config &cfg, Ssd1306Panel *panel);
  void LoadWidget(IWidget *widget);
  void SetAppState(AppState state);
  void SetErrorCode(ErrorCode code);
  void SetErrorRecoverable(bool recoverable);

  DisplayRenderer &Renderer() { return renderer_; }
  const DisplayRenderer &Renderer() const { return renderer_; }
  void NotifyUserActivity();
  void SetInactivityTimeoutSeconds(uint8_t timeout_s);
  void DisableInactivityTimeout();
  void SetFadeOut(uint8_t interval);
  void DisableFadeOut();
  void DisplayOn();
  void DisplayOff();
  bool IsScreenOn() const;
  bool IsTransitionActive() const;
  void StartMosaicTransition(
      TimeMs now, TimeMs duration_ms = kDefaultMosaicTransitionDurationMs,
      uint8_t max_block_size = kDefaultMosaicBlockSizePx);

  TimeMs GetFrameIntervalMs() const;
  size_t Width() const { return kWidth; }
  size_t Height() const { return kHeight; }
  const DisplayCanvas &Canvas() const { return canvas_; }

 private:
  friend class System;

  Ui();
  static void TaskEntry(void *param);
  void Task();
  void Step(TimeMs now, WidgetContext &ctx);
  void WakeTask() const;
  void ServiceTransition(TimeMs now);
  AppState CurrentAppState() const;
  ErrorCode CurrentErrorCode() const;
  bool CurrentErrorRecoverable() const;
  uint8_t CurrentInactivityTimeoutSeconds() const;
  MainScreen DeriveMainScreen(AppState state) const;
  void SyncPresentation(TimeMs now);
  void RenderMainScreenSnapshot(MainScreen screen, TimeMs now,
                                DisplayCanvas &dst);
  void StartMosaicTransitionToScreen(MainScreen next_screen, TimeMs now,
                                     TimeMs duration_ms,
                                     uint8_t max_block_size);
  void StartMainScreenTransition(MainScreen next_screen, TimeMs now,
                                 SlideDirection direction, TimeMs duration_ms,
                                 float accel_ratio, float decel_ratio);
  bool RenderActiveTransition(TimeMs now);
  void StopTransition();
  bool ApplyPendingWidget(WidgetContext &ctx);
  void FlushIfDirty();
  void UpdatePowerState(TimeMs now);
  SlideDirection TransitionDirectionForScreens(MainScreen from,
                                               MainScreen to) const;
  bool ShouldSkipMainScreenTransition(MainScreen from, MainScreen to) const;
  bool ShouldUseMosaicMainScreenTransition(MainScreen from,
                                           MainScreen to) const;
  TimeMs MosaicDurationForScreens(MainScreen from, MainScreen to) const;
  uint8_t ScreenGroup(MainScreen screen) const;

  Config cfg_{};
  Ssd1306Panel *panel_ = nullptr;
  DisplayCanvas canvas_{};
  DisplayRenderer renderer_;
  BootWidget *boot_widget_ = nullptr;
  MainUiWidget *main_ui_widget_ = nullptr;
  ErrorWidget *error_widget_ = nullptr;
  IWidget *current_widget_ = nullptr;
  IWidget *pending_widget_ = nullptr;
  void *task_handle_ = nullptr;  // TaskHandle_t
  AppState app_state_ = AppState::kBooting;
  ErrorCode error_code_ = ErrorCode::kUnknown;
  bool error_recoverable_ = false;
  MainScreen main_screen_ = MainScreen::kBooting;
  TimeMs next_step_ms_ = 0;
  bool transition_active_ = false;
  struct TransitionState {
    TransitionEffect effect = TransitionEffect::kSlide;
    MainScreen target_screen = MainScreen::kBooting;
    SlideDirection direction = SlideDirection::kLeft;
    TimeMs start_ms = 0;
    TimeMs duration_ms = 180;
    float accel_ratio = 0.18f;
    float decel_ratio = 0.22f;
    uint8_t mosaic_block_size = 2;
  } transition_{};
  DisplayCanvas transition_from_canvas_{};
  DisplayCanvas transition_to_canvas_{};
  TimeMs last_user_activity_ms_ = 0;
  TimeMs inactivity_fade_start_ms_ = 0;
  bool display_on_ = true;
  bool inactivity_fade_active_ = false;

  ~Ui() = default;
  Ui(const Ui &) = delete;
  Ui &operator=(const Ui &) = delete;
};
