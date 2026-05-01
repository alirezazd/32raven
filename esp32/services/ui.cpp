#include "ui.hpp"

#include <algorithm>
#include <cmath>

#include "boot_widget.hpp"
#include "error_widget.hpp"
#include "main_ui_widget.hpp"
#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {

constexpr const char *kTag = "ui";
constexpr uint32_t kTaskStackDepthWords = 3072;
constexpr TimeMs kUiFadeOutDurationMs = 2000;
constexpr uint8_t kUiFadeOutInterval = 0;
portMUX_TYPE g_ui_lock = portMUX_INITIALIZER_UNLOCKED;

struct TransitionPreset {
  TimeMs duration_ms = 220;
  float accel_ratio = 0.05f;
  float decel_ratio = 0.45f;
};

bool ReadPackedPixel(const uint8_t *bitmap_data, size_t width, size_t height,
                     size_t x, size_t y) {
  if (bitmap_data == nullptr || x >= width || y >= height) {
    return false;
  }
  const size_t page = y / 8;
  const size_t bit = y % 8;
  const uint8_t value = bitmap_data[(page * width) + x];
  return (value & (1u << bit)) != 0;
}

float Clamp01(float value) { return std::clamp(value, 0.0f, 1.0f); }

float SwipeProgress(float t, float accel_ratio, float decel_ratio) {
  t = Clamp01(t);
  accel_ratio = std::max(0.0f, accel_ratio);
  decel_ratio = std::max(0.0f, decel_ratio);

  const float total_ease = accel_ratio + decel_ratio;
  if (total_ease >= 0.95f && total_ease > 0.0f) {
    const float scale = 0.95f / total_ease;
    accel_ratio *= scale;
    decel_ratio *= scale;
  }

  const float cruise_ratio = std::max(0.0f, 1.0f - accel_ratio - decel_ratio);
  const float velocity =
      1.0f / (cruise_ratio + 0.5f * (accel_ratio + decel_ratio));

  if (accel_ratio > 0.0f && t < accel_ratio) {
    return 0.5f * velocity * t * t / accel_ratio;
  }

  const float accel_distance = 0.5f * velocity * accel_ratio;
  if (t < accel_ratio + cruise_ratio) {
    return accel_distance + velocity * (t - accel_ratio);
  }

  if (decel_ratio <= 0.0f) {
    return 1.0f;
  }

  const float u = t - accel_ratio - cruise_ratio;
  const float cruise_distance = velocity * cruise_ratio;
  const float decel_distance =
      velocity * u - 0.5f * velocity * u * u / decel_ratio;
  return Clamp01(accel_distance + cruise_distance + decel_distance);
}

TickType_t WaitTicksFromMs(TimeMs wait_ms) {
  return pdMS_TO_TICKS(wait_ms > 0 ? wait_ms : 1);
}

BootWidget &BootWidgetInstance() {
  static BootWidget widget;
  return widget;
}

TransitionPreset TransitionPresetForSpeed(uint8_t speed_x) {
  switch (speed_x) {
    case 1:
      return {
          .duration_ms = 440,
          .accel_ratio = 0.08f,
          .decel_ratio = 0.52f,
      };
    case 3:
      return {
          .duration_ms = 147,
          .accel_ratio = 0.03f,
          .decel_ratio = 0.34f,
      };
    case 2:
    default:
      // Keep 2x aligned with the current transition feel.
      return {
          .duration_ms = 220,
          .accel_ratio = 0.05f,
          .decel_ratio = 0.45f,
      };
  }
}

}  // namespace

Ui::Ui()
    : renderer_(&canvas_),
      boot_widget_(&BootWidgetInstance()),
      main_ui_widget_(&MainUiWidget::GetInstance()),
      error_widget_(&ErrorWidget::GetInstance()) {}

void DisplayCanvas::Clear() { Fill(false); }

void DisplayCanvas::MarkDirtyRange(size_t page, size_t x_begin, size_t x_end) {
  if (page >= kPageCount || x_begin >= x_end || x_begin >= kWidth) {
    return;
  }

  x_end = std::min(x_end, kWidth);
  DirtyRange &range = dirty_ranges_[page];
  if (!range.dirty) {
    range.dirty = true;
    range.x_begin = x_begin;
    range.x_end = x_end;
    return;
  }

  range.x_begin = std::min(range.x_begin, x_begin);
  range.x_end = std::max(range.x_end, x_end);
}

void DisplayCanvas::Fill(bool on) {
  const uint8_t fill_value = on ? 0xFF : 0x00;

  for (size_t page = 0; page < kPageCount; ++page) {
    size_t dirty_begin = kWidth;
    size_t dirty_end = 0;
    const size_t page_offset = page * kWidth;

    for (size_t x = 0; x < kWidth; ++x) {
      uint8_t &value = buffer_[page_offset + x];
      if (value == fill_value) {
        continue;
      }

      value = fill_value;
      dirty_begin = std::min(dirty_begin, x);
      dirty_end = std::max(dirty_end, x + 1);
    }

    if (dirty_begin < dirty_end) {
      MarkDirtyRange(page, dirty_begin, dirty_end);
    }
  }
}

bool DisplayCanvas::SetPixel(size_t x, size_t y, bool on) {
  if (x >= kWidth || y >= kHeight) {
    return false;
  }

  const size_t page = y / 8;
  const uint8_t mask = static_cast<uint8_t>(1u << (y % 8));
  uint8_t &value = buffer_[(page * kWidth) + x];
  const uint8_t previous = value;
  if (on) {
    value |= mask;
  } else {
    value &= static_cast<uint8_t>(~mask);
  }
  if (value != previous) {
    MarkDirtyRange(page, x, x + 1);
  }
  return true;
}

bool DisplayCanvas::DrawPackedBitmap(const uint8_t *bitmap_data, size_t width,
                                     size_t height, size_t offset_x,
                                     size_t offset_y) {
  if (bitmap_data == nullptr || width == 0 || height == 0 || width > kWidth ||
      height > kHeight || offset_x + width > kWidth ||
      offset_y + height > kHeight) {
    return false;
  }

  for (size_t src_y = 0; src_y < height; ++src_y) {
    for (size_t src_x = 0; src_x < width; ++src_x) {
      if (!ReadPackedPixel(bitmap_data, width, height, src_x, src_y)) {
        continue;
      }
      SetPixel(offset_x + src_x, offset_y + src_y, true);
    }
  }
  return true;
}

bool DisplayCanvas::HasDirtyRanges() const {
  for (const DirtyRange &range : dirty_ranges_) {
    if (range.dirty) {
      return true;
    }
  }
  return false;
}

void DisplayCanvas::ClearDirtyRanges() { dirty_ranges_.fill({}); }

void WidgetContext::LoadWidget(IWidget *widget) const {
  if (ui != nullptr) {
    ui->LoadWidget(widget);
  }
}

void Ui::Init(const Config &cfg, Ssd1306Panel *panel) {
  static StaticTask_t task_buffer;
  static StackType_t task_stack[kTaskStackDepthWords];

  cfg_ = cfg;
  if (panel == nullptr || cfg_.fps_cap == 0) {
    Panic(ErrorCode::kUiInitFailed);
  }

  panel_ = panel;
  SetAppState(AppState::kBooting);
  main_screen_ = MainScreen::kBooting;
  transition_ = {};
  transition_active_ = false;
  if (main_ui_widget_ != nullptr) {
    main_ui_widget_->SetMode(MainScreen::kBooting);
  }
  last_user_activity_ms_ = Sys().Timebase().NowMs();
  inactivity_fade_start_ms_ = 0;
  display_on_ = true;
  inactivity_fade_active_ = false;
  next_step_ms_ = 0;
  boot_widget_->SetNextWidget(main_ui_widget_);
  if (task_handle_ == nullptr) {
    task_handle_ = xTaskCreateStatic(TaskEntry, "display", kTaskStackDepthWords,
                                     this, 1, task_stack, &task_buffer);
    if (task_handle_ == nullptr) {
      Panic(ErrorCode::kUiInitFailed);
    }
  }

  if (cfg_.boot_logo_timeout_s > 0) {
    boot_widget_->SetTimeoutMs(static_cast<TimeMs>(cfg_.boot_logo_timeout_s) *
                               1000u);
    LoadWidget(boot_widget_);
  } else {
    LoadWidget(main_ui_widget_);
  }

  ESP_LOGI(kTag,
           "initialized fps_cap=%u boot_timeout=%us ui_timeout=%us "
           "transition_speed=%ux",
           static_cast<unsigned>(cfg_.fps_cap),
           static_cast<unsigned>(cfg_.boot_logo_timeout_s),
           static_cast<unsigned>(cfg_.ui_timeout_s),
           static_cast<unsigned>(cfg_.transition_speed_x));
}

void Ui::LoadWidget(IWidget *widget) {
  taskENTER_CRITICAL(&g_ui_lock);
  pending_widget_ = widget;
  taskEXIT_CRITICAL(&g_ui_lock);
  WakeTask();
}

void Ui::SetAppState(AppState state) {
  taskENTER_CRITICAL(&g_ui_lock);
  app_state_ = state;
  taskEXIT_CRITICAL(&g_ui_lock);
  WakeTask();
}

void Ui::SetErrorCode(ErrorCode code) {
  taskENTER_CRITICAL(&g_ui_lock);
  error_code_ = code;
  taskEXIT_CRITICAL(&g_ui_lock);
  WakeTask();
}

void Ui::SetErrorRecoverable(bool recoverable) {
  taskENTER_CRITICAL(&g_ui_lock);
  error_recoverable_ = recoverable;
  taskEXIT_CRITICAL(&g_ui_lock);
  WakeTask();
}

Ui::AppState Ui::CurrentAppState() const {
  taskENTER_CRITICAL(&g_ui_lock);
  const AppState state = app_state_;
  taskEXIT_CRITICAL(&g_ui_lock);
  return state;
}

ErrorCode Ui::CurrentErrorCode() const {
  taskENTER_CRITICAL(&g_ui_lock);
  const ErrorCode code = error_code_;
  taskEXIT_CRITICAL(&g_ui_lock);
  return code;
}

bool Ui::CurrentErrorRecoverable() const {
  taskENTER_CRITICAL(&g_ui_lock);
  const bool recoverable = error_recoverable_;
  taskEXIT_CRITICAL(&g_ui_lock);
  return recoverable;
}

uint8_t Ui::CurrentInactivityTimeoutSeconds() const {
  taskENTER_CRITICAL(&g_ui_lock);
  const uint8_t timeout_s = cfg_.ui_timeout_s;
  taskEXIT_CRITICAL(&g_ui_lock);
  return timeout_s;
}

Ui::MainScreen Ui::DeriveMainScreen(AppState state) const {
  switch (state) {
    case AppState::kBooting:
      return MainScreen::kBooting;
    case AppState::kServing:
      return MainScreen::kServing;
    case AppState::kDfu:
      return Sys().Wifi().HasAssociatedStations()
                 ? MainScreen::kDfuIdleConnected
                 : MainScreen::kDfuDisconnected;
    case AppState::kMavlinkWifi:
      return Sys().Wifi().HasAssociatedStations()
                 ? MainScreen::kMavlinkWifiConnected
                 : MainScreen::kMavlinkWifiDisconnected;
    case AppState::kProgram:
      if (Sys().Programmer().IsVerifying()) {
        return MainScreen::kVerifying;
      }
      if (Sys().Programmer().Done() && main_screen_ == MainScreen::kVerifying) {
        return MainScreen::kVerifying;
      }
      return MainScreen::kProgramming;
    case AppState::kHardError:
    default:
      return MainScreen::kServing;
  }
}

uint8_t Ui::ScreenGroup(MainScreen screen) const {
  switch (screen) {
    case MainScreen::kServing:
      return 1;
    case MainScreen::kMavlinkWifiDisconnected:
    case MainScreen::kMavlinkWifiConnected:
      return 2;
    case MainScreen::kDfuDisconnected:
    case MainScreen::kDfuIdleConnected:
      return 3;
    case MainScreen::kProgramming:
    case MainScreen::kVerifying:
      return 4;
    case MainScreen::kBooting:
    default:
      return 0;
  }
}

Ui::SlideDirection Ui::TransitionDirectionForScreens(MainScreen from,
                                                     MainScreen to) const {
  return (ScreenGroup(to) >= ScreenGroup(from)) ? SlideDirection::kLeft
                                                : SlideDirection::kRight;
}

bool Ui::ShouldSkipMainScreenTransition(MainScreen from, MainScreen to) const {
  return from == MainScreen::kDfuIdleConnected &&
         (to == MainScreen::kProgramming || to == MainScreen::kVerifying);
}

bool Ui::ShouldUseMosaicMainScreenTransition(MainScreen from,
                                             MainScreen to) const {
  if (from == MainScreen::kProgramming && to == MainScreen::kVerifying) {
    return true;
  }

  return from == MainScreen::kVerifying &&
         (to == MainScreen::kDfuIdleConnected ||
          to == MainScreen::kDfuDisconnected);
}

TimeMs Ui::MosaicDurationForScreens(MainScreen from, MainScreen to) const {
  static_cast<void>(from);
  static_cast<void>(to);
  return kDefaultMosaicTransitionDurationMs;
}

void Ui::RenderMainScreenSnapshot(MainScreen screen, TimeMs now,
                                  DisplayCanvas &dst) {
  if (main_ui_widget_ == nullptr) {
    return;
  }

  dst.Clear();
  DisplayRenderer renderer(&dst);
  WidgetContext ctx{
      .ui = this,
      .renderer = &renderer,
  };

  main_ui_widget_->SetMode(screen);
  main_ui_widget_->OnStep(ctx, now);
  dst.ClearDirtyRanges();
}

void Ui::StartMainScreenTransition(MainScreen next_screen, TimeMs now,
                                   SlideDirection direction, TimeMs duration_ms,
                                   float accel_ratio, float decel_ratio) {
  transition_from_canvas_ = canvas_;
  transition_from_canvas_.ClearDirtyRanges();
  RenderMainScreenSnapshot(next_screen, now, transition_to_canvas_);

  transition_.effect = TransitionEffect::kSlide;
  transition_.target_screen = next_screen;
  transition_.direction = direction;
  transition_.start_ms = now;
  transition_.duration_ms = std::max<TimeMs>(1, duration_ms);
  transition_.accel_ratio = accel_ratio;
  transition_.decel_ratio = decel_ratio;
  main_screen_ = next_screen;
  next_step_ms_ = 0;
  transition_active_ = true;
}

void Ui::StartMosaicTransitionToScreen(MainScreen next_screen, TimeMs now,
                                       TimeMs duration_ms,
                                       uint8_t max_block_size) {
  transition_from_canvas_ = canvas_;
  transition_from_canvas_.ClearDirtyRanges();
  RenderMainScreenSnapshot(next_screen, now, transition_to_canvas_);

  transition_.effect = TransitionEffect::kMosaic;
  transition_.target_screen = next_screen;
  transition_.direction = SlideDirection::kLeft;
  transition_.start_ms = now;
  transition_.duration_ms = std::max<TimeMs>(1, duration_ms);
  transition_.accel_ratio = 0.0f;
  transition_.decel_ratio = 0.0f;
  transition_.mosaic_block_size = std::max<uint8_t>(max_block_size, 1u);
  main_screen_ = next_screen;
  next_step_ms_ = 0;
  transition_active_ = true;
}

bool Ui::RenderActiveTransition(TimeMs now) {
  const float linear_t =
      Clamp01(static_cast<float>(now - transition_.start_ms) /
              static_cast<float>(transition_.duration_ms));
  switch (transition_.effect) {
    case TransitionEffect::kMosaic:
      renderer_.DrawMosaicTransition(
          transition_from_canvas_.Data(), transition_to_canvas_.Data(), kWidth,
          kHeight, linear_t, transition_.mosaic_block_size);
      break;

    case TransitionEffect::kSlide:
    default: {
      const float eased_t = SwipeProgress(linear_t, transition_.accel_ratio,
                                          transition_.decel_ratio);
      const int16_t width = static_cast<int16_t>(kWidth);
      const int16_t shift_px = static_cast<int16_t>(
          std::lround(eased_t * static_cast<float>(width)));

      int16_t from_offset_x = 0;
      int16_t to_offset_x = 0;
      if (transition_.direction == SlideDirection::kLeft) {
        from_offset_x = static_cast<int16_t>(-shift_px);
        to_offset_x = static_cast<int16_t>(width - shift_px);
      } else {
        from_offset_x = shift_px;
        to_offset_x = static_cast<int16_t>(shift_px - width);
      }

      canvas_.Clear();
      const auto draw_shifted = [&](const DisplayCanvas &src,
                                    int16_t offset_x) {
        for (size_t y = 0; y < kHeight; ++y) {
          for (size_t x = 0; x < kWidth; ++x) {
            if (!ReadPackedPixel(src.Data(), kWidth, kHeight, x, y)) {
              continue;
            }

            const int16_t dst_x =
                static_cast<int16_t>(static_cast<int16_t>(x) + offset_x);
            if (dst_x < 0 || dst_x >= width) {
              continue;
            }

            canvas_.SetPixel(static_cast<size_t>(dst_x), y, true);
          }
        }
      };

      draw_shifted(transition_from_canvas_, from_offset_x);
      draw_shifted(transition_to_canvas_, to_offset_x);
      break;
    }
  }

  return linear_t >= 1.0f;
}

void Ui::SyncPresentation(TimeMs now) {
  const AppState app_state = CurrentAppState();
  const MainScreen desired_main_screen = DeriveMainScreen(app_state);
  IWidget *current = nullptr;
  IWidget *pending = nullptr;

  taskENTER_CRITICAL(&g_ui_lock);
  current = current_widget_;
  pending = pending_widget_;
  taskEXIT_CRITICAL(&g_ui_lock);

  if (main_ui_widget_ != nullptr && app_state != AppState::kHardError) {
    const bool main_widget_visible = current == main_ui_widget_ &&
                                     pending != boot_widget_ &&
                                     pending != error_widget_;
    const bool same_group =
        ScreenGroup(main_screen_) == ScreenGroup(desired_main_screen);
    const bool skip_transition =
        ShouldSkipMainScreenTransition(main_screen_, desired_main_screen);
    const bool mosaic_transition =
        ShouldUseMosaicMainScreenTransition(main_screen_, desired_main_screen);
    if (main_widget_visible && desired_main_screen != main_screen_ &&
        mosaic_transition) {
      StartMosaicTransitionToScreen(
          desired_main_screen, now,
          MosaicDurationForScreens(main_screen_, desired_main_screen),
          kDefaultMosaicBlockSizePx);
    } else if (main_widget_visible && desired_main_screen != main_screen_ &&
               !same_group && !skip_transition) {
      const TransitionPreset preset =
          TransitionPresetForSpeed(cfg_.transition_speed_x);
      StartMainScreenTransition(
          desired_main_screen, now,
          TransitionDirectionForScreens(main_screen_, desired_main_screen),
          preset.duration_ms, preset.accel_ratio, preset.decel_ratio);
    } else {
      if (skip_transition) {
        StopTransition();
      }
      main_ui_widget_->SetMode(desired_main_screen);
      main_screen_ = desired_main_screen;
    }
  }

  if (error_widget_ != nullptr && app_state == AppState::kHardError) {
    error_widget_->SetErrorCode(CurrentErrorCode());
    error_widget_->SetRecoverable(CurrentErrorRecoverable());
  }

  IWidget *desired = nullptr;
  if (app_state == AppState::kHardError) {
    desired = error_widget_;
  } else {
    const bool boot_active = current == boot_widget_ || pending == boot_widget_;
    if (!boot_active) {
      desired = main_ui_widget_;
    }
  }

  if (desired != nullptr && desired != current && desired != pending) {
    LoadWidget(desired);
  }
}

void Ui::NotifyUserActivity() {
  last_user_activity_ms_ = Sys().Timebase().NowMs();

  if (panel_ != nullptr) {
    if (inactivity_fade_active_) {
      DisableFadeOut();
      display_on_ = true;
    } else if (!display_on_) {
      DisplayOn();
    }
  }
  WakeTask();
}

void Ui::SetInactivityTimeoutSeconds(uint8_t timeout_s) {
  taskENTER_CRITICAL(&g_ui_lock);
  cfg_.ui_timeout_s = timeout_s;
  taskEXIT_CRITICAL(&g_ui_lock);

  last_user_activity_ms_ = Sys().Timebase().NowMs();
  inactivity_fade_start_ms_ = 0;
  inactivity_fade_active_ = false;

  if (timeout_s == 0) {
    DisableFadeOut();
    if (!display_on_) {
      DisplayOn();
    }
  }
  WakeTask();
}

void Ui::DisableInactivityTimeout() { SetInactivityTimeoutSeconds(0); }

void Ui::SetFadeOut(uint8_t interval) {
  if (panel_ == nullptr) {
    return;
  }

  FlushIfDirty();
  panel_->SetFadeOut(interval);
}

void Ui::DisableFadeOut() {
  if (panel_ == nullptr) {
    return;
  }

  panel_->Flush(canvas_.Data(), canvas_.Size());
  panel_->DisableFadeOut();
  canvas_.ClearDirtyRanges();
  inactivity_fade_active_ = false;
}

void Ui::DisplayOn() {
  if (panel_ == nullptr) {
    return;
  }

  panel_->DisableFadeOut();
  panel_->DisplayOn();
  panel_->Flush(canvas_.Data(), canvas_.Size());
  canvas_.ClearDirtyRanges();
  display_on_ = true;
  inactivity_fade_active_ = false;
}

void Ui::DisplayOff() {
  if (panel_ == nullptr) {
    return;
  }

  FlushIfDirty();
  panel_->DisplayOff();
  display_on_ = false;
  inactivity_fade_active_ = false;
}

bool Ui::IsScreenOn() const { return display_on_; }

bool Ui::IsTransitionActive() const {
  return transition_active_;
}

void Ui::StopTransition() {
  transition_active_ = false;
  next_step_ms_ = 0;
}

void Ui::ServiceTransition(TimeMs now) {
  if (!transition_active_) {
    return;
  }

  if (RenderActiveTransition(now)) {
    transition_active_ = false;
    next_step_ms_ = 0;
    return;
  }

  next_step_ms_ = TimeAfter(now, GetFrameIntervalMs());
}

void Ui::StartMosaicTransition(TimeMs now, TimeMs duration_ms,
                               uint8_t max_block_size) {
  MainScreen next_screen = DeriveMainScreen(CurrentAppState());
  if (next_screen == MainScreen::kBooting) {
    next_screen = MainScreen::kServing;
  }
  StartMosaicTransitionToScreen(next_screen, now, duration_ms, max_block_size);
}

TimeMs Ui::GetFrameIntervalMs() const {
  return static_cast<TimeMs>(
      (1000u + static_cast<uint32_t>(cfg_.fps_cap) - 1u) /
      static_cast<uint32_t>(cfg_.fps_cap));
}

void Ui::TaskEntry(void *param) { static_cast<Ui *>(param)->Task(); }

void Ui::Task() {
  WidgetContext ctx{
      .ui = this,
      .renderer = &renderer_,
  };

  while (true) {
    const TimeMs now = Sys().Timebase().NowMs();
    Step(now, ctx);

    TimeMs wait_ms = GetFrameIntervalMs();
    if (next_step_ms_ != 0) {
      const TimeMs after_step = Sys().Timebase().NowMs();
      if (!TimeReached(after_step, next_step_ms_)) {
        wait_ms = next_step_ms_ - after_step;
      }
    }

    ulTaskNotifyTake(pdTRUE, WaitTicksFromMs(wait_ms));
  }
}

void Ui::WakeTask() const {
  if (task_handle_ != nullptr) {
    xTaskNotifyGive((TaskHandle_t)task_handle_);
  }
}

void Ui::Step(TimeMs now, WidgetContext &ctx) {
  if (panel_ == nullptr || cfg_.fps_cap == 0) {
    return;
  }

  bool widget_changed = ApplyPendingWidget(ctx);
  SyncPresentation(now);
  widget_changed = ApplyPendingWidget(ctx) || widget_changed;
  if (widget_changed) {
    StopTransition();
    FlushIfDirty();
  }

  IWidget *widget = nullptr;
  taskENTER_CRITICAL(&g_ui_lock);
  widget = current_widget_;
  taskEXIT_CRITICAL(&g_ui_lock);

  if (widget == nullptr) {
    FlushIfDirty();
    return;
  }

  UpdatePowerState(now);
  if (!display_on_ && !inactivity_fade_active_) {
    return;
  }

  if (IsTransitionActive()) {
    if (!TimeReached(now, next_step_ms_)) {
      FlushIfDirty();
      return;
    }

    ServiceTransition(now);
    FlushIfDirty();
    if (IsTransitionActive()) {
      return;
    }
  }

  if (!TimeReached(now, next_step_ms_)) {
    FlushIfDirty();
    return;
  }

  widget->OnStep(ctx, now);
  FlushIfDirty();
  UpdatePowerState(Sys().Timebase().NowMs());
  next_step_ms_ = TimeAfter(now, GetFrameIntervalMs());
}

void Ui::UpdatePowerState(TimeMs now) {
  const uint8_t timeout_s = CurrentInactivityTimeoutSeconds();
  if (panel_ == nullptr || timeout_s == 0) {
    return;
  }

  const TimeMs timeout_ms =
      static_cast<TimeMs>(timeout_s) * static_cast<TimeMs>(1000u);

  if (display_on_ && !inactivity_fade_active_) {
    if ((now - last_user_activity_ms_) >= timeout_ms) {
      SetFadeOut(kUiFadeOutInterval);
      inactivity_fade_active_ = true;
      inactivity_fade_start_ms_ = now;
    }
    return;
  }

  if (inactivity_fade_active_ &&
      (now - inactivity_fade_start_ms_) >= kUiFadeOutDurationMs) {
    DisplayOff();
  }
}

bool Ui::ApplyPendingWidget(WidgetContext &ctx) {
  bool changed = false;
  while (true) {
    IWidget *next = nullptr;

    taskENTER_CRITICAL(&g_ui_lock);
    if (pending_widget_ == current_widget_) {
      taskEXIT_CRITICAL(&g_ui_lock);
      return changed;
    }
    next = pending_widget_;
    current_widget_ = pending_widget_;
    taskEXIT_CRITICAL(&g_ui_lock);
    changed = true;

    if (next != nullptr) {
      next->OnEnter(ctx);
    }
  }
}

void Ui::FlushIfDirty() {
  if (panel_ == nullptr || !canvas_.HasDirtyRanges()) {
    return;
  }

  for (size_t page = 0; page < DisplayCanvas::kPageCount; ++page) {
    const DisplayCanvas::DirtyRange &range = canvas_.GetDirtyRange(page);
    if (!range.dirty) {
      continue;
    }

    panel_->FlushPageRange(static_cast<uint8_t>(page), canvas_.PageData(page),
                           range.x_begin, range.x_end - range.x_begin);
  }

  canvas_.ClearDirtyRanges();
}
