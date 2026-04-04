#include "display_manager.hpp"

#include <algorithm>

#include "boot_widget.hpp"
#include "main_ui_widget.hpp"
#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {

constexpr const char *kTag = "display_manager";
constexpr uint32_t kTaskStackBytes = 3072;
constexpr TimeMs kUiFadeOutDurationMs = 2000;
constexpr uint8_t kUiFadeOutInterval = 0;
portMUX_TYPE g_display_manager_lock = portMUX_INITIALIZER_UNLOCKED;

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

TickType_t WaitTicksFromMs(TimeMs wait_ms) {
  return pdMS_TO_TICKS(wait_ms > 0 ? wait_ms : 1);
}

}  // namespace

DisplayManager::DisplayManager() : renderer_(&canvas_) {}

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
  if (display != nullptr) {
    display->LoadWidget(widget);
  }
}

void StateMachineWidget::OnEnter(WidgetContext &ctx) {
  ctx_ = ctx;
  ctx_.sm = &sm_;
  sm_.Start(InitialState());
}

void StateMachineWidget::OnStep(WidgetContext &ctx, TimeMs now) {
  (void)ctx;
  sm_.Step(now);
}

void StateMachineWidget::OnExit(WidgetContext &ctx) {
  (void)ctx;
  ctx_.sm = nullptr;
}

void DisplayManager::Init(const Config &cfg, Ssd1306Panel *panel) {
  static StaticTask_t task_buffer;
  static StackType_t task_stack[kTaskStackBytes];
  static BootWidget boot_widget;
  MainUiWidget &main_ui_widget = MainUiWidget::GetInstance();

  cfg_ = cfg;
  if (panel == nullptr || cfg_.fps_cap == 0) {
    Panic(ErrorCode::kDisplayManagerInitFailed);
  }

  panel_ = panel;
  main_ui_widget.SetStatus("Booting");
  last_user_activity_ms_ = Sys().Timebase().NowMs();
  inactivity_fade_start_ms_ = 0;
  display_on_ = true;
  inactivity_fade_active_ = false;
  boot_widget.SetNextWidget(&main_ui_widget);
  task_handle_ =
      xTaskCreateStatic(TaskEntry, "display", kTaskStackBytes, this, 1,
                        task_stack, &task_buffer);
  if (task_handle_ == nullptr) {
    Panic(ErrorCode::kDisplayManagerInitFailed);
  }

  if (cfg_.boot_logo_timeout_s > 0) {
    boot_widget.SetTimeoutMs(static_cast<TimeMs>(cfg_.boot_logo_timeout_s) *
                             1000u);
    LoadWidget(&boot_widget);
  } else {
    LoadWidget(&main_ui_widget);
  }

  ESP_LOGI(kTag, "initialized fps_cap=%u boot_timeout=%us ui_timeout=%us",
           static_cast<unsigned>(cfg_.fps_cap),
           static_cast<unsigned>(cfg_.boot_logo_timeout_s),
           static_cast<unsigned>(cfg_.ui_timeout_s));
}

void DisplayManager::LoadWidget(IWidget *widget) {
  taskENTER_CRITICAL(&g_display_manager_lock);
  pending_widget_ = widget;
  taskEXIT_CRITICAL(&g_display_manager_lock);

  if (task_handle_ != nullptr) {
    xTaskNotifyGive((TaskHandle_t)task_handle_);
  }
}

void DisplayManager::NotifyUserActivity() {
  last_user_activity_ms_ = Sys().Timebase().NowMs();

  if (panel_ != nullptr) {
    if (inactivity_fade_active_) {
      DisableFadeOut();
      display_on_ = true;
    } else if (!display_on_) {
      DisplayOn();
    }
  }

  if (task_handle_ != nullptr) {
    xTaskNotifyGive((TaskHandle_t)task_handle_);
  }
}

void DisplayManager::SetFadeOut(uint8_t interval) {
  if (panel_ == nullptr) {
    return;
  }

  FlushIfDirty();
  panel_->SetFadeOut(interval);
}

void DisplayManager::DisableFadeOut() {
  if (panel_ == nullptr) {
    return;
  }

  panel_->Flush(canvas_.Data(), canvas_.Size());
  panel_->DisableFadeOut();
  canvas_.ClearDirtyRanges();
  inactivity_fade_active_ = false;
}

void DisplayManager::DisplayOn() {
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

void DisplayManager::DisplayOff() {
  if (panel_ == nullptr) {
    return;
  }

  FlushIfDirty();
  panel_->DisplayOff();
  display_on_ = false;
  inactivity_fade_active_ = false;
}

TimeMs DisplayManager::GetFrameIntervalMs() const {
  return static_cast<TimeMs>(
      (1000u + static_cast<uint32_t>(cfg_.fps_cap) - 1u) /
      static_cast<uint32_t>(cfg_.fps_cap));
}

void DisplayManager::TaskEntry(void *param) {
  static_cast<DisplayManager *>(param)->Task();
}

void DisplayManager::Task() {
  WidgetContext ctx{
      .display = this,
      .renderer = &renderer_,
  };
  TimeMs next_step_ms = 0;

  while (true) {
    if (ApplyPendingWidget(ctx)) {
      next_step_ms = 0;
      FlushIfDirty();
    }

    IWidget *widget = nullptr;
    taskENTER_CRITICAL(&g_display_manager_lock);
    widget = current_widget_;
    taskEXIT_CRITICAL(&g_display_manager_lock);

    if (widget == nullptr) {
      FlushIfDirty();
      ulTaskNotifyTake(pdTRUE, WaitTicksFromMs(GetFrameIntervalMs()));
      continue;
    }

    const TimeMs now = Sys().Timebase().NowMs();
    UpdatePowerState(now);
    if (!display_on_ && !inactivity_fade_active_) {
      ulTaskNotifyTake(pdTRUE, WaitTicksFromMs(GetFrameIntervalMs()));
      continue;
    }
    if (TimeReached(now, next_step_ms)) {
      widget->OnStep(ctx, now);
      FlushIfDirty();
      UpdatePowerState(Sys().Timebase().NowMs());
      next_step_ms = TimeAfter(now, GetFrameIntervalMs());
      continue;
    }

    FlushIfDirty();
    ulTaskNotifyTake(pdTRUE, WaitTicksFromMs(next_step_ms - now));
  }
}

void DisplayManager::UpdatePowerState(TimeMs now) {
  if (panel_ == nullptr || cfg_.ui_timeout_s == 0) {
    return;
  }

  const TimeMs timeout_ms =
      static_cast<TimeMs>(cfg_.ui_timeout_s) * static_cast<TimeMs>(1000u);

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

bool DisplayManager::ApplyPendingWidget(WidgetContext &ctx) {
  bool changed = false;
  while (true) {
    IWidget *current = nullptr;
    IWidget *next = nullptr;

    taskENTER_CRITICAL(&g_display_manager_lock);
    if (pending_widget_ == current_widget_) {
      taskEXIT_CRITICAL(&g_display_manager_lock);
      return changed;
    }
    current = current_widget_;
    next = pending_widget_;
    current_widget_ = pending_widget_;
    taskEXIT_CRITICAL(&g_display_manager_lock);
    changed = true;

    if (current != nullptr) {
      current->OnExit(ctx);
    }
    if (next != nullptr) {
      next->OnEnter(ctx);
    }
  }
}

void DisplayManager::FlushIfDirty() {
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
