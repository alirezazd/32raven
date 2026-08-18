// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "display_renderer.hpp"
#include "error_code.hpp"
#include "message.hpp"
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

  void Clear() override;
  void Fill(Ink ink) override;
  size_t Width() const override { return kWidth; }
  size_t Height() const override { return kHeight; }
  bool SetPixel(size_t x, size_t y, Ink ink) override;
  bool DrawPackedBitmap(const PackedBitmap &bitmap, size_t offset_x,
                        size_t offset_y) override;
  PackedBitmap AsBitmap() const { return {buffer_, kWidth, kHeight}; }
  bool HasDirtyRanges() const;
  const DirtyRange &GetDirtyRange(size_t page) const {
    return dirty_ranges_[page];
  }
  std::span<const uint8_t, kWidth> PageSpan(size_t page) const {
    return std::span<const uint8_t, kWidth>(buffer_.data() + (page * kWidth),
                                            kWidth);
  }
  void ClearDirtyRanges();
  const uint8_t *Data() const { return buffer_.data(); }
  [[nodiscard]] std::span<const uint8_t, kBufferSize> Buffer() const {
    return buffer_;
  }

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
    kMavlinkUsb,
    kProgram,
    kEscConfig,
    kWifiLog,
    kUsbLog,
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
    // Three stages of one screen; the STM32's USB report picks between them.
    kEscConfigArmed,          // refused: the vehicle is armed
    kEscConfigDisconnected,   // no host has enumerated the port
    kEscConfigIdleConnected,  // enumerated, no configurator attached yet
    kEscConfigConnected,      // configurator opened the port
    kWifiLogDisconnected,     // AP up, no station associated
    kWifiLogConnected,        // a station joined; pulls animate the lanes
    kUsbLogIdle,              // MSC granted, no host has enumerated the disk
    kUsbLogActive,            // host mounted the card
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
  static Ui &GetInstance();

  // Lives here because the display is its only consumer. Decoding the wire
  // message into this shape is the dispatcher's job.
  struct PeerUsbState {
    bool attached = false;
    bool configured = false;
    bool port_open = false;
    message::UsbMode mode = message::UsbMode::kNone;
    // Wrapping: compare against what you saw last, never read as a total.
    // In MSC mode they carry SD block counts instead of protocol frames.
    uint8_t rx_frames = 0;
    uint8_t tx_frames = 0;
  };

  void UpdatePeerUsb(const PeerUsbState &state, uint32_t now_ms);

  // nullopt once the STM32 has gone quiet -- silence is not evidence that USB
  // came up.
  std::optional<PeerUsbState> PeerUsb(uint32_t now_ms) const;

  // WiFi-log pull traffic: requests out, replies back.
  // Wrapping: compare against what you saw last, never read as a total.
  struct LogTraffic {
    uint16_t rx_frames = 0;
    uint16_t tx_frames = 0;
  };

  void UpdateLogTraffic(uint16_t rx_frames, uint16_t tx_frames);
  LogTraffic GetLogTraffic() const;

  void Init(const Config &cfg, Ssd1306Panel *panel);
  void LoadWidget(IWidget *widget);
  void SetAppState(AppState state);
  void SetErrorCode(uint32_t code);
  void SetErrorRecoverable(bool recoverable);

  DisplayRenderer &Renderer() { return renderer_; }
  const DisplayRenderer &Renderer() const { return renderer_; }
  // Returns whether the screen was already awake; asking afterwards is
  // useless, since the wake itself makes IsScreenOn() true.
  bool NotifyUserActivity();
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

  // Written by the state-machine task, read by the UI task. volatile is for
  // ordering, not tearing: PeerUsb must load the timestamp first or it pairs a
  // fresh one with the previous report. Zero means nothing heard yet.
  volatile uint32_t peer_usb_report_ = 0;
  volatile uint32_t peer_usb_update_ms_ = 0;
  // rx in the low half, tx in the high; one word so the pair cannot tear.
  volatile uint32_t log_traffic_word_ = 0;

  Ui();
  static void TaskEntry(void *param);
  void Task();
  void Step(TimeMs now, WidgetContext &ctx);
  void WakeTask() const;
  void ServiceTransition(TimeMs now);
  AppState CurrentAppState() const;
  uint32_t CurrentErrorCode() const;
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
  // Order is the left/right slide direction (see
  // TransitionDirectionForScreens).
  enum class ScreenGroup : uint8_t {
    kBoot,
    kServing,
    kMavlink,
    kLog,
    kDfu,
    kProgram,
    kEscConfig,
  };

  ScreenGroup ScreenGroupFor(MainScreen screen) const;

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
  uint32_t error_code_ = static_cast<uint32_t>(ErrorCode::Common::kUnknown);
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
