// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "panic.hpp"

#include <algorithm>

#include "driver/gpio.h"
#include "error_code.hpp"
#include "esp32_config.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
#include "system.hpp"

static constexpr const char *kTag = "panic";

namespace {

// The Service recovery loop runs here rather than on the stack of
// whatever panicked, and measures ~3.7 KB deep, so there is little to give
// back. The priority has to outrank anything that could be wedged when it is
// woken.
static constexpr uint32_t kPanicTaskStackBytes = 4096;
static constexpr UBaseType_t kPanicTaskPrio = 24;
static_assert(kPanicTaskPrio < configMAX_PRIORITIES);
static StaticTask_t s_panic_task_buffer;
static StackType_t s_panic_task_stack[kPanicTaskStackBytes];
static TaskHandle_t s_panic_task_handle = nullptr;

[[noreturn]] void RunPanicLoop(uint32_t code);

void PanicTask(void *) {
  while (true) {
    uint32_t notified_code = static_cast<uint32_t>(ErrorCode::Common::kUnknown);
    (void)xTaskNotifyWait(0, UINT32_MAX, &notified_code, portMAX_DELAY);
    RunPanicLoop(notified_code);
  }
}

void EnsurePanicTaskStarted() {
  if (s_panic_task_handle != nullptr) {
    return;
  }

  s_panic_task_handle = xTaskCreateStaticPinnedToCore(
      PanicTask, "panic", kPanicTaskStackBytes, nullptr, kPanicTaskPrio,
      s_panic_task_stack, &s_panic_task_buffer, 0);
}

// constexpr helper — `case Raw(ErrorCode::Esp32::kFoo):` is shorter than
// `case static_cast<uint32_t>(ErrorCode::Esp32::kFoo):` and lets us mix
// domains in one switch.
template <typename E>
constexpr uint32_t Raw(E code) {
  return static_cast<uint32_t>(code);
}

bool SupportsServiceRecovery(uint32_t code) {
  switch (code) {
    case Raw(ErrorCode::Common::kUnknown):
    case Raw(ErrorCode::Esp32::kButtonGpioConfigFailed):
    case Raw(ErrorCode::Esp32::kWifiNvsInitFailed):
    case Raw(ErrorCode::Esp32::kWifiNetifInitFailed):
    case Raw(ErrorCode::Esp32::kWifiEventLoopFailed):
    case Raw(ErrorCode::Esp32::kWifiInitFailed):
    case Raw(ErrorCode::Esp32::kWifiSetStorageFailed):
    case Raw(ErrorCode::Esp32::kI2cParamConfigFailed):
    case Raw(ErrorCode::Esp32::kI2cInitFailed):
    case Raw(ErrorCode::Esp32::kI2cInvalidArg):
    case Raw(ErrorCode::Esp32::kI2cOperationFailed):
    case Raw(ErrorCode::Esp32::kUartParamConfigFailed):
    case Raw(ErrorCode::Esp32::kUartSetPinFailed):
    case Raw(ErrorCode::Esp32::kUartDriverInstallFailed):
    case Raw(ErrorCode::Esp32::kUartInvalidNumber):
    case Raw(ErrorCode::Esp32::kUartNotInitialized):
    case Raw(ErrorCode::Esp32::kUartInvalidArg):
    case Raw(ErrorCode::Esp32::kUartOperationFailed):
    case Raw(ErrorCode::Esp32::kProgrammerUartNull):
    case Raw(ErrorCode::Esp32::kProgrammerHandshakeFailed):
    case Raw(ErrorCode::Esp32::kProgrammerBufferOverflow):
    case Raw(ErrorCode::Esp32::kProgrammerEraseFailed):
    case Raw(ErrorCode::Esp32::kProgrammerWriteFailed):
    case Raw(ErrorCode::Esp32::kProgrammerReadFailed):
    case Raw(ErrorCode::Esp32::kProgrammerVerifyFailed):
    case Raw(ErrorCode::Esp32::kProgrammerOtaPartitionNotFound):
    case Raw(ErrorCode::Esp32::kProgrammerOtaBeginFailed):
    case Raw(ErrorCode::Esp32::kProgrammerOtaWriteFailed):
    case Raw(ErrorCode::Esp32::kProgrammerOtaEndFailed):
    case Raw(ErrorCode::Esp32::kProgrammerOtaSetBootFailed):
    case Raw(ErrorCode::Esp32::kProgrammerTimedOut):
    case Raw(ErrorCode::Esp32::kDisplayPanelInitFailed):
    case Raw(ErrorCode::Esp32::kUiInitFailed):
    case Raw(ErrorCode::Esp32::kTcpServerStartFailed):
    case Raw(ErrorCode::Esp32::kTcpServerAcceptFailed):
    case Raw(ErrorCode::Esp32::kTcpServerError):
      return false;
    default:
      return true;
  }
}

void ShowPanicUi(uint32_t code, bool recoverable) {
  Sys().Ui().SetErrorCode(code);
  Sys().Ui().SetErrorRecoverable(recoverable);
  Sys().Ui().SetAppState(Ui::AppState::kHardError);
  Sys().Ui().DisableInactivityTimeout();
  Sys().Ui().NotifyUserActivity();
}

uint32_t EnterRecoveryServiceMode() {
  System &sys = Sys();
  sys.Button().FlushEvents();
  sys.Ui().SetAppState(Ui::AppState::kService);
  sys.Ui().NotifyUserActivity();
  // Reached from the panic task, so a Panic() here would nest another
  // RunPanicLoop on the same static stack. The checks below report instead.
  sys.StartNetwork();
  sys.Tcp().CloseDataRx();
  // As the app's Service mode: the host links get the USB port to themselves.
  sys.Mavlink().SetTelemetryLink(false);
  sys.UsbHost().Start();

  if (!sys.Wifi().IsOn()) {
    return Raw(ErrorCode::Esp32::kWifiInitFailed);
  }
  if (!sys.Tcp().Running()) {
    return Raw(ErrorCode::Esp32::kTcpServerStartFailed);
  }

  return Raw(ErrorCode::Common::kOk);
}

[[nodiscard]] uint32_t EnterRecoveryProgramMode(const HostLink &link) {
  System &sys = Sys();
  sys.Button().FlushEvents();
  sys.Ui().SetAppState(Ui::AppState::kProgram);
  sys.Ui().NotifyUserActivity();
  const HostLink::BeginArgs &begin = link.Begin();
  sys.Programmer().Start(begin.size, begin.crc);
  return sys.Programmer().Written();
}

class RecoverySession {
 public:
  explicit RecoverySession(System &sys);

  uint32_t RunUntilFailure();

 private:
  enum class Mode : uint8_t {
    kService,
    kProgram,
  };

  enum class NetworkAction : uint8_t {
    kKeepNetwork,
    kStopNetwork,
  };

  bool EnterServiceMode(TimeMs now, NetworkAction network_on_error);
  void EnterProgramMode(TimeMs now);
  void StepServiceMode(TimeMs now);
  void StepProgramMode(TimeMs now);
  void Dispatch(TimeMs now, const HostLink::Event &ev);
  void Exit(uint32_t code, NetworkAction network);

  System &sys_;
  TcpServer &tcp_;
  UsbHostLink &usb_;
  Programmer &prog_;
  // The link whose BEGIN armed the current transfer; Program answers only it.
  HostLink *link_ = nullptr;
  Mode mode_ = Mode::kService;
  uint32_t result_ = Raw(ErrorCode::Common::kOk);
  bool exit_ = false;
  TimeMs last_activity_ = 0;
  uint32_t last_written_ = 0;
};

RecoverySession::RecoverySession(System &sys)
    : sys_(sys),
      tcp_(sys.Tcp()),
      usb_(sys.UsbHost()),
      prog_(sys.Programmer()) {}

bool RecoverySession::EnterServiceMode(TimeMs now,
                                       NetworkAction network_on_error) {
  const uint32_t recovery_error = EnterRecoveryServiceMode();
  if (recovery_error != Raw(ErrorCode::Common::kOk)) {
    Exit(recovery_error, network_on_error);
    return false;
  }

  mode_ = Mode::kService;
  last_written_ = prog_.Written();
  last_activity_ = now;
  return true;
}

void RecoverySession::StepServiceMode(TimeMs now) {
  while (auto ev = tcp_.PopEvent()) {
    Dispatch(now, *ev);
    if (mode_ != Mode::kService || exit_) return;
  }
  while (auto ev = usb_.PopEvent()) {
    Dispatch(now, *ev);
    if (mode_ != Mode::kService || exit_) return;
  }

  tcp_.ClearLinkDrop();
  usb_.ClearLinkDrop();
}

void RecoverySession::Dispatch(TimeMs now, const HostLink::Event &ev) {
  HostLink &link = *ev.origin;
  switch (ev.id) {
    case HostLink::EventId::kBegin:
      link.SendCtrlLine("OK\n");
      link.BeginTransfer(ev.begin);
      prog_.SetTarget(ev.begin.target);
      link_ = &link;
      EnterProgramMode(now);
      return;
    case HostLink::EventId::kAbort:
      prog_.Abort();
      link.EndTransfer();
      (void)EnterServiceMode(now, NetworkAction::kStopNetwork);
      return;
    case HostLink::EventId::kReset:
      link.CloseDataRx();
      (void)prog_.Boot();
      esp_restart();
      return;
    case HostLink::EventId::kLogList:
    case HostLink::EventId::kLogGet:
      // Queued unanswered by the parser, and recovery serves no logs.
      link.SendCtrlLine("ERR wrong_mode\n");
      return;
    case HostLink::EventId::kNone:
    default:
      return;
  }
}

void RecoverySession::EnterProgramMode(TimeMs now) {
  // Program serves the armed link only. Stopped, the USB one drops what
  // arrives meanwhile instead of banking commands for the return to Service.
  if (link_ != &usb_) usb_.Stop();
  mode_ = Mode::kProgram;
  last_written_ = EnterRecoveryProgramMode(*link_);
  last_activity_ = now;
}

void RecoverySession::StepProgramMode(TimeMs now) {
  HostLink &link = *link_;
  prog_.Poll();

  if (prog_.Error()) {
    const uint32_t programmer_error = prog_.LastErrorCode();
    link.EndTransfer();
    prog_.Abort();
    Exit(programmer_error, NetworkAction::kStopNetwork);
    return;
  }

  if (prog_.Done()) {
    HostLink::Status st{};
    st.rx = prog_.Written();
    st.total = prog_.Total();
    st.state = HostLink::Status::kDone;
    link.EndTransfer();
    link.SetStatus(st);
    (void)prog_.Boot();
    (void)EnterServiceMode(now, NetworkAction::kStopNetwork);
    return;
  }

  while (auto ev = link.PopEvent()) {
    Dispatch(now, *ev);
    if (mode_ != Mode::kProgram || exit_) return;
  }

  if (link.TakeLinkDrop()) {
    prog_.Abort();
    link.EndTransfer();
    (void)EnterServiceMode(now, NetworkAction::kStopNetwork);
    return;
  }

  if (prog_.IsVerifying()) {
    HostLink::Status verifying = link.GetStatus();
    verifying.rx = prog_.VerifyOffset();
    verifying.state = HostLink::Status::kVerifying;
    link.SetStatus(verifying);
    return;
  }

  HostLink::Status st = link.GetStatus();
  st.rx = prog_.Written();
  link.SetStatus(st);

  uint8_t buf[512];
  size_t budget = prog_.TargetWriteChunkLimit();
  while (budget > 0) {
    const size_t room = std::min({prog_.Free(), budget, sizeof(buf)});
    if (room == 0) break;
    const size_t n = link.ReadDataRx({buf, room});
    if (n == 0) break;
    prog_.PushBytes({buf, n});
    last_activity_ = now;
    budget -= n;
  }

  const uint32_t current_written = prog_.Written();
  if (current_written != last_written_) {
    last_activity_ = now;
    last_written_ = current_written;
  }

  if (!prog_.Done() && (now - last_activity_) > Programmer::kStallTimeoutMs) {
    prog_.Abort();
    Exit(Raw(ErrorCode::Esp32::kProgrammerTimedOut),
         NetworkAction::kStopNetwork);
  }
}

void RecoverySession::Exit(uint32_t code, NetworkAction network) {
  if (network == NetworkAction::kStopNetwork) {
    sys_.StopNetwork();
  }
  usb_.Stop();
  result_ = code;
  exit_ = true;
}

uint32_t RecoverySession::RunUntilFailure() {
  if (!EnterServiceMode(sys_.Timebase().NowMs(), NetworkAction::kKeepNetwork)) {
    return result_;
  }

  while (true) {
    const TimeMs now = sys_.Timebase().NowMs();
    sys_.Button().Poll();
    tcp_.Poll();
    usb_.Poll();

    switch (mode_) {
      case Mode::kService:
        StepServiceMode(now);
        break;
      case Mode::kProgram:
        StepProgramMode(now);
        break;
    }

    if (exit_) {
      return result_;
    }

    vTaskDelay(1);
  }
}

uint32_t RunRecoverableLoop() {
  System &sys = Sys();
  RecoverySession recovery(sys);
  return recovery.RunUntilFailure();
}

// The blink wants a tick this short; the console line does not. Held to one
// line every two seconds so a monitor attached long after the board stopped
// still learns why, without the reason burying everything else the console
// has to say.
constexpr uint32_t kPanicBlinkMs = 40;
constexpr uint32_t kNestedPanicBlinkMs = 15;
constexpr uint32_t kPanicLogPeriodMs = 2000;

[[noreturn]] void RunPanicLoop(uint32_t code) {
  Sys().Halt();
  bool recoverable = SupportsServiceRecovery(code);
  const char *msg = GetMessage(code);
  Sys().TonePlayer().PlayBuiltinNow(::TonePlayer::BuiltinTone::kError);
  ShowPanicUi(code, recoverable);
  gpio_reset_pin(kPinMap.led);
  gpio_set_direction(kPinMap.led, GPIO_MODE_OUTPUT);
  ESP_LOGE(kTag, "PANIC [0x%08lX]: %s", (unsigned long)code, msg);
  Sys().Mavlink().ReportPanic(Mavlink::PanicSource::kEsp32, code);
  if (recoverable) {
    Sys().Button().FlushEvents();
  }
  constexpr uint32_t kLogEveryTicks = kPanicLogPeriodMs / kPanicBlinkMs;
  // The line above just went out, so the next one is a window away.
  uint32_t ticks_since_log = 0;
  bool led_on = false;
  while (true) {
    if (recoverable) {
      Sys().Button().Poll();
      if (Sys().Button().ConsumeLongPress()) {
        code = RunRecoverableLoop();
        recoverable = SupportsServiceRecovery(code);
        msg = GetMessage(code);
        ShowPanicUi(code, recoverable);
        Sys().Mavlink().ReportPanic(Mavlink::PanicSource::kEsp32, code);
        if (recoverable) {
          Sys().Button().FlushEvents();
        }
        // A different code is worth saying now rather than at the next window.
        ticks_since_log = kLogEveryTicks;
      }
    }
    gpio_set_level(kPinMap.led, led_on ? 1 : 0);
    led_on = !led_on;
    if (ticks_since_log >= kLogEveryTicks) {
      ESP_LOGE(kTag, "PANIC [0x%08lX]: %s", (unsigned long)code, msg);
      ticks_since_log = 0;
    }
    ++ticks_since_log;
    vTaskDelay(pdMS_TO_TICKS(kPanicBlinkMs));
  }
}

// Reached when something inside RunPanicLoop panics -- the recovery paths are
// the realistic source, since they drive services the first panic halted.
// Re-entering the loop would nest it on the same static stack, so this reports
// and stops: no UI, no tone, no recovery, and a faster blink than the loop it
// is standing in for.
[[noreturn]] void ReportNestedPanic(uint32_t code) {
  const char *msg = GetMessage(code);
  constexpr uint32_t kLogEveryTicks = kPanicLogPeriodMs / kNestedPanicBlinkMs;
  // Nothing has said this one yet, so the first pass says it.
  uint32_t ticks_since_log = kLogEveryTicks;
  bool led_on = false;
  while (true) {
    gpio_set_level(kPinMap.led, led_on ? 1 : 0);
    led_on = !led_on;
    if (ticks_since_log >= kLogEveryTicks) {
      ESP_LOGE(kTag, "PANIC IN PANIC [0x%08lX]: %s", (unsigned long)code, msg);
      ticks_since_log = 0;
    }
    ++ticks_since_log;
    vTaskDelay(pdMS_TO_TICKS(kNestedPanicBlinkMs));
  }
}

}  // namespace

[[noreturn]] void PanicImpl(uint32_t code) {
  EnsurePanicTaskStarted();
  Sys().Halt();

  if (s_panic_task_handle == xTaskGetCurrentTaskHandle()) {
    ReportNestedPanic(code);
  }

  if (s_panic_task_handle != nullptr &&
      s_panic_task_handle != xTaskGetCurrentTaskHandle()) {
    (void)xTaskNotify(s_panic_task_handle, code, eSetValueWithOverwrite);
    vTaskSuspend(nullptr);
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  RunPanicLoop(code);
}
