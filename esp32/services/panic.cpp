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

// The DFU recovery loop runs here rather than on the stack of
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

bool SupportsDfuRecovery(uint32_t code) {
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

uint32_t EnterRecoveryDfuMode() {
  System &sys = Sys();
  sys.Button().FlushEvents();
  sys.Ui().SetAppState(Ui::AppState::kDfu);
  sys.Ui().NotifyUserActivity();
  // Reached from the panic task, so a Panic() here would nest another
  // RunPanicLoop on the same static stack. The checks below report instead.
  sys.StartNetwork();
  sys.Tcp().DisableBridge();

  if (!sys.Wifi().IsOn()) {
    return Raw(ErrorCode::Esp32::kWifiInitFailed);
  }
  if (!sys.Tcp().Running()) {
    return Raw(ErrorCode::Esp32::kTcpServerStartFailed);
  }

  return Raw(ErrorCode::Common::kOk);
}

[[nodiscard]] uint32_t EnterRecoveryProgramMode() {
  System &sys = Sys();
  sys.Button().FlushEvents();
  sys.Ui().SetAppState(Ui::AppState::kProgram);
  sys.Ui().NotifyUserActivity();
  sys.Programmer().Start(sys.Tcp().GetStatus().total);
  return sys.Programmer().Written();
}

class RecoverySession {
 public:
  explicit RecoverySession(System &sys);

  uint32_t RunUntilFailure();

 private:
  enum class Mode : uint8_t {
    kDfu,
    kProgram,
  };

  enum class NetworkAction : uint8_t {
    kKeepNetwork,
    kStopNetwork,
  };

  bool EnterDfuMode(TimeMs now, NetworkAction network_on_error);
  void EnterProgramMode(TimeMs now);
  void StepDfuMode(TimeMs now);
  void StepProgramMode(TimeMs now);
  void Exit(uint32_t code, NetworkAction network);

  System &sys_;
  TcpServer &tcp_;
  Programmer &prog_;
  Mode mode_ = Mode::kDfu;
  uint32_t result_ = Raw(ErrorCode::Common::kOk);
  bool exit_ = false;
  TimeMs last_activity_ = 0;
  uint32_t last_written_ = 0;
};

RecoverySession::RecoverySession(System &sys)
    : sys_(sys), tcp_(sys.Tcp()), prog_(sys.Programmer()) {}

bool RecoverySession::EnterDfuMode(TimeMs now, NetworkAction network_on_error) {
  const uint32_t recovery_error = EnterRecoveryDfuMode();
  if (recovery_error != Raw(ErrorCode::Common::kOk)) {
    Exit(recovery_error, network_on_error);
    return false;
  }

  mode_ = Mode::kDfu;
  last_written_ = prog_.Written();
  last_activity_ = now;
  return true;
}

void RecoverySession::StepDfuMode(TimeMs now) {
  while (auto ev = tcp_.PopEvent()) {
    switch (ev->id) {
      case TcpServer::EventId::kBegin:
        tcp_.SendCtrlLine("OK\n");
        tcp_.StartDownload(ev->begin.size);
        prog_.SetTarget(ev->begin.target);
        EnterProgramMode(now);
        return;
      case TcpServer::EventId::kAbort: {
        prog_.Abort();
        tcp_.StopDownload();
        if (!EnterDfuMode(now, NetworkAction::kStopNetwork)) {
          return;
        }
        break;
      }
      case TcpServer::EventId::kReset:
        tcp_.DisableBridge();
        (void)prog_.Boot();
        esp_restart();
        break;
      case TcpServer::EventId::kBridge:
        tcp_.EnableBridge();
        break;
      case TcpServer::EventId::kNone:
      case TcpServer::EventId::kCtrlUp:
      case TcpServer::EventId::kCtrlDown:
      case TcpServer::EventId::kDataUp:
      case TcpServer::EventId::kDataDown:
      default:
        break;
    }
  }
}

void RecoverySession::EnterProgramMode(TimeMs now) {
  mode_ = Mode::kProgram;
  last_written_ = EnterRecoveryProgramMode();
  last_activity_ = now;
}

void RecoverySession::StepProgramMode(TimeMs now) {
  prog_.Poll();

  if (prog_.Error()) {
    const uint32_t programmer_error = prog_.LastErrorCode();
    tcp_.StopDownload();
    prog_.Abort();
    Exit(programmer_error, NetworkAction::kStopNetwork);
    return;
  }

  if (prog_.Done()) {
    TcpServer::Status st{};
    st.rx = prog_.Written();
    st.total = prog_.Total();
    st.state = 1;
    tcp_.StopDownload();
    tcp_.SetStatus(st);
    (void)prog_.Boot();
    (void)EnterDfuMode(now, NetworkAction::kStopNetwork);
    return;
  }

  while (auto ev = tcp_.PopEvent()) {
    switch (ev->id) {
      case TcpServer::EventId::kBegin:
        tcp_.SendCtrlLine("OK\n");
        tcp_.StartDownload(ev->begin.size);
        prog_.SetTarget(ev->begin.target);
        EnterProgramMode(now);
        return;
      case TcpServer::EventId::kAbort:
        prog_.Abort();
        tcp_.StopDownload();
        (void)EnterDfuMode(now, NetworkAction::kStopNetwork);
        return;
      case TcpServer::EventId::kReset:
        tcp_.DisableBridge();
        (void)prog_.Boot();
        esp_restart();
        break;
      case TcpServer::EventId::kBridge:
        tcp_.EnableBridge();
        break;
      case TcpServer::EventId::kCtrlDown:
      case TcpServer::EventId::kDataDown:
        prog_.Abort();
        tcp_.StopDownload();
        (void)EnterDfuMode(now, NetworkAction::kStopNetwork);
        return;
      case TcpServer::EventId::kNone:
      case TcpServer::EventId::kCtrlUp:
      case TcpServer::EventId::kDataUp:
      default:
        break;
    }
  }

  if (prog_.IsVerifying()) {
    return;
  }

  TcpServer::Status st = tcp_.GetStatus();
  st.rx = prog_.Written();
  tcp_.SetStatus(st);

  const size_t free = prog_.Free();
  if (free > 0) {
    uint8_t buf[512];
    const size_t read_size = std::min(free, sizeof(buf));
    const size_t n = tcp_.ReadDownload({buf, read_size});
    if (n > 0) {
      prog_.PushBytes({buf, n});
      last_activity_ = now;
    }
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
  result_ = code;
  exit_ = true;
}

uint32_t RecoverySession::RunUntilFailure() {
  if (!EnterDfuMode(sys_.Timebase().NowMs(), NetworkAction::kKeepNetwork)) {
    return result_;
  }

  while (true) {
    const TimeMs now = sys_.Timebase().NowMs();
    sys_.Button().Poll();
    tcp_.Poll();

    switch (mode_) {
      case Mode::kDfu:
        StepDfuMode(now);
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

[[noreturn]] void RunPanicLoop(uint32_t code) {
  Sys().Halt();
  bool recoverable = SupportsDfuRecovery(code);
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
  int level = 0;
  while (true) {
    if (recoverable) {
      Sys().Button().Poll();
      if (Sys().Button().ConsumeLongPress()) {
        code = RunRecoverableLoop();
        recoverable = SupportsDfuRecovery(code);
        msg = GetMessage(code);
        ShowPanicUi(code, recoverable);
        Sys().Mavlink().ReportPanic(Mavlink::PanicSource::kEsp32, code);
        if (recoverable) {
          Sys().Button().FlushEvents();
        }
      }
    }
    gpio_set_level(kPinMap.led, level);
    level = !level;
    ESP_LOGE(kTag, "PANIC [0x%08lX]: %s", (unsigned long)code, msg);
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

}  // namespace

[[noreturn]] void PanicImpl(uint32_t code) {
  EnsurePanicTaskStarted();
  Sys().Halt();

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
