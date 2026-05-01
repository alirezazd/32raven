#include "panic.hpp"

#include "driver/gpio.h"
#include "esp32_config.hpp"
#include "esp32_limits.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
#include "system.hpp"

static constexpr const char *kTag = "panic";

namespace {

static constexpr uint32_t kPanicTaskStackWords =
    static_cast<uint32_t>(esp32_limits::kPanicTaskStackDepthWords);
static_assert(esp32_limits::kPanicTaskPriority < configMAX_PRIORITIES);
static constexpr UBaseType_t kPanicTaskPrio =
    static_cast<UBaseType_t>(esp32_limits::kPanicTaskPriority);
static StaticTask_t s_panic_task_buffer;
static StackType_t s_panic_task_stack[kPanicTaskStackWords];
static TaskHandle_t s_panic_task_handle = nullptr;

[[noreturn]] void RunPanicLoop(ErrorCode code);

void PanicTask(void *) {
  while (true) {
    uint32_t notified_code = static_cast<uint32_t>(ErrorCode::kUnknown);
    (void)xTaskNotifyWait(0, UINT32_MAX, &notified_code, portMAX_DELAY);
    RunPanicLoop(static_cast<ErrorCode>(notified_code));
  }
}

void EnsurePanicTaskStarted() {
  if (s_panic_task_handle != nullptr) {
    return;
  }

  s_panic_task_handle = xTaskCreateStaticPinnedToCore(
      PanicTask, "panic", kPanicTaskStackWords, nullptr, kPanicTaskPrio,
      s_panic_task_stack, &s_panic_task_buffer, 0);
}

bool SupportsDfuRecovery(ErrorCode code) {
  switch (code) {
    case ErrorCode::kUnknown:
    case ErrorCode::kButtonGpioConfigFailed:
    case ErrorCode::kWifiNvsInitFailed:
    case ErrorCode::kWifiNetifInitFailed:
    case ErrorCode::kWifiEventLoopFailed:
    case ErrorCode::kWifiInitFailed:
    case ErrorCode::kWifiSetStorageFailed:
    case ErrorCode::kI2cParamConfigFailed:
    case ErrorCode::kI2cInitFailed:
    case ErrorCode::kI2cInvalidArg:
    case ErrorCode::kI2cOperationFailed:
    case ErrorCode::kUartParamConfigFailed:
    case ErrorCode::kUartSetPinFailed:
    case ErrorCode::kUartDriverInstallFailed:
    case ErrorCode::kUartInvalidNumber:
    case ErrorCode::kUartNotInitialized:
    case ErrorCode::kUartInvalidArg:
    case ErrorCode::kUartOperationFailed:
    case ErrorCode::kProgrammerUartNull:
    case ErrorCode::kProgrammerHandshakeFailed:
    case ErrorCode::kProgrammerBufferOverflow:
    case ErrorCode::kProgrammerEraseFailed:
    case ErrorCode::kProgrammerWriteFailed:
    case ErrorCode::kProgrammerReadFailed:
    case ErrorCode::kProgrammerVerifyFailed:
    case ErrorCode::kProgrammerOtaPartitionNotFound:
    case ErrorCode::kProgrammerOtaBeginFailed:
    case ErrorCode::kProgrammerOtaWriteFailed:
    case ErrorCode::kProgrammerOtaEndFailed:
    case ErrorCode::kProgrammerOtaSetBootFailed:
    case ErrorCode::kProgrammerTimedOut:
    case ErrorCode::kDisplayPanelInitFailed:
    case ErrorCode::kUiInitFailed:
    case ErrorCode::kTcpServerStartFailed:
    case ErrorCode::kTcpServerAcceptFailed:
    case ErrorCode::kTcpServerError:
      return false;
    default:
      return true;
  }
}

void ShowPanicUi(ErrorCode code, bool recoverable) {
  Sys().Ui().SetErrorCode(code);
  Sys().Ui().SetErrorRecoverable(recoverable);
  Sys().Ui().SetAppState(Ui::AppState::kHardError);
  Sys().Ui().DisableInactivityTimeout();
  Sys().Ui().NotifyUserActivity();
}

ErrorCode EnterRecoveryDfuMode() {
  System &sys = Sys();
  sys.Button().FlushEvents();
  sys.Ui().SetAppState(Ui::AppState::kDfu);
  sys.Ui().NotifyUserActivity();
  sys.StartNetwork();
  sys.Tcp().DisableBridge();

  if (!sys.Wifi().IsOn()) {
    return ErrorCode::kWifiInitFailed;
  }
  if (!sys.Tcp().Running()) {
    return ErrorCode::kTcpServerStartFailed;
  }

  return ErrorCode::kOk;
}

void EnterRecoveryProgramMode(SmTick now, SmTick *last_activity,
                              uint32_t *last_written) {
  System &sys = Sys();
  sys.Button().FlushEvents();
  sys.Ui().SetAppState(Ui::AppState::kProgram);
  sys.Ui().NotifyUserActivity();
  sys.Programmer().Start(sys.Tcp().GetStatus().total);
  if (last_activity != nullptr) {
    *last_activity = now;
  }
  if (last_written != nullptr) {
    *last_written = sys.Programmer().Written();
  }
}

class RecoverySession {
 public:
  explicit RecoverySession(System &sys);

  ErrorCode RunUntilFailure();

 private:
  enum class Mode : uint8_t {
    kDfu,
    kProgram,
  };

  bool EnterDfuMode(SmTick now, bool stop_network_on_error);
  void EnterProgramMode(SmTick now);
  void StepDfuMode(SmTick now);
  void StepProgramMode(SmTick now);
  void Exit(ErrorCode code, bool stop_network);

  System &sys_;
  TcpServer &tcp_;
  Programmer &prog_;
  Mode mode_ = Mode::kDfu;
  ErrorCode result_ = ErrorCode::kOk;
  bool exit_ = false;
  SmTick last_activity_ = 0;
  uint32_t last_written_ = 0;
};

RecoverySession::RecoverySession(System &sys)
    : sys_(sys), tcp_(sys.Tcp()), prog_(sys.Programmer()) {}

bool RecoverySession::EnterDfuMode(SmTick now, bool stop_network_on_error) {
  const ErrorCode recovery_error = EnterRecoveryDfuMode();
  if (recovery_error != ErrorCode::kOk) {
    Exit(recovery_error, stop_network_on_error);
    return false;
  }

  mode_ = Mode::kDfu;
  last_written_ = prog_.Written();
  last_activity_ = now;
  return true;
}

void RecoverySession::StepDfuMode(SmTick now) {
  while (auto ev = tcp_.PopEvent()) {
    switch (ev->id) {
      case TcpServer::EventId::kBegin:
        tcp_.StartDownload(ev->begin.size);
        prog_.SetTarget(ev->begin.target);
        EnterProgramMode(now);
        return;
      case TcpServer::EventId::kAbort: {
        prog_.Abort(now);
        tcp_.StopDownload();
        if (!EnterDfuMode(now, true)) {
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

void RecoverySession::EnterProgramMode(SmTick now) {
  mode_ = Mode::kProgram;
  EnterRecoveryProgramMode(now, &last_activity_, &last_written_);
}

void RecoverySession::StepProgramMode(SmTick now) {
  prog_.Poll(now);

  if (prog_.Error()) {
    const ErrorCode programmer_error = prog_.LastErrorCode();
    tcp_.StopDownload();
    prog_.Abort(now);
    Exit(programmer_error, true);
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
    (void)EnterDfuMode(now, true);
    return;
  }

  while (auto ev = tcp_.PopEvent()) {
    switch (ev->id) {
      case TcpServer::EventId::kBegin:
        tcp_.StartDownload(ev->begin.size);
        prog_.SetTarget(ev->begin.target);
        EnterProgramMode(now);
        return;
      case TcpServer::EventId::kAbort:
        prog_.Abort(now);
        tcp_.StopDownload();
        (void)EnterDfuMode(now, true);
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
        prog_.Abort(now);
        tcp_.StopDownload();
        (void)EnterDfuMode(now, true);
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

  size_t free = prog_.Free();
  if (free > 0) {
    uint8_t buf[512];
    size_t n =
        tcp_.ReadDownload(buf, (free < sizeof(buf)) ? free : sizeof(buf));
    if (n > 0) {
      prog_.PushBytes(buf, n, now);
      last_activity_ = now;
    }
  }

  const uint32_t current_written = prog_.Written();
  if (current_written != last_written_) {
    last_activity_ = now;
    last_written_ = current_written;
  }

  if (!prog_.Done() && (now - last_activity_) > 3000) {
    prog_.Abort(now);
    Exit(ErrorCode::kProgrammerTimedOut, true);
  }
}

void RecoverySession::Exit(ErrorCode code, bool stop_network) {
  if (stop_network) {
    sys_.StopNetwork();
  }
  result_ = code;
  exit_ = true;
}

ErrorCode RecoverySession::RunUntilFailure() {
  if (!EnterDfuMode(sys_.Timebase().NowMs(), false)) {
    return result_;
  }

  while (true) {
    const SmTick now = sys_.Timebase().NowMs();
    sys_.Button().Poll();
    tcp_.Poll(now);

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

ErrorCode RunRecoverableLoop() {
  System &sys = Sys();
  RecoverySession recovery(sys);
  return recovery.RunUntilFailure();
}

[[noreturn]] void RunPanicLoop(ErrorCode code) {
  Sys().Halt();
  bool recoverable = SupportsDfuRecovery(code);
  const char *msg = GetMessage(code);
  Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
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

[[noreturn]] void Panic(ErrorCode code) {
  EnsurePanicTaskStarted();
  Sys().Halt();

  if (s_panic_task_handle != nullptr &&
      s_panic_task_handle != xTaskGetCurrentTaskHandle()) {
    (void)xTaskNotify(s_panic_task_handle, static_cast<uint32_t>(code),
                      eSetValueWithOverwrite);
    vTaskSuspend(nullptr);
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  RunPanicLoop(code);
}
