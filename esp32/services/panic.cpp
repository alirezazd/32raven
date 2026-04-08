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
static constexpr UBaseType_t kPanicTaskPrio = 24;
static StaticTask_t s_panic_task_buffer;
static StackType_t s_panic_task_stack[kPanicTaskStackWords];
static TaskHandle_t s_panic_task_handle = nullptr;

[[noreturn]] void RunPanicLoop(ErrorCode code);

enum class RecoveryState : uint8_t {
  kDfu,
  kProgram,
};

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

ErrorCode RunRecoverableLoop() {
  System &sys = Sys();
  Button &button = sys.Button();
  TcpServer &tcp = sys.Tcp();
  Programmer &prog = sys.Programmer();

  ErrorCode recovery_error = EnterRecoveryDfuMode();
  if (recovery_error != ErrorCode::kOk) {
    return recovery_error;
  }

  RecoveryState state = RecoveryState::kDfu;
  SmTick last_activity = sys.Timebase().NowMs();
  uint32_t last_written = prog.Written();

  while (true) {
    const SmTick now = sys.Timebase().NowMs();
    button.Poll();

    tcp.Poll(now);

    if (state == RecoveryState::kProgram) {
      prog.Poll(now);

      if (prog.Error()) {
        prog.Abort(now);
        sys.StopNetwork();
        return prog.LastErrorCode();
      }

      if (prog.Done()) {
        TcpServer::Status st = tcp.GetStatus();
        st.state = 1;
        tcp.SetStatus(st);
        (void)prog.Boot();
        recovery_error = EnterRecoveryDfuMode();
        if (recovery_error != ErrorCode::kOk) {
          sys.StopNetwork();
          return recovery_error;
        }
        state = RecoveryState::kDfu;
        continue;
      }
    }

    while (auto ev = tcp.PopEvent()) {
      switch (ev->id) {
        case TcpServer::EventId::kBegin:
          tcp.StartDownload(ev->begin.size);
          prog.SetTarget(ev->begin.target);
          EnterRecoveryProgramMode(now, &last_activity, &last_written);
          state = RecoveryState::kProgram;
          break;
        case TcpServer::EventId::kAbort:
          prog.Abort(now);
          tcp.StopDownload();
          recovery_error = EnterRecoveryDfuMode();
          if (recovery_error != ErrorCode::kOk) {
            sys.StopNetwork();
            return recovery_error;
          }
          state = RecoveryState::kDfu;
          break;
        case TcpServer::EventId::kReset:
          tcp.DisableBridge();
          (void)prog.Boot();
          esp_restart();
          break;
        case TcpServer::EventId::kBridge:
          tcp.EnableBridge();
          break;
        case TcpServer::EventId::kCtrlDown:
        case TcpServer::EventId::kDataDown:
          if (state == RecoveryState::kProgram) {
            prog.Abort(now);
            tcp.StopDownload();
            recovery_error = EnterRecoveryDfuMode();
            if (recovery_error != ErrorCode::kOk) {
              sys.StopNetwork();
              return recovery_error;
            }
            state = RecoveryState::kDfu;
          }
          break;
        case TcpServer::EventId::kNone:
        case TcpServer::EventId::kCtrlUp:
        case TcpServer::EventId::kDataUp:
        default:
          break;
      }
    }

    if (state == RecoveryState::kProgram) {
      if (prog.IsVerifying()) {
        vTaskDelay(1);
        continue;
      }

      TcpServer::Status st = tcp.GetStatus();
      st.rx = prog.Written();
      tcp.SetStatus(st);

      size_t free = prog.Free();
      if (free > 0) {
        uint8_t buf[512];
        size_t n =
            tcp.ReadDownload(buf, (free < sizeof(buf)) ? free : sizeof(buf));
        if (n > 0) {
          prog.PushBytes(buf, n, now);
          last_activity = now;
        }
      }

      const uint32_t current_written = prog.Written();
      if (current_written != last_written) {
        last_activity = now;
        last_written = current_written;
      }

      if (!prog.Done() && (now - last_activity) > 3000) {
        prog.Abort(now);
        sys.StopNetwork();
        return ErrorCode::kProgrammerTimedOut;
      }
    }

    vTaskDelay(1);
  }
}

[[noreturn]] void RunPanicLoop(ErrorCode code) {
  Sys().HaltSystem();
  bool recoverable = SupportsDfuRecovery(code);
  const char *msg = GetMessage(code);
  Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
  ShowPanicUi(code, recoverable);
  gpio_reset_pin(kPinMap.led);
  gpio_set_direction(kPinMap.led, GPIO_MODE_OUTPUT);
  ESP_LOGE(kTag, "PANIC [0x%08lX]: %s", (unsigned long)code, msg);
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
  Sys().HaltSystem();

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
