// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "states.hpp"

#include <cstdio>

#include "ctx.hpp"
#include "esp32_config.hpp"
#include "fc_link.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
#include "timebase.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "ESP32-SM";

// LED cadence per page, so the board says where it is without the screen.
// The three tool pages share one rate; Service blinks faster because it is the
// only page waiting on a host to connect.
static constexpr uint32_t kServingBreatheMs = 3000;
static constexpr uint32_t kServiceBlinkMs = 400;
static constexpr uint32_t kToolPageBlinkMs = 800;

// Same patience as the boot handshake: the same STM32 on the same link.
static constexpr uint16_t kStm32RequestAttempts =
    FcLink::HandshakeAttempts(kFcLinkConfig.handshake_window_s);

// Mavlink().Poll stays at the call sites, so a page that wants the radio has
// to say so.
static void DrainFcLink(AppContext &ctx) {
  ctx.sys->FcLink().Poll();
  while (auto packet = ctx.sys->FcLink().PopPacket()) {
    ctx.sys->CommandHandler().Dispatch(ctx, *packet);
  }
}

struct MenuTarget {
  IState<AppContext> &state;
  const char *name;
};

// The press that wakes the display is spent doing that, so a short press only
// acts on an awake screen; a long press always acts.
static bool CycleOnButton(AppContext &ctx, const char *from,
                          const MenuTarget &press,
                          const MenuTarget &long_press,
                          void (*on_exit)(AppContext &) = nullptr) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (button.ConsumePress() && ctx.sys->Ui().NotifyUserActivity()) {
    ESP_LOGI(kTag, "%s -> %s (press)", from, press.name);
    if (on_exit != nullptr) {
      on_exit(ctx);
    }
    ctx.sm->ReqTransition(press.state);
    return true;
  }
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "%s -> %s (long press)", from, long_press.name);
    if (on_exit != nullptr) {
      on_exit(ctx);
    }
    ctx.sm->ReqTransition(long_press.state);
    return true;
  }
  return false;
}

// Navigation model. Two menus, one gesture each:
//   short press — cycle within the current menu
//     normal: Serving -> MavlinkWifi -> WifiLog -> UsbLog -> MavlinkUsb
//             -> Serving
//     config: Service -> EscConfig -> Service
//   hold        — swap menus, from anywhere
//     normal -> Service (config entry point), config -> Serving
// Program is transient and sits outside the cycle; a hold aborts it back to
// Service rather than leaving the config menu mid-flash.

// Serving State
void ServingState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Serving");
  ctx.sys->Ui().SetAppState(Ui::AppState::kServing);
  // Telem UART is the always-on default link — a SiK radio (or any
  // transparent serial peer) starts seeing heartbeats the moment it's
  // plugged in, with no user action required.
  ctx.sys->Mavlink().SetTransport(&ctx.sys->Telem());
  ctx.sys->Mavlink().SetTelemetryLink(true);
  // The STM32 keeps streaming FC link packets while Service/Program modes are not
  // polling them, so resume in a fresh resync state instead of parsing stale
  // buffered bytes as fatal corruption.
  ctx.sys->FcLink().ResetRxState();
  ctx.sys->StopNetwork();
  ctx.sys->Led().SetPattern(LED::Pattern::kBreathe, kServingBreatheMs);
}

void ServingState::OnStep(AppContext &ctx) {
  if (CycleOnButton(ctx, "Serving", {*ctx.mavlink_wifi_state, "MavlinkWifi"},
                    {*ctx.service_state, "Service"})) {
    return;
  }
  DrainFcLink(ctx);
  ctx.sys->Mavlink().Poll(ctx.now_ms);
}

// MavlinkWifi State
void MavlinkWifiState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering MavlinkWifi");
  ctx.sys->Ui().SetAppState(Ui::AppState::kMavlinkWifi);
  ctx.sys->Mavlink().SetTransport(&ctx.sys->Udp());
  ctx.sys->Mavlink().SetTelemetryLink(true);
  ctx.sys->Tcp().Stop();
  ctx.sys->Wifi().StartAp();
  // Telemetry only: a dead socket must not take the whole bench tool down.
  ctx.sys->Udp().Start();
}

void MavlinkWifiState::OnStep(AppContext &ctx) {
  if (CycleOnButton(ctx, "MavlinkWifi", {*ctx.wifi_log_state, "WifiLog"},
                    {*ctx.service_state, "Service"})) {
    return;
  }
  DrainFcLink(ctx);
  ctx.sys->Mavlink().Poll(ctx.now_ms);
}

// MavlinkUsb State
void MavlinkUsbState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering MavlinkUsb");
  // For now the UI re-uses the WiFi screen — the widget reads the active
  // transport from the Mavlink service to render the right text.
  ctx.sys->Ui().SetAppState(Ui::AppState::kMavlinkUsb);
  ctx.sys->Mavlink().SetTransport(&ctx.sys->UsbCdc());
  ctx.sys->Mavlink().SetTelemetryLink(true);
  // No AP / UDP socket needed for USB CDC; tear them down so an already
  // associated WiFi peer doesn't keep consuming radio.
  ctx.sys->StopNetwork();
}

void MavlinkUsbState::OnStep(AppContext &ctx) {
  if (CycleOnButton(ctx, "MavlinkUsb", {*ctx.serving_state, "Serving"},
                    {*ctx.service_state, "Service"})) {
    return;
  }
  DrainFcLink(ctx);
  ctx.sys->Mavlink().Poll(ctx.now_ms);
}

// Service State
void ServiceState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Service");
  ctx.sys->Ui().SetAppState(Ui::AppState::kService);
  ctx.sys->Mavlink().SetTelemetryLink(false);
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, kServiceBlinkMs);
  // Panicking here would be unrecoverable (kTcpServerStartFailed is not
  // Service-recoverable), so a failed start only leaves nothing listening.
  ctx.sys->StartNetwork();
  ctx.sys->Tcp().CloseDataRx();
}

void ServiceState::OnStep(AppContext &ctx) {
  if (CycleOnButton(ctx, "Service", {*ctx.esc_config_state, "EscConfig"},
                    {*ctx.serving_state, "Serving"})) {
    return;
  }

  ctx.sys->Tcp().Poll();

  while (auto ev = ctx.sys->Tcp().PopEvent()) {
    switch (ctx.sys->CommandHandler().Dispatch(ctx, *ev)) {
      case CommandHandler::ServiceTcpAction::kEnterProgram:
        ctx.sys->Tcp().SendCtrlLine("OK\n");
        ctx.sm->ReqTransition(*ctx.program_state);
        return;
      case CommandHandler::ServiceTcpAction::kEnterLogPull:
        // Logs are served from the WiFi log page, where the screen shows
        // the transfer.
        ctx.sys->Tcp().SendCtrlLine("ERR wrong_page\n");
        break;
      case CommandHandler::ServiceTcpAction::kStayInService:
        break;
    }
  }

  (void)ctx.sys->Tcp().TakeLinkDrops();
}

// Program State
void ProgramState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering Program mode");
  ctx.sys->Ui().SetAppState(Ui::AppState::kProgram);
  // Treat programming start like user activity so the progress UI is visible.
  ctx.sys->Ui().NotifyUserActivity();
  ctx.sys->Mavlink().SetTelemetryLink(false);
  ctx.sys->Programmer().Start(ctx.sys->Tcp().GetStatus().total);
  ctx.sys->Led().Off();
}

void ProgramState::OnStep(AppContext &ctx) {
  auto &button = ctx.sys->Button();
  button.Poll();

  if (button.ConsumePress()) {
    ctx.sys->Ui().NotifyUserActivity();
  }
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "Program -> Service (long press)");
    ctx.sys->Programmer().Abort();
    ctx.sys->Tcp().EndTransfer();
    ctx.sm->ReqTransition(*ctx.service_state);
    return;
  }

  ctx.sys->Tcp().Poll();
  ctx.sys->Programmer().Poll();

  auto &tcp = ctx.sys->Tcp();
  auto &prog = ctx.sys->Programmer();

  if (prog.Done()) {
    ESP_LOGI(kTag, "Prog Done -> Transitioning to Service");
    TcpServer::Status st{};
    st.rx = prog.Written();
    st.total = prog.Total();
    st.state = TcpServer::Status::kDone;
    tcp.EndTransfer();
    tcp.SetStatus(st);

    ctx.sys->Programmer().Boot();
    ctx.sm->ReqTransition(*ctx.service_state);
    return;
  }

  while (auto ev = tcp.PopEvent()) {
    if (ev->id == TcpServer::EventId::kAbort) {
      ESP_LOGE(kTag, "ProgramState: ABORT");
      tcp.EndTransfer();
      prog.Abort();
      ctx.sm->ReqTransition(*ctx.service_state);
      return;
    }
  }

  const TcpServer::LinkDrops drops = tcp.TakeLinkDrops();
  if (drops.ctrl || drops.data) {
    ESP_LOGE(kTag, "ProgramState: link drop -> Abort");
    tcp.EndTransfer();
    prog.Abort();
    // Service, where a completed flash also lands: the network stays up and
    // the host can retry without walking the menu again. Only the unasked-for
    // endings sound -- an ABORT line is the host's own doing, a dropped
    // socket is not, and the STM32 left half-written says so on next boot.
    ctx.sys->TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kWarning);
    ctx.sm->ReqTransition(*ctx.service_state);
    return;
  }

  if (prog.IsVerifying()) {
    ctx.sys->Led().Toggle();
    // Writing is over, so rx would otherwise sit frozen at the last written
    // byte for the whole verify pass. state distinguishes the two counts.
    TcpServer::Status verifying = tcp.GetStatus();
    verifying.rx = prog.VerifyOffset();
    verifying.state = TcpServer::Status::kVerifying;
    tcp.SetStatus(verifying);
    return;
  }

  TcpServer::Status st = tcp.GetStatus();
  st.rx = prog.Written();
  tcp.SetStatus(st);

  size_t free = prog.Free();
  if (free > 0) {
    uint8_t buf[512];
    const std::span<uint8_t> chunk{buf};
    const size_t n = tcp.ReadDataRx(
        chunk.first((free < chunk.size()) ? free : chunk.size()));
    if (n > 0) {
      prog.PushBytes(chunk.first(n));
      ctx.sys->Led().Toggle();
    }
  }
}

static void SendUsbMode(AppContext &ctx, message::UsbMode mode) {
  ctx.sys->FcLink().SendPacket(
      message::MsgId::kSetUsbMode,
      message::SetUsbModeMsg{.mode = static_cast<uint8_t>(mode)});
}

static void DropUsbMode(AppContext &ctx) {
  SendUsbMode(ctx, message::UsbMode::kNone);
}

// EscConfig State
void EscConfigState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering EscConfig");
  ctx.sys->Ui().SetAppState(Ui::AppState::kEscConfig);
  // The configurator reaches the STM32 over its USB, so the telem UART is idle
  // here. Keeping the radio up means the ground is told the vehicle is not
  // flight-ready rather than being told nothing at all.
  ctx.sys->Mavlink().SetTransport(&ctx.sys->Telem());
  ctx.sys->Mavlink().SetTelemetryLink(true);
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, kToolPageBlinkMs);
  ctx.sys->StopNetwork();
  ctx.sys->FcLink().ResetRxState();
  warned_armed_ = false;
  stream_seen_ = false;
  activity_.Reset(0);
  grant_.Begin(ctx.now_ms);
  SendUsbMode(ctx, message::UsbMode::kEscConfig);
}

void EscConfigState::OnStep(AppContext &ctx) {
  if (CycleOnButton(ctx, "EscConfig", {*ctx.service_state, "Service"},
                    {*ctx.serving_state, "Serving"},
                    DropUsbMode)) {
    return;
  }

  DrainFcLink(ctx);
  ctx.sys->Mavlink().Poll(ctx.now_ms);

  // The STM32 publishes kUsbStatus only while it is in ESC config, so the
  // stream arriving at all is the grant -- no flag has to carry it, and its
  // absence is the only evidence a lost request leaves. Retrying on the
  // handshake cadence rather than on the report closes the loop that a
  // report-driven retry cannot: no reports means nothing to answer.
  if (!stream_seen_) {
    if (ctx.sys->Ui().PeerUsb(ctx.now_ms).has_value()) {
      stream_seen_ = true;
    } else if (grant_.Due(ctx.now_ms, FcLink::kHandshakeRetryPeriodMs)) {
      if (grant_.Exhausted(kStm32RequestAttempts)) {
        ESP_LOGW(kTag, "EscConfig -> Serving (STM32 never opened the port)");
        ctx.sys->TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kWarning);
        ctx.sys->Ui().NotifyUserActivity();
        ctx.sm->ReqTransition(*ctx.serving_state);
        return;
      }
      grant_.Sent(ctx.now_ms);
      SendUsbMode(ctx, message::UsbMode::kEscConfig);
    }
  }

  // Edge triggered because the report arrives every second either way, so
  // anything less would hold the screen awake for the whole session.
  if (const auto usb = ctx.sys->Ui().PeerUsb(ctx.now_ms)) {
    const uint32_t frames =
        (static_cast<uint32_t>(usb->rx_frames) << 8) | usb->tx_frames;
    if (activity_.Advanced(frames)) {
      ctx.sys->Ui().NotifyUserActivity();
    }
  }

  // The port stays shut while armed, and the screen says so -- but the screen
  // may well be asleep, so say it out loud too. Edge-triggered: the condition
  // holds for as long as the vehicle stays armed.
  const bool armed = ctx.sys->Mavlink().PeerArmed(ctx.now_ms).value_or(false);
  if (armed && !warned_armed_) {
    ctx.sys->TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kWarning);
    ctx.sys->Ui().NotifyUserActivity();
  }
  warned_armed_ = armed;
}

// UsbLog State
void UsbLogState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering UsbLog");
  ctx.sys->Ui().SetAppState(Ui::AppState::kUsbLog);
  ctx.sys->Mavlink().SetTelemetryLink(false);
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, kToolPageBlinkMs);
  ctx.sys->StopNetwork();
  ctx.sys->FcLink().ResetRxState();
  stream_seen_ = false;
  activity_.Reset(0);
  grant_.Begin(ctx.now_ms);
  SendUsbMode(ctx, message::UsbMode::kMsc);
}

void UsbLogState::OnStep(AppContext &ctx) {
  if (CycleOnButton(ctx, "UsbLog", {*ctx.mavlink_usb_state, "MavlinkUsb"},
                    {*ctx.service_state, "Service"},
                    DropUsbMode)) {
    return;
  }

  DrainFcLink(ctx);

  // The kUsbStatus stream exists only inside the session, so its arrival is
  // the grant.
  if (!stream_seen_) {
    if (ctx.sys->Ui().PeerUsb(ctx.now_ms).has_value()) {
      stream_seen_ = true;
    } else if (grant_.Due(ctx.now_ms, FcLink::kHandshakeRetryPeriodMs)) {
      if (grant_.Exhausted(kStm32RequestAttempts)) {
        ESP_LOGW(kTag, "UsbLog -> Serving (STM32 never granted MSC)");
        ctx.sys->TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kWarning);
        ctx.sys->Ui().NotifyUserActivity();
        ctx.sm->ReqTransition(*ctx.serving_state);
        return;
      }
      grant_.Sent(ctx.now_ms);
      SendUsbMode(ctx, message::UsbMode::kMsc);
    }
  }

  // Block counters ride the frame fields; movement means the host is copying.
  if (const auto usb = ctx.sys->Ui().PeerUsb(ctx.now_ms)) {
    const uint32_t frames =
        (static_cast<uint32_t>(usb->rx_frames) << 8) | usb->tx_frames;
    if (activity_.Advanced(frames)) {
      ctx.sys->Ui().NotifyUserActivity();
    }
  }
}

// WifiLog State
void WifiLogState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering WifiLog");
  ctx.sys->Ui().SetAppState(Ui::AppState::kWifiLog);
  ctx.sys->Mavlink().SetTelemetryLink(false);
  ctx.sys->Led().SetPattern(LED::Pattern::kBlink, kToolPageBlinkMs);
  ctx.sys->FcLink().ResetRxState();
  // Best effort: a failed start only leaves nothing listening.
  ctx.sys->StartNetwork();
  ctx.sys->Tcp().CloseDataRx();
}

void WifiLogState::OnStep(AppContext &ctx) {
  if (CycleOnButton(ctx, "WifiLog", {*ctx.usb_log_state, "UsbLog"},
                    {*ctx.service_state, "Service"})) {
    return;
  }

  ctx.sys->Tcp().Poll();
  DrainFcLink(ctx);

  while (auto ev = ctx.sys->Tcp().PopEvent()) {
    switch (ctx.sys->CommandHandler().Dispatch(ctx, *ev)) {
      case CommandHandler::ServiceTcpAction::kEnterLogPull:
        ctx.sys->Tcp().SendCtrlLine("OK\n");
        ctx.sm->ReqTransition(*ctx.log_pull_state);
        return;
      case CommandHandler::ServiceTcpAction::kEnterProgram:
        // BEGIN's dispatch already opened the transfer, so close it first.
        ctx.sys->Tcp().EndTransfer();
        ctx.sys->Tcp().SendCtrlLine("ERR wrong_page\n");
        break;
      case CommandHandler::ServiceTcpAction::kStayInService:
        break;
    }
  }

  (void)ctx.sys->Tcp().TakeLinkDrops();
}

// LogPull State
namespace {
constexpr size_t kSha256Bytes = 32;
constexpr size_t kSha256HexSize = (kSha256Bytes * 2) + 1;
constexpr uint32_t kLogPullRetryMs = 400;
// A peer that vanishes without closing leaves the socket blocking rather
// than erroring, so refusing to wait forever is the only way out.
constexpr uint32_t kLogPullStallMs = 10000;
constexpr uint8_t kLogPullMaxAttempts = 5;
}  // namespace

void LogPullState::PrepareList() {
  op_ = Op::kList;
  list_first_ = 0;
}

void LogPullState::PrepareGet(const char *name) {
  op_ = Op::kGet;
  std::snprintf(name_, sizeof(name_), "%s", name);
  offset_ = 0;
}

void LogPullState::OnEnter(AppContext &ctx) {
  ESP_LOGI(kTag, "entering LogPull (%s)", op_ == Op::kList ? "list" : name_);
  ctx_ = &ctx;
  // Stays on the WiFi log screen, where the lanes show the transfer.
  ctx.sys->Ui().SetAppState(Ui::AppState::kWifiLog);
  reply_pending_ = false;
  done_ = false;
  reply_.Clear();
  chunk_valid_ = false;
  chunk_sent_ = 0;
  total_bytes_ = 0;
  rx_frames_ = 0;
  tx_frames_ = 0;
  activity_.Reset(0);
  last_progress_ms_ = ctx.now_ms;
  ctx.sys->Ui().UpdateLogTraffic(rx_frames_, tx_frames_);
  mbedtls_sha256_init(&sha_);
  mbedtls_sha256_starts(&sha_, 0);
  SendRequest(ctx);
}

void LogPullState::SendRequest(AppContext &ctx) {
  if (op_ == Op::kList) {
    ctx.sys->FcLink().SendPacket(message::MsgId::kLogList,
                                 message::LogListMsg{.first = list_first_});
  } else {
    message::LogReadMsg req{};
    std::memcpy(req.name, name_, message::kLogNameLen);
    req.offset = offset_;
    req.len = message::kLogDataMaxBytes;
    ctx.sys->FcLink().SendPacket(message::MsgId::kLogRead, req);
  }
  reply_pending_ = true;
  reply_.Sent(ctx.now_ms);
  ++tx_frames_;
  ctx.sys->Ui().UpdateLogTraffic(rx_frames_, tx_frames_);
}

void LogPullState::OnListReply(const message::LogListReplyMsg &reply) {
  reply_pending_ = false;
  reply_.Clear();
  ++rx_frames_;
  ctx_->sys->Ui().UpdateLogTraffic(rx_frames_, tx_frames_);
  if (static_cast<message::LogStatus>(reply.status) !=
      message::LogStatus::kOk) {
    Finish(*ctx_, "ERR busy\n");
    return;
  }
  char line[48];
  for (uint8_t i = 0; i < reply.count && i < message::kLogListMaxEntries; ++i) {
    char name[13] = {};
    std::memcpy(name, reply.entries[i].name, message::kLogNameLen);
    std::snprintf(line, sizeof(line), "LOG %s %u\n", name,
                  (unsigned)reply.entries[i].size_bytes);
    ctx_->sys->Tcp().SendCtrlLine(line);
  }
  const uint8_t next = static_cast<uint8_t>(reply.first + reply.count);
  if (reply.count == message::kLogListMaxEntries && next < reply.total) {
    list_first_ = next;
    SendRequest(*ctx_);
    return;
  }
  std::snprintf(line, sizeof(line), "DONE total=%u\n", (unsigned)reply.total);
  Finish(*ctx_, line);
}

void LogPullState::OnData(const message::LogDataMsg &data) {
  if (data.offset != offset_) {
    return;  // an answer to a request this state already gave up on
  }
  reply_pending_ = false;
  reply_.Clear();
  ++rx_frames_;
  ctx_->sys->Ui().UpdateLogTraffic(rx_frames_, tx_frames_);
  if (static_cast<message::LogStatus>(data.status) != message::LogStatus::kOk) {
    Finish(*ctx_, static_cast<message::LogStatus>(data.status) ==
                          message::LogStatus::kNotFound
                      ? "ERR not_found\n"
                      : "ERR busy\n");
    return;
  }
  if (data.len == 0u) {
    done_ = true;
    return;
  }
  chunk_ = data;
  chunk_sent_ = 0;
  chunk_valid_ = true;
  last_progress_ms_ = ctx_->now_ms;
  mbedtls_sha256_update(&sha_, data.data, data.len);
  total_bytes_ += data.len;
  offset_ += data.len;
}

void LogPullState::Finish(AppContext &ctx, const char *ctrl_line) {
  ctx.sys->Tcp().SendCtrlLine(ctrl_line);
  mbedtls_sha256_free(&sha_);
  ctx.sm->ReqTransition(*ctx.wifi_log_state);
}

void LogPullState::OnStep(AppContext &ctx) {
  auto &button = ctx.sys->Button();
  button.Poll();
  if (button.ConsumeLongPress()) {
    ctx.sys->Ui().NotifyUserActivity();
    Finish(ctx, "ERR aborted\n");
    return;
  }

  ctx.sys->Tcp().Poll();
  DrainFcLink(ctx);

  // A pull outlasts the inactivity timeout many times over, so traffic keeps
  // the screen up.
  const uint32_t frames =
      (static_cast<uint32_t>(rx_frames_) << 16) | tx_frames_;
  if (activity_.Advanced(frames)) {
    ctx.sys->Ui().NotifyUserActivity();
  }

  // Forward before requesting more: the unsent tail is the flow control.
  if (chunk_valid_) {
    const int sent = ctx.sys->Tcp().SendData(&chunk_.data[chunk_sent_],
                                             chunk_.len - chunk_sent_);
    // Negative is the socket reporting the peer gone; zero is only
    // backpressure, which the stall deadline below bounds.
    if (sent < 0) {
      Finish(ctx, "ERR peer_gone\n");
      return;
    }
    if (sent > 0) {
      chunk_sent_ = static_cast<uint16_t>(chunk_sent_ + sent);
      last_progress_ms_ = ctx.now_ms;
    }
    if (chunk_sent_ < chunk_.len) {
      if ((ctx.now_ms - last_progress_ms_) >= kLogPullStallMs) {
        Finish(ctx, "ERR peer_stalled\n");
      }
      return;
    }
    chunk_valid_ = false;
    if (!done_) {
      SendRequest(ctx);
      return;
    }
  }

  if (done_ && !chunk_valid_) {
    char line[96];
    uint8_t hash[kSha256Bytes];
    mbedtls_sha256_finish(&sha_, hash);
    char hex[kSha256HexSize];
    for (size_t i = 0; i < kSha256Bytes; ++i) {
      std::snprintf(&hex[i * 2], 3, "%02x", hash[i]);
    }
    std::snprintf(line, sizeof(line), "DONE size=%u sha256=%s\n",
                  (unsigned)total_bytes_, hex);
    Finish(ctx, line);
    return;
  }

  if (reply_pending_ && reply_.Due(ctx.now_ms, kLogPullRetryMs)) {
    if (reply_.Exhausted(kLogPullMaxAttempts)) {
      Finish(ctx, "ERR fc_timeout\n");
      return;
    }
    SendRequest(ctx);
  }
}
