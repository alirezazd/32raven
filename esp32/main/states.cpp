// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "states.hpp"

#include <algorithm>
#include <cstdio>

#include "esp32_config.hpp"
#include "fc_link.hpp"
#include "host_link.hpp"
#include "state_machine_context.hpp"
#include "system.hpp"
#include "tcp_server.hpp"
#include "timebase.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

static constexpr const char *kTag = "ESP32-SM";

// LED cadence per mode, so the board says where it is without the screen.
// The three tool modes share one rate; Service blinks faster because it is the
// only mode waiting on a host to connect.
static constexpr uint32_t kServingBreatheMs = 3000;
static constexpr uint32_t kServiceBlinkMs = 400;
static constexpr uint32_t kToolModeBlinkMs = 800;

// Same patience as the boot handshake: the same STM32 on the same link.
static constexpr uint16_t kStm32RequestAttempts =
    FcLink::HandshakeAttempts(kFcLinkConfig.handshake_window_s);

// Mavlink().Poll stays at the call sites, so a mode that wants the radio has
// to say so.
static void DrainFcLink(StateMachineContext &ctx) {
  System::GetInstance().FcLink().Poll();
  while (auto packet = System::GetInstance().FcLink().PopPacket()) {
    System::GetInstance().CommandHandler().Dispatch(ctx, *packet);
  }
}

struct MenuTarget {
  IState<StateMachineContext> &state;
  const char *name;
};

// The press that wakes the display is spent doing that, so a short press only
// acts on an awake screen; a long press always acts.
static bool CycleOnButton(StateMachineContext &ctx, const char *from,
                          const MenuTarget &press,
                          const MenuTarget &long_press) {
  auto &button = System::GetInstance().Button();
  button.Poll();

  if (button.ConsumePress() &&
      System::GetInstance().Ui().NotifyUserActivity()) {
    ESP_LOGI(kTag, "%s -> %s (press)", from, press.name);
    ctx.sm->ReqTransition(press.state);
    return true;
  }
  if (button.ConsumeLongPress()) {
    System::GetInstance().Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "%s -> %s (long press)", from, long_press.name);
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
void ServingState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering Serving");
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kServing);
  // Telem UART is the always-on default link — a SiK radio (or any
  // transparent serial peer) starts seeing heartbeats the moment it's
  // plugged in, with no user action required.
  System::GetInstance().Mavlink().SetTransport(&System::GetInstance().Telem());
  System::GetInstance().Mavlink().SetTelemetryLink(true);
  System::GetInstance().StopNetwork();
  System::GetInstance().Led().SetPattern(LED::Pattern::kBreathe,
                                         kServingBreatheMs);
}

void ServingState::OnStep(StateMachineContext &ctx) {
  if (CycleOnButton(ctx, "Serving", {ctx.mavlink_wifi_state, "MavlinkWifi"},
                    {ctx.service_state, "Service"})) {
    return;
  }
  DrainFcLink(ctx);
  System::GetInstance().Mavlink().Poll(ctx.now_ms);
}

// MavlinkWifi State
void MavlinkWifiState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering MavlinkWifi");
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kMavlinkWifi);
  System::GetInstance().Mavlink().SetTransport(&System::GetInstance().Udp());
  System::GetInstance().Mavlink().SetTelemetryLink(true);
  System::GetInstance().Tcp().Stop();
  System::GetInstance().Wifi().StartAp();
  // Telemetry only: a dead socket must not take the whole bench tool down.
  System::GetInstance().Udp().Start();
}

void MavlinkWifiState::OnStep(StateMachineContext &ctx) {
  if (CycleOnButton(ctx, "MavlinkWifi", {ctx.wifi_log_state, "WifiLog"},
                    {ctx.service_state, "Service"})) {
    return;
  }
  DrainFcLink(ctx);
  System::GetInstance().Mavlink().Poll(ctx.now_ms);
}

// MavlinkUsb State
void MavlinkUsbState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering MavlinkUsb");
  // For now the UI re-uses the WiFi screen — the widget reads the active
  // transport from the Mavlink service to render the right text.
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kMavlinkUsb);
  System::GetInstance().Mavlink().SetTransport(&System::GetInstance().UsbCdc());
  System::GetInstance().Mavlink().SetTelemetryLink(true);
  // No AP / UDP socket needed for USB CDC; tear them down so an already
  // associated WiFi peer doesn't keep consuming radio.
  System::GetInstance().StopNetwork();
}

void MavlinkUsbState::OnStep(StateMachineContext &ctx) {
  if (CycleOnButton(ctx, "MavlinkUsb", {ctx.serving_state, "Serving"},
                    {ctx.service_state, "Service"})) {
    return;
  }
  DrainFcLink(ctx);
  System::GetInstance().Mavlink().Poll(ctx.now_ms);
}

// Service State
void ServiceState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering Service");
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kService);
  System::GetInstance().Mavlink().SetTelemetryLink(false);
  System::GetInstance().Led().SetPattern(LED::Pattern::kBlink, kServiceBlinkMs);
  // Panicking here would be unrecoverable (kTcpServerStartFailed is not
  // Service-recoverable), so a failed start only leaves nothing listening.
  System::GetInstance().StartNetwork();
  System::GetInstance().Tcp().CloseDataRx();
  System::GetInstance().UsbHost().Start();
}

// This mode never drains FcLink, so the STM32's stream has been piling up
// unparsed the whole time. Handed on as a resync rather than as a buffer the
// next mode would read as fatal corruption. Program is the other such mode.
void ServiceState::OnExit(StateMachineContext &ctx) {
  System::GetInstance().FcLink().ResetRxState();
  // The USB session is only valid while a mode is attending it, and Program
  // is the only one that keeps doing so -- for the link that armed its
  // transfer, and no other. Anything else leaves it unattended, so it is
  // stopped here and the next entry to Service opens a clean one rather than
  // answering a command that has been sitting in the queue since.
  if (ctx.host_link != &System::GetInstance().UsbHost())
    System::GetInstance().UsbHost().Stop();
}

void ServiceState::OnStep(StateMachineContext &ctx) {
  if (CycleOnButton(ctx, "Service", {ctx.esc_config_state, "EscConfig"},
                    {ctx.serving_state, "Serving"})) {
    return;
  }

  HostLink *const links[] = {&System::GetInstance().Tcp(),
                             &System::GetInstance().UsbHost()};
  for (HostLink *link : links) {
    link->Poll();
    while (auto ev = link->PopEvent()) {
      System::GetInstance().CommandHandler().Dispatch(ctx, *ev);
    }
    link->ClearLinkDrop();
  }
}

// Program State
void ProgramState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering Program mode");
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kProgram);
  // Treat programming start like user activity so the progress UI is visible.
  System::GetInstance().Ui().NotifyUserActivity();
  System::GetInstance().Mavlink().SetTelemetryLink(false);
  const HostLink::BeginArgs &begin = ctx.host_link->Begin();
  System::GetInstance().Programmer().Start(begin.size, begin.crc);
  System::GetInstance().Led().Off();
}

// As ServiceState::OnExit, and the STM32 may also just have been rebooted.
void ProgramState::OnExit(StateMachineContext &ctx) {
  System::GetInstance().FcLink().ResetRxState();
  ctx.host_link = nullptr;
}

void ProgramState::OnStep(StateMachineContext &ctx) {
  auto &button = System::GetInstance().Button();
  button.Poll();

  if (button.ConsumePress()) {
    System::GetInstance().Ui().NotifyUserActivity();
  }
  if (button.ConsumeLongPress()) {
    System::GetInstance().Ui().NotifyUserActivity();
    ESP_LOGI(kTag, "Program -> Service (long press)");
    System::GetInstance().Programmer().Abort();
    ctx.host_link->EndTransfer();
    ctx.sm->ReqTransition(ctx.service_state);
    return;
  }

  ctx.host_link->Poll();
  System::GetInstance().Programmer().Poll();

  auto &link = *ctx.host_link;
  auto &prog = System::GetInstance().Programmer();

  if (prog.Done()) {
    ESP_LOGI(kTag, "Prog Done -> Transitioning to Service");
    HostLink::Status st{};
    st.rx = prog.Written();
    st.total = prog.Total();
    st.state = HostLink::Status::kDone;
    link.EndTransfer();
    link.SetStatus(st);

    System::GetInstance().Programmer().Boot();
    ctx.sm->ReqTransition(ctx.service_state);
    return;
  }

  while (auto ev = link.PopEvent()) {
    // ABORT means the transfer, not just the link, so it is handled here
    // rather than by the shared dispatch.
    if (ev->id == HostLink::EventId::kAbort) {
      ESP_LOGE(kTag, "ProgramState: ABORT");
      link.EndTransfer();
      prog.Abort();
      ctx.sm->ReqTransition(ctx.service_state);
      return;
    }
    // BEGIN and the LOG pair are queued unanswered, so whoever pops one owes
    // the host a reply; dropping it leaves the host waiting out its timeout.
    System::GetInstance().CommandHandler().Dispatch(ctx, *ev);
  }

  if (link.TakeLinkDrop()) {
    ESP_LOGE(kTag, "ProgramState: link drop -> Abort");
    link.EndTransfer();
    prog.Abort();
    // Service, where a completed flash also lands: the network stays up and
    // the host can retry without walking the menu again. Only the unasked-for
    // endings sound -- an ABORT line is the host's own doing, a dropped
    // link is not, and the STM32 left half-written says so on next boot.
    System::GetInstance().TonePlayer().PlayBuiltin(
        ::TonePlayer::BuiltinTone::kWarning);
    ctx.sm->ReqTransition(ctx.service_state);
    return;
  }

  if (prog.IsVerifying()) {
    System::GetInstance().Led().Toggle();
    // Writing is over, so rx would otherwise sit frozen at the last written
    // byte for the whole verify pass. state distinguishes the two counts.
    HostLink::Status verifying = link.GetStatus();
    verifying.rx = prog.VerifyOffset();
    verifying.state = HostLink::Status::kVerifying;
    link.SetStatus(verifying);
    return;
  }

  HostLink::Status st = link.GetStatus();
  st.rx = prog.Written();
  link.SetStatus(st);

  uint8_t buf[512];
  const std::span<uint8_t> chunk{buf};
  size_t budget = prog.TargetWriteChunkLimit();
  bool moved = false;
  while (budget > 0) {
    const size_t room = std::min({prog.Free(), budget, chunk.size()});
    if (room == 0) break;
    const size_t n = link.ReadDataRx(chunk.first(room));
    if (n == 0) break;
    prog.PushBytes(chunk.first(n));
    budget -= n;
    moved = true;
  }
  if (moved) {
    System::GetInstance().Led().Toggle();
  }
}

static void SendUsbMode(StateMachineContext &ctx, message::UsbMode mode) {
  System::GetInstance().FcLink().SendPacket(
      message::MsgId::kSetUsbMode,
      message::SetUsbModeMsg{.mode = static_cast<uint8_t>(mode)});
}

static void DropUsbMode(StateMachineContext &ctx) {
  SendUsbMode(ctx, message::UsbMode::kNone);
}

// EscConfig State
// Every way out, including giving up on the grant: an STM32 that opened the
// port after the last retry is otherwise left holding it with nobody to say
// the session is over.
void EscConfigState::OnExit(StateMachineContext &ctx) { DropUsbMode(ctx); }

void EscConfigState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering EscConfig");
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kEscConfig);
  // The configurator reaches the STM32 over its USB, so the telem UART is idle
  // here. Keeping the radio up means the ground is told the vehicle is not
  // flight-ready rather than being told nothing at all.
  System::GetInstance().Mavlink().SetTransport(&System::GetInstance().Telem());
  System::GetInstance().Mavlink().SetTelemetryLink(true);
  System::GetInstance().Led().SetPattern(LED::Pattern::kBlink,
                                         kToolModeBlinkMs);
  System::GetInstance().StopNetwork();
  warned_armed_ = false;
  stream_seen_ = false;
  activity_.Reset(0);
  grant_.Begin(ctx.now_ms);
  SendUsbMode(ctx, message::UsbMode::kEscConfig);
}

void EscConfigState::OnStep(StateMachineContext &ctx) {
  if (CycleOnButton(ctx, "EscConfig", {ctx.service_state, "Service"},
                    {ctx.serving_state, "Serving"})) {
    return;
  }

  DrainFcLink(ctx);
  System::GetInstance().Mavlink().Poll(ctx.now_ms);

  // The STM32 publishes kUsbStatus only while it is in ESC config, so the
  // stream arriving at all is the grant -- no flag has to carry it, and its
  // absence is the only evidence a lost request leaves. Retrying on the
  // handshake cadence rather than on the report closes the loop that a
  // report-driven retry cannot: no reports means nothing to answer.
  if (!stream_seen_) {
    if (System::GetInstance().Ui().PeerUsb(ctx.now_ms).has_value()) {
      stream_seen_ = true;
    } else if (grant_.Due(ctx.now_ms, FcLink::kHandshakeRetryPeriodMs)) {
      if (grant_.Exhausted(kStm32RequestAttempts)) {
        ESP_LOGW(kTag, "EscConfig -> Serving (STM32 never opened the port)");
        System::GetInstance().TonePlayer().PlayBuiltin(
            ::TonePlayer::BuiltinTone::kWarning);
        System::GetInstance().Ui().NotifyUserActivity();
        ctx.sm->ReqTransition(ctx.serving_state);
        return;
      }
      grant_.Sent(ctx.now_ms);
      SendUsbMode(ctx, message::UsbMode::kEscConfig);
    }
  }

  // Edge triggered because the report arrives every second either way, so
  // anything less would hold the screen awake for the whole session.
  if (const auto usb = System::GetInstance().Ui().PeerUsb(ctx.now_ms)) {
    const uint32_t frames =
        (static_cast<uint32_t>(usb->rx_frames) << 8) | usb->tx_frames;
    if (activity_.Advanced(frames)) {
      System::GetInstance().Ui().NotifyUserActivity();
    }
  }

  // The port stays shut while armed, and the screen says so -- but the screen
  // may well be asleep, so say it out loud too. Edge-triggered: the condition
  // holds for as long as the vehicle stays armed.
  const bool armed =
      System::GetInstance().Mavlink().PeerArmed(ctx.now_ms).value_or(false);
  if (armed && !warned_armed_) {
    System::GetInstance().TonePlayer().PlayBuiltin(
        ::TonePlayer::BuiltinTone::kWarning);
    System::GetInstance().Ui().NotifyUserActivity();
  }
  warned_armed_ = armed;
}

// UsbLog State
// As EscConfigState::OnExit.
void UsbLogState::OnExit(StateMachineContext &ctx) { DropUsbMode(ctx); }

void UsbLogState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering UsbLog");
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kUsbLog);
  System::GetInstance().Mavlink().SetTelemetryLink(false);
  System::GetInstance().Led().SetPattern(LED::Pattern::kBlink,
                                         kToolModeBlinkMs);
  System::GetInstance().StopNetwork();
  stream_seen_ = false;
  activity_.Reset(0);
  grant_.Begin(ctx.now_ms);
  SendUsbMode(ctx, message::UsbMode::kMsc);
}

void UsbLogState::OnStep(StateMachineContext &ctx) {
  if (CycleOnButton(ctx, "UsbLog", {ctx.mavlink_usb_state, "MavlinkUsb"},
                    {ctx.service_state, "Service"})) {
    return;
  }

  DrainFcLink(ctx);

  // The kUsbStatus stream exists only inside the session, so its arrival is
  // the grant.
  if (!stream_seen_) {
    if (System::GetInstance().Ui().PeerUsb(ctx.now_ms).has_value()) {
      stream_seen_ = true;
    } else if (grant_.Due(ctx.now_ms, FcLink::kHandshakeRetryPeriodMs)) {
      if (grant_.Exhausted(kStm32RequestAttempts)) {
        ESP_LOGW(kTag, "UsbLog -> Serving (STM32 never granted MSC)");
        System::GetInstance().TonePlayer().PlayBuiltin(
            ::TonePlayer::BuiltinTone::kWarning);
        System::GetInstance().Ui().NotifyUserActivity();
        ctx.sm->ReqTransition(ctx.serving_state);
        return;
      }
      grant_.Sent(ctx.now_ms);
      SendUsbMode(ctx, message::UsbMode::kMsc);
    }
  }

  // Block counters ride the frame fields; movement means the host is copying.
  if (const auto usb = System::GetInstance().Ui().PeerUsb(ctx.now_ms)) {
    const uint32_t frames =
        (static_cast<uint32_t>(usb->rx_frames) << 8) | usb->tx_frames;
    if (activity_.Advanced(frames)) {
      System::GetInstance().Ui().NotifyUserActivity();
    }
  }
}

// WifiLog State
void WifiLogState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering WifiLog");
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kWifiLog);
  System::GetInstance().Mavlink().SetTelemetryLink(false);
  System::GetInstance().Led().SetPattern(LED::Pattern::kBlink,
                                         kToolModeBlinkMs);
  // Best effort: a failed start only leaves nothing listening.
  System::GetInstance().StartNetwork();
  System::GetInstance().Tcp().CloseDataRx();
}

void WifiLogState::OnStep(StateMachineContext &ctx) {
  if (CycleOnButton(ctx, "WifiLog", {ctx.usb_log_state, "UsbLog"},
                    {ctx.service_state, "Service"})) {
    return;
  }

  System::GetInstance().Tcp().Poll();
  DrainFcLink(ctx);

  while (auto ev = System::GetInstance().Tcp().PopEvent()) {
    System::GetInstance().CommandHandler().Dispatch(ctx, *ev);
  }

  System::GetInstance().Tcp().ClearLinkDrop();
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

void LogPullState::OnEnter(StateMachineContext &ctx) {
  ESP_LOGI(kTag, "entering LogPull (%s)", op_ == Op::kList ? "list" : name_);
  ctx_ = &ctx;
  // Stays on the WiFi log screen, where the lanes show the transfer.
  System::GetInstance().Ui().SetBridgeState(Ui::BridgeState::kWifiLog);
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
  System::GetInstance().Ui().UpdateLogTraffic(rx_frames_, tx_frames_);
  mbedtls_sha256_init(&sha_);
  mbedtls_sha256_starts(&sha_, 0);
  SendRequest(ctx);
}

void LogPullState::SendRequest(StateMachineContext &ctx) {
  if (op_ == Op::kList) {
    System::GetInstance().FcLink().SendPacket(message::MsgId::kLogList,
                                 message::LogListMsg{.first = list_first_});
  } else {
    message::LogReadMsg req{};
    std::memcpy(req.name, name_, message::kLogNameLen);
    req.offset = offset_;
    req.len = message::kLogDataMaxBytes;
    System::GetInstance().FcLink().SendPacket(message::MsgId::kLogRead, req);
  }
  reply_pending_ = true;
  reply_.Sent(ctx.now_ms);
  ++tx_frames_;
  System::GetInstance().Ui().UpdateLogTraffic(rx_frames_, tx_frames_);
}

void LogPullState::OnListReply(const message::LogListReplyMsg &reply) {
  reply_pending_ = false;
  reply_.Clear();
  ++rx_frames_;
  System::GetInstance().Ui().UpdateLogTraffic(rx_frames_, tx_frames_);
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
    System::GetInstance().Tcp().SendCtrlLine(line);
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
  System::GetInstance().Ui().UpdateLogTraffic(rx_frames_, tx_frames_);
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

void LogPullState::Finish(StateMachineContext &ctx, const char *ctrl_line) {
  System::GetInstance().Tcp().SendCtrlLine(ctrl_line);
  mbedtls_sha256_free(&sha_);
  ctx.sm->ReqTransition(ctx.wifi_log_state);
}

void LogPullState::OnStep(StateMachineContext &ctx) {
  auto &button = System::GetInstance().Button();
  button.Poll();
  if (button.ConsumeLongPress()) {
    System::GetInstance().Ui().NotifyUserActivity();
    Finish(ctx, "ERR aborted\n");
    return;
  }

  System::GetInstance().Tcp().Poll();
  while (auto ev = System::GetInstance().Tcp().PopEvent()) {
    // The pull is what ABORT ends; everything else the dispatch refuses,
    // because a queued verb nobody answers hangs the host.
    if (ev->id == HostLink::EventId::kAbort) {
      Finish(ctx, "ERR aborted\n");
      return;
    }
    System::GetInstance().CommandHandler().Dispatch(ctx, *ev);
  }
  DrainFcLink(ctx);

  // A pull outlasts the inactivity timeout many times over, so traffic keeps
  // the screen up.
  const uint32_t frames =
      (static_cast<uint32_t>(rx_frames_) << 16) | tx_frames_;
  if (activity_.Advanced(frames)) {
    System::GetInstance().Ui().NotifyUserActivity();
  }

  // Forward before requesting more: the unsent tail is the flow control.
  if (chunk_valid_) {
    const int sent = System::GetInstance().Tcp().SendData(
        &chunk_.data[chunk_sent_], chunk_.len - chunk_sent_);
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
