#pragma once
#include "state_machine.hpp"
#include "system.hpp"

struct IdleState;
struct NotIdleState;

struct AppContext {
  System *sys;
  StateMachine<AppContext> *sm;

  IdleState *idle;
  NotIdleState *not_idle;

  uint32_t telemetry_interval_ms;
};

#include "dispatcher.hpp"
#include "message.hpp"

struct IdleState : public IState<AppContext> {
  const char *Name() const override { return "Idle"; }
  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;

private:
  // Epistole Parser State
  enum class RxState { kMagic1, kMagic2, kId, kLen, kPayload, kCrc1, kCrc2 };
  RxState rx_state_ = RxState::kMagic1;
  uint8_t rx_idx_ = 0;
  uint8_t rx_len_ = 0;
  struct {
    uint8_t id;
    uint8_t len;
    uint8_t payload[255];
    uint16_t crc;
  } rx_pkt_internal_;

  uint32_t last_time_sync_ms_ = 0;
  uint64_t last_imu_send_us_ = 0;
};

struct NotIdleState : public IState<AppContext> {
  const char *Name() const override { return "NotIdle"; }
  void OnEnter(AppContext &ctx, SmTick now) override;
  void OnStep(AppContext &ctx, SmTick now) override;
  void OnExit(AppContext &ctx, SmTick now) override;
};
