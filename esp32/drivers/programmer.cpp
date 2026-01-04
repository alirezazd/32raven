#include "programmer.hpp"

void Programmer::EnterBootState::OnEnter(Ctx &c, SmTick now) {
  c.st_enter = this;
}

void Programmer::Init(const Config &cfg, Uart *uart) {
  if (initialized_)
    return;
  ctx_.cfg = cfg;
  ctx_.uart = uart;

  // Set up state pointers
  ctx_.st_idle = &st_idle_;
  ctx_.st_enter = &st_enter_;
  ctx_.st_sync = &st_sync_;
  ctx_.st_ready = &st_ready_;
  ctx_.st_error = &st_error_;

  sm_.Start(st_idle_, 0);

  initialized_ = true;
}

void Programmer::Step(SmTick now) {
  if (!initialized_)
    return;
  sm_.Step(now);
}