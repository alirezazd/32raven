// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include <cstdint>

using SmTick = uint32_t;

template <typename Context>
struct IState {
  virtual ~IState() = default;
  virtual const char *Name() const = 0;
  virtual void OnEnter(Context &ctx) { (void)ctx; }
  virtual void OnStep(Context &ctx, SmTick now) = 0;
};

template <typename Context>
class StateMachine {
 public:
  explicit StateMachine(Context &ctx) : ctx_(ctx) {}

  void Start(IState<Context> &initial) {
    current_ = &initial;
    next_ = nullptr;
    current_->OnEnter(ctx_);
  }

  void Step(SmTick now) {
    // Apply any pending transition first
    if (next_) {
      IState<Context> *t = next_;
      next_ = nullptr;  // clear BEFORE transition
      TransitionTo(*t, now);
    }

    if (current_) current_->OnStep(ctx_, now);

    // Apply transition requested during on_step
    if (next_) {
      IState<Context> *t = next_;
      next_ = nullptr;
      TransitionTo(*t, now);
    }
  }

  void ReqTransition(IState<Context> &target) { next_ = &target; }

  const char *CurrentName() const {
    return current_ ? current_->Name() : "(none)";
  }

  const IState<Context> *CurrentState() const { return current_; }

 private:
  void TransitionTo(IState<Context> &target, SmTick now) {
    if (current_ == &target) return;  // ignore self transition
    current_ = &target;
    current_->OnEnter(ctx_);
  }

  Context &ctx_;
  IState<Context> *current_ = nullptr;
  IState<Context> *next_ = nullptr;
};
