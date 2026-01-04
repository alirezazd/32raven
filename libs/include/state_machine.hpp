#pragma once
#include <cstdint>

using SmTick = uint32_t;

struct SmEvent {
  uint32_t id = 0;
  uint32_t a = 0;
  uint32_t b = 0;
};

template <typename Context> struct IState {
  virtual ~IState() = default;
  virtual const char *Name() const = 0;
  virtual void OnEnter(Context &ctx, SmTick now) = 0;
  virtual void OnStep(Context &ctx, SmTick now) = 0;
  virtual void OnExit(Context &ctx, SmTick now) = 0;

  virtual void OnEvent(Context &ctx, const SmEvent &ev, SmTick now) {
    (void)ctx;
    (void)ev;
    (void)now;
  }
};

template <typename Context> class StateMachine {
public:
  explicit StateMachine(Context &ctx) : ctx_(ctx) {}

  void Start(IState<Context> &initial, SmTick now) {
    current_ = &initial;
    next_ = nullptr;
    current_->OnEnter(ctx_, now);
  }

  void Step(SmTick now) {
    // Apply any pending transition first
    if (next_) {
      IState<Context> *t = next_;
      next_ = nullptr; // clear BEFORE transition
      TransitionTo(*t, now);
    }

    if (current_)
      current_->OnStep(ctx_, now);

    // Apply transition requested during on_step
    if (next_) {
      IState<Context> *t = next_;
      next_ = nullptr;
      TransitionTo(*t, now);
    }
  }

  void PostEvent(const SmEvent &ev, SmTick now) {
    if (current_)
      current_->OnEvent(ctx_, ev, now);
  }

  void ReqTransition(IState<Context> &target) { next_ = &target; }

  const char *CurrentName() const {
    return current_ ? current_->Name() : "(none)";
  }

private:
  void TransitionTo(IState<Context> &target, SmTick now) {
    if (current_ == &target)
      return; // ignore self transition
    if (current_)
      current_->OnExit(ctx_, now);
    current_ = &target;
    current_->OnEnter(ctx_, now);
  }

  Context &ctx_;
  IState<Context> *current_ = nullptr;
  IState<Context> *next_ = nullptr;
};
