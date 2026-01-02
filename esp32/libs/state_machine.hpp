#pragma once
#include <stdint.h>

using sm_tick_t = uint32_t;

struct SmEvent {
  uint32_t id = 0;
  uint32_t a = 0;
  uint32_t b = 0;
};

template <typename Context> struct IState {
  virtual ~IState() = default;
  virtual const char *name() const = 0;
  virtual void on_enter(Context &ctx, sm_tick_t now) = 0;
  virtual void on_step(Context &ctx, sm_tick_t now) = 0;
  virtual void on_exit(Context &ctx, sm_tick_t now) = 0;

  virtual void on_event(Context &ctx, const SmEvent &ev, sm_tick_t now) {
    (void)ctx;
    (void)ev;
    (void)now;
  }
};

template <typename Context> class StateMachine {
public:
  explicit StateMachine(Context &ctx) : ctx_(ctx) {}

  void start(IState<Context> &initial, sm_tick_t now) {
    current_ = &initial;
    next_ = nullptr;
    current_->on_enter(ctx_, now);
  }

  void step(sm_tick_t now) {
    // Apply any pending transition first
    if (next_) {
      IState<Context> *t = next_;
      next_ = nullptr; // clear BEFORE transition
      transition_to(*t, now);
    }

    if (current_)
      current_->on_step(ctx_, now);

    // Apply transition requested during on_step
    if (next_) {
      IState<Context> *t = next_;
      next_ = nullptr;
      transition_to(*t, now);
    }
  }

  void post_event(const SmEvent &ev, sm_tick_t now) {
    if (current_)
      current_->on_event(ctx_, ev, now);
  }

  void request_transition(IState<Context> &target) { next_ = &target; }

  const char *current_name() const {
    return current_ ? current_->name() : "(none)";
  }

private:
  void transition_to(IState<Context> &target, sm_tick_t now) {
    if (current_ == &target)
      return; // ignore self transition
    if (current_)
      current_->on_exit(ctx_, now);
    current_ = &target;
    current_->on_enter(ctx_, now);
  }

  Context &ctx_;
  IState<Context> *current_ = nullptr;
  IState<Context> *next_ = nullptr;
};
