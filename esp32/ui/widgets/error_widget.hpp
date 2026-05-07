#pragma once

#include <cstdint>

#include "ui.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
}

class ErrorWidget : public IWidget {
 public:
  static ErrorWidget &GetInstance() {
    static ErrorWidget instance;
    return instance;
  }

  const char *Name() const override { return "error"; }

  void SetErrorCode(uint32_t code);
  void SetRecoverable(bool recoverable);
  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  uint32_t CurrentErrorCode() const;
  bool CurrentRecoverable() const;
  void Render(WidgetContext &ctx, TimeMs now);

  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  uint32_t error_code_ = 0;
  bool recoverable_ = false;

  ErrorWidget() = default;
};
