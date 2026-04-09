#pragma once

#include "ui.hpp"
#include "error_code.hpp"

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

  void SetErrorCode(ErrorCode code);
  void SetRecoverable(bool recoverable);
  void OnEnter(WidgetContext &ctx) override;
  void OnStep(WidgetContext &ctx, TimeMs now) override;

 private:
  ErrorCode CurrentErrorCode() const;
  bool CurrentRecoverable() const;
  void Render(WidgetContext &ctx, TimeMs now);

  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  ErrorCode error_code_ = ErrorCode::kUnknown;
  bool recoverable_ = false;

  ErrorWidget() = default;
};
