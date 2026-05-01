#pragma once

#include <cstddef>
#include <cstdint>

#include "esp32_limits.hpp"
#include "i2c.hpp"

class Ssd1306Panel {
 public:
  struct Config {
    struct I2cDevice {
      uint16_t address = 0x3C;
      uint32_t scl_speed_hz = 400000;
      uint32_t scl_wait_us = 20000;
    };

    struct Output {
      bool invert = false;
      bool rotate_180 = false;
    };

    bool enabled = false;
    uint16_t settle_time_ms = 50;
    I2cDevice i2c{};
    Output output{};
  };

  static constexpr size_t kWidth = esp32_limits::kDisplayPanelWidth;
  static constexpr size_t kHeight = esp32_limits::kDisplayPanelHeight;
  static constexpr size_t kPageCount = kHeight / 8;
  static constexpr size_t kFramebufferSize = kWidth * kPageCount;

  static Ssd1306Panel &GetInstance() {
    static Ssd1306Panel instance;
    return instance;
  }

  void Init(const Config &cfg, I2cDisplay *i2c);
  void Flush(const uint8_t *framebuffer, size_t size);
  void FlushPageRange(uint8_t page, const uint8_t *page_data, size_t x_begin,
                      size_t count);
  void SetFadeOut(uint8_t interval);
  void DisableFadeOut();
  void DisplayOn();
  void DisplayOff();

 private:
  friend class System;

  static constexpr size_t kControllerWidth =
      esp32_limits::kDisplayPanelControllerWidth;
  static constexpr size_t kColumnOffset =
      esp32_limits::kDisplayPanelColumnOffset;

  void SendCommands(const uint8_t *commands, size_t count);
  void SetPageAddress(uint8_t page, uint8_t column);

  Config cfg_{};
  I2cDisplay *i2c_ = nullptr;
  i2c_master_dev_handle_t dev_ = nullptr;

  Ssd1306Panel() = default;
  ~Ssd1306Panel() = default;
  Ssd1306Panel(const Ssd1306Panel &) = delete;
  Ssd1306Panel &operator=(const Ssd1306Panel &) = delete;
};
