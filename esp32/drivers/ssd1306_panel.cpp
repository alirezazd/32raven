#include "ssd1306_panel.hpp"

#include <cstring>

#include "panic.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {

constexpr const char *kTag = "ssd1306";
constexpr uint8_t kControlCommand = 0x00;
constexpr uint8_t kControlData = 0x40;
constexpr TickType_t kProbeRetryPeriod = pdMS_TO_TICKS(10);
constexpr size_t kProbeRetryCount = 20;

}  // namespace

void Ssd1306Panel::Init(const Config &cfg, I2cDisplay *i2c) {
  cfg_ = cfg;
  if (!cfg_.enabled) {
    ESP_LOGI(kTag, "disabled");
    return;
  }
  if (i2c == nullptr || cfg_.i2c.address > 0x7F || cfg_.i2c.scl_speed_hz == 0) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  i2c_ = i2c;
  if (cfg_.settle_time_ms > 0) {
    vTaskDelay(pdMS_TO_TICKS(cfg_.settle_time_ms));
  }

  bool found = false;
  for (size_t attempt = 0; attempt < kProbeRetryCount; ++attempt) {
    if (i2c_->Probe(cfg_.i2c.address)) {
      found = true;
      break;
    }
    vTaskDelay(kProbeRetryPeriod);
  }
  if (!found) {
    ESP_LOGE(kTag, "display probe failed addr=0x%02X", cfg_.i2c.address);
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  dev_ = i2c_->AddDevice(I2cDeviceConfig{
      .address = cfg_.i2c.address,
      .timing =
          {
              .clock_hz = cfg_.i2c.scl_speed_hz,
              .scl_wait_us = cfg_.i2c.scl_wait_us,
          },
  });

  const uint8_t segment_remap = cfg_.output.rotate_180 ? 0xA0 : 0xA1;
  const uint8_t com_scan_direction = cfg_.output.rotate_180 ? 0xC0 : 0xC8;
  const uint8_t display_mode = cfg_.output.invert ? 0xA7 : 0xA6;
  const uint8_t init_cmds[] = {
      0xAE,
      0xD5,
      0xF0,
      0xA8,
      0x27,
      0xD3,
      0x00,
      0x40,
      0x8D,
      0x14,
      0x20,
      0x02,
      segment_remap,
      com_scan_direction,
      0xDA,
      0x12,
      0xAD,
      0x30,
      0x81,
      0x2F,
      0xD9,
      0x22,
      0xDB,
      0x20,
      display_mode,
      0xA4,
      0xAF,
  };
  SendCommands(init_cmds, sizeof(init_cmds));

  ESP_LOGI(kTag, "initialized addr=0x%02X invert=%u rotate180=%u",
           cfg_.i2c.address, static_cast<unsigned>(cfg_.output.invert),
           static_cast<unsigned>(cfg_.output.rotate_180));
}

void Ssd1306Panel::SendCommands(const uint8_t *commands, size_t count) {
  if (dev_ == nullptr || i2c_ == nullptr) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }
  if (commands == nullptr || count == 0 || count > 31) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  uint8_t buffer[32]{};
  buffer[0] = kControlCommand;
  std::memcpy(buffer + 1, commands, count);
  i2c_->Transmit(dev_, buffer, count + 1);
}

void Ssd1306Panel::SetPageAddress(uint8_t page, uint8_t column) {
  if (page >= kPageCount || column >= kControllerWidth) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  const uint8_t commands[3] = {
      static_cast<uint8_t>(0xB0u | (page & 0x0Fu)),
      static_cast<uint8_t>(column & 0x0Fu),
      static_cast<uint8_t>(0x10u | ((column >> 4) & 0x0Fu)),
  };
  SendCommands(commands, sizeof(commands));
}

void Ssd1306Panel::Flush(const uint8_t *framebuffer, size_t size) {
  if (dev_ == nullptr || i2c_ == nullptr) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }
  if (framebuffer == nullptr || size != kFramebufferSize) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  uint8_t buffer[1 + kControllerWidth]{};
  buffer[0] = kControlData;

  for (uint8_t page = 0; page < kPageCount; ++page) {
    SetPageAddress(page, 0);
    std::memset(buffer + 1, 0x00, kControllerWidth);
    std::memcpy(buffer + 1 + kColumnOffset, framebuffer + (page * kWidth),
                kWidth);
    i2c_->Transmit(dev_, buffer, sizeof(buffer));
  }
}

void Ssd1306Panel::FlushPageRange(uint8_t page, const uint8_t *page_data,
                                  size_t x_begin, size_t count) {
  if (dev_ == nullptr || i2c_ == nullptr) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }
  if (page >= kPageCount || page_data == nullptr || count == 0 ||
      x_begin >= kWidth || (x_begin + count) > kWidth) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  const size_t start_column = kColumnOffset + x_begin;
  if ((start_column + count) > kControllerWidth) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  uint8_t buffer[1 + kWidth]{};
  buffer[0] = kControlData;
  std::memcpy(buffer + 1, page_data + x_begin, count);
  SetPageAddress(page, static_cast<uint8_t>(start_column));
  i2c_->Transmit(dev_, buffer, count + 1);
}

void Ssd1306Panel::SetFadeOut(uint8_t interval) {
  if (dev_ == nullptr || i2c_ == nullptr) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  const uint8_t commands[2] = {
      0x23,
      static_cast<uint8_t>(0x20u | (interval & 0x0Fu)),
  };
  SendCommands(commands, sizeof(commands));
}

void Ssd1306Panel::DisableFadeOut() {
  if (dev_ == nullptr || i2c_ == nullptr) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  const uint8_t commands[2] = {
      0x23,
      0x00,
  };
  SendCommands(commands, sizeof(commands));
}

void Ssd1306Panel::DisplayOn() {
  if (dev_ == nullptr || i2c_ == nullptr) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  const uint8_t commands[1] = {0xAF};
  SendCommands(commands, sizeof(commands));
}

void Ssd1306Panel::DisplayOff() {
  if (dev_ == nullptr || i2c_ == nullptr) {
    Panic(ErrorCode::kDisplayPanelInitFailed);
  }

  const uint8_t commands[1] = {0xAE};
  SendCommands(commands, sizeof(commands));
}
