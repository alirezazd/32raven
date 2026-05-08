#include "usb_cdc_server.hpp"

#include "error_code.hpp"
#include "panic.hpp"

extern "C" {
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

// USB Full-Speed delivers MAVLink frames in <1 ms when a host is reading;
// 50 ms is a generous upper bound that lets a momentarily slow host catch
// up without stalling the MAVLink TX scheduler.
static constexpr TickType_t kSendTimeoutTicks = pdMS_TO_TICKS(50);

void UsbCdcServer::Init(const Config &cfg) {
  cfg_ = cfg;

  usb_serial_jtag_driver_config_t drv_cfg = {
      .tx_buffer_size = cfg_.tx_buffer_bytes,
      .rx_buffer_size = cfg_.rx_buffer_bytes,
  };
  const esp_err_t err = usb_serial_jtag_driver_install(&drv_cfg);
  // ESP-IDF console (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y) installs the
  // driver during boot; a second install returns ESP_ERR_INVALID_STATE.
  // That's fine — we just share the already-running driver. Read/write
  // calls work either way. The cost is ESP_LOG output interleaving with the
  // MAVLink stream; pymavlink resyncs on the magic byte (0xFD).
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Panic(ErrorCode::Esp32::kUsbCdcStartFailed);
  }
  driver_installed_ = true;
}

int UsbCdcServer::Receive(uint8_t *dst, size_t max_len) {
  if (!driver_installed_ || dst == nullptr || max_len == 0) {
    return 0;
  }
  // ticks_to_wait=0 so the call is non-blocking and matches UdpServer's
  // semantics. Returns the number of bytes actually read.
  return usb_serial_jtag_read_bytes(dst, max_len, /*ticks_to_wait=*/0);
}

int UsbCdcServer::Send(const uint8_t *data, size_t len) {
  if (!driver_installed_ || data == nullptr || len == 0) {
    return 0;
  }
  return usb_serial_jtag_write_bytes(data, len, kSendTimeoutTicks);
}

bool UsbCdcServer::IsReady() const {
  // ESP-IDF exposes usb_serial_jtag_is_connected() in 5.x. Treat the driver
  // being installed as readiness — sends queue into the TX FIFO and drain
  // when a host attaches; we don't gate on enumeration state.
  return driver_installed_;
}

void UsbCdcServer::ClearPeer() {
  // No peer state to clear — USB CDC is point-to-point with the host.
}
