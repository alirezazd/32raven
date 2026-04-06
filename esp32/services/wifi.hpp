#pragma once

#include <atomic>

#include "esp_event_base.h"
#include "esp_netif_types.h"
#include "esp_wifi_types.h"

class WifiController {
 public:
  static WifiController &GetInstance() {
    static WifiController instance;
    return instance;
  }
  struct Config {
    struct AccessPoint {
      const char *ssid = nullptr;
      const char *password = "";
      uint8_t channel = 1;
      uint8_t max_connections = 1;
      uint16_t beacon_interval_tu = 100;
      bool hidden = false;
    } ap;

    struct ProtectedManagementFrames {
      bool capable = true;
      bool required = false;
    } pmf;

    wifi_ps_type_t power_save = WIFI_PS_NONE;
  };

  bool StartAp();
  void Stop();

  bool IsOn() const { return wifi_on_; }
  bool HasAssociatedStations() const {
    return associated_station_count_.load() > 0;
  }
  const char *ApSsid() const {
    return cfg_.ap.ssid != nullptr ? cfg_.ap.ssid : "";
  }
  const char *ApPassword() const {
    return cfg_.ap.password != nullptr ? cfg_.ap.password : "";
  }

 private:
  friend class System;
  static void HandleWifiEvent(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data);
  void Init(const Config &cfg);
  void HandleApStaConnected();
  void HandleApStaDisconnected();
  Config cfg_{};
  bool wifi_on_ = false;
  bool event_handler_registered_ = false;
  esp_netif_t *ap_netif_ = nullptr;
  std::atomic<uint8_t> associated_station_count_{0};
  WifiController() = default;
  ~WifiController() = default;
  WifiController(const WifiController &) = delete;
  WifiController &operator=(const WifiController &) = delete;
};
