#pragma once

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
  bool IsInitialized() const { return initialized_; }

 private:
  friend class System;
  void Init(const Config &cfg);
  Config cfg_{};
  bool wifi_on_ = false;
  esp_netif_t *ap_netif_ = nullptr;
  bool initialized_ = false;

  WifiController() = default;
  ~WifiController() = default;
  WifiController(const WifiController &) = delete;
  WifiController &operator=(const WifiController &) = delete;
};
