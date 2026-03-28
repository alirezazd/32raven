#pragma once

#include "esp_netif_types.h"

class WifiController {
public:
  static WifiController &GetInstance() {
    static WifiController instance;
    return instance;
  }
  struct Config {
    const char *ssid = "32Raven";
    const char *pass = "32Raven@1234";
    int channel = 1;
    int max_conn = 1;
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
