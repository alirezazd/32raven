#include "wifi.hpp"

extern "C" {
#include "driver/gpio.h" // IWYU pragma: keep
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
}
extern "C" {
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
}

#include <string.h>

static constexpr const char *kTag = "wifi";

static inline void LogErr(const char *what, esp_err_t e) {
  if (e != ESP_OK)
    ESP_LOGE(kTag, "%s: %s", what, esp_err_to_name(e));
}

void WifiController::Init(const Config &cfg, ErrorHandler error_handler) {
  if (initialized_)
    return;
  cfg_ = cfg;

  // 1. NVS
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    e = nvs_flash_erase();
    LogErr("nvs_flash_erase", e);
    e = nvs_flash_init();
  }
  LogErr("nvs_flash_init", e);
  if (e != ESP_OK) {
    if (error_handler)
      error_handler("NVS Init Failed");
    return;
  }

  // 2. Netif (LwIP)
  e = esp_netif_init();
  LogErr("esp_netif_init", e);
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
    if (error_handler)
      error_handler("Netif Init Failed");
    return;
  }

  // 3. Event Loop
  e = esp_event_loop_create_default();
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
    LogErr("esp_event_loop_create_default", e);
    if (error_handler)
      error_handler("Event Loop Create Failed");
    return;
  }

  // 4. WiFi Init
  wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
  e = esp_wifi_init(&wcfg);
  if (e == ESP_ERR_WIFI_INIT_STATE) {
    // already init
  } else if (e != ESP_OK) {
    LogErr("esp_wifi_init", e);
    if (error_handler)
      error_handler("Wifi Init Failed");
    return;
  }

  // 5. Storage RAM
  e = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  LogErr("esp_wifi_set_storage", e);
  // Storage failure might be non-critical, but let's be safe?
  // User asked for error handling. Let's assume critical.
  if (e != ESP_OK) {
    if (error_handler)
      error_handler("Wifi Set Storage Failed");
    return;
  }

  initialized_ = true;
  ESP_LOGI(kTag, "initialized");
}

bool WifiController::StartAp() {
  if (!initialized_) {
    ESP_LOGE(kTag, "StartAp called before init");
    return false;
  }
  if (wifi_on_)
    return true;

  // 1. Create AP Netif if needed
  if (!ap_netif_) {
    esp_netif_inherent_config_t inherent = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();
    esp_netif_config_t ncfg = ESP_NETIF_DEFAULT_WIFI_AP();
    ncfg.base = &inherent;
    ap_netif_ = esp_netif_new(&ncfg);
    if (!ap_netif_) {
      ESP_LOGE(kTag, "esp_netif_new(AP) failed");
      return false;
    }

    // Attach AP netif to Wi-Fi
    esp_err_t e = esp_netif_attach_wifi_ap(ap_netif_);
    LogErr("esp_netif_attach_wifi_ap", e);
    if (e != ESP_OK)
      return false;

    // Register default handlers
    e = esp_wifi_set_default_wifi_ap_handlers();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
      LogErr("esp_wifi_set_default_wifi_ap_handlers", e);
      return false;
    }
  }

  // 2. Set Mode
  esp_err_t e = esp_wifi_set_mode(WIFI_MODE_AP);
  LogErr("esp_wifi_set_mode", e);
  if (e != ESP_OK)
    return false;

  // 3. Config
  wifi_config_t ap{};
  strlcpy((char *)ap.ap.ssid, cfg_.ssid, sizeof(ap.ap.ssid));
  strlcpy((char *)ap.ap.password, cfg_.pass, sizeof(ap.ap.password));
  ap.ap.ssid_len = strlen(cfg_.ssid);
  ap.ap.channel = cfg_.channel;
  ap.ap.max_connection = cfg_.max_conn;
  ap.ap.beacon_interval = 100;
  ap.ap.authmode =
      (cfg_.pass && cfg_.pass[0] != '\0') ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
  ap.ap.pmf_cfg.capable = true;
  ap.ap.pmf_cfg.required = false;

  e = esp_wifi_set_config(WIFI_IF_AP, &ap);
  LogErr("esp_wifi_set_config", e);
  if (e != ESP_OK)
    return false;

  // 4. Start
  e = esp_wifi_start();
  LogErr("esp_wifi_start", e);
  if (e != ESP_OK)
    return false;

  wifi_on_ = true;
  ESP_LOGI(kTag, "AP started SSID=%s", cfg_.ssid);

  // Disable power save to prevent "Assoc Expired" (reason=4) loops
  esp_wifi_set_ps(WIFI_PS_NONE);

  return true;
}

void WifiController::Stop() {
  if (!wifi_on_)
    return;

  esp_err_t e = esp_wifi_stop();
  if (e != ESP_OK)
    ESP_LOGW(kTag, "esp_wifi_stop: %s", esp_err_to_name(e));

  // No need to deinit, we usually keep driver alive
  wifi_on_ = false;
  ESP_LOGI(kTag, "stopped");
}
