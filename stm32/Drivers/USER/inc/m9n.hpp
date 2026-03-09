#pragma once

#include "error_code.hpp"
#include <cstdint>

class M9N {
public:
  enum class BaudRate : uint32_t {
    k9600 = 9600,
    k19200 = 19200,
    k38400 = 38400,
    k57600 = 57600,
    k115200 = 115200,
    k230400 = 230400,
    k460800 = 460800,
    k921600 = 921600,
  };

  enum class DynamicModel : uint8_t {
    kPortable = 0,
    kStationary = 2,
    kPedestrian = 3,
    kAutomotive = 4,
    kSea = 5,
    kAirborne1g = 6,
    kAirborne2g = 7,
    kAirborne4g = 8,
    kWrist = 9,
    kBike = 10,
  };

  static constexpr uint32_t ToBaudRateValue(BaudRate baud_rate) {
    return static_cast<uint32_t>(baud_rate);
  }

  static M9N &GetInstance() {
    static M9N instance;
    return instance;
  }

  struct Config {
    bool flash_config;
    BaudRate baud_rate;

    struct Protocols {
      bool outprot_ubx;
      bool outprot_nmea;
    } protocols;

    struct Messages {
      bool nav_pvt;
      bool nav_dop;
      bool nav_cov;
      bool nav_eoe;
    } messages;

    struct Navigation {
      uint16_t rate_meas_ms;
      DynamicModel dyn_model;
    } nav;

    struct Constellations {
      bool gps_enable;
      bool glo_enable;
      bool gal_enable;
      bool bds_enable;
      bool sbas_enable;
      bool itfm_enable;
    } gnss;

    struct Timepulse {
      bool ena;
      uint32_t period;
      uint32_t len;
      uint8_t timegrid;
      bool sync_gnss;
      bool use_locked;
      bool align_to_tow;
      bool pol_rising;
      uint32_t period_lock;
      uint32_t len_lock;
    } tp1;

    uint32_t ack_timeout_us;
  };

  bool Read(uint8_t &b);

private:
  friend class System;

  void Init(const Config &config);
  void FlashConfig(BaudRate current_baud = BaudRate::k38400);

  void ApplyConfig(uint8_t layer);

  template <typename T>
  void SendCfgValSetRaw(uint32_t key, T value, uint8_t layer);

  template <typename T>
  bool SendCfgValSet(uint32_t key, T value, uint8_t layer);

  void SendCfgValGet(uint32_t key, uint8_t layer);

  template <typename T> bool WaitForValget(uint32_t key, T expected_value);

  void WaitForReady();
  bool WaitForAck(uint8_t want_cls, uint8_t want_id);

  Config config_{};

  M9N() = default;
  ~M9N() = default;
  M9N(const M9N &) = delete;
  M9N &operator=(const M9N &) = delete;
};
