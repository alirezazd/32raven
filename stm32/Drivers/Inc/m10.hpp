// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include <cstdint>

class M10 {
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

  enum class TimeGrid : uint8_t {
    kUtc = 0,
    kGps = 1,
    kGlonass = 2,
    kBeiDou = 3,
    kGalileo = 4,
  };

  enum class UartStopBits : uint8_t {
    kHalf = 0,
    k1 = 1,
    k1Point5 = 2,
    k2 = 3,
  };

  enum class UartDataBits : uint8_t {
    k8 = 0,
    k7 = 1,
  };

  enum class UartParity : uint8_t {
    kNone = 0,
    kOdd = 1,
    kEven = 2,
  };

  static constexpr uint32_t ToBaudRateValue(BaudRate baud_rate) {
    return static_cast<uint32_t>(baud_rate);
  }

  static M10 &GetInstance() {
    static M10 instance;
    return instance;
  }

  struct Config {
    BaudRate baud_rate;

    struct Uart1 {
      bool enabled;
      UartStopBits stop_bits;
      UartDataBits data_bits;
      UartParity parity;
    } uart1;

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
      TimeGrid timegrid;
      bool sync_gnss;
      bool align_to_tow;
      bool pol_rising;
    } tp1;

    uint32_t ack_timeout_us;
  };

  bool Read(uint8_t &b);

 private:
  friend class System;

  // M10 config is applied to RAM on each boot; this module variant has no
  // flash. VALSET layers are a bitmask while VALGET layers are an enumeration,
  // so 0x01 means RAM in one space and BBR in the other — separate types keep
  // the two from being passed to the wrong call.
  enum class ValsetLayer : uint8_t { kRam = 0x01 };
  enum class ValgetLayer : uint8_t {
    kRam = 0,
    kBbr = 1,
    kFlash = 2,
    kDefault = 7,
  };

  void Init(const Config &config);
  void ApplyConfig(ValsetLayer layer);

  template <typename T>
  void SendCfgValSetRaw(uint32_t key, T value, ValsetLayer layer);

  template <typename T>
  bool SendCfgValSet(uint32_t key, T value, ValsetLayer layer);

  void SendCfgValGet(uint32_t key, ValgetLayer layer);

  template <typename T>
  bool WaitForValget(uint32_t key, T expected_value);

  void WaitForReady();
  bool WaitForAck(uint8_t want_cls, uint8_t want_id);

  Config config_{};

  M10() = default;
  ~M10() = default;
  M10(const M10 &) = delete;
  M10 &operator=(const M10 &) = delete;
};
