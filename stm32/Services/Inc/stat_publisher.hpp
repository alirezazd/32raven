#pragma once

#include <cstdint>

class Icm42688p;
struct AppContext;
struct BatteryData;

namespace message {
struct SystemStatusMsg;
struct VehicleStatusMsg;
}  // namespace message

class StatPublisher {
 public:
  static StatPublisher &GetInstance() {
    static StatPublisher instance;
    return instance;
  }

  void Publish(AppContext &ctx, uint32_t now_us, uint32_t loop_counter);

 private:
  StatPublisher() = default;
  ~StatPublisher() = default;
  StatPublisher(const StatPublisher &) = delete;
  StatPublisher &operator=(const StatPublisher &) = delete;

  static uint16_t BatteryVoltageMv(const BatteryData &battery);
  static int16_t BatteryCurrentCa(const BatteryData &battery);
  static int8_t BatteryRemainingPct(const BatteryData &battery);
  static message::SystemStatusMsg BuildSystemStatusMsg(AppContext &ctx,
                                                       uint32_t now_us,
                                                       uint32_t loop_counter,
                                                       const Icm42688p &imu);
  static message::VehicleStatusMsg BuildVehicleStatusMsg();
};
