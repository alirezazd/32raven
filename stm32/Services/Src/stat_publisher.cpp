#include "stat_publisher.hpp"

#include <cstdint>

#include "ctx.hpp"
#include "icm42688p.hpp"
#include "message.hpp"
#include "system.hpp"
#include "vehicle_state.hpp"

uint16_t StatPublisher::BatteryVoltageMv(const BatteryData &battery) {
  if (battery.voltage <= 0.0f) {
    return 0;
  }

  const uint32_t voltage_mv =
      static_cast<uint32_t>(battery.voltage * 1000.0f + 0.5f);
  return voltage_mv > UINT16_MAX ? UINT16_MAX
                                 : static_cast<uint16_t>(voltage_mv);
}

int16_t StatPublisher::BatteryCurrentCa(const BatteryData &battery) {
  if (battery.voltage <= 0.0f) {
    return -1;
  }

  const int32_t current_ca = static_cast<int32_t>(battery.current * 100.0f);
  if (current_ca > INT16_MAX) {
    return INT16_MAX;
  }
  if (current_ca < INT16_MIN) {
    return INT16_MIN;
  }
  return static_cast<int16_t>(current_ca);
}

int8_t StatPublisher::BatteryRemainingPct(const BatteryData &battery) {
  if (battery.voltage <= 0.0f) {
    return -1;
  }
  return battery.percentage > 100u ? 100
                                   : static_cast<int8_t>(battery.percentage);
}

message::SystemStatusMsg StatPublisher::BuildSystemStatusMsg(
    AppContext &ctx, uint32_t now_us, uint32_t loop_counter,
    const Icm42688p &imu) {
  const VehicleState &vehicle_state = ctx.sys->GetVehicleState();
  const GpsData &gps = vehicle_state.GetGps();
  const BatteryData &battery = vehicle_state.GetBattery();
  const RcData &rc = vehicle_state.GetRc();

  uint32_t sensors_present = 0;
  uint32_t sensors_health = 0;

  if (imu.IsInitialized()) {
    sensors_present |= message::kSystemSensorFlagImu;
    if (imu.ImuPathOverrun() == 0u && imu.DmaStartFailCount() == 0u &&
        imu.ParseFailCount() == 0u) {
      sensors_health |= message::kSystemSensorFlagImu;
    }
  }

  if (gps.timestamp_us != 0u) {
    sensors_present |= message::kSystemSensorFlagGps;
    if (gps.fix_type >= 2u) {
      sensors_health |= message::kSystemSensorFlagGps;
    }
  }

  if (battery.voltage > 0.0f) {
    sensors_present |= message::kSystemSensorFlagBattery;
    sensors_health |= message::kSystemSensorFlagBattery;
  }

  if (rc.timestamp_us != 0u) {
    sensors_present |= message::kSystemSensorFlagRcReceiver;
    if (rc.rx_online) {
      sensors_health |= message::kSystemSensorFlagRcReceiver;
    }
  }

  message::SystemStatusMsg msg{};
  msg.uptime_ms = now_us / 1000u;
  msg.loop_counter = loop_counter;
  msg.error_code = ErrorCode::kOk;
  msg.sensor_present_flags = sensors_present;
  msg.sensor_health_flags = sensors_health;
  msg.batt_voltage = BatteryVoltageMv(battery);
  msg.batt_current = BatteryCurrentCa(battery);
  msg.batt_remaining = BatteryRemainingPct(battery);
  msg.boot_state = message::kSystemBootStateReady;
  msg.flags = message::kSystemStatusFlagLoopAlive;
  return msg;
}

message::VehicleStatusMsg StatPublisher::BuildVehicleStatusMsg() {
  message::VehicleStatusMsg msg{};
  msg.armed_state = message::kVehicleArmedStateDisarmed;
  msg.failsafe_flags = 0u;
  return msg;
}

void StatPublisher::Publish(AppContext &ctx, uint32_t now_us,
                            uint32_t loop_counter) {
  const Icm42688p &imu = ctx.sys->GetImu42688p();
  ctx.sys->GetFcLink().SendSystemStatus(
      BuildSystemStatusMsg(ctx, now_us, loop_counter, imu));
  ctx.sys->GetFcLink().SendVehicleStatus(BuildVehicleStatusMsg());
}
