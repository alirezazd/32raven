// Per-fast-tick IMU burst aggregator + attitude integrator.
//
// Today (Phase A): walks the 8-sample IMU burst, averages the body-rate
// vector for the rate controller's measurement input, and integrates
// the gyro stream into a unit quaternion (first-order Euler with
// per-sample sub-stepping at ≈125 µs). The 8 samples are not dropped;
// every sample contributes one integration sub-step.
//
// Phase C: the same Process() entry point will be the MEKF predict +
// update calls. VehicleState still consumes the same ImuState struct.
// Only the implementation of Process() changes — none of the callers,
// none of the rest of the flight stack, do.
//
// Owned by System like Mixer / EscService. The state machine treats
// this as pure orchestration:
//   const ImuState imu = ctx.sys->GetAttitudeEstimator().Process(batch);
//   ctx.sys->GetVehicleState().UpdateImu(imu);

#pragma once

#include "icm42688p.hpp"
#include "vehicle_state.hpp"

namespace attitude_estimator {

class AttitudeEstimator {
 public:
  // No tunables today. The struct exists so Phase C can add MEKF
  // process-noise / measurement-noise gains here without churning
  // the constructor signature.
  struct Config {};

  AttitudeEstimator() = default;

  // Init. Panics on invalid config; called once by System::InitComponent.
  void Init(const Config &cfg);

  // Reset the integrated attitude to identity and zero out the
  // averaged gyro. Called on arm transitions and from tests.
  void Reset();

  // Process one IMU sample burst. Reads the IMU via the singleton
  // (`Icm42688p::GetInstance()`) for scaling — accepts raw samples
  // rather than scaled ones so the call site stays cheap.
  //
  // Idempotent on an empty batch: returns the last published state
  // unchanged.
  ImuState Process(const Icm42688p::SampleBatch &batch);

  // Read-only access to the latest published state. Same data
  // VehicleState got after the most recent Process() call.
  const ImuState &Current() const { return state_; }

 private:
  Config cfg_{};
  ImuState state_{};
};

}  // namespace attitude_estimator
