#pragma once

#include "IrrigationTypes.h"

class IrrigationMotionDetector {
 public:
  explicit IrrigationMotionDetector(uint32_t moving_hold_sec = 120, float motion_start_threshold_g = 0.2f);

  void update(float accel_rms_g, float gps_speed_mps, float heading_delta_deg, bool gps_fix, uint64_t now_ms);
  IrrigationMotionState motion_state() const;
  bool transitioned_to_moving() const;
  bool transitioned_to_stationary() const;

 private:
  uint32_t moving_hold_ms_;
  float motion_start_threshold_g_;
  IrrigationMotionState motion_state_;
  bool transitioned_to_moving_;
  bool transitioned_to_stationary_;
  uint64_t last_motion_ms_;
};
