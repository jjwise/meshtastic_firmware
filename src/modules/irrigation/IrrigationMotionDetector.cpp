#include "IrrigationMotionDetector.h"

namespace {
constexpr float k_speed_threshold_mps = 0.40f;
constexpr float k_heading_threshold_deg = 12.0f;
}  // namespace

IrrigationMotionDetector::IrrigationMotionDetector(uint32_t moving_hold_sec, float motion_start_threshold_g)
    : moving_hold_ms_(moving_hold_sec * 1000UL),
      motion_start_threshold_g_(motion_start_threshold_g),
      motion_state_(IrrigationMotionState::STATIONARY),
      transitioned_to_moving_(false),
      transitioned_to_stationary_(false),
      last_motion_ms_(0) {}

void IrrigationMotionDetector::update(
    float accel_rms_g, float gps_speed_mps, float heading_delta_deg, bool gps_fix, uint64_t now_ms) {
  transitioned_to_moving_ = false;
  transitioned_to_stationary_ = false;

  const bool accel_motion = accel_rms_g >= motion_start_threshold_g_;
  const bool speed_motion = gps_fix && gps_speed_mps >= k_speed_threshold_mps;
  const bool heading_motion = gps_fix && gps_speed_mps > 0.2f && heading_delta_deg >= k_heading_threshold_deg;
  const bool motion_now = accel_motion || speed_motion || heading_motion;

  if (motion_now) {
    last_motion_ms_ = now_ms;
    if (motion_state_ != IrrigationMotionState::MOVING) {
      motion_state_ = IrrigationMotionState::MOVING;
      transitioned_to_moving_ = true;
    }
    return;
  }

  if (motion_state_ == IrrigationMotionState::MOVING && now_ms - last_motion_ms_ > moving_hold_ms_) {
    motion_state_ = IrrigationMotionState::STATIONARY;
    transitioned_to_stationary_ = true;
  }
}

IrrigationMotionState IrrigationMotionDetector::motion_state() const {
  return motion_state_;
}

bool IrrigationMotionDetector::transitioned_to_moving() const {
  return transitioned_to_moving_;
}

bool IrrigationMotionDetector::transitioned_to_stationary() const {
  return transitioned_to_stationary_;
}
