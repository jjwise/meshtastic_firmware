#pragma once

#include <Arduino.h>

enum class IrrigationNodeProfile {
  ENDPOINT_POD,
  RELAY_FIXED,
  GATEWAY_CENTRAL
};

enum class IrrigationMotionState {
  STATIONARY,
  MOVING
};

struct IrrigationTelemetryEvent {
  String msg_id;
  uint64_t ts_utc_ms;
  String tracker_id;
  String line_id;
  String endpoint_role;
  double lat;
  double lon;
  float hdop;
  float speed_mps;
  float heading_deg;
  IrrigationMotionState motion_state;
  uint16_t battery_mv;
  bool fix;
  uint8_t hop_count;
};

struct IrrigationSensorSample {
  bool gps_fix;
  double lat;
  double lon;
  float hdop;
  float speed_mps;
  float heading_deg;
  float accel_rms_g;
  uint16_t battery_mv;
};

