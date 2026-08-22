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

enum class IrrigationPublishReason {
  HEARTBEAT,
  HEARTBEAT_NO_FIX,
  POST_MOVE_FIX,
  SETTLE_CONFIRM,
  MOTION_DETECTED_NO_FIX
};

enum class IrrigationTimeQuality {
  UNKNOWN,
  RTC,
  GNSS
};

struct IrrigationTelemetryEvent {
  String msg_id;
  uint64_t ts_utc_ms;
  String tracker_id;
  String line_id;
  String endpoint_role;
  IrrigationPublishReason publish_reason;
  double lat;
  double lon;
  float hdop;
  float speed_mps;
  float heading_deg;
  bool heading_valid;
  IrrigationMotionState motion_state;
  uint16_t battery_mv;
  bool fix;
  bool stale;
  IrrigationTimeQuality time_quality;
  uint8_t hop_count;
};

struct IrrigationSensorSample {
  uint64_t ts_utc_ms;
  IrrigationTimeQuality time_quality;
  bool gps_fix;
  double lat;
  double lon;
  float hdop;
  float speed_mps;
  float heading_deg;
  bool heading_valid;
  float accel_rms_g;
  uint16_t battery_mv;
};
