#pragma once

#include <Arduino.h>

enum class IrrigationNodeProfile {
  ENDPOINT_POD,
  BASIC_TRACKER,
  VALVE_ACTUATOR,
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
  MOTION_DETECTED_NO_FIX,
  PERIODIC_POSITION,
  VALVE_STATE,
  COMMAND_ACK,
  COMMAND_REJECTED
};

enum class IrrigationCommandAction {
  UNKNOWN,
  SET_POSITION_INTERVAL,
  SET_VALVE
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
  IrrigationNodeProfile node_profile;
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
  String command_id;
  bool has_valve_state;
  bool valve_open;
  bool has_position_interval;
  uint32_t position_interval_sec;
};

struct IrrigationCommand {
  String command_id;
  String tracker_id;
  IrrigationCommandAction action;
  uint64_t issued_at_utc_ms;
  uint64_t expires_at_utc_ms;
  bool valve_open;
  uint32_t duration_sec;
  uint32_t position_interval_sec;
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
