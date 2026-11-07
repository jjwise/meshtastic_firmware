#include "IrrigationPacketCodec.h"

#include <SHA256.h>

#include <cmath>
#include <cstdio>

namespace {
constexpr uint8_t k_lat_lon_precision = 5;

String quote(const String &value) {
  String output = "\"";
  output += value;
  output += "\"";
  return output;
}

int find_value_start(const String &payload_json, const String &key) {
  const String token = "\"" + key + "\"";
  const int key_pos = payload_json.indexOf(token);
  if (key_pos < 0) {
    return -1;
  }
  int colon_pos = payload_json.indexOf(':', key_pos + token.length());
  if (colon_pos < 0) {
    return -1;
  }
  colon_pos++;
  while (colon_pos < payload_json.length() && isspace(static_cast<unsigned char>(payload_json[colon_pos]))) {
    colon_pos++;
  }
  return colon_pos;
}

bool extract_quoted_string(const String &payload_json, const String &key, String &value) {
  const int start = find_value_start(payload_json, key);
  if (start < 0 || start >= payload_json.length() || payload_json[start] != '"') {
    return false;
  }
  const int end = payload_json.indexOf('"', start + 1);
  if (end < 0) {
    return false;
  }
  value = payload_json.substring(start + 1, end);
  return true;
}

bool extract_raw_token(const String &payload_json, const String &key, String &value) {
  int start = find_value_start(payload_json, key);
  if (start < 0) {
    return false;
  }
  int end = start;
  while (end < payload_json.length() && payload_json[end] != ',' && payload_json[end] != '}') {
    end++;
  }
  value = payload_json.substring(start, end);
  value.trim();
  return !value.isEmpty();
}

bool extract_quoted_string_any(const String &payload_json, const char *const *keys, size_t key_count, String &value) {
  for (size_t index = 0; index < key_count; ++index) {
    if (extract_quoted_string(payload_json, keys[index], value)) {
      return true;
    }
  }
  return false;
}

bool extract_raw_token_any(const String &payload_json, const char *const *keys, size_t key_count, String &value) {
  for (size_t index = 0; index < key_count; ++index) {
    if (extract_raw_token(payload_json, keys[index], value)) {
      return true;
    }
  }
  return false;
}

String format_decimal(double value, uint8_t precision) {
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%.*f", static_cast<int>(precision), value);
  return String(buffer);
}

String format_u64(uint64_t value) {
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%llu", static_cast<unsigned long long>(value));
  return String(buffer);
}
}  // namespace

String irrigation_motion_state_to_string(IrrigationMotionState motion_state) {
  switch (motion_state) {
    case IrrigationMotionState::MOVING:
      return "M";
    case IrrigationMotionState::STATIONARY:
    default:
      return "S";
  }
}

IrrigationMotionState irrigation_motion_state_from_string(const String &value) {
  if (value == "M" || value == "MOVING") {
    return IrrigationMotionState::MOVING;
  }
  return IrrigationMotionState::STATIONARY;
}

String irrigation_publish_reason_to_string(IrrigationPublishReason publish_reason) {
  switch (publish_reason) {
    case IrrigationPublishReason::HEARTBEAT:
      return "H";
    case IrrigationPublishReason::HEARTBEAT_NO_FIX:
      return "N";
    case IrrigationPublishReason::POST_MOVE_FIX:
      return "P";
    case IrrigationPublishReason::SETTLE_CONFIRM:
      return "C";
    case IrrigationPublishReason::MOTION_DETECTED_NO_FIX:
      return "X";
    case IrrigationPublishReason::PERIODIC_POSITION:
      return "T";
    case IrrigationPublishReason::VALVE_STATE:
      return "V";
    case IrrigationPublishReason::COMMAND_ACK:
      return "A";
    case IrrigationPublishReason::COMMAND_REJECTED:
      return "R";
    default:
      return "H";
  }
}

IrrigationPublishReason irrigation_publish_reason_from_string(const String &value) {
  if (value == "N" || value == "HEARTBEAT_NO_FIX") {
    return IrrigationPublishReason::HEARTBEAT_NO_FIX;
  }
  if (value == "P" || value == "POST_MOVE_FIX") {
    return IrrigationPublishReason::POST_MOVE_FIX;
  }
  if (value == "C" || value == "SETTLE_CONFIRM") {
    return IrrigationPublishReason::SETTLE_CONFIRM;
  }
  if (value == "X" || value == "MOTION_DETECTED_NO_FIX") {
    return IrrigationPublishReason::MOTION_DETECTED_NO_FIX;
  }
  if (value == "T" || value == "PERIODIC_POSITION") {
    return IrrigationPublishReason::PERIODIC_POSITION;
  }
  if (value == "V" || value == "VALVE_STATE") {
    return IrrigationPublishReason::VALVE_STATE;
  }
  if (value == "A" || value == "COMMAND_ACK") {
    return IrrigationPublishReason::COMMAND_ACK;
  }
  if (value == "R" || value == "COMMAND_REJECTED") {
    return IrrigationPublishReason::COMMAND_REJECTED;
  }
  return IrrigationPublishReason::HEARTBEAT;
}

String irrigation_node_profile_to_string(IrrigationNodeProfile profile) {
  switch (profile) {
    case IrrigationNodeProfile::BASIC_TRACKER:
      return "BASIC_TRACKER";
    case IrrigationNodeProfile::VALVE_ACTUATOR:
      return "VALVE_ACTUATOR";
    case IrrigationNodeProfile::RELAY_FIXED:
      return "RELAY_FIXED";
    case IrrigationNodeProfile::GATEWAY_CENTRAL:
      return "GATEWAY_CENTRAL";
    case IrrigationNodeProfile::ENDPOINT_POD:
    default:
      return "ENDPOINT_POD";
  }
}

IrrigationNodeProfile irrigation_node_profile_from_string(const String &value) {
  if (value == "BASIC_TRACKER") {
    return IrrigationNodeProfile::BASIC_TRACKER;
  }
  if (value == "VALVE_ACTUATOR") {
    return IrrigationNodeProfile::VALVE_ACTUATOR;
  }
  if (value == "RELAY_FIXED") {
    return IrrigationNodeProfile::RELAY_FIXED;
  }
  if (value == "GATEWAY_CENTRAL") {
    return IrrigationNodeProfile::GATEWAY_CENTRAL;
  }
  return IrrigationNodeProfile::ENDPOINT_POD;
}

String irrigation_time_quality_to_string(IrrigationTimeQuality time_quality) {
  switch (time_quality) {
    case IrrigationTimeQuality::GNSS:
      return "G";
    case IrrigationTimeQuality::RTC:
      return "R";
    default:
      return "U";
  }
}

IrrigationTimeQuality irrigation_time_quality_from_string(const String &value) {
  if (value == "G" || value == "GNSS") {
    return IrrigationTimeQuality::GNSS;
  }
  if (value == "R" || value == "RTC") {
    return IrrigationTimeQuality::RTC;
  }
  return IrrigationTimeQuality::UNKNOWN;
}

String irrigation_message_id(const String &tracker_id, uint64_t timestamp, uint64_t uptime,
                             uint32_t sequence, uint64_t boot_nonce) {
  const String source = tracker_id + "|" + format_u64(timestamp) + "|" + format_u64(uptime) +
                        "|" + String(sequence) + "|" + format_u64(boot_nonce);
  uint8_t digest[32];
  SHA256 hash;
  hash.reset();
  hash.update(source.c_str(), source.length());
  hash.finalize(digest, sizeof(digest));
  static const char hex[] = "0123456789abcdef";
  String identifier;
  identifier.reserve(32);
  for (size_t index = 0; index < 16; ++index) {
    identifier += hex[digest[index] >> 4];
    identifier += hex[digest[index] & 15];
  }
  return identifier;
}

bool irrigation_event_to_json(const IrrigationTelemetryEvent &event, String &output_json) {
  const bool command_result = event.publish_reason == IrrigationPublishReason::COMMAND_ACK ||
                              event.publish_reason == IrrigationPublishReason::COMMAND_REJECTED;
  output_json = "{";
  output_json += "\"i\":" + quote(event.msg_id) + ",";
  output_json += "\"t\":" + format_u64(event.ts_utc_ms) + ",";
  output_json += "\"r\":" + quote(event.tracker_id) + ",";
  output_json += "\"l\":" + quote(event.line_id) + ",";
  output_json += "\"e\":" + quote(event.endpoint_role) + ",";
  output_json += "\"n\":" + quote(irrigation_node_profile_to_string(event.node_profile)) + ",";
  output_json += "\"p\":" + quote(irrigation_publish_reason_to_string(event.publish_reason));
  if (command_result) {
    output_json += ",\"c\":" + quote(event.command_id);
  } else {
    if (!std::isfinite(event.lat) || !std::isfinite(event.lon) ||
        event.lat < -90 || event.lat > 90 || event.lon < -180 || event.lon > 180) {
      output_json = "";
      return false;
    }
    output_json += ",\"a\":" + format_decimal(event.lat, k_lat_lon_precision);
    output_json += ",\"o\":" + format_decimal(event.lon, k_lat_lon_precision);
    if (event.motion_state == IrrigationMotionState::MOVING) {
      output_json += ",\"m\":\"M\"";
    }
    if (event.fix) {
      output_json += ",\"f\":1";
    }
    if (event.stale) {
      output_json += ",\"s\":1";
    }
  }
  if (event.has_valve_state) {
    output_json += event.valve_open ? ",\"w\":1" : ",\"w\":0";
  }
  if (event.has_position_interval) {
    output_json += ",\"u\":" + String(event.position_interval_sec);
  }
  const auto append_optional = [&output_json](const String &fields) {
    if (output_json.length() + fields.length() + 1 <= k_irrigation_mesh_payload_max) {
      output_json += fields;
    }
  };
  if (event.battery_mv != 0) {
    append_optional(",\"b\":" + String(event.battery_mv));
  }
  if (event.time_quality != IrrigationTimeQuality::UNKNOWN) {
    append_optional(",\"q\":" + quote(irrigation_time_quality_to_string(event.time_quality)));
  }
  if (!command_result) {
    if (event.heading_valid && std::isfinite(event.heading_deg)) {
      append_optional(",\"d\":" + format_decimal(event.heading_deg, 1) + ",\"v\":1");
    }
    if (event.hop_count != 0) {
      append_optional(",\"h\":" + String(event.hop_count));
    }
  }
  output_json += "}";
  if (output_json.length() > k_irrigation_mesh_payload_max) {
    output_json = "";
    return false;
  }
  return true;
}

bool irrigation_event_from_json(const String &payload_json, IrrigationTelemetryEvent &event) {
  static const char *const k_msg_id_keys[] = {"i", "msg_id"};
  static const char *const k_ts_keys[] = {"t", "ts_utc_ms"};
  static const char *const k_tracker_keys[] = {"r", "tracker_id"};
  static const char *const k_line_keys[] = {"l", "line_id"};
  static const char *const k_endpoint_keys[] = {"e", "endpoint_role"};
  static const char *const k_profile_keys[] = {"n", "node_profile"};
  static const char *const k_publish_reason_keys[] = {"p", "publish_reason"};
  static const char *const k_lat_keys[] = {"a", "lat"};
  static const char *const k_lon_keys[] = {"o", "long", "lon"};
  static const char *const k_motion_keys[] = {"m", "motion_state"};
  static const char *const k_fix_keys[] = {"f", "fix"};
  static const char *const k_stale_keys[] = {"s", "stale"};
  static const char *const k_heading_keys[] = {"d", "heading_deg"};
  static const char *const k_heading_valid_keys[] = {"v", "heading_valid"};
  static const char *const k_battery_keys[] = {"b", "battery_mv"};
  static const char *const k_time_quality_keys[] = {"q", "time_quality"};
  static const char *const k_hop_keys[] = {"h", "hop_count"};
  static const char *const k_command_id_keys[] = {"c", "command_id"};
  static const char *const k_valve_state_keys[] = {"w", "valve_open"};
  static const char *const k_position_interval_keys[] = {"u", "position_interval_sec"};
  String token;
  String text_value;

  if (!extract_quoted_string_any(payload_json, k_msg_id_keys, 2, event.msg_id)) {
    return false;
  }
  if (extract_raw_token_any(payload_json, k_ts_keys, 2, token)) {
    event.ts_utc_ms = strtoull(token.c_str(), nullptr, 10);
  } else {
    event.ts_utc_ms = 0;
  }
  extract_quoted_string_any(payload_json, k_tracker_keys, 2, event.tracker_id);
  extract_quoted_string_any(payload_json, k_line_keys, 2, event.line_id);
  extract_quoted_string_any(payload_json, k_endpoint_keys, 2, event.endpoint_role);
  if (extract_quoted_string_any(payload_json, k_profile_keys, 2, text_value)) {
    event.node_profile = irrigation_node_profile_from_string(text_value);
  } else {
    event.node_profile = IrrigationNodeProfile::ENDPOINT_POD;
  }
  if (extract_quoted_string_any(payload_json, k_publish_reason_keys, 2, text_value)) {
    event.publish_reason = irrigation_publish_reason_from_string(text_value);
  } else {
    event.publish_reason = IrrigationPublishReason::HEARTBEAT;
  }

  if (extract_raw_token_any(payload_json, k_lat_keys, 2, token)) {
    event.lat = atof(token.c_str());
  } else {
    event.lat = 0.0;
  }

  if (extract_raw_token_any(payload_json, k_lon_keys, 3, token)) {
    event.lon = atof(token.c_str());
  } else {
    event.lon = 0.0;
  }

  event.hdop = 0.0f;
  event.speed_mps = 0.0f;
  if (extract_raw_token_any(payload_json, k_heading_keys, 2, token)) {
    event.heading_deg = atof(token.c_str());
  } else {
    event.heading_deg = 0.0f;
  }

  if (extract_quoted_string_any(payload_json, k_motion_keys, 2, text_value)) {
    event.motion_state = irrigation_motion_state_from_string(text_value);
  } else {
    event.motion_state = IrrigationMotionState::STATIONARY;
  }

  if (extract_raw_token_any(payload_json, k_battery_keys, 2, token)) {
    event.battery_mv = static_cast<uint16_t>(atoi(token.c_str()));
  } else {
    event.battery_mv = 0;
  }

  if (extract_raw_token_any(payload_json, k_fix_keys, 2, token)) {
    token.toLowerCase();
    event.fix = token == "true" || token == "1";
  } else {
    event.fix = false;
  }

  if (extract_raw_token_any(payload_json, k_stale_keys, 2, token)) {
    token.toLowerCase();
    event.stale = token == "true" || token == "1";
  } else {
    event.stale = false;
  }

  if (extract_raw_token_any(payload_json, k_heading_valid_keys, 2, token)) {
    token.toLowerCase();
    event.heading_valid = token == "true" || token == "1";
  } else {
    event.heading_valid = false;
  }

  if (extract_quoted_string_any(payload_json, k_time_quality_keys, 2, text_value)) {
    event.time_quality = irrigation_time_quality_from_string(text_value);
  } else {
    event.time_quality = IrrigationTimeQuality::UNKNOWN;
  }

  if (extract_raw_token_any(payload_json, k_hop_keys, 2, token)) {
    event.hop_count = static_cast<uint8_t>(atoi(token.c_str()));
  } else {
    event.hop_count = 0;
  }

  event.command_id = "";
  extract_quoted_string_any(payload_json, k_command_id_keys, 2, event.command_id);
  event.has_valve_state = extract_raw_token_any(payload_json, k_valve_state_keys, 2, token);
  if (event.has_valve_state) {
    token.toLowerCase();
    event.valve_open = token == "true" || token == "1";
  } else {
    event.valve_open = false;
  }
  event.has_position_interval = extract_raw_token_any(payload_json, k_position_interval_keys, 2, token);
  event.position_interval_sec = event.has_position_interval ? strtoul(token.c_str(), nullptr, 10) : 0;

  return true;
}

bool irrigation_command_from_json(const String &payload_json, IrrigationCommand &command) {
  static const char *const k_kind_keys[] = {"k", "kind"};
  static const char *const k_command_id_keys[] = {"i", "command_id"};
  static const char *const k_tracker_keys[] = {"r", "tracker_id"};
  static const char *const k_action_keys[] = {"a", "action"};
  static const char *const k_issued_keys[] = {"x", "issued_at_utc_ms"};
  static const char *const k_expires_keys[] = {"z", "expires_at_utc_ms"};
  static const char *const k_valve_open_keys[] = {"w", "valve_open"};
  static const char *const k_duration_keys[] = {"d", "duration_sec"};
  static const char *const k_interval_keys[] = {"u", "position_interval_sec"};
  static const char *const k_signature_keys[] = {"s", "signature"};
  String token;
  String value;

  if (!extract_quoted_string_any(payload_json, k_kind_keys, 2, value) ||
      !(value == "C" || value == "COMMAND")) {
    return false;
  }
  if (!extract_quoted_string_any(payload_json, k_command_id_keys, 2, command.command_id) ||
      !extract_quoted_string_any(payload_json, k_tracker_keys, 2, command.tracker_id) ||
      !extract_quoted_string_any(payload_json, k_action_keys, 2, value)) {
    return false;
  }

  if (value == "I" || value == "SET_POSITION_INTERVAL") {
    command.action = IrrigationCommandAction::SET_POSITION_INTERVAL;
  } else if (value == "V" || value == "SET_VALVE") {
    command.action = IrrigationCommandAction::SET_VALVE;
  } else {
    command.action = IrrigationCommandAction::UNKNOWN;
    return false;
  }

  command.issued_at_utc_ms = extract_raw_token_any(payload_json, k_issued_keys, 2, token)
                                 ? strtoull(token.c_str(), nullptr, 10)
                                 : 0;
  command.expires_at_utc_ms = extract_raw_token_any(payload_json, k_expires_keys, 2, token)
                                  ? strtoull(token.c_str(), nullptr, 10)
                                  : 0;
  command.valve_open = false;
  if (extract_raw_token_any(payload_json, k_valve_open_keys, 2, token)) {
    token.toLowerCase();
    command.valve_open = token == "true" || token == "1";
  }
  command.duration_sec = extract_raw_token_any(payload_json, k_duration_keys, 2, token)
                             ? strtoul(token.c_str(), nullptr, 10)
                             : 0;
  command.position_interval_sec = extract_raw_token_any(payload_json, k_interval_keys, 2, token)
                                       ? strtoul(token.c_str(), nullptr, 10)
                                       : 0;
  command.signature = "";
  extract_quoted_string_any(payload_json, k_signature_keys, 2, command.signature);
  return !command.command_id.isEmpty() && !command.tracker_id.isEmpty();
}

String build_irrigation_mqtt_topic(const String &farm_id, const String &line_id, const String &tracker_id) {
  return "farm/" + farm_id + "/lines/" + line_id + "/trackers/" + tracker_id + "/telemetry";
}

String build_irrigation_command_topic(const String &farm_id, const String &tracker_id) {
  return "farm/" + farm_id + "/nodes/" + tracker_id + "/commands";
}
