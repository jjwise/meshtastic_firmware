#include "IrrigationPacketCodec.h"

namespace {
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
}  // namespace

String irrigation_motion_state_to_string(IrrigationMotionState motion_state) {
  switch (motion_state) {
    case IrrigationMotionState::MOVING:
      return "MOVING";
    case IrrigationMotionState::STATIONARY:
    default:
      return "STATIONARY";
  }
}

IrrigationMotionState irrigation_motion_state_from_string(const String &value) {
  if (value == "MOVING") {
    return IrrigationMotionState::MOVING;
  }
  return IrrigationMotionState::STATIONARY;
}

bool irrigation_event_to_json(const IrrigationTelemetryEvent &event, String &output_json) {
  output_json = "{";
  output_json += "\"msg_id\":" + quote(event.msg_id) + ",";
  output_json += "\"ts_utc_ms\":" + String(static_cast<unsigned long long>(event.ts_utc_ms)) + ",";
  output_json += "\"tracker_id\":" + quote(event.tracker_id) + ",";
  output_json += "\"line_id\":" + quote(event.line_id) + ",";
  output_json += "\"endpoint_role\":" + quote(event.endpoint_role) + ",";
  output_json += "\"lat\":" + String(event.lat, 7) + ",";
  output_json += "\"long\":" + String(event.lon, 7) + ",";
  output_json += "\"hdop\":" + String(event.hdop, 3) + ",";
  output_json += "\"speed_mps\":" + String(event.speed_mps, 3) + ",";
  output_json += "\"heading_deg\":" + String(event.heading_deg, 3) + ",";
  output_json += "\"motion_state\":" + quote(irrigation_motion_state_to_string(event.motion_state)) + ",";
  output_json += "\"battery_mv\":" + String(event.battery_mv) + ",";
  output_json += "\"fix\":";
  output_json += event.fix ? "true" : "false";
  output_json += ",";
  output_json += "\"hop_count\":" + String(event.hop_count);
  output_json += "}";
  return true;
}

bool irrigation_event_from_json(const String &payload_json, IrrigationTelemetryEvent &event) {
  String token;
  String text_value;

  if (!extract_quoted_string(payload_json, "msg_id", event.msg_id)) {
    return false;
  }
  if (extract_raw_token(payload_json, "ts_utc_ms", token)) {
    event.ts_utc_ms = strtoull(token.c_str(), nullptr, 10);
  } else {
    event.ts_utc_ms = 0;
  }
  extract_quoted_string(payload_json, "tracker_id", event.tracker_id);
  extract_quoted_string(payload_json, "line_id", event.line_id);
  extract_quoted_string(payload_json, "endpoint_role", event.endpoint_role);

  if (extract_raw_token(payload_json, "lat", token)) {
    event.lat = atof(token.c_str());
  } else {
    event.lat = 0.0;
  }

  if (extract_raw_token(payload_json, "long", token) || extract_raw_token(payload_json, "lon", token)) {
    event.lon = atof(token.c_str());
  } else {
    event.lon = 0.0;
  }

  if (extract_raw_token(payload_json, "hdop", token)) {
    event.hdop = atof(token.c_str());
  } else {
    event.hdop = 0.0f;
  }
  if (extract_raw_token(payload_json, "speed_mps", token)) {
    event.speed_mps = atof(token.c_str());
  } else {
    event.speed_mps = 0.0f;
  }
  if (extract_raw_token(payload_json, "heading_deg", token)) {
    event.heading_deg = atof(token.c_str());
  } else {
    event.heading_deg = 0.0f;
  }

  if (extract_quoted_string(payload_json, "motion_state", text_value)) {
    event.motion_state = irrigation_motion_state_from_string(text_value);
  } else {
    event.motion_state = IrrigationMotionState::STATIONARY;
  }

  if (extract_raw_token(payload_json, "battery_mv", token)) {
    event.battery_mv = static_cast<uint16_t>(atoi(token.c_str()));
  } else {
    event.battery_mv = 0;
  }

  if (extract_raw_token(payload_json, "fix", token)) {
    token.toLowerCase();
    event.fix = token == "true" || token == "1";
  } else {
    event.fix = false;
  }

  if (extract_raw_token(payload_json, "hop_count", token)) {
    event.hop_count = static_cast<uint8_t>(atoi(token.c_str()));
  } else {
    event.hop_count = 0;
  }

  return true;
}

String build_irrigation_mqtt_topic(const String &farm_id, const String &line_id, const String &tracker_id) {
  return "farm/" + farm_id + "/lines/" + line_id + "/trackers/" + tracker_id + "/telemetry";
}
