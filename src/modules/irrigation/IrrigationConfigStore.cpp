#include "IrrigationConfigStore.h"

#include "DebugConfiguration.h"

#include <stdlib.h>
#include <stdint.h>

#ifdef ARCH_ESP32
#include <LittleFS.h>
#endif

namespace {
String trim_copy(const String &value) {
  String output = value;
  output.trim();
  return output;
}

bool ensure_filesystem_ready() {
#ifdef ARCH_ESP32
  static bool initialized = false;
  static bool ready = false;
  if (!initialized) {
    ready = LittleFS.begin(true);
    initialized = true;
  }
  return ready;
#else
  return false;
#endif
}

String profile_to_string(IrrigationNodeProfile profile) {
  switch (profile) {
    case IrrigationNodeProfile::ENDPOINT_POD:
      return "ENDPOINT_POD";
    case IrrigationNodeProfile::RELAY_FIXED:
      return "RELAY_FIXED";
    case IrrigationNodeProfile::GATEWAY_CENTRAL:
      return "GATEWAY_CENTRAL";
    default:
      return "ENDPOINT_POD";
  }
}

bool parse_profile(const String &value, IrrigationNodeProfile &profile) {
  if (value.equalsIgnoreCase("ENDPOINT_POD")) {
    profile = IrrigationNodeProfile::ENDPOINT_POD;
    return true;
  }
  if (value.equalsIgnoreCase("RELAY_FIXED")) {
    profile = IrrigationNodeProfile::RELAY_FIXED;
    return true;
  }
  if (value.equalsIgnoreCase("GATEWAY_CENTRAL")) {
    profile = IrrigationNodeProfile::GATEWAY_CENTRAL;
    return true;
  }
  return false;
}

bool parse_bool(const String &value, bool &output) {
  if (value.equalsIgnoreCase("1") || value.equalsIgnoreCase("true") || value.equalsIgnoreCase("yes") ||
      value.equalsIgnoreCase("on")) {
    output = true;
    return true;
  }
  if (value.equalsIgnoreCase("0") || value.equalsIgnoreCase("false") || value.equalsIgnoreCase("no") ||
      value.equalsIgnoreCase("off")) {
    output = false;
    return true;
  }
  return false;
}

bool parse_u32(const String &value, uint32_t &output) {
  const String token = trim_copy(value);
  if (token.isEmpty()) {
    return false;
  }

  char *end_ptr = nullptr;
  const unsigned long parsed = strtoul(token.c_str(), &end_ptr, 10);
  if (end_ptr == token.c_str() || *end_ptr != '\0') {
    return false;
  }
  output = static_cast<uint32_t>(parsed);
  return true;
}

bool parse_u16(const String &value, uint16_t &output) {
  uint32_t parsed = 0;
  if (!parse_u32(value, parsed) || parsed > UINT16_MAX) {
    return false;
  }
  output = static_cast<uint16_t>(parsed);
  return true;
}

bool parse_u8(const String &value, uint8_t &output) {
  uint32_t parsed = 0;
  if (!parse_u32(value, parsed) || parsed > UINT8_MAX) {
    return false;
  }
  output = static_cast<uint8_t>(parsed);
  return true;
}

bool parse_i32(const String &value, int32_t &output) {
  const String token = trim_copy(value);
  if (token.isEmpty()) {
    return false;
  }

  char *end_ptr = nullptr;
  const long parsed = strtol(token.c_str(), &end_ptr, 10);
  if (end_ptr == token.c_str() || *end_ptr != '\0') {
    return false;
  }
  output = static_cast<int32_t>(parsed);
  return true;
}

bool parse_float(const String &value, float &output) {
  const String token = trim_copy(value);
  if (token.isEmpty()) {
    return false;
  }

  char *end_ptr = nullptr;
  const float parsed = strtof(token.c_str(), &end_ptr);
  if (end_ptr == token.c_str() || *end_ptr != '\0') {
    return false;
  }
  output = parsed;
  return true;
}

bool apply_config_value(IrrigationConfig &config, const String &key, const String &value) {
  if (key == "farm_id") {
    config.farm_id = value;
    return true;
  }
  if (key == "line_id") {
    config.line_id = value;
    return true;
  }
  if (key == "tracker_id") {
    config.tracker_id = value;
    return true;
  }
  if (key == "endpoint_role") {
    config.endpoint_role = value;
    return true;
  }
  if (key == "node_profile") {
    return parse_profile(value, config.node_profile);
  }
  if (key == "heartbeat_interval_sec") {
    return parse_u32(value, config.heartbeat_interval_sec);
  }
  if (key == "stationary_interval_sec") {
    return parse_u32(value, config.stationary_interval_sec);
  }
  if (key == "moving_interval_sec") {
    return parse_u32(value, config.moving_interval_sec);
  }
  if (key == "moving_hold_sec") {
    return parse_u32(value, config.moving_hold_sec);
  }
  if (key == "settle_confirm_delay_sec") {
    return parse_u32(value, config.settle_confirm_delay_sec);
  }
  if (key == "gnss_fix_timeout_sec") {
    return parse_u32(value, config.gnss_fix_timeout_sec);
  }
  if (key == "motion_start_threshold_mg") {
    return parse_u16(value, config.motion_start_threshold_mg);
  }
  if (key == "stable_fix_min_samples") {
    return parse_u8(value, config.stable_fix_min_samples);
  }
  if (key == "stable_fix_max_hdop") {
    return parse_float(value, config.stable_fix_max_hdop);
  }
  if (key == "imu_interrupt_pin") {
    return parse_i32(value, config.imu_interrupt_pin);
  }
  if (key == "default_hop_limit") {
    return parse_u8(value, config.default_hop_limit);
  }
  if (key == "max_hop_override") {
    return parse_u8(value, config.max_hop_override);
  }
  if (key == "wifi_ssid") {
    config.wifi_ssid = value;
    return true;
  }
  if (key == "wifi_password") {
    config.wifi_password = value;
    return true;
  }
  if (key == "mqtt_host") {
    config.mqtt_host = value;
    return true;
  }
  if (key == "mqtt_port") {
    return parse_u16(value, config.mqtt_port);
  }
  if (key == "mqtt_username") {
    config.mqtt_username = value;
    return true;
  }
  if (key == "mqtt_password") {
    config.mqtt_password = value;
    return true;
  }
  if (key == "mqtt_client_id") {
    config.mqtt_client_id = value;
    return true;
  }
  if (key == "gateway_buffer_max_records") {
    return parse_u16(value, config.gateway_buffer_max_records);
  }
  if (key == "enabled") {
    return parse_bool(value, config.enabled);
  }
  return false;
}

bool configs_equal(const IrrigationConfig &lhs, const IrrigationConfig &rhs) {
  return lhs.farm_id == rhs.farm_id && lhs.line_id == rhs.line_id && lhs.tracker_id == rhs.tracker_id &&
         lhs.endpoint_role == rhs.endpoint_role && lhs.node_profile == rhs.node_profile &&
         lhs.heartbeat_interval_sec == rhs.heartbeat_interval_sec &&
         lhs.stationary_interval_sec == rhs.stationary_interval_sec &&
         lhs.moving_interval_sec == rhs.moving_interval_sec && lhs.moving_hold_sec == rhs.moving_hold_sec &&
         lhs.settle_confirm_delay_sec == rhs.settle_confirm_delay_sec &&
         lhs.gnss_fix_timeout_sec == rhs.gnss_fix_timeout_sec &&
         lhs.motion_start_threshold_mg == rhs.motion_start_threshold_mg &&
         lhs.stable_fix_min_samples == rhs.stable_fix_min_samples &&
         lhs.stable_fix_max_hdop == rhs.stable_fix_max_hdop &&
         lhs.imu_interrupt_pin == rhs.imu_interrupt_pin &&
         lhs.default_hop_limit == rhs.default_hop_limit && lhs.max_hop_override == rhs.max_hop_override &&
         lhs.wifi_ssid == rhs.wifi_ssid && lhs.wifi_password == rhs.wifi_password &&
         lhs.mqtt_host == rhs.mqtt_host && lhs.mqtt_port == rhs.mqtt_port &&
         lhs.mqtt_username == rhs.mqtt_username && lhs.mqtt_password == rhs.mqtt_password &&
         lhs.mqtt_client_id == rhs.mqtt_client_id &&
         lhs.gateway_buffer_max_records == rhs.gateway_buffer_max_records && lhs.enabled == rhs.enabled;
}

#ifdef ARCH_ESP32
bool write_line(File &file, const char *key, const String &value) {
  return file.print(key) && file.print("=") && file.println(value);
}

bool write_line(File &file, const char *key, uint32_t value) {
  return write_line(file, key, String(value));
}

bool write_line(File &file, const char *key, bool value) {
  return write_line(file, key, value ? "true" : "false");
}
#endif
}  // namespace

IrrigationConfigStore::IrrigationConfigStore() : file_path_("/irrigation.cfg") {}

bool IrrigationConfigStore::begin(const String &file_path) {
  file_path_ = file_path.isEmpty() ? "/irrigation.cfg" : file_path;
  return ensure_filesystem_ready();
}

bool IrrigationConfigStore::load(IrrigationConfig &config) const {
#ifdef ARCH_ESP32
  if (!ensure_filesystem_ready() || !LittleFS.exists(file_path_)) {
    return false;
  }

  File config_file = LittleFS.open(file_path_, FILE_READ);
  if (!config_file) {
    return false;
  }

  while (config_file.available()) {
    String line = config_file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty() || line.startsWith("#")) {
      continue;
    }

    const int separator_index = line.indexOf('=');
    if (separator_index <= 0) {
      continue;
    }

    String key = line.substring(0, separator_index);
    key.trim();
    String value = line.substring(separator_index + 1);
    value.trim();
    apply_config_value(config, key, value);
  }

  config_file.close();
  return true;
#else
  (void)config;
  return false;
#endif
}

bool IrrigationConfigStore::save(const IrrigationConfig &config) const {
#ifdef ARCH_ESP32
  if (!ensure_filesystem_ready()) {
    return false;
  }

  if (LittleFS.exists(file_path_) && !LittleFS.remove(file_path_)) {
    return false;
  }

  File config_file = LittleFS.open(file_path_, FILE_WRITE);
  if (!config_file) {
    return false;
  }

  bool success = true;
  success = success && config_file.println("# irrigation module config");
  success = success && write_line(config_file, "farm_id", config.farm_id);
  success = success && write_line(config_file, "line_id", config.line_id);
  success = success && write_line(config_file, "tracker_id", config.tracker_id);
  success = success && write_line(config_file, "endpoint_role", config.endpoint_role);
  success = success && write_line(config_file, "node_profile", profile_to_string(config.node_profile));
  success = success && write_line(config_file, "heartbeat_interval_sec", config.heartbeat_interval_sec);
  success = success && write_line(config_file, "stationary_interval_sec", config.stationary_interval_sec);
  success = success && write_line(config_file, "moving_interval_sec", config.moving_interval_sec);
  success = success && write_line(config_file, "moving_hold_sec", config.moving_hold_sec);
  success = success && write_line(config_file, "settle_confirm_delay_sec", config.settle_confirm_delay_sec);
  success = success && write_line(config_file, "gnss_fix_timeout_sec", config.gnss_fix_timeout_sec);
  success = success && write_line(config_file, "motion_start_threshold_mg", static_cast<uint32_t>(config.motion_start_threshold_mg));
  success = success && write_line(config_file, "stable_fix_min_samples", static_cast<uint32_t>(config.stable_fix_min_samples));
  success = success && write_line(config_file, "stable_fix_max_hdop", String(config.stable_fix_max_hdop, 2));
  success = success && write_line(config_file, "imu_interrupt_pin", String(config.imu_interrupt_pin));
  success = success && write_line(config_file, "default_hop_limit", static_cast<uint32_t>(config.default_hop_limit));
  success = success && write_line(config_file, "max_hop_override", static_cast<uint32_t>(config.max_hop_override));
  success = success && write_line(config_file, "wifi_ssid", config.wifi_ssid);
  success = success && write_line(config_file, "wifi_password", config.wifi_password);
  success = success && write_line(config_file, "mqtt_host", config.mqtt_host);
  success = success && write_line(config_file, "mqtt_port", static_cast<uint32_t>(config.mqtt_port));
  success = success && write_line(config_file, "mqtt_username", config.mqtt_username);
  success = success && write_line(config_file, "mqtt_password", config.mqtt_password);
  success = success && write_line(config_file, "mqtt_client_id", config.mqtt_client_id);
  success = success &&
            write_line(config_file, "gateway_buffer_max_records", static_cast<uint32_t>(config.gateway_buffer_max_records));
  success = success && write_line(config_file, "enabled", config.enabled);
  config_file.close();
  return success;
#else
  (void)config;
  return false;
#endif
}

bool IrrigationConfigStore::save_if_changed(const IrrigationConfig &config) const {
  IrrigationConfig persisted = make_default_irrigation_config();
  if (load(persisted) && configs_equal(persisted, config)) {
    return true;
  }
  return save(config);
}

bool IrrigationConfigStore::exists() const {
#ifdef ARCH_ESP32
  return ensure_filesystem_ready() && LittleFS.exists(file_path_);
#else
  return false;
#endif
}

const String &IrrigationConfigStore::file_path() const { return file_path_; }
