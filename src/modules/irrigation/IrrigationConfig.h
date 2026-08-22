#pragma once

#include <Arduino.h>

#include "IrrigationTypes.h"

struct IrrigationConfig {
  String farm_id;
  String line_id;
  String tracker_id;
  String endpoint_role;
  IrrigationNodeProfile node_profile;

  uint32_t heartbeat_interval_sec;
  uint32_t stationary_interval_sec;
  uint32_t moving_interval_sec;
  uint32_t moving_hold_sec;
  uint32_t settle_confirm_delay_sec;
  uint32_t gnss_fix_timeout_sec;
  uint16_t motion_start_threshold_mg;
  uint8_t stable_fix_min_samples;
  float stable_fix_max_hdop;
  int32_t imu_interrupt_pin;

  uint8_t default_hop_limit;
  uint8_t max_hop_override;

  String wifi_ssid;
  String wifi_password;
  String mqtt_host;
  uint16_t mqtt_port;
  String mqtt_username;
  String mqtt_password;
  String mqtt_client_id;

  uint16_t gateway_buffer_max_records;
  bool enabled;
};

/**
 * Returns conservative defaults to bootstrap the module.
 *
 * Meshtastic integration should load the module-specific persisted config on
 * top of these defaults, then apply any build-time or runtime overrides.
 */
inline IrrigationConfig make_default_irrigation_config() {
  IrrigationConfig config;
  config.farm_id = "farm_demo";
  config.line_id = "line_demo_01";
  config.tracker_id = "tracker_demo";
  config.endpoint_role = "ENDPOINT_A";
  config.node_profile = IrrigationNodeProfile::ENDPOINT_POD;
  config.heartbeat_interval_sec = 12 * 60 * 60;
  config.stationary_interval_sec = 3600;
  config.moving_interval_sec = 10;
  config.moving_hold_sec = 120;
  config.settle_confirm_delay_sec = 5 * 60;
  config.gnss_fix_timeout_sec = 10 * 60;
  config.motion_start_threshold_mg = 200;
  config.stable_fix_min_samples = 3;
  config.stable_fix_max_hdop = 2.5f;
  config.imu_interrupt_pin = -1;
  config.default_hop_limit = 3;
  config.max_hop_override = 5;
  config.wifi_ssid = "";
  config.wifi_password = "";
  config.mqtt_host = "";
  config.mqtt_port = 8883;
  config.mqtt_username = "";
  config.mqtt_password = "";
  config.mqtt_client_id = "tracker_demo_gateway";
  config.gateway_buffer_max_records = 12000;
  config.enabled = true;
  return config;
}
