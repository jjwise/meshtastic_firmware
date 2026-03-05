#pragma once

#include <Arduino.h>

#include "IrrigationTypes.h"

struct IrrigationConfig {
  String farm_id;
  String line_id;
  String tracker_id;
  String endpoint_role;
  IrrigationNodeProfile node_profile;

  uint32_t stationary_interval_sec;
  uint32_t moving_interval_sec;
  uint32_t moving_hold_sec;

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
 * In Meshtastic fork, bind this to device config persistence.
 */
inline IrrigationConfig make_default_irrigation_config() {
  IrrigationConfig config;
  config.farm_id = "farm_demo";
  config.line_id = "line_demo_01";
  config.tracker_id = "tracker_demo";
  config.endpoint_role = "ENDPOINT_A";
  config.node_profile = IrrigationNodeProfile::ENDPOINT_POD;
  config.stationary_interval_sec = 3600;
  config.moving_interval_sec = 10;
  config.moving_hold_sec = 120;
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

