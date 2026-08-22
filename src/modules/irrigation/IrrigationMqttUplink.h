#pragma once

#include <Arduino.h>

#include "configuration.h"
#include "IrrigationConfig.h"

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#endif

class IrrigationMqttUplink {
 public:
  IrrigationMqttUplink();

  bool begin(const IrrigationConfig &config);
  void tick(uint64_t now_ms);
  bool is_connected();
  bool publish_payload(const String &topic, const String &payload, uint8_t qos_level);

 private:
  bool ensure_native_wifi_started();
  bool ensure_wifi_connected(uint64_t now_ms);
  bool ensure_mqtt_connected(uint64_t now_ms);
  bool probe_tcp_socket();

  IrrigationConfig config_;
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  WiFiClient plain_client_;
  WiFiClientSecure secure_client_;
  MqttClient plain_mqtt_client_;
  MqttClient secure_mqtt_client_;
  MqttClient *mqtt_client_;
#endif
  uint64_t last_wifi_attempt_ms_;
  uint64_t last_mqtt_attempt_ms_;
  bool wifi_reported_connected_;
  bool mqtt_reported_connected_;
  bool native_wifi_started_;
  bool mqtt_uses_tls_;
  int32_t last_wifi_status_;
};
