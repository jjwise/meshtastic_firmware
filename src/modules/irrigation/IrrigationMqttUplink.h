#pragma once

#include <Arduino.h>

#include "IrrigationConfig.h"

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
#include <PubSubClient.h>
#include <WiFi.h>
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
  bool ensure_wifi_connected(uint64_t now_ms);
  bool ensure_mqtt_connected(uint64_t now_ms);

  IrrigationConfig config_;
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  WiFiClientSecure secure_client_;
  PubSubClient mqtt_client_;
#endif
  uint64_t last_wifi_attempt_ms_;
  uint64_t last_mqtt_attempt_ms_;
};
