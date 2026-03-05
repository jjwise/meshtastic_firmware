#include "IrrigationMqttUplink.h"

namespace {
constexpr uint64_t k_wifi_retry_ms = 10000;
constexpr uint64_t k_mqtt_retry_ms = 5000;
}  // namespace

IrrigationMqttUplink::IrrigationMqttUplink()
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
    : mqtt_client_(secure_client_), last_wifi_attempt_ms_(0), last_mqtt_attempt_ms_(0) {}
#else
    : last_wifi_attempt_ms_(0), last_mqtt_attempt_ms_(0) {}
#endif

bool IrrigationMqttUplink::begin(const IrrigationConfig &config) {
  config_ = config;
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  WiFi.mode(WIFI_STA);
  secure_client_.setInsecure();
  if (config_.mqtt_host.isEmpty()) {
    return false;
  }
  mqtt_client_.setServer(config_.mqtt_host.c_str(), config_.mqtt_port);
  return true;
#else
  (void)config_;
  return false;
#endif
}

void IrrigationMqttUplink::tick(uint64_t now_ms) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (!ensure_wifi_connected(now_ms)) {
    return;
  }
  ensure_mqtt_connected(now_ms);
  mqtt_client_.loop();
#else
  (void)now_ms;
#endif
}

bool IrrigationMqttUplink::is_connected() {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  return mqtt_client_.connected();
#else
  return false;
#endif
}

bool IrrigationMqttUplink::publish_payload(const String &topic, const String &payload, uint8_t qos_level) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  (void)qos_level;
  const uint64_t now_ms = millis();
  if (!ensure_wifi_connected(now_ms) || !ensure_mqtt_connected(now_ms)) {
    return false;
  }
  return mqtt_client_.publish(topic.c_str(), payload.c_str(), false);
#else
  (void)topic;
  (void)payload;
  (void)qos_level;
  return false;
#endif
}

bool IrrigationMqttUplink::ensure_wifi_connected(uint64_t now_ms) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  if (now_ms - last_wifi_attempt_ms_ < k_wifi_retry_ms) {
    return false;
  }
  last_wifi_attempt_ms_ = now_ms;
  if (config_.wifi_ssid.isEmpty()) {
    return false;
  }
  WiFi.begin(config_.wifi_ssid.c_str(), config_.wifi_password.c_str());
  return false;
#else
  (void)now_ms;
  return false;
#endif
}

bool IrrigationMqttUplink::ensure_mqtt_connected(uint64_t now_ms) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (mqtt_client_.connected()) {
    return true;
  }
  if (now_ms - last_mqtt_attempt_ms_ < k_mqtt_retry_ms) {
    return false;
  }
  last_mqtt_attempt_ms_ = now_ms;
  return mqtt_client_.connect(
      config_.mqtt_client_id.c_str(), config_.mqtt_username.c_str(), config_.mqtt_password.c_str());
#else
  (void)now_ms;
  return false;
#endif
}
