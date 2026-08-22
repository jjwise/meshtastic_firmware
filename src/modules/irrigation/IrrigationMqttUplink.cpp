#include "IrrigationMqttUplink.h"

#include "DebugConfiguration.h"
#include "NodeDB.h"
#include "modules/AdminModule.h"
#include "mesh/wifi/WiFiAPClient.h"
#include "sleep.h"
#include "IrrigationSecrets.h"

#include <cstring>

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
#include <esp_wifi.h>
#endif

namespace {
constexpr uint64_t k_wifi_retry_ms = 10000;
constexpr uint64_t k_mqtt_retry_ms = 5000;

bool gateway_prefers_wifi(const IrrigationConfig &config) {
  return config.node_profile == IrrigationNodeProfile::GATEWAY_CENTRAL && !config.wifi_ssid.isEmpty();
}

const char *wifi_status_name(int32_t wifi_status) {
  switch (wifi_status) {
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID";
    case WL_SCAN_COMPLETED:
      return "SCAN_COMPLETED";
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    case WL_NO_SHIELD:
      return "NO_SHIELD";
    default:
      return "UNKNOWN";
  }
}

bool wifi_status_has_disconnect_reason(int32_t wifi_status) {
  return wifi_status == WL_NO_SSID_AVAIL || wifi_status == WL_CONNECT_FAILED ||
         wifi_status == WL_CONNECTION_LOST || wifi_status == WL_DISCONNECTED;
}

void enforce_gateway_radio_policy(const IrrigationConfig &config) {
  if (!gateway_prefers_wifi(config)) {
    return;
  }
  if (!::config.bluetooth.enabled) {
    return;
  }
  const bool bluetooth_was_enabled = ::config.bluetooth.enabled;
  ::config.bluetooth.enabled = false;
  disableBluetooth();
  if (bluetooth_was_enabled) {
    LOG_INFO("Irrigation gateway radio policy: wifi=on bluetooth=off previous_bt=1");
  }
}

String normalize_mqtt_host(const String &raw_host) {
  String host = raw_host;
  host.trim();

  if (host.startsWith("mqtt://")) {
    host.remove(0, strlen("mqtt://"));
  } else if (host.startsWith("tcp://")) {
    host.remove(0, strlen("tcp://"));
  } else if (host.startsWith("http://")) {
    host.remove(0, strlen("http://"));
  } else if (host.startsWith("https://")) {
    host.remove(0, strlen("https://"));
  }

  while (host.endsWith("/")) {
    host.remove(host.length() - 1);
  }

  return host;
}
}  // namespace

IrrigationMqttUplink::IrrigationMqttUplink()
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
    : plain_mqtt_client_(plain_client_),
      secure_mqtt_client_(secure_client_),
      mqtt_client_(nullptr),
      last_wifi_attempt_ms_(0),
      last_mqtt_attempt_ms_(0),
      wifi_reported_connected_(false),
      mqtt_reported_connected_(false),
      native_wifi_started_(false),
      mqtt_uses_tls_(true),
      last_wifi_status_(WL_IDLE_STATUS) {}
#else
    : last_wifi_attempt_ms_(0),
      last_mqtt_attempt_ms_(0),
      wifi_reported_connected_(false),
      mqtt_reported_connected_(false),
      native_wifi_started_(false),
      mqtt_uses_tls_(false),
      last_wifi_status_(0) {}
#endif

bool IrrigationMqttUplink::begin(const IrrigationConfig &config) {
  config_ = config;
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  const String normalized_host = normalize_mqtt_host(config_.mqtt_host);
  if (normalized_host != config_.mqtt_host) {
    config_.mqtt_host = normalized_host;
  }
  if (config_.mqtt_host.isEmpty()) {
    return false;
  }
  mqtt_uses_tls_ = config_.mqtt_port != 1883;
  if (mqtt_uses_tls_) {
    const char *ca_certificate = irrigation_mqtt_ca_certificate();
    if (ca_certificate == nullptr || ca_certificate[0] == '\0') {
      LOG_ERROR("Irrigation MQTT TLS refused: no trusted CA configured");
      return false;
    }
    secure_client_.setCACert(ca_certificate);
    secure_client_.setHandshakeTimeout(15);
    mqtt_client_ = &secure_mqtt_client_;
  } else {
    mqtt_client_ = &plain_mqtt_client_;
  }
  mqtt_client_->setId(config_.mqtt_client_id);
  mqtt_client_->setUsernamePassword(config_.mqtt_username, config_.mqtt_password);
  mqtt_client_->setCleanSession(false);
  mqtt_client_->setKeepAliveInterval(60 * 1000UL);
  mqtt_client_->setConnectionTimeout(15 * 1000UL);
  mqtt_client_->setTxPayloadSize(512);
  LOG_INFO("Irrigation MQTT transport: tls=%d port=%u", mqtt_uses_tls_ ? 1 : 0, config_.mqtt_port);
  native_wifi_started_ = ensure_native_wifi_started();
  last_wifi_status_ = WiFi.status();
  return true;
#else
  (void)config_;
  return false;
#endif
}

bool IrrigationMqttUplink::ensure_native_wifi_started() {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (native_wifi_started_) {
    return true;
  }
  if (wifiReconnect != nullptr || WiFi.status() == WL_CONNECTED) {
    native_wifi_started_ = true;
    return true;
  }
  if (config_.wifi_ssid.isEmpty()) {
    return false;
  }

  config.network.wifi_enabled = true;
  strncpy(config.network.wifi_ssid, config_.wifi_ssid.c_str(), sizeof(config.network.wifi_ssid) - 1);
  config.network.wifi_ssid[sizeof(config.network.wifi_ssid) - 1] = '\0';
  strncpy(config.network.wifi_psk, config_.wifi_password.c_str(), sizeof(config.network.wifi_psk) - 1);
  config.network.wifi_psk[sizeof(config.network.wifi_psk) - 1] = '\0';

  enforce_gateway_radio_policy(config_);

  setCPUFast(true);
  native_wifi_started_ = initWifi();
  if (native_wifi_started_) {
    if (wifiReconnect != nullptr) {
      wifiReconnect->setIntervalFromNow(0);
    }
  }
  if (!native_wifi_started_) {
    LOG_WARN("Irrigation WiFi bootstrap failed: wifi_enabled=%d ssid_len=%u",
             config.network.wifi_enabled ? 1 : 0, strlen(config.network.wifi_ssid));
  } else {
    LOG_INFO("Irrigation WiFi bootstrap: ssid=%s", config.network.wifi_ssid);
  }
  return native_wifi_started_;
#else
  return false;
#endif
}

void IrrigationMqttUplink::tick(uint64_t now_ms) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (!ensure_wifi_connected(now_ms)) {
    return;
  }
  ensure_mqtt_connected(now_ms);
  if (mqtt_client_ != nullptr && mqtt_client_->connected()) {
    mqtt_client_->poll();
  }
#else
  (void)now_ms;
#endif
}

bool IrrigationMqttUplink::is_connected() {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  return mqtt_client_ != nullptr && mqtt_client_->connected();
#else
  return false;
#endif
}

bool IrrigationMqttUplink::publish_payload(const String &topic, const String &payload, uint8_t qos_level) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  const uint64_t now_ms = millis();
  if (!ensure_wifi_connected(now_ms) || !ensure_mqtt_connected(now_ms)) {
    LOG_WARN("Irrigation MQTT publish skipped: connectivity unavailable topic=%s", topic.c_str());
    return false;
  }
  if (qos_level > 2 || mqtt_client_ == nullptr) {
    LOG_WARN("Irrigation MQTT invalid QoS/client: qos=%u", qos_level);
    return false;
  }
  if (!mqtt_client_->beginMessage(topic.c_str(), false, qos_level, false)) {
    LOG_WARN("Irrigation MQTT begin publish failed: topic=%s qos=%u", topic.c_str(), qos_level);
    return false;
  }
  const size_t written =
      mqtt_client_->write(reinterpret_cast<const uint8_t *>(payload.c_str()), payload.length());
  const int publish_result = mqtt_client_->endMessage();
  const bool published = written == payload.length() && publish_result == 1;
  if (published) {
    LOG_DEBUG("Irrigation MQTT published: topic=%s bytes=%u qos=%u", topic.c_str(), payload.length(), qos_level);
  } else {
    LOG_WARN("Irrigation MQTT publish failed: topic=%s qos=%u written=%u result=%d",
             topic.c_str(), qos_level, static_cast<unsigned int>(written), publish_result);
  }
  return published;
#else
  (void)topic;
  (void)payload;
  (void)qos_level;
  return false;
#endif
}

bool IrrigationMqttUplink::ensure_wifi_connected(uint64_t now_ms) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (!ensure_native_wifi_started()) {
    if (now_ms - last_wifi_attempt_ms_ >= k_wifi_retry_ms) {
      last_wifi_attempt_ms_ = now_ms;
      LOG_WARN("Irrigation WiFi unavailable: bootstrap not started");
    }
    return false;
  }

  const int32_t wifi_status = WiFi.status();
  enforce_gateway_radio_policy(config_);
  if (wifi_status != last_wifi_status_) {
    last_wifi_status_ = wifi_status;
    if (wifi_status != WL_CONNECTED) {
      const char *status_name = wifi_status_name(wifi_status);
#ifdef ARCH_ESP32
      if (wifi_status_has_disconnect_reason(wifi_status)) {
        LOG_WARN("Irrigation WiFi status=%d(%s) reason=%s",
                 wifi_status, status_name,
                 WiFi.disconnectReasonName(static_cast<wifi_err_reason_t>(getWifiDisconnectReason())));
      } else {
        LOG_DEBUG("Irrigation WiFi status=%d(%s)", wifi_status, status_name);
      }
#else
      LOG_DEBUG("Irrigation WiFi status=%d(%s)", wifi_status, status_name);
#endif
    }
  }

  if (wifi_status == WL_CONNECTED) {
    if (!wifi_reported_connected_) {
      wifi_reported_connected_ = true;
      LOG_INFO("Irrigation WiFi connected: ssid=%s ip=%s",
               config_.wifi_ssid.c_str(), WiFi.localIP().toString().c_str());
    }
    return true;
  }
  if (wifi_reported_connected_) {
    wifi_reported_connected_ = false;
    LOG_WARN("Irrigation WiFi disconnected");
  }
  if (now_ms - last_wifi_attempt_ms_ < k_wifi_retry_ms) {
    return false;
  }
  last_wifi_attempt_ms_ = now_ms;
  if (config_.wifi_ssid.isEmpty()) {
    LOG_WARN("Irrigation WiFi not configured: wifi_ssid empty");
    return false;
  }
  if (wifiReconnect != nullptr) {
    wifiReconnect->setIntervalFromNow(0);
  }
  LOG_INFO("Irrigation WiFi waiting for connect: ssid=%s status=%d(%s)",
           config_.wifi_ssid.c_str(), wifi_status, wifi_status_name(wifi_status));
  return false;
#else
  (void)now_ms;
  return false;
#endif
}

bool IrrigationMqttUplink::ensure_mqtt_connected(uint64_t now_ms) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (mqtt_client_ != nullptr && mqtt_client_->connected()) {
    if (!mqtt_reported_connected_) {
      mqtt_reported_connected_ = true;
      LOG_INFO("Irrigation MQTT connected: host=%s port=%u client_id=%s",
               config_.mqtt_host.c_str(), config_.mqtt_port, config_.mqtt_client_id.c_str());
    }
    return true;
  }
  if (mqtt_reported_connected_) {
    mqtt_reported_connected_ = false;
    LOG_WARN("Irrigation MQTT disconnected: connect_error=%d",
             mqtt_client_ != nullptr ? mqtt_client_->connectError() : -1);
  }
  if (now_ms - last_mqtt_attempt_ms_ < k_mqtt_retry_ms) {
    return false;
  }
  last_mqtt_attempt_ms_ = now_ms;
  LOG_INFO("Irrigation MQTT connect attempt: host=%s port=%u client_id=%s",
           config_.mqtt_host.c_str(), config_.mqtt_port, config_.mqtt_client_id.c_str());
  IPAddress resolved_ip;
  const bool dns_resolved = WiFi.hostByName(config_.mqtt_host.c_str(), resolved_ip) == 1;
  if (dns_resolved) {
    LOG_INFO("Irrigation MQTT DNS resolved: host=%s ip=%s",
             config_.mqtt_host.c_str(), resolved_ip.toString().c_str());
  } else {
    LOG_WARN("Irrigation MQTT DNS resolution failed: host=%s", config_.mqtt_host.c_str());
  }
  if (!probe_tcp_socket()) {
    LOG_WARN("Irrigation MQTT TCP probe failed: host=%s port=%u transport=%s",
             config_.mqtt_host.c_str(), config_.mqtt_port, mqtt_uses_tls_ ? "tls" : "plain_tcp");
  } else {
    LOG_INFO("Irrigation MQTT TCP probe succeeded: host=%s port=%u transport=%s",
             config_.mqtt_host.c_str(), config_.mqtt_port, mqtt_uses_tls_ ? "tls" : "plain_tcp");
  }
  if (mqtt_uses_tls_) {
    secure_client_.stop();
  } else {
    plain_client_.stop();
  }
  if (mqtt_client_ == nullptr) {
    return false;
  }
  const bool connected = mqtt_client_->connect(config_.mqtt_host.c_str(), config_.mqtt_port) == 1;
  if (!connected) {
    if (mqtt_uses_tls_) {
      char tls_error[96] = {0};
      const int tls_error_code = secure_client_.lastError(tls_error, sizeof(tls_error));
      LOG_WARN("Irrigation MQTT connect failed: connect_error=%d tls_error=%d detail=%s",
               mqtt_client_->connectError(), tls_error_code, tls_error[0] ? tls_error : "n/a");
    } else {
      LOG_WARN("Irrigation MQTT connect failed: connect_error=%d transport=plain_tcp",
               mqtt_client_->connectError());
    }
  }
  return connected;
#else
  (void)now_ms;
  return false;
#endif
}

bool IrrigationMqttUplink::probe_tcp_socket() {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  WiFiClient probe_client;
  probe_client.setTimeout(2000);
  const bool connected = probe_client.connect(config_.mqtt_host.c_str(), config_.mqtt_port, 2000);
  if (connected) {
    probe_client.stop();
  }
  return connected;
#else
  return false;
#endif
}
