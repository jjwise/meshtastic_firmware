#include "IrrigationMqttUplink.h"
#include "IrrigationWiFiBootstrap.h"

#include "DebugConfiguration.h"
#include "NodeDB.h"
#include "modules/AdminModule.h"
#include "mesh/wifi/WiFiAPClient.h"
#include "sleep.h"
#include "IrrigationSecrets.h"

#include <cstring>

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
#include <esp_heap_caps.h>
#include <esp_wifi.h>
#ifdef IRRIGATION_MQTT_TLS_INSECURE_DIAGNOSTIC
#include <cerrno>
#include <lwip/sockets.h>
#endif
#endif

namespace {
constexpr uint64_t k_wifi_retry_ms = 10000;
constexpr uint64_t k_mqtt_retry_ms = 5000;

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
void log_tls_heap(const char *stage) {
  constexpr uint32_t k_internal_8bit_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  constexpr uint32_t k_dma_caps = MALLOC_CAP_DMA;
  LOG_INFO("Irrigation TLS heap: stage=%s free=%u largest=%u minimum=%u dma_free=%u dma_largest=%u dma_minimum=%u",
           stage,
           static_cast<unsigned int>(heap_caps_get_free_size(k_internal_8bit_caps)),
           static_cast<unsigned int>(heap_caps_get_largest_free_block(k_internal_8bit_caps)),
           static_cast<unsigned int>(heap_caps_get_minimum_free_size(k_internal_8bit_caps)),
           static_cast<unsigned int>(heap_caps_get_free_size(k_dma_caps)),
           static_cast<unsigned int>(heap_caps_get_largest_free_block(k_dma_caps)),
           static_cast<unsigned int>(heap_caps_get_minimum_free_size(k_dma_caps)));
}

#ifdef IRRIGATION_MQTT_TLS_INSECURE_DIAGNOSTIC
const char *tls_handshake_state_name(int state) {
  switch (state) {
    case MBEDTLS_SSL_HELLO_REQUEST:
      return "HELLO_REQUEST";
    case MBEDTLS_SSL_CLIENT_HELLO:
      return "CLIENT_HELLO";
    case MBEDTLS_SSL_SERVER_HELLO:
      return "SERVER_HELLO";
    case MBEDTLS_SSL_SERVER_CERTIFICATE:
      return "SERVER_CERTIFICATE";
    case MBEDTLS_SSL_SERVER_KEY_EXCHANGE:
      return "SERVER_KEY_EXCHANGE";
    case MBEDTLS_SSL_CERTIFICATE_REQUEST:
      return "CERTIFICATE_REQUEST";
    case MBEDTLS_SSL_SERVER_HELLO_DONE:
      return "SERVER_HELLO_DONE";
    case MBEDTLS_SSL_CLIENT_CERTIFICATE:
      return "CLIENT_CERTIFICATE";
    case MBEDTLS_SSL_CLIENT_KEY_EXCHANGE:
      return "CLIENT_KEY_EXCHANGE";
    case MBEDTLS_SSL_CERTIFICATE_VERIFY:
      return "CERTIFICATE_VERIFY";
    case MBEDTLS_SSL_CLIENT_CHANGE_CIPHER_SPEC:
      return "CLIENT_CHANGE_CIPHER_SPEC";
    case MBEDTLS_SSL_CLIENT_FINISHED:
      return "CLIENT_FINISHED";
    case MBEDTLS_SSL_SERVER_CHANGE_CIPHER_SPEC:
      return "SERVER_CHANGE_CIPHER_SPEC";
    case MBEDTLS_SSL_SERVER_FINISHED:
      return "SERVER_FINISHED";
    case MBEDTLS_SSL_FLUSH_BUFFERS:
      return "FLUSH_BUFFERS";
    case MBEDTLS_SSL_HANDSHAKE_WRAPUP:
      return "HANDSHAKE_WRAPUP";
    case MBEDTLS_SSL_HANDSHAKE_OVER:
      return "HANDSHAKE_OVER";
    default:
      return "UNKNOWN";
  }
}

class IrrigationTlsDiagnosticClient final : public WiFiClientSecure {
 public:
  int connect(const char *host, uint16_t port) override {
    IPAddress address;
    if (WiFi.hostByName(host, address) != 1) {
      _lastError = -1;
      LOG_WARN("Irrigation TLS state probe DNS failed: host=%s", host);
      return 0;
    }

    errno = 0;
    const int result = start_ssl_client(sslclient, address, port, host, _timeout, _CA_cert, _use_ca_bundle,
                                        _cert, _private_key, nullptr, nullptr, _use_insecure, _alpn_protos);
    const int saved_errno = errno;
    _lastError = result;

    const int socket_fd = sslclient->socket;
    const int state = sslclient->ssl_ctx.state;
    const size_t input_left = sslclient->ssl_ctx.in_left;
    const size_t output_left = sslclient->ssl_ctx.out_left;
    int socket_error = 0;
    socklen_t socket_error_size = sizeof(socket_error);
    const int socket_option_result =
        socket_fd >= 0 ? getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_size) : -1;

    LOG_WARN("Irrigation TLS state probe: result=%d state=%d(%s) in_left=%u out_left=%u socket=%d errno=%d "
             "so_error=%d so_result=%d",
             result, state, tls_handshake_state_name(state), static_cast<unsigned int>(input_left),
             static_cast<unsigned int>(output_left), socket_fd, saved_errno, socket_error, socket_option_result);

    if (result < 0) {
      stop();
      return 0;
    }
    _connected = true;
    return 1;
  }
};

bool run_tcp_write_probe(const String &host, uint16_t port) {
  static constexpr char k_probe_marker[] = "IRRIGATION_TCP_PROBE";
  WiFiClient probe_client;
  const bool connected = probe_client.connect(host.c_str(), port, 3000);
  if (!connected) {
    LOG_WARN("Irrigation raw TCP write probe failed to connect: host=%s port=%u", host.c_str(), port);
    return false;
  }

  // Remove Nagle buffering as a possible explanation for a short diagnostic
  // marker and keep the socket alive long enough for the Wi-Fi TX task to run.
  probe_client.setNoDelay(true);
  const size_t requested = sizeof(k_probe_marker) - 1;
  errno = 0;
  const size_t written = probe_client.write(reinterpret_cast<const uint8_t *>(k_probe_marker), requested);
  const int write_errno = errno;
  const int socket_fd = probe_client.fd();
  int socket_error = 0;
  socklen_t socket_error_size = sizeof(socket_error);
  const int socket_option_result =
      socket_fd >= 0 ? getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_size) : -1;
  LOG_WARN("Irrigation raw TCP write probe: host=%s port=%u socket=%d requested=%u written=%u errno=%d "
           "so_error=%d so_result=%d",
           host.c_str(), port, socket_fd, static_cast<unsigned int>(requested),
           static_cast<unsigned int>(written), write_errno, socket_error, socket_option_result);

  delay(5000);
  socket_error = 0;
  socket_error_size = sizeof(socket_error);
  const int post_wait_option_result =
      socket_fd >= 0 ? getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_size) : -1;
  LOG_WARN("Irrigation raw TCP write probe after wait: socket=%d connected=%d available=%d so_error=%d so_result=%d",
           socket_fd, probe_client.connected() ? 1 : 0, probe_client.available(), socket_error,
           post_wait_option_result);
  probe_client.stop();
  return written == requested;
}
#endif
#endif

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
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
#endif
}  // namespace

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
void applyIrrigationGatewayWiFiBootstrap() {
#ifdef IRRIGATION_WIFI_SSID
  if (IRRIGATION_WIFI_SSID[0] == '\0') {
    return;
  }

  ::config.network.wifi_enabled = true;
  strncpy(::config.network.wifi_ssid, IRRIGATION_WIFI_SSID, sizeof(::config.network.wifi_ssid) - 1);
  ::config.network.wifi_ssid[sizeof(::config.network.wifi_ssid) - 1] = '\0';
#ifdef IRRIGATION_WIFI_PASSWORD
  strncpy(::config.network.wifi_psk, IRRIGATION_WIFI_PASSWORD, sizeof(::config.network.wifi_psk) - 1);
  ::config.network.wifi_psk[sizeof(::config.network.wifi_psk) - 1] = '\0';
#else
  ::config.network.wifi_psk[0] = '\0';
#endif
  LOG_INFO("Irrigation gateway WiFi credentials applied before native startup: ssid=%s",
           ::config.network.wifi_ssid);
#endif
}
#endif

IrrigationMqttUplink::IrrigationMqttUplink()
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
    : plain_mqtt_client_(plain_client_),
      secure_mqtt_client_(secure_client_),
      mqtt_client_(nullptr),
      last_wifi_attempt_ms_(0),
      last_mqtt_attempt_ms_(0),
      wifi_reported_connected_(false),
      mqtt_reported_connected_(false),
      command_subscription_active_(false),
      command_pending_(false),
      native_wifi_started_(false),
      mqtt_uses_tls_(true),
      last_wifi_status_(WL_IDLE_STATUS) {}
#else
    : last_wifi_attempt_ms_(0),
      last_mqtt_attempt_ms_(0),
      wifi_reported_connected_(false),
      mqtt_reported_connected_(false),
      command_subscription_active_(false),
      command_pending_(false),
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
#ifdef IRRIGATION_MQTT_TLS_INSECURE_DIAGNOSTIC
    LOG_WARN("Irrigation MQTT TLS diagnostic mode: certificate verification DISABLED");
    secure_client_.setInsecure();
#else
    const char *ca_certificate = irrigation_mqtt_ca_certificate();
    if (ca_certificate == nullptr || ca_certificate[0] == '\0') {
      LOG_ERROR("Irrigation MQTT TLS refused: no trusted CA configured");
      return false;
    }
    secure_client_.setCACert(ca_certificate);
#endif
    secure_client_.setHandshakeTimeout(45);
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
  command_subscription_active_ = false;
  command_pending_ = false;
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
    ensure_command_subscription();
    poll_command_message();
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

bool IrrigationMqttUplink::publish_payload(
    const String &topic, const String &payload, uint8_t qos_level, uint64_t now_ms) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
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
  (void)now_ms;
  return false;
#endif
}

bool IrrigationMqttUplink::pop_received_command(String &topic, String &payload) {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (!command_pending_) {
    return false;
  }
  topic = command_topic_;
  payload = command_payload_;
  command_topic_ = "";
  command_payload_ = "";
  command_pending_ = false;
  return true;
#else
  (void)topic;
  (void)payload;
  return false;
#endif
}

bool IrrigationMqttUplink::ensure_command_subscription() {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (command_subscription_active_) {
    return true;
  }
  if (mqtt_client_ == nullptr || !mqtt_client_->connected()) {
    return false;
  }
  const String topic = "farm/" + config_.farm_id + "/nodes/+/commands";
  const bool subscribed = mqtt_client_->subscribe(topic, 1) == 1;
  command_subscription_active_ = subscribed;
  if (subscribed) {
    LOG_INFO("Irrigation MQTT subscribed: topic=%s qos=1", topic.c_str());
  } else {
    LOG_WARN("Irrigation MQTT subscribe failed: topic=%s", topic.c_str());
  }
  return subscribed;
#else
  return false;
#endif
}

void IrrigationMqttUplink::poll_command_message() {
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  if (command_pending_ || mqtt_client_ == nullptr || !mqtt_client_->connected()) {
    return;
  }

  const int message_size = mqtt_client_->parseMessage();
  if (message_size <= 0) {
    return;
  }
  const String topic = mqtt_client_->messageTopic();
  const int retained = mqtt_client_->messageRetain();
  String payload;
  payload.reserve(static_cast<unsigned int>(message_size));
  while (mqtt_client_->available()) {
    payload += static_cast<char>(mqtt_client_->read());
  }

  const String prefix = "farm/" + config_.farm_id + "/nodes/";
  const bool valid_topic = topic.startsWith(prefix) && topic.endsWith("/commands");
  if (!valid_topic || retained == 1 || payload.length() > 512) {
    LOG_WARN("Irrigation MQTT command dropped: topic=%s retained=%d bytes=%u",
             topic.c_str(), retained, payload.length());
    return;
  }
  command_topic_ = topic;
  command_payload_ = payload;
  command_pending_ = true;
  LOG_INFO("Irrigation MQTT command received: topic=%s bytes=%u", topic.c_str(), payload.length());
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
    command_subscription_active_ = false;
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
#ifdef IRRIGATION_MQTT_TLS_INSECURE_DIAGNOSTIC
  static uint8_t diagnostic_phase = 0;
  if (diagnostic_phase == 0) {
    diagnostic_phase = 1;
    setCPUFast(true);
    log_tls_heap("before_raw_tcp_probe");
    const bool tcp_write_succeeded = run_tcp_write_probe(config_.mqtt_host, config_.mqtt_port);
    log_tls_heap("after_raw_tcp_probe");
    LOG_WARN("Irrigation raw-first diagnostic completed: raw_tcp_write=%d tls_not_started=1",
             tcp_write_succeeded ? 1 : 0);
    return false;
  }
  if (diagnostic_phase == 1) {
    diagnostic_phase = 2;
    setCPUFast(true);
    log_tls_heap("before_state_probe");
    IrrigationTlsDiagnosticClient diagnostic_client;
    diagnostic_client.setInsecure();
    diagnostic_client.setHandshakeTimeout(45);
    const int tls_result = diagnostic_client.connect(config_.mqtt_host.c_str(), config_.mqtt_port);
    log_tls_heap("after_state_probe");
    LOG_WARN("Irrigation TLS state probe completed: connected=%d", tls_result == 1 ? 1 : 0);
    diagnostic_client.stop();
    return false;
  }
  return false;
#endif
  // Do not open a diagnostic socket immediately before TLS. Its short-lived
  // allocations can fragment the internal heap needed by mbedTLS.
  if (mqtt_uses_tls_) {
    secure_client_.stop();
    setCPUFast(true);
    log_tls_heap("before_connect");
  } else {
    plain_client_.stop();
  }
  if (mqtt_client_ == nullptr) {
    return false;
  }
  const bool connected = mqtt_client_->connect(config_.mqtt_host.c_str(), config_.mqtt_port) == 1;
  if (connected) {
    command_subscription_active_ = false;
    ensure_command_subscription();
  }
  if (!connected) {
    if (mqtt_uses_tls_) {
      char tls_error[96] = {0};
      const int tls_error_code = secure_client_.lastError(tls_error, sizeof(tls_error));
      log_tls_heap("after_failure");
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
