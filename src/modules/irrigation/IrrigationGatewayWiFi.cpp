#include "configuration.h"
#include "IrrigationWiFiBootstrap.h"

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
#include "DebugConfiguration.h"
#include "IrrigationConfig.h"
#include "IrrigationSecrets.h"
#include "NodeDB.h"
#include "mesh/Throttle.h"

#include <cstring>

namespace {
struct WiFiCredentials {
  const char *ssid;
  const char *password;
};

#ifndef IRRIGATION_WIFI_SSID
#define IRRIGATION_WIFI_SSID ""
#endif
#ifndef IRRIGATION_WIFI_PASSWORD
#define IRRIGATION_WIFI_PASSWORD ""
#endif
#ifndef IRRIGATION_WIFI_NETWORKS
#define IRRIGATION_WIFI_NETWORKS {{IRRIGATION_WIFI_SSID, IRRIGATION_WIFI_PASSWORD}}
#endif

const WiFiCredentials networks[] = IRRIGATION_WIFI_NETWORKS;
constexpr size_t network_count = sizeof(networks) / sizeof(networks[0]);
static_assert(network_count > 0, "IRRIGATION_WIFI_NETWORKS must contain at least one entry");
constexpr uint32_t attempt_timeout_ms = 20000;
size_t active_network = 0;
uint32_t attempt_started_ms = 0;
bool network_selected = false;
bool attempt_started = false;

bool valid_network(size_t index) {
  const auto &network = networks[index];
  return network.ssid != nullptr && network.password != nullptr && network.ssid[0] != '\0' &&
         strlen(network.ssid) < sizeof(config.network.wifi_ssid) &&
         strlen(network.password) < sizeof(config.network.wifi_psk);
}

bool find_network(size_t start, size_t &index) {
  for (size_t offset = 0; offset < network_count; ++offset) {
    const size_t candidate = (start + offset) % network_count;
    if (valid_network(candidate)) {
      index = candidate;
      return true;
    }
  }
  return false;
}

void select_network(size_t index) {
  const auto &network = networks[index];
  strcpy(config.network.wifi_ssid, network.ssid);
  strcpy(config.network.wifi_psk, network.password);
  config.network.wifi_enabled = true;
  active_network = index;
  network_selected = true;
  LOG_INFO("Irrigation WiFi selected: network=%u/%u", static_cast<unsigned int>(index + 1),
           static_cast<unsigned int>(network_count));
}
}  // namespace

void applyIrrigationGatewayWiFiConfig(IrrigationConfig &config) {
  size_t index;
  if (find_network(0, index)) {
    config.wifi_ssid = networks[index].ssid;
    config.wifi_password = networks[index].password;
  }
}

void applyIrrigationGatewayWiFiBootstrap() {
  size_t index;
  network_selected = false;
  attempt_started = false;
  if (find_network(0, index)) {
    select_network(index);
  }
}

bool advanceIrrigationGatewayWiFi(bool connected) {
  if (!network_selected || !config.network.wifi_enabled) {
    return false;
  }
  if (connected) {
    attempt_started = false;
    return false;
  }
  if (!attempt_started) {
    attempt_started_ms = millis();
    attempt_started = true;
    return false;
  }
  if (Throttle::isWithinTimespanMs(attempt_started_ms, attempt_timeout_ms)) {
    return false;
  }
  attempt_started_ms = millis();
  size_t next;
  if (!find_network((active_network + 1) % network_count, next) || next == active_network) {
    return false;
  }
  select_network(next);
  return true;
}
#endif
