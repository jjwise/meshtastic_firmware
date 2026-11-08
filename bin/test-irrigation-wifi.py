#!/usr/bin/env python3
"""Exercise ordered gateway Wi-Fi selection and the real native reconnect callback."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent

ARDUINO = r"""
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <utility>
class String : public std::string {
 public:
  using std::string::string;
  String(const std::string &s) : std::string(s) {}
  bool isEmpty() const { return empty(); }
};
inline uint32_t now_ms = 0;
inline uint32_t millis() { return now_ms; }
"""
NODE_DB = r"""
#pragma once
struct NativeConfig {
  struct { bool wifi_enabled = false; char wifi_ssid[33] = {}; char wifi_psk[65] = {}; } network;
};
inline NativeConfig config;
"""
SECRETS = r"""
#pragma once
#if TEST_CASE == 1
#define IRRIGATION_WIFI_SSID "legacy-ignored"
#define IRRIGATION_WIFI_PASSWORD "legacy-password"
#define IRRIGATION_WIFI_NETWORKS {{"alpha", "alpha-password"}, {"beta", "beta-password"}, {"gamma", ""}}
#elif TEST_CASE == 2
#define IRRIGATION_WIFI_SSID "legacy"
#define IRRIGATION_WIFI_PASSWORD "legacy-password"
#elif TEST_CASE == 3
#define IRRIGATION_WIFI_NETWORKS {{"", ""}, {"123456789012345678901234567890123", "p"}, {"bad-pass", "12345678901234567890123456789012345678901234567890123456789012345"}, {nullptr, "p"}, {"null-password", nullptr}, {"Reseau d\u2019\u00e9t\u00e9", "unicode-password"}, {"second", ""}}
#elif TEST_CASE == 5
#define IRRIGATION_WIFI_NETWORKS {{"same", "old-password"}, {"same", "new-password"}}
#endif
"""
HARNESS = r"""
#include "configuration.h"
#include "DebugConfiguration.h"
#include "Arduino.h"
#include "NodeDB.h"
#include "IrrigationConfig.h"
#include "IrrigationWiFiBootstrap.h"
#include <cassert>
#include <cstring>
#include <iostream>
struct Transport {
 bool connected = false;
 unsigned disconnects = 0;
 std::vector<std::pair<std::string,std::string>> attempts;
 bool isConnected() { return connected; }
 void disconnect(bool, bool) { connected = false; ++disconnects; }
 void begin(const char *ssid, const char *password) { attempts.emplace_back(ssid, password ? password : ""); }
} WiFi;
bool needReconnect = true, isReconnecting = false, wifiReconnectPending = false;
unsigned long wifiReconnectStartMillis = 0;
void onNetworkConnected() {}
"""
TEST = r"""
int main() {
 (void)&reconnectWiFi;
 IrrigationConfig runtime{};
#if TEST_CASE == 1
 applyIrrigationGatewayWiFiConfig(runtime);
 assert(runtime.wifi_ssid == "alpha" && runtime.wifi_password == "alpha-password");
 applyIrrigationGatewayWiFiBootstrap();
 assert(config.network.wifi_enabled && std::string(config.network.wifi_ssid) == "alpha");
 assert(reconnectWiFi() == 5000 && WiFi.attempts.empty());
 now_ms = 4999; assert(reconnectWiFi() == 100);
 now_ms = 5000; assert(reconnectWiFi() == 1000);
 assert(WiFi.attempts.back() == std::make_pair(std::string("alpha"),std::string("alpha-password")));
 // Native retries do not restart the per-network deadline.
 now_ms = 8000; needReconnect = true; assert(reconnectWiFi() == 5000);
 now_ms = 13000; reconnectWiFi(); assert(WiFi.attempts.back().first == "alpha");
 now_ms = 19999; reconnectWiFi(); assert(std::string(config.network.wifi_ssid) == "alpha");
 now_ms = 20000; assert(reconnectWiFi() == 5000); assert(std::string(config.network.wifi_ssid) == "beta");
 now_ms = 25000; reconnectWiFi(); assert(WiFi.attempts.back().first == "beta");
 WiFi.connected = true; now_ms = 26000; assert(reconnectWiFi() == 300000);
 now_ms = 1000000; assert(reconnectWiFi() == 300000); assert(std::string(config.network.wifi_ssid) == "beta");
 // Retry the working network first after a loss, then advance on timeout.
 WiFi.connected = false; needReconnect = true; now_ms = 1000001; reconnectWiFi();
 now_ms += 5000; reconnectWiFi(); assert(WiFi.attempts.back().first == "beta");
 now_ms = 1020001; reconnectWiFi(); assert(std::string(config.network.wifi_ssid) == "gamma");
 now_ms += 5000; reconnectWiFi(); assert(WiFi.attempts.back().second.empty());
 // No disconnect event is required to escape a stalled connection/DHCP attempt.
 now_ms = 1040001; reconnectWiFi(); assert(std::string(config.network.wifi_ssid) == "alpha");
 applyIrrigationGatewayWiFiBootstrap(); now_ms = UINT32_MAX - 10000U;
 assert(!advanceIrrigationGatewayWiFi(false)); now_ms += 19999U;
 assert(!advanceIrrigationGatewayWiFi(false)); ++now_ms;
 assert(advanceIrrigationGatewayWiFi(false) && std::string(config.network.wifi_ssid) == "beta");
 config.network.wifi_enabled = false; now_ms += 20000;
 assert(!advanceIrrigationGatewayWiFi(false) && !config.network.wifi_enabled);
 applyIrrigationGatewayWiFiBootstrap(); assert(std::string(config.network.wifi_ssid) == "alpha");
#elif TEST_CASE == 2
 applyIrrigationGatewayWiFiConfig(runtime); assert(runtime.wifi_ssid == "legacy");
 applyIrrigationGatewayWiFiBootstrap(); assert(std::string(config.network.wifi_ssid) == "legacy");
 assert(!advanceIrrigationGatewayWiFi(false)); now_ms = 100000;
 assert(!advanceIrrigationGatewayWiFi(false));
#elif TEST_CASE == 3
 applyIrrigationGatewayWiFiConfig(runtime); assert(runtime.wifi_ssid == "Reseau d\u2019\u00e9t\u00e9");
 applyIrrigationGatewayWiFiBootstrap(); assert(std::string(config.network.wifi_ssid) == "Reseau d\u2019\u00e9t\u00e9");
 assert(!advanceIrrigationGatewayWiFi(false)); now_ms = 20000;
 assert(advanceIrrigationGatewayWiFi(false) && std::string(config.network.wifi_ssid) == "second");
#elif TEST_CASE == 4
 strcpy(config.network.wifi_ssid, "native-config"); runtime.wifi_ssid = "runtime-config";
 applyIrrigationGatewayWiFiConfig(runtime); applyIrrigationGatewayWiFiBootstrap();
 assert(runtime.wifi_ssid == "runtime-config" && std::string(config.network.wifi_ssid) == "native-config");
 assert(!advanceIrrigationGatewayWiFi(false));
#elif TEST_CASE == 5
 applyIrrigationGatewayWiFiBootstrap(); assert(std::string(config.network.wifi_psk) == "old-password");
 assert(!advanceIrrigationGatewayWiFi(false)); now_ms = 20000;
 assert(advanceIrrigationGatewayWiFi(false) && std::string(config.network.wifi_psk) == "new-password");
#endif
 std::cout << "PASS Wi-Fi scenario " << TEST_CASE << "\n";
}
"""

def main():
    source = (ROOT / "src/mesh/wifi/WiFiAPClient.cpp").read_text()
    callback = source[source.index("static int32_t reconnectWiFi()"):
                      source.index("bool isWifiAvailable()")]
    with tempfile.TemporaryDirectory(prefix="irrigation-wifi-") as temp:
        directory = pathlib.Path(temp)
        (directory / "mesh").mkdir()
        stubs = {
            "Arduino.h": ARDUINO, "NodeDB.h": NODE_DB,
            "configuration.h": "#pragma once\n#include <Arduino.h>\n#ifdef ARDUINO_ARCH_ESP32\n#define ARCH_ESP32\n#endif\n",
            "DebugConfiguration.h": "#pragma once\n#include \"configuration.h\"\n#define LOG_INFO(...) ((void)0)\n",
            "IrrigationGatewaySecrets.generated.h": SECRETS,
            "test.cpp": HARNESS + callback + TEST,
        }
        for name, content in stubs.items():
            (directory / name).write_text(content)
        # Copy only public production source/headers; never copy the real generated secrets.
        for name in ("IrrigationGatewayWiFi.cpp", "IrrigationWiFiBootstrap.h", "IrrigationSecrets.h", "IrrigationConfig.h", "IrrigationTypes.h"):
            (directory / name).write_bytes((ROOT / "src/modules/irrigation" / name).read_bytes())
        (directory / "mesh/Throttle.h").write_bytes((ROOT / "src/mesh/Throttle.h").read_bytes())
        (directory / "Throttle.h").write_bytes((ROOT / "src/mesh/Throttle.h").read_bytes())
        (directory / "Throttle.cpp").write_bytes((ROOT / "src/mesh/Throttle.cpp").read_bytes())
        for case in range(1, 6):
            binary = directory / ("wifi-test-" + str(case))
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-DARDUINO_ARCH_ESP32", "-DIRRIGATION_ENABLE_MQTT_UPLINK", "-DDISABLE_NTP", f"-DTEST_CASE={case}", "-I", str(directory), str(directory / "test.cpp"), str(directory / "IrrigationGatewayWiFi.cpp"), str(directory / "Throttle.cpp"), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    print("PASS ordered fallback, native retries/deadlines, loss/recovery, stalled attempts, rollover, legacy credentials, invalid/UTF-8 entries and duplicate SSIDs")

if __name__ == "__main__":
    main()
