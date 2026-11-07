#!/usr/bin/env python3
"""Exercise actual uplink methods with a host transport double, without credentials."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent

HARNESS = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>
#define ARCH_ESP32
#define IRRIGATION_ENABLE_MQTT_UPLINK
#define LOG_WARN(...) ((void)0)
#define LOG_INFO(...) ((void)0)
#define LOG_DEBUG(...) ((void)0)
class String : public std::string {
public:
  using std::string::string;
  String(const std::string &s) : std::string(s) {}
  static inline size_t reservations = 0;
  static inline bool fail_reserve = false;
  bool reserve(unsigned int n) {
    ++reservations;
    if (fail_reserve) return false;
    std::string::reserve(n);
    return true;
  }
  bool startsWith(const String &s) const { return rfind(s, 0) == 0; }
  bool endsWith(const String &s) const {
    return size() >= s.size() && compare(size() - s.size(), s.size(), s) == 0;
  }
};
constexpr int WL_CONNECTED = 3;
struct { int state = WL_CONNECTED; int status() { return state; } } WiFi;
struct Client {
  bool online = true, partial = false;
  int begin_result = 1, end_result = 1, begins = 0, ends = 0, stops = 0;
  unsigned long length = 0;
  uint8_t qos = 0;
  std::string sent;
  String incoming_topic = "farm/f/nodes/n/commands";
  std::string incoming = "{}";
  int retained = 0, fail_read_at = -1, reads = 0;
  size_t position = 0;
  bool connected() { return online; }
  int beginMessage(const char *, unsigned long n, bool retain, uint8_t q, bool dup) {
    assert(!retain && !dup); ++begins; length = n; qos = q; return begin_result;
  }
  size_t write(const uint8_t *p, size_t n) {
    if (partial && n) --n;
    sent.assign(reinterpret_cast<const char *>(p), n);
    return n;
  }
  int endMessage() { ++ends; return end_result; }
  void stop() { ++stops; online = false; }
  int parseMessage() { return static_cast<int>(incoming.size()); }
  String messageTopic() { return incoming_topic; }
  int messageRetain() { return retained; }
  int available() { return static_cast<int>(incoming.size() - position); }
  int read() {
    ++reads;
    if (static_cast<int>(position) == fail_read_at) return -1;
    return position < incoming.size() ? static_cast<unsigned char>(incoming[position++]) : -1;
  }
};
class IrrigationMqttUplink {
public:
  Client *mqtt_client_ = nullptr;
  struct { String farm_id = "f"; } config_;
  bool command_pending_ = false;
  String command_topic_, command_payload_;
  bool is_connected() { return mqtt_client_ && mqtt_client_->connected(); }
  bool publish_payload(const String &, const String &, uint8_t, uint64_t);
  void poll_command_message();
};
'''

TEST = r'''
int main() {
  Client client;
  IrrigationMqttUplink uplink;
  uplink.mqtt_client_ = &client;
  const String payload(900, 'x');
  for (uint8_t qos = 0; qos <= 2; ++qos) {
    client = Client{};
    assert(uplink.publish_payload("topic", payload, qos, 0));
    assert(client.begins == 1 && client.ends == 1 && client.length == payload.size());
    assert(client.sent == payload && client.qos == qos);
  }
  client = Client{}; WiFi.state = 0;
  assert(!uplink.publish_payload("topic", payload, 1, 0) && client.begins == 0);
  WiFi.state = WL_CONNECTED; client.online = false;
  assert(!uplink.publish_payload("topic", payload, 1, 0) && client.begins == 0);
  client = Client{};
  assert(!uplink.publish_payload("topic", payload, 3, 0) && client.begins == 0);
  client.partial = true;
  assert(!uplink.publish_payload("topic", payload, 1, 0));
  assert(client.stops == 1 && client.ends == 0);
  client = Client{}; client.begin_result = 0;
  assert(!uplink.publish_payload("topic", payload, 1, 0) && client.ends == 0);
  client = Client{}; client.end_result = 0;
  assert(!uplink.publish_payload("topic", payload, 1, 0));

  client = Client{};
  uplink.poll_command_message();
  assert(uplink.command_pending_ && uplink.command_payload_ == "{}");
  assert(uplink.command_topic_ == client.incoming_topic);
  int reads = client.reads;
  uplink.poll_command_message();
  assert(client.reads == reads);
  uplink.command_pending_ = false;
  for (int mode = 0; mode < 3; ++mode) {
    client = Client{};
    if (mode == 0) client.incoming.assign(513, 'x');
    if (mode == 1) client.retained = 1;
    if (mode == 2) client.incoming_topic = "unrelated/topic";
    String::reservations = 0;
    uplink.poll_command_message();
    assert(!uplink.command_pending_ && String::reservations == 0);
    assert(client.available() == 0);
  }
  client = Client{}; client.incoming.assign(512, 'x');
  uplink.poll_command_message();
  assert(uplink.command_pending_ && uplink.command_payload_.size() == 512);
  uplink.command_pending_ = false;
  client = Client{}; client.fail_read_at = 1;
  uplink.poll_command_message();
  assert(!uplink.command_pending_ && uplink.command_payload_.empty() && client.stops == 1);
  client = Client{}; client.retained = 1; client.fail_read_at = 0;
  uplink.poll_command_message();
  assert(!uplink.command_pending_ && client.stops == 1 && client.reads == 1);
  client = Client{}; String::fail_reserve = true;
  uplink.poll_command_message();
  assert(!uplink.command_pending_ && client.stops == 1 && client.reads == 0);
  std::cout << "PASS: streamed publish/QoS, disconnected/partial/failed sends, bounded commands/read failures\n";
}
'''


def main():
    source = (ROOT / "src/modules/irrigation/IrrigationMqttUplink.cpp").read_text()
    # Compile unchanged production method bodies; hardware setup stays outside this host test.
    publish = source[source.index("bool IrrigationMqttUplink::publish_payload("):
                     source.index("bool IrrigationMqttUplink::pop_received_command(")]
    receive = source[source.index("void IrrigationMqttUplink::poll_command_message()"):
                     source.index("bool IrrigationMqttUplink::ensure_wifi_connected(")]
    with tempfile.TemporaryDirectory(prefix="irrigation-mqtt-") as temp:
        directory = pathlib.Path(temp)
        test = directory / "test.cpp"
        test.write_text(HARNESS + publish + receive + TEST)
        binary = directory / "mqtt-test"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        str(test), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
