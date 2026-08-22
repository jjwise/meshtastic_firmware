#include "IrrigationGatewayStore.h"

#include "DebugConfiguration.h"

#ifdef ARCH_ESP32
#include <LittleFS.h>
#endif

namespace {
bool ensure_filesystem_ready() {
#ifdef ARCH_ESP32
  static bool initialized = false;
  static bool ready = false;
  if (!initialized) {
    ready = LittleFS.begin(true);
    initialized = true;
  }
  return ready;
#else
  return false;
#endif
}
}  // namespace

IrrigationGatewayStore::IrrigationGatewayStore() : file_path_("/irrigation_queue.log"), max_records_(500) {}

bool IrrigationGatewayStore::begin(const String &file_path, size_t max_records) {
  file_path_ = file_path;
  max_records_ = max_records;
#ifdef ARCH_ESP32
  if (!ensure_filesystem_ready()) {
    LOG_WARN("Irrigation gateway store begin failed: filesystem unavailable path=%s", file_path_.c_str());
    return false;
  }
  if (!LittleFS.exists(file_path_)) {
    File queue_file = LittleFS.open(file_path_, FILE_WRITE);
    if (!queue_file) {
      LOG_WARN("Irrigation gateway store begin failed: unable to create path=%s", file_path_.c_str());
      return false;
    }
    queue_file.close();
  }
  return true;
#else
  return false;
#endif
}

bool IrrigationGatewayStore::append(const String &payload) {
  std::vector<String> lines;
  if (!load_lines(lines)) {
    return false;
  }
  lines.push_back(payload);
  while (lines.size() > max_records_) {
    lines.erase(lines.begin());
  }
  return save_lines(lines);
}

bool IrrigationGatewayStore::peek_next(String &payload) {
  std::vector<String> lines;
  if (!load_lines(lines) || lines.empty()) {
    return false;
  }
  payload = lines.front();
  return true;
}

bool IrrigationGatewayStore::pop_next() {
  std::vector<String> lines;
  if (!load_lines(lines) || lines.empty()) {
    return false;
  }
  lines.erase(lines.begin());
  return save_lines(lines);
}

size_t IrrigationGatewayStore::size() const {
  std::vector<String> lines;
  if (!load_lines(lines)) {
    return 0;
  }
  return lines.size();
}

bool IrrigationGatewayStore::load_lines(std::vector<String> &lines) const {
  lines.clear();
#ifdef ARCH_ESP32
  if (!ensure_filesystem_ready()) {
    LOG_WARN("Irrigation gateway store read failed: filesystem unavailable path=%s", file_path_.c_str());
    return false;
  }
  File queue_file = LittleFS.open(file_path_, FILE_READ);
  if (!queue_file) {
    LOG_WARN("Irrigation gateway store read failed: open path=%s", file_path_.c_str());
    return false;
  }

  while (queue_file.available()) {
    String line = queue_file.readStringUntil('\n');
    line.trim();
    if (!line.isEmpty()) {
      lines.push_back(line);
    }
  }
  queue_file.close();
  return true;
#else
  return false;
#endif
}

bool IrrigationGatewayStore::save_lines(const std::vector<String> &lines) {
#ifdef ARCH_ESP32
  if (!ensure_filesystem_ready()) {
    LOG_WARN("Irrigation gateway store write failed: filesystem unavailable path=%s", file_path_.c_str());
    return false;
  }
  if (LittleFS.exists(file_path_) && !LittleFS.remove(file_path_)) {
    LOG_WARN("Irrigation gateway store write failed: remove path=%s", file_path_.c_str());
    return false;
  }
  File queue_file = LittleFS.open(file_path_, FILE_WRITE);
  if (!queue_file) {
    LOG_WARN("Irrigation gateway store write failed: open path=%s", file_path_.c_str());
    return false;
  }
  bool success = true;
  for (const String &line : lines) {
    success = success && queue_file.println(line);
  }
  queue_file.close();
  return success;
#else
  (void)lines;
  return false;
#endif
}
