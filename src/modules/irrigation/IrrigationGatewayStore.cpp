#include "IrrigationGatewayStore.h"

#include "DebugConfiguration.h"

#include <utility>

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
  return rewrite(&payload);
}

bool IrrigationGatewayStore::peek_next(String &payload) {
#ifdef ARCH_ESP32
  if (!ensure_filesystem_ready()) {
    return false;
  }
  File queue_file = LittleFS.open(file_path_, FILE_READ);
  if (!queue_file) {
    return false;
  }
  while (queue_file.available()) {
    String line = queue_file.readStringUntil('\n');
    line.trim();
    if (!line.isEmpty()) {
      payload = std::move(line);
      return true;
    }
  }
#else
  (void)payload;
#endif
  return false;
}

bool IrrigationGatewayStore::pop_next() {
  return rewrite(nullptr);
}

size_t IrrigationGatewayStore::size() const {
  size_t count = 0;
  return count_records(count) ? count : 0;
}

bool IrrigationGatewayStore::count_records(size_t &count) const {
  count = 0;
#ifdef ARCH_ESP32
  if (!ensure_filesystem_ready()) {
    return false;
  }
  File queue_file = LittleFS.open(file_path_, FILE_READ);
  if (!queue_file) {
    return false;
  }
  while (queue_file.available()) {
    String line = queue_file.readStringUntil('\n');
    line.trim();
    if (!line.isEmpty()) {
      ++count;
    }
  }
  return true;
#else
  return false;
#endif
}

bool IrrigationGatewayStore::rewrite(const String *appended_payload) {
#ifdef ARCH_ESP32
  size_t count = 0;
  if (!count_records(count) || (appended_payload == nullptr && count == 0)) {
    return false;
  }

  const String temp_path = file_path_ + ".tmp";
  File input = LittleFS.open(file_path_, FILE_READ);
  if (!input) {
    return false;
  }
  File output = LittleFS.open(temp_path, FILE_WRITE);
  if (!output) {
    return false;
  }

  size_t skip = 1;
  if (appended_payload != nullptr) {
    const size_t keep = max_records_ > 0 ? max_records_ - 1 : 0;
    skip = count > keep ? count - keep : 0;
  }
  bool success = true;
  // Keep at most one existing record in RAM, regardless of the backlog size.
  while (input.available()) {
    String line = input.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) {
      continue;
    }
    if (skip > 0) {
      --skip;
      continue;
    }
    if (output.println(line) != line.length() + 2) {
      success = false;
      break;
    }
  }
  if (success && appended_payload != nullptr && max_records_ > 0) {
    success = output.println(*appended_payload) == appended_payload->length() + 2;
  }
  output.flush();
  output.close();
  input.close();

  // Replace only a fully written file; LittleFS rename replaces the old target.
  if (success && LittleFS.rename(temp_path, file_path_)) {
    return true;
  }
  LittleFS.remove(temp_path);
  LOG_WARN("Irrigation gateway store write failed");
  return false;
#else
  (void)appended_payload;
  return false;
#endif
}
