#include "IrrigationCommandStore.h"

#include "configuration.h"

#include <cstring>

#ifdef ARCH_ESP32
#include <nvs.h>
#endif

namespace {
constexpr size_t k_max_command_id_length = 128;
constexpr size_t k_header_size = 10;
constexpr uint64_t k_min_valid_epoch_ms = 1577836800000ULL;
}

bool IrrigationCommandStore::load(uint64_t &issued_at_utc_ms, String &command_id) {
  issued_at_utc_ms = 0;
  command_id = "";
#ifdef ARCH_ESP32
  nvs_handle_t handle;
  if (nvs_open("irrigation-cmd", NVS_READWRITE, &handle) != ESP_OK) {
    return false;
  }
  uint8_t record[k_header_size + k_max_command_id_length] = {};
  size_t length = sizeof(record);
  const esp_err_t result = nvs_get_blob(handle, "checkpoint", record, &length);
  nvs_close(handle);
  if (result == ESP_ERR_NVS_NOT_FOUND) {
    return true;
  }
  if (result != ESP_OK || length < k_header_size || record[0] != 1 ||
      record[9] == 0 || record[9] > k_max_command_id_length ||
      length != k_header_size + record[9]) {
    return false;
  }
  uint64_t timestamp = 0;
  for (size_t index = 0; index < 8; ++index) {
    timestamp |= static_cast<uint64_t>(record[1 + index]) << (8 * index);
  }
  if (timestamp < k_min_valid_epoch_ms || memchr(record + k_header_size, 0, record[9]) != nullptr) {
    return false;
  }
  char identifier[k_max_command_id_length + 1] = {};
  memcpy(identifier, record + k_header_size, record[9]);
  issued_at_utc_ms = timestamp;
  command_id = identifier;
  return command_id.length() == record[9];
#else
  return false;
#endif
}

bool IrrigationCommandStore::save(uint64_t issued_at_utc_ms, const String &command_id) {
#ifdef ARCH_ESP32
  if (issued_at_utc_ms < k_min_valid_epoch_ms || command_id.isEmpty() ||
      command_id.length() > k_max_command_id_length) {
    return false;
  }
  uint8_t record[k_header_size + k_max_command_id_length] = {};
  record[0] = 1;
  for (size_t index = 0; index < 8; ++index) {
    record[1 + index] = static_cast<uint8_t>(issued_at_utc_ms >> (8 * index));
  }
  record[9] = static_cast<uint8_t>(command_id.length());
  const size_t length = k_header_size + command_id.length();
  memcpy(record + k_header_size, command_id.c_str(), command_id.length());
  nvs_handle_t handle;
  if (nvs_open("irrigation-cmd", NVS_READWRITE, &handle) != ESP_OK) {
    return false;
  }
  // Commit one checkpoint before changing an output; power loss then consumes the command.
  const bool saved = nvs_set_blob(handle, "checkpoint", record, length) == ESP_OK && nvs_commit(handle) == ESP_OK;
  nvs_close(handle);
  uint64_t stored_time = 0;
  String stored_id;
  return saved && load(stored_time, stored_id) && stored_time == issued_at_utc_ms && stored_id == command_id;
#else
  return false;
#endif
}
