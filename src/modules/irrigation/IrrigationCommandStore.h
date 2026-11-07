#pragma once

#include <Arduino.h>

class IrrigationCommandStore {
 public:
  bool load(uint64_t &issued_at_utc_ms, String &command_id);
  bool save(uint64_t issued_at_utc_ms, const String &command_id);
};
