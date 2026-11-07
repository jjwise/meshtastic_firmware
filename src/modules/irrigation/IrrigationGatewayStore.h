#pragma once

#include <Arduino.h>

class IrrigationGatewayStore {
 public:
  IrrigationGatewayStore();

  bool begin(const String &file_path, size_t max_records);
  bool append(const String &payload);
  bool peek_next(String &payload);
  bool pop_next();
  size_t size() const;

 private:
  bool rewrite(const String *appended_payload);
  bool count_records(size_t &count) const;

  String file_path_;
  size_t max_records_;
};

