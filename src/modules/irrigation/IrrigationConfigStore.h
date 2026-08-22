#pragma once

#include <Arduino.h>

#include "IrrigationConfig.h"

/**
 * Lightweight persistence for irrigation-specific settings.
 *
 * This deliberately stays outside Meshtastic's native config protobufs so the
 * irrigation overlay remains isolated from core admin/app config flows.
 */
class IrrigationConfigStore {
 public:
  IrrigationConfigStore();

  /**
   * Initializes persistent storage access.
   *
   * The default file path keeps the module self-contained in the device
   * filesystem. Call this once before load/save operations.
   */
  bool begin(const String &file_path = "/irrigation.cfg");

  /**
   * Merges any persisted key/value pairs into the provided config.
   *
   * The input config is treated as the caller's baseline, which allows a clean
   * precedence chain such as defaults -> persisted config -> build overrides.
   */
  bool load(IrrigationConfig &config) const;

  /**
   * Persists the full effective config snapshot.
   */
  bool save(const IrrigationConfig &config) const;

  /**
   * Persists the config only when the stored snapshot differs.
   */
  bool save_if_changed(const IrrigationConfig &config) const;

  bool exists() const;
  const String &file_path() const;

 private:
  String file_path_;
};
