#pragma once

#include <Arduino.h>

#include <vector>

#include "IrrigationConfig.h"
#include "IrrigationGatewayStore.h"
#include "IrrigationMotionDetector.h"
#include "IrrigationMqttUplink.h"
#include "IrrigationPacketCodec.h"

/**
 * Isolated irrigation module intended for Meshtastic fork integration.
 *
 * Integration points expected from Meshtastic core:
 * - call begin() once config is available
 * - call tick(now_ms) from scheduler
 * - call on_packet_rx(...) on irrigation port packets
 * - call on_sensor_sample(...) when new GNSS/IMU sample is available
 * - consume queued outbound packets via pop_next_mesh_tx(...)
 */
class IrrigationModule {
 public:
  IrrigationModule();

  bool begin(const IrrigationConfig &config);
  void tick(uint64_t now_ms);

  void on_sensor_sample(const IrrigationSensorSample &sample, uint64_t now_ms);
  bool on_packet_rx(const String &payload_json);

  bool pop_next_mesh_tx(String &payload_json, uint8_t &hop_limit);
  bool has_pending_mesh_tx() const;

 private:
  struct PendingMeshTx {
    String payload_json;
    uint8_t hop_limit;
  };

  void maybe_enqueue_endpoint_event(uint64_t now_ms);
  void flush_gateway_backlog(uint64_t now_ms, size_t max_messages);
  uint8_t compute_hop_limit(const IrrigationSensorSample &sample) const;
  IrrigationTelemetryEvent build_event(uint64_t now_ms);

  IrrigationConfig config_;
  bool started_;
  bool has_sample_;
  IrrigationSensorSample last_sample_;
  float last_heading_deg_;
  uint64_t last_endpoint_publish_ms_;
  uint32_t sequence_;

  IrrigationMotionDetector motion_detector_;
  IrrigationGatewayStore gateway_store_;
  IrrigationMqttUplink mqtt_uplink_;

  std::vector<PendingMeshTx> pending_mesh_tx_;
  uint64_t last_gateway_flush_ms_;
};
