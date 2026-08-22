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
  enum class EndpointLifecycleState {
    MONITORING,
    WAIT_FOR_SETTLE,
    ACQUIRE_FINAL_FIX,
    WAIT_SETTLE_CONFIRM
  };

  struct PendingMeshTx {
    String payload_json;
    uint8_t hop_limit;
  };

  struct PendingGatewayRx {
    String payload_json;
  };

  void maybe_enqueue_endpoint_event(uint64_t now_ms);
  void tick_endpoint(uint64_t now_ms);
  void maybe_publish_heartbeat(uint64_t now_ms);
  void process_post_motion_fix(uint64_t now_ms);
  bool enqueue_event_from_sample(
      IrrigationPublishReason publish_reason,
      const IrrigationSensorSample &sample,
      bool stale,
      bool force_fix,
      bool heading_valid,
      IrrigationTimeQuality time_quality,
      uint64_t now_ms);
  bool enqueue_event_from_last_good_fix(IrrigationPublishReason publish_reason, uint64_t now_ms);
  bool sample_has_stable_fix(const IrrigationSensorSample &sample) const;
  bool sample_has_usable_fix(const IrrigationSensorSample &sample) const;
  void remember_last_good_fix(const IrrigationSensorSample &sample, uint64_t now_ms);
  void process_gateway_ingress(size_t max_messages);
  void flush_gateway_backlog(uint64_t now_ms, size_t max_messages);
  uint8_t compute_hop_limit(const IrrigationSensorSample &sample) const;
  IrrigationTelemetryEvent build_event(
      IrrigationPublishReason publish_reason,
      const IrrigationSensorSample &sample,
      bool stale,
      bool force_fix,
      bool heading_valid,
      IrrigationTimeQuality time_quality,
      uint64_t now_ms);

  IrrigationConfig config_;
  bool started_;
  bool has_sample_;
  bool has_last_good_fix_;
  IrrigationSensorSample last_sample_;
  IrrigationSensorSample last_good_fix_sample_;
  float last_heading_deg_;
  uint64_t last_good_fix_ms_;
  uint64_t last_heartbeat_publish_ms_;
  uint64_t last_endpoint_publish_ms_;
  uint64_t motion_cycle_start_ms_;
  uint64_t settle_confirm_due_ms_;
  uint32_t sequence_;
  uint8_t stable_fix_sample_count_;
  EndpointLifecycleState endpoint_state_;

  IrrigationMotionDetector motion_detector_;
  IrrigationGatewayStore gateway_store_;
  IrrigationMqttUplink mqtt_uplink_;

  std::vector<PendingMeshTx> pending_mesh_tx_;
  std::vector<PendingGatewayRx> pending_gateway_rx_;
  uint64_t last_gateway_flush_ms_;
};
