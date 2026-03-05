#include "IrrigationModule.h"

#include <cmath>

namespace {
constexpr uint64_t k_gateway_flush_interval_ms = 2000;
constexpr float k_border_hop_hdop_threshold = 3.5f;
}  // namespace

IrrigationModule::IrrigationModule()
    : started_(false),
      has_sample_(false),
      last_heading_deg_(0.0f),
      last_endpoint_publish_ms_(0),
      sequence_(0),
      motion_detector_(120),
      last_gateway_flush_ms_(0) {}

bool IrrigationModule::begin(const IrrigationConfig &config) {
  config_ = config;
  motion_detector_ = IrrigationMotionDetector(config_.moving_hold_sec);
  started_ = config_.enabled;
  has_sample_ = false;
  last_heading_deg_ = 0.0f;
  last_endpoint_publish_ms_ = 0;
  sequence_ = 0;
  pending_mesh_tx_.clear();

  if (!started_) {
    return false;
  }

  if (config_.node_profile == IrrigationNodeProfile::GATEWAY_CENTRAL) {
    gateway_store_.begin("/irrigation_queue.log", config_.gateway_buffer_max_records);
    mqtt_uplink_.begin(config_);
  }

  return true;
}

void IrrigationModule::tick(uint64_t now_ms) {
  if (!started_) {
    return;
  }

  if (config_.node_profile == IrrigationNodeProfile::ENDPOINT_POD) {
    maybe_enqueue_endpoint_event(now_ms);
    return;
  }

  if (config_.node_profile == IrrigationNodeProfile::GATEWAY_CENTRAL) {
    mqtt_uplink_.tick(now_ms);
    if (now_ms - last_gateway_flush_ms_ >= k_gateway_flush_interval_ms) {
      last_gateway_flush_ms_ = now_ms;
      flush_gateway_backlog(now_ms, 20);
    }
  }
}

void IrrigationModule::on_sensor_sample(const IrrigationSensorSample &sample, uint64_t now_ms) {
  if (!started_) {
    return;
  }

  last_sample_ = sample;
  has_sample_ = true;
  const float heading_delta_deg = fabsf(sample.heading_deg - last_heading_deg_);
  last_heading_deg_ = sample.heading_deg;

  motion_detector_.update(sample.accel_rms_g, sample.speed_mps, heading_delta_deg, sample.gps_fix, now_ms);
}

bool IrrigationModule::on_packet_rx(const String &payload_json) {
  if (!started_ || config_.node_profile != IrrigationNodeProfile::GATEWAY_CENTRAL) {
    return false;
  }

  IrrigationTelemetryEvent event;
  if (!irrigation_event_from_json(payload_json, event)) {
    return false;
  }

  return gateway_store_.append(payload_json);
}

bool IrrigationModule::pop_next_mesh_tx(String &payload_json, uint8_t &hop_limit) {
  if (pending_mesh_tx_.empty()) {
    return false;
  }

  const PendingMeshTx next_item = pending_mesh_tx_.front();
  pending_mesh_tx_.erase(pending_mesh_tx_.begin());
  payload_json = next_item.payload_json;
  hop_limit = next_item.hop_limit;
  return true;
}

bool IrrigationModule::has_pending_mesh_tx() const {
  return !pending_mesh_tx_.empty();
}

void IrrigationModule::maybe_enqueue_endpoint_event(uint64_t now_ms) {
  if (!has_sample_) {
    return;
  }

  const IrrigationMotionState state = motion_detector_.motion_state();
  const uint32_t interval_sec =
      state == IrrigationMotionState::MOVING ? config_.moving_interval_sec : config_.stationary_interval_sec;

  const bool due_by_interval =
      last_endpoint_publish_ms_ == 0 || now_ms - last_endpoint_publish_ms_ >= static_cast<uint64_t>(interval_sec) * 1000;
  const bool due_by_transition = motion_detector_.transitioned_to_moving();
  if (!due_by_interval && !due_by_transition) {
    return;
  }

  IrrigationTelemetryEvent event = build_event(now_ms);
  event.motion_state = state;
  event.hop_count = compute_hop_limit(last_sample_);

  String payload_json;
  if (!irrigation_event_to_json(event, payload_json)) {
    return;
  }

  PendingMeshTx tx_item;
  tx_item.payload_json = payload_json;
  tx_item.hop_limit = event.hop_count;
  pending_mesh_tx_.push_back(tx_item);
  last_endpoint_publish_ms_ = now_ms;
}

void IrrigationModule::flush_gateway_backlog(uint64_t now_ms, size_t max_messages) {
  (void)now_ms;
  if (!mqtt_uplink_.is_connected()) {
    return;
  }

  size_t sent_count = 0;
  while (sent_count < max_messages) {
    String payload_json;
    if (!gateway_store_.peek_next(payload_json)) {
      break;
    }

    IrrigationTelemetryEvent event;
    if (!irrigation_event_from_json(payload_json, event)) {
      gateway_store_.pop_next();
      continue;
    }

    const String topic = build_irrigation_mqtt_topic(config_.farm_id, event.line_id, event.tracker_id);
    if (!mqtt_uplink_.publish_payload(topic, payload_json, 1)) {
      break;
    }

    gateway_store_.pop_next();
    sent_count++;
  }
}

uint8_t IrrigationModule::compute_hop_limit(const IrrigationSensorSample &sample) const {
  if (!sample.gps_fix || sample.hdop > k_border_hop_hdop_threshold) {
    return config_.max_hop_override;
  }
  return config_.default_hop_limit;
}

IrrigationTelemetryEvent IrrigationModule::build_event(uint64_t now_ms) {
  IrrigationTelemetryEvent event;
  event.msg_id = config_.tracker_id + "-" + String(now_ms) + "-" + String(sequence_++);
  event.ts_utc_ms = now_ms;
  event.tracker_id = config_.tracker_id;
  event.line_id = config_.line_id;
  event.endpoint_role = config_.endpoint_role;
  event.lat = last_sample_.lat;
  event.lon = last_sample_.lon;
  event.hdop = last_sample_.hdop;
  event.speed_mps = last_sample_.speed_mps;
  event.heading_deg = last_sample_.heading_deg;
  event.motion_state = IrrigationMotionState::STATIONARY;
  event.battery_mv = last_sample_.battery_mv;
  event.fix = last_sample_.gps_fix;
  event.hop_count = config_.default_hop_limit;
  return event;
}
