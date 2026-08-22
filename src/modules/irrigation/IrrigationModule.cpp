#include "IrrigationModule.h"

#include "DebugConfiguration.h"

#include <cmath>

namespace {
constexpr uint64_t k_gateway_flush_interval_ms = 2000;
constexpr float k_border_hop_hdop_threshold = 3.5f;
constexpr size_t k_gateway_rx_queue_limit = 16;

const char *profile_to_cstr(IrrigationNodeProfile profile) {
  switch (profile) {
    case IrrigationNodeProfile::ENDPOINT_POD:
      return "ENDPOINT_POD";
    case IrrigationNodeProfile::RELAY_FIXED:
      return "RELAY_FIXED";
    case IrrigationNodeProfile::GATEWAY_CENTRAL:
      return "GATEWAY_CENTRAL";
    default:
      return "UNKNOWN";
  }
}

const char *motion_state_to_cstr(IrrigationMotionState state) {
  switch (state) {
    case IrrigationMotionState::STATIONARY:
      return "STATIONARY";
    case IrrigationMotionState::MOVING:
      return "MOVING";
    default:
      return "UNKNOWN";
  }
}

const char *publish_reason_to_cstr(IrrigationPublishReason publish_reason) {
  switch (publish_reason) {
    case IrrigationPublishReason::HEARTBEAT:
      return "HEARTBEAT";
    case IrrigationPublishReason::HEARTBEAT_NO_FIX:
      return "HEARTBEAT_NO_FIX";
    case IrrigationPublishReason::POST_MOVE_FIX:
      return "POST_MOVE_FIX";
    case IrrigationPublishReason::SETTLE_CONFIRM:
      return "SETTLE_CONFIRM";
    case IrrigationPublishReason::MOTION_DETECTED_NO_FIX:
      return "MOTION_DETECTED_NO_FIX";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

IrrigationModule::IrrigationModule()
    : started_(false),
      has_sample_(false),
      has_last_good_fix_(false),
      last_heading_deg_(0.0f),
      last_good_fix_ms_(0),
      last_heartbeat_publish_ms_(0),
      last_endpoint_publish_ms_(0),
      motion_cycle_start_ms_(0),
      settle_confirm_due_ms_(0),
      sequence_(0),
      stable_fix_sample_count_(0),
      endpoint_state_(EndpointLifecycleState::MONITORING),
      motion_detector_(120, 0.2f),
      last_gateway_flush_ms_(0) {}

bool IrrigationModule::begin(const IrrigationConfig &config) {
  config_ = config;
  if (config_.heartbeat_interval_sec == 0) {
    config_.heartbeat_interval_sec = 12 * 60 * 60;
  }
  if (config_.moving_hold_sec == 0) {
    config_.moving_hold_sec = 120;
  }
  if (config_.gnss_fix_timeout_sec == 0) {
    config_.gnss_fix_timeout_sec = 10 * 60;
  }
  if (config_.stable_fix_min_samples == 0) {
    config_.stable_fix_min_samples = 1;
  }
  motion_detector_ = IrrigationMotionDetector(config_.moving_hold_sec, config_.motion_start_threshold_mg / 1000.0f);
  started_ = config_.enabled;
  has_sample_ = false;
  has_last_good_fix_ = false;
  last_heading_deg_ = 0.0f;
  last_good_fix_ms_ = 0;
  last_heartbeat_publish_ms_ = 0;
  last_endpoint_publish_ms_ = 0;
  motion_cycle_start_ms_ = 0;
  settle_confirm_due_ms_ = 0;
  sequence_ = 0;
  stable_fix_sample_count_ = 0;
  endpoint_state_ = EndpointLifecycleState::MONITORING;
  pending_mesh_tx_.clear();
  pending_gateway_rx_.clear();

  if (!started_) {
    return false;
  }

  if (config_.node_profile == IrrigationNodeProfile::GATEWAY_CENTRAL) {
    const bool queue_ok = gateway_store_.begin("/irrigation_queue.log", config_.gateway_buffer_max_records);
    const bool mqtt_ok = mqtt_uplink_.begin(config_);
    LOG_INFO("Irrigation gateway services: queue=%d mqtt=%d", queue_ok ? 1 : 0, mqtt_ok ? 1 : 0);
  }

  return true;
}

void IrrigationModule::tick(uint64_t now_ms) {
  if (!started_) {
    return;
  }

  if (config_.node_profile == IrrigationNodeProfile::ENDPOINT_POD) {
    tick_endpoint(now_ms);
    return;
  }

  if (config_.node_profile == IrrigationNodeProfile::GATEWAY_CENTRAL) {
    mqtt_uplink_.tick(now_ms);
    process_gateway_ingress(4);
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
  if (sample_has_usable_fix(sample)) {
    remember_last_good_fix(sample, now_ms);
  }
  const float heading_delta_deg = fabsf(sample.heading_deg - last_heading_deg_);
  last_heading_deg_ = sample.heading_deg;

  motion_detector_.update(sample.accel_rms_g, sample.speed_mps, heading_delta_deg, sample.gps_fix, now_ms);
}

bool IrrigationModule::on_packet_rx(const String &payload_json) {
  if (!started_ || config_.node_profile != IrrigationNodeProfile::GATEWAY_CENTRAL) {
    return false;
  }

  if (pending_gateway_rx_.size() >= k_gateway_rx_queue_limit) {
    LOG_WARN("Irrigation gateway rx queue full: queued=%u dropped_bytes=%u",
             pending_gateway_rx_.size(), payload_json.length());
    return false;
  }

  PendingGatewayRx pending_rx;
  pending_rx.payload_json = payload_json;
  pending_gateway_rx_.push_back(pending_rx);
  return true;
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
  tick_endpoint(now_ms);
}

void IrrigationModule::tick_endpoint(uint64_t now_ms) {
  if (!has_sample_) {
    return;
  }

  if (motion_detector_.transitioned_to_moving()) {
    endpoint_state_ = EndpointLifecycleState::WAIT_FOR_SETTLE;
    motion_cycle_start_ms_ = now_ms;
    settle_confirm_due_ms_ = 0;
    stable_fix_sample_count_ = 0;
    LOG_INFO("Irrigation endpoint motion start: threshold_mg=%u", config_.motion_start_threshold_mg);
  }

  switch (endpoint_state_) {
    case EndpointLifecycleState::MONITORING:
      maybe_publish_heartbeat(now_ms);
      break;
    case EndpointLifecycleState::WAIT_FOR_SETTLE:
      if (motion_detector_.transitioned_to_stationary()) {
        endpoint_state_ = EndpointLifecycleState::ACQUIRE_FINAL_FIX;
        stable_fix_sample_count_ = 0;
        LOG_INFO("Irrigation endpoint settle detected: quiet_sec=%u", config_.moving_hold_sec);
      }
      break;
    case EndpointLifecycleState::ACQUIRE_FINAL_FIX:
      process_post_motion_fix(now_ms);
      break;
    case EndpointLifecycleState::WAIT_SETTLE_CONFIRM:
      if (motion_detector_.transitioned_to_moving()) {
        endpoint_state_ = EndpointLifecycleState::WAIT_FOR_SETTLE;
        motion_cycle_start_ms_ = now_ms;
        settle_confirm_due_ms_ = 0;
        stable_fix_sample_count_ = 0;
        break;
      }
      if (settle_confirm_due_ms_ != 0 && now_ms >= settle_confirm_due_ms_) {
        const bool published = sample_has_stable_fix(last_sample_)
                                   ? enqueue_event_from_sample(
                                         IrrigationPublishReason::SETTLE_CONFIRM,
                                         last_sample_,
                                         false,
                                         true,
                                         last_sample_.heading_valid,
                                         IrrigationTimeQuality::GNSS,
                                         now_ms)
                                   : enqueue_event_from_last_good_fix(IrrigationPublishReason::SETTLE_CONFIRM, now_ms);
        if (published) {
          endpoint_state_ = EndpointLifecycleState::MONITORING;
          motion_cycle_start_ms_ = 0;
          settle_confirm_due_ms_ = 0;
        }
      }
      break;
  }
}

void IrrigationModule::maybe_publish_heartbeat(uint64_t now_ms) {
  const uint64_t heartbeat_interval_ms = static_cast<uint64_t>(config_.heartbeat_interval_sec) * 1000ULL;
  if (last_heartbeat_publish_ms_ != 0 && now_ms - last_heartbeat_publish_ms_ < heartbeat_interval_ms) {
    return;
  }

  if (sample_has_stable_fix(last_sample_)) {
    enqueue_event_from_sample(
        IrrigationPublishReason::HEARTBEAT,
        last_sample_,
        false,
        true,
        last_sample_.heading_valid && motion_detector_.motion_state() == IrrigationMotionState::STATIONARY,
        IrrigationTimeQuality::GNSS,
        now_ms);
    return;
  }

  enqueue_event_from_last_good_fix(IrrigationPublishReason::HEARTBEAT_NO_FIX, now_ms);
}

void IrrigationModule::process_post_motion_fix(uint64_t now_ms) {
  if (sample_has_stable_fix(last_sample_)) {
    if (stable_fix_sample_count_ < UINT8_MAX) {
      stable_fix_sample_count_++;
    }
  } else {
    stable_fix_sample_count_ = 0;
  }

  if (stable_fix_sample_count_ >= config_.stable_fix_min_samples) {
    if (enqueue_event_from_sample(
            IrrigationPublishReason::POST_MOVE_FIX,
            last_sample_,
            false,
            true,
            last_sample_.heading_valid,
            IrrigationTimeQuality::GNSS,
            now_ms)) {
      endpoint_state_ = EndpointLifecycleState::WAIT_SETTLE_CONFIRM;
      motion_cycle_start_ms_ = 0;
      settle_confirm_due_ms_ = now_ms + static_cast<uint64_t>(config_.settle_confirm_delay_sec) * 1000ULL;
      stable_fix_sample_count_ = 0;
    }
    return;
  }

  const uint64_t fix_timeout_ms = static_cast<uint64_t>(config_.gnss_fix_timeout_sec) * 1000ULL;
  if (motion_cycle_start_ms_ != 0 && now_ms - motion_cycle_start_ms_ >= fix_timeout_ms) {
    if (enqueue_event_from_last_good_fix(IrrigationPublishReason::MOTION_DETECTED_NO_FIX, now_ms)) {
      endpoint_state_ = EndpointLifecycleState::MONITORING;
      motion_cycle_start_ms_ = 0;
      stable_fix_sample_count_ = 0;
      settle_confirm_due_ms_ = 0;
    }
  }
}

bool IrrigationModule::enqueue_event_from_sample(
    IrrigationPublishReason publish_reason,
    const IrrigationSensorSample &sample,
    bool stale,
    bool force_fix,
    bool heading_valid,
    IrrigationTimeQuality time_quality,
    uint64_t now_ms) {
  IrrigationTelemetryEvent event = build_event(
      publish_reason, sample, stale, force_fix, heading_valid, time_quality, now_ms);

  String payload_json;
  if (!irrigation_event_to_json(event, payload_json)) {
    return false;
  }

  PendingMeshTx tx_item;
  tx_item.payload_json = payload_json;
  tx_item.hop_limit = event.hop_count;
  pending_mesh_tx_.push_back(tx_item);
  last_endpoint_publish_ms_ = now_ms;
  last_heartbeat_publish_ms_ = now_ms;
  LOG_INFO("Irrigation endpoint queued telemetry: msg_id=%s reason=%s motion=%s hop_limit=%u bytes=%u stale=%u fix=%u",
           event.msg_id.c_str(), publish_reason_to_cstr(event.publish_reason), motion_state_to_cstr(event.motion_state),
           event.hop_count, payload_json.length(), event.stale ? 1 : 0, event.fix ? 1 : 0);
  return true;
}

bool IrrigationModule::enqueue_event_from_last_good_fix(IrrigationPublishReason publish_reason, uint64_t now_ms) {
  IrrigationSensorSample fallback_sample{};
  if (has_sample_) {
    fallback_sample = last_sample_;
  }

  if (!has_last_good_fix_) {
    fallback_sample.gps_fix = false;
    fallback_sample.lat = 0.0;
    fallback_sample.lon = 0.0;
    fallback_sample.hdop = 0.0f;
    fallback_sample.speed_mps = 0.0f;
    fallback_sample.heading_deg = 0.0f;
    fallback_sample.heading_valid = false;
    return enqueue_event_from_sample(
        publish_reason, fallback_sample, true, false, false, IrrigationTimeQuality::RTC, now_ms);
  }

  fallback_sample = last_good_fix_sample_;
  if (has_sample_) {
    fallback_sample.battery_mv = last_sample_.battery_mv;
    fallback_sample.ts_utc_ms = last_sample_.ts_utc_ms;
    fallback_sample.time_quality = last_sample_.time_quality;
  }
  fallback_sample.gps_fix = false;
  fallback_sample.heading_valid = false;
  return enqueue_event_from_sample(
      publish_reason, fallback_sample, true, false, false, IrrigationTimeQuality::RTC, now_ms);
}

bool IrrigationModule::sample_has_stable_fix(const IrrigationSensorSample &sample) const {
  return sample_has_usable_fix(sample);
}

bool IrrigationModule::sample_has_usable_fix(const IrrigationSensorSample &sample) const {
  return sample.gps_fix && sample.hdop > 0.0f && sample.hdop <= config_.stable_fix_max_hdop;
}

void IrrigationModule::remember_last_good_fix(const IrrigationSensorSample &sample, uint64_t now_ms) {
  last_good_fix_sample_ = sample;
  has_last_good_fix_ = true;
  last_good_fix_ms_ = now_ms;
}

void IrrigationModule::process_gateway_ingress(size_t max_messages) {
  size_t processed_count = 0;
  while (processed_count < max_messages && !pending_gateway_rx_.empty()) {
    const PendingGatewayRx next_item = pending_gateway_rx_.front();
    pending_gateway_rx_.erase(pending_gateway_rx_.begin());

    IrrigationTelemetryEvent event;
    if (!irrigation_event_from_json(next_item.payload_json, event)) {
      const String preview = next_item.payload_json.substring(0, 96);
      LOG_WARN("Irrigation gateway drop: invalid payload_json bytes=%u preview=%s",
               next_item.payload_json.length(), preview.c_str());
      processed_count++;
      continue;
    }

    const bool appended = gateway_store_.append(next_item.payload_json);
    if (appended) {
      LOG_INFO("Irrigation gateway queued: msg_id=%s tracker_id=%s line_id=%s bytes=%u",
               event.msg_id.c_str(), event.tracker_id.c_str(), event.line_id.c_str(), next_item.payload_json.length());
    } else {
      const String topic = build_irrigation_mqtt_topic(config_.farm_id, event.line_id, event.tracker_id);
      LOG_WARN("Irrigation gateway queue append failed: msg_id=%s topic=%s", event.msg_id.c_str(), topic.c_str());
      if (mqtt_uplink_.publish_payload(topic, next_item.payload_json, 1)) {
        LOG_WARN("Irrigation gateway fallback publish succeeded: msg_id=%s topic=%s",
                 event.msg_id.c_str(), topic.c_str());
      } else {
        LOG_WARN("Irrigation gateway fallback publish failed: msg_id=%s topic=%s",
                 event.msg_id.c_str(), topic.c_str());
      }
    }

    processed_count++;
  }
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
      LOG_WARN("Irrigation gateway backlog drop: invalid payload_json");
      gateway_store_.pop_next();
      continue;
    }

    const String topic = build_irrigation_mqtt_topic(config_.farm_id, event.line_id, event.tracker_id);
    if (!mqtt_uplink_.publish_payload(topic, payload_json, 1)) {
      LOG_WARN("Irrigation gateway publish blocked: msg_id=%s topic=%s", event.msg_id.c_str(), topic.c_str());
      break;
    }

    gateway_store_.pop_next();
    LOG_INFO("Irrigation gateway published: msg_id=%s topic=%s bytes=%u",
             event.msg_id.c_str(), topic.c_str(), payload_json.length());
    sent_count++;
  }

  if (sent_count > 0) {
    LOG_INFO("Irrigation gateway flush complete: sent=%u", static_cast<unsigned int>(sent_count));
  }
}

uint8_t IrrigationModule::compute_hop_limit(const IrrigationSensorSample &sample) const {
  if (!sample.gps_fix || sample.hdop > k_border_hop_hdop_threshold) {
    return config_.max_hop_override;
  }
  return config_.default_hop_limit;
}

IrrigationTelemetryEvent IrrigationModule::build_event(
    IrrigationPublishReason publish_reason,
    const IrrigationSensorSample &sample,
    bool stale,
    bool force_fix,
    bool heading_valid,
    IrrigationTimeQuality time_quality,
    uint64_t now_ms) {
  IrrigationTelemetryEvent event;
  const uint64_t event_time_ms = sample.ts_utc_ms;
  event.msg_id = config_.tracker_id + "-" + String(event_time_ms) + "-" + String(now_ms) + "-" + String(sequence_++);
  event.ts_utc_ms = event_time_ms;
  event.tracker_id = config_.tracker_id;
  event.line_id = config_.line_id;
  event.endpoint_role = config_.endpoint_role;
  event.publish_reason = publish_reason;
  event.lat = sample.lat;
  event.lon = sample.lon;
  event.hdop = sample.hdop;
  event.speed_mps = sample.speed_mps;
  event.heading_deg = heading_valid ? sample.heading_deg : 0.0f;
  event.heading_valid = heading_valid;
  event.motion_state = motion_detector_.motion_state();
  event.battery_mv = sample.battery_mv;
  event.fix = force_fix && sample.gps_fix;
  event.stale = stale;
  event.time_quality =
      sample.time_quality != IrrigationTimeQuality::UNKNOWN ? sample.time_quality : time_quality;
  event.hop_count = compute_hop_limit(sample);
  return event;
}
