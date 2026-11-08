#include "IrrigationModule.h"

#include "IrrigationCommandAuth.h"

#include "DebugConfiguration.h"

#include <cmath>
#include <utility>

#ifdef ARCH_ESP32
#include <esp_system.h>
#endif

namespace {
constexpr uint64_t k_gateway_flush_interval_ms = 2000;
constexpr float k_border_hop_hdop_threshold = 3.5f;
constexpr size_t k_gateway_rx_queue_limit = 16;
constexpr uint32_t k_min_position_interval_sec = 60;
constexpr uint32_t k_max_position_interval_sec = 24 * 60 * 60;
constexpr uint64_t k_min_valid_epoch_ms = 1577836800000ULL;

const char *profile_to_cstr(IrrigationNodeProfile profile) {
  switch (profile) {
    case IrrigationNodeProfile::ENDPOINT_POD:
      return "ENDPOINT_POD";
    case IrrigationNodeProfile::BASIC_TRACKER:
      return "BASIC_TRACKER";
    case IrrigationNodeProfile::VALVE_ACTUATOR:
      return "VALVE_ACTUATOR";
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
    case IrrigationPublishReason::PERIODIC_POSITION:
      return "PERIODIC_POSITION";
    case IrrigationPublishReason::VALVE_STATE:
      return "VALVE_STATE";
    case IrrigationPublishReason::COMMAND_ACK:
      return "COMMAND_ACK";
    case IrrigationPublishReason::COMMAND_REJECTED:
      return "COMMAND_REJECTED";
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
      last_profile_publish_ms_(0),
      valve_opened_ms_(0),
      valve_open_duration_sec_(0),
      valve_open_(false),
      last_command_accepted_(false),
      config_update_pending_(false),
      command_store_ready_(false),
      motion_cycle_start_ms_(0),
      settle_confirm_due_ms_(0),
      sequence_(0),
      boot_nonce_(0),
      stable_fix_sample_count_(0),
      endpoint_state_(EndpointLifecycleState::MONITORING),
      motion_detector_(120, 0.2f),
      last_gateway_flush_ms_(0),
      last_command_issued_at_utc_ms_(0) {}

bool IrrigationModule::begin(const IrrigationConfig &config) {
  config_ = config;
  if (config_.heartbeat_interval_sec == 0) {
    config_.heartbeat_interval_sec = 12 * 60 * 60;
  }
  if (config_.position_interval_sec < k_min_position_interval_sec ||
      config_.position_interval_sec > k_max_position_interval_sec) {
    config_.position_interval_sec = 15 * 60;
  }
  if (config_.valve_max_open_sec == 0) {
    config_.valve_max_open_sec = 60 * 60;
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
  last_profile_publish_ms_ = 0;
  valve_opened_ms_ = 0;
  valve_open_duration_sec_ = 0;
  valve_open_ = false;
  last_command_issued_at_utc_ms_ = 0;
  last_command_id_ = "";
  last_command_accepted_ = false;
  config_update_pending_ = false;
  motion_cycle_start_ms_ = 0;
  settle_confirm_due_ms_ = 0;
  sequence_ = 0;
#ifdef ARCH_ESP32
  boot_nonce_ = (static_cast<uint64_t>(esp_random()) << 32) | esp_random();
#else
  boot_nonce_ = (static_cast<uint64_t>(micros()) << 32) | millis();
#endif
  command_store_ready_ = false;
  stable_fix_sample_count_ = 0;
  endpoint_state_ = EndpointLifecycleState::MONITORING;
  pending_mesh_tx_.clear();
  pending_gateway_rx_.clear();

  if (config_.node_profile == IrrigationNodeProfile::VALVE_ACTUATOR) {
    if (config_.valve_control_pin >= 0) {
      set_valve_output(false);
      pinMode(config_.valve_control_pin, OUTPUT);
    } else {
      LOG_ERROR("Irrigation valve disabled: valve_control_pin is not configured");
    }
  }

  if (!started_) {
    return false;
  }
  if (config_.node_profile == IrrigationNodeProfile::VALVE_ACTUATOR ||
      config_.node_profile == IrrigationNodeProfile::BASIC_TRACKER) {
    command_store_ready_ = command_store_.load(last_command_issued_at_utc_ms_, last_command_id_);
    if (!command_store_ready_) {
      LOG_ERROR("Irrigation command checkpoint unavailable; opening and configuration commands disabled");
    }
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

  if (config_.node_profile == IrrigationNodeProfile::BASIC_TRACKER) {
    tick_basic_tracker(now_ms);
    return;
  }

  if (config_.node_profile == IrrigationNodeProfile::VALVE_ACTUATOR) {
    tick_valve_actuator(now_ms);
    return;
  }

  if (config_.node_profile == IrrigationNodeProfile::GATEWAY_CENTRAL) {
    mqtt_uplink_.tick(now_ms);
    process_gateway_commands(4);
    process_gateway_ingress(now_ms, 4);
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

bool IrrigationModule::on_packet_rx(const String &payload_json, uint64_t now_ms) {
  if (!started_) {
    return false;
  }

  if (config_.node_profile == IrrigationNodeProfile::GATEWAY_CENTRAL) {
    if (pending_gateway_rx_.size() >= k_gateway_rx_queue_limit) {
      LOG_WARN("Irrigation gateway rx queue full: queued=%u dropped_bytes=%u",
               pending_gateway_rx_.size(), payload_json.length());
      return false;
    }

    PendingGatewayRx pending_rx;
    pending_rx.payload_json = payload_json;
    pending_gateway_rx_.push_back(std::move(pending_rx));
    return true;
  }

  if (config_.node_profile == IrrigationNodeProfile::BASIC_TRACKER ||
      config_.node_profile == IrrigationNodeProfile::VALVE_ACTUATOR) {
    IrrigationCommand command{};
    if (irrigation_command_from_json(payload_json, command) && command.tracker_id == config_.tracker_id) {
      return handle_command(command, now_ms);
    }
  }
  return false;
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

bool IrrigationModule::pop_config_update(IrrigationConfig &config) {
  if (!config_update_pending_) {
    return false;
  }
  config = config_;
  config_update_pending_ = false;
  return true;
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

void IrrigationModule::tick_basic_tracker(uint64_t now_ms) {
  const uint64_t interval_ms = static_cast<uint64_t>(config_.position_interval_sec) * 1000ULL;
  if (last_profile_publish_ms_ != 0 && now_ms - last_profile_publish_ms_ < interval_ms) {
    return;
  }
  if (enqueue_profile_status(IrrigationPublishReason::PERIODIC_POSITION, now_ms)) {
    last_profile_publish_ms_ = now_ms;
  }
}

void IrrigationModule::tick_valve_actuator(uint64_t now_ms) {
  if (valve_open_ && valve_open_duration_sec_ > 0 &&
      now_ms - valve_opened_ms_ >= static_cast<uint64_t>(valve_open_duration_sec_) * 1000ULL) {
    set_valve_output(false);
    valve_open_duration_sec_ = 0;
    enqueue_profile_status(IrrigationPublishReason::VALVE_STATE, now_ms);
    last_profile_publish_ms_ = now_ms;
    LOG_WARN("Irrigation valve auto-closed: max duration reached");
  }

  const uint64_t interval_ms = static_cast<uint64_t>(config_.heartbeat_interval_sec) * 1000ULL;
  if (last_profile_publish_ms_ == 0 || now_ms - last_profile_publish_ms_ >= interval_ms) {
    if (enqueue_profile_status(IrrigationPublishReason::VALVE_STATE, now_ms)) {
      last_profile_publish_ms_ = now_ms;
    }
  }
}

bool IrrigationModule::handle_command(const IrrigationCommand &command, uint64_t now_ms) {
  if (!irrigation_command_signature_is_valid(command, config_.command_hmac_key)) {
    LOG_WARN("Irrigation command authentication failed: command_id=%s", command.command_id.c_str());
    enqueue_command_result(command.command_id, false, now_ms);
    return false;
  }

  String acknowledgement;
  if (!build_command_result_payload(command.command_id, false, now_ms, acknowledgement) ||
      !build_command_result_payload(command.command_id, true, now_ms, acknowledgement, &command)) {
    LOG_WARN("Irrigation command rejected: acknowledgement cannot fit radio payload");
    return false;
  }

  if (command.command_id == last_command_id_) {
    enqueue_command_result(command.command_id, last_command_accepted_, now_ms);
    return last_command_accepted_;
  }

  const uint64_t device_utc_ms = has_sample_ ? last_sample_.ts_utc_ms : 0;
  const bool valid_time = device_utc_ms >= k_min_valid_epoch_ms &&
                          command.issued_at_utc_ms >= k_min_valid_epoch_ms &&
                          command.expires_at_utc_ms >= command.issued_at_utc_ms &&
                          device_utc_ms <= command.expires_at_utc_ms &&
                          device_utc_ms + 60000ULL >= command.issued_at_utc_ms;
  const bool command_order_valid = command.issued_at_utc_ms > last_command_issued_at_utc_ms_;
  const bool checkpoint_required = valid_time && command_order_valid && command_store_ready_ &&
      ((command.action == IrrigationCommandAction::SET_POSITION_INTERVAL &&
        config_.node_profile == IrrigationNodeProfile::BASIC_TRACKER &&
        command.position_interval_sec >= k_min_position_interval_sec &&
        command.position_interval_sec <= k_max_position_interval_sec) ||
       (command.action == IrrigationCommandAction::SET_VALVE &&
        config_.node_profile == IrrigationNodeProfile::VALVE_ACTUATOR &&
        (!command.valve_open || (config_.valve_control_pin >= 0 && command.duration_sec > 0 &&
                                command.duration_sec <= config_.valve_max_open_sec))));
  if (checkpoint_required) {
    command_store_ready_ = command_store_.save(command.issued_at_utc_ms, command.command_id);
    if (!command_store_ready_) {
      LOG_ERROR("Irrigation command checkpoint write failed; opening and configuration commands disabled");
    }
  }
  const bool checkpoint_saved = checkpoint_required && command_store_ready_;

  bool accepted = false;
  if (command.action == IrrigationCommandAction::SET_POSITION_INTERVAL &&
      config_.node_profile == IrrigationNodeProfile::BASIC_TRACKER &&
      checkpoint_saved &&
      command.position_interval_sec >= k_min_position_interval_sec &&
      command.position_interval_sec <= k_max_position_interval_sec) {
    config_.position_interval_sec = command.position_interval_sec;
    config_update_pending_ = true;
    last_profile_publish_ms_ = 0;
    accepted = true;
  } else if (command.action == IrrigationCommandAction::SET_VALVE &&
             config_.node_profile == IrrigationNodeProfile::VALVE_ACTUATOR) {
    if (!command.valve_open) {
      set_valve_output(false);
      valve_open_duration_sec_ = 0;
      accepted = true;
    } else {
      const bool valid_duration = command.duration_sec > 0 && command.duration_sec <= config_.valve_max_open_sec;
      if (config_.valve_control_pin >= 0 && checkpoint_saved && valid_duration) {
        set_valve_output(true);
        valve_opened_ms_ = now_ms;
        valve_open_duration_sec_ = command.duration_sec;
        accepted = true;
      }
    }
  }

  if (accepted && checkpoint_saved) {
    last_command_id_ = command.command_id;
    if (command.issued_at_utc_ms > last_command_issued_at_utc_ms_) {
      last_command_issued_at_utc_ms_ = command.issued_at_utc_ms;
    }
    last_command_accepted_ = true;
  }
  if (!accepted) {
    LOG_WARN("Irrigation command rejection checks: command_id=%s time_valid=%d order_valid=%d checkpoint_saved=%d "
             "pin=%d duration_sec=%u max_open_sec=%u",
             command.command_id.c_str(), valid_time ? 1 : 0, command_order_valid ? 1 : 0,
             checkpoint_saved ? 1 : 0, static_cast<int>(config_.valve_control_pin),
             static_cast<unsigned int>(command.duration_sec), static_cast<unsigned int>(config_.valve_max_open_sec));
  }
  enqueue_command_result(command.command_id, accepted, now_ms);
  LOG_WARN("Irrigation command handled: command_id=%s profile=%s accepted=%d",
           command.command_id.c_str(), profile_to_cstr(config_.node_profile), accepted ? 1 : 0);
  return accepted;
}

bool IrrigationModule::build_command_result_payload(
    const String &command_id, bool accepted, uint64_t now_ms, String &payload_json,
    const IrrigationCommand *prospective) {
  IrrigationSensorSample sample{};
  if (has_sample_) {
    sample = last_sample_;
  }
  const bool usable_fix = sample_has_usable_fix(sample);
  IrrigationTelemetryEvent event = build_event(
      accepted ? IrrigationPublishReason::COMMAND_ACK : IrrigationPublishReason::COMMAND_REJECTED,
      sample,
      !usable_fix,
      usable_fix,
      usable_fix && sample.heading_valid,
      usable_fix ? IrrigationTimeQuality::GNSS : IrrigationTimeQuality::RTC,
      now_ms);
  event.command_id = command_id;
  if (prospective != nullptr) {
    if (event.has_valve_state && prospective->action == IrrigationCommandAction::SET_VALVE) {
      event.valve_open = prospective->valve_open;
    }
    if (event.has_position_interval && prospective->action == IrrigationCommandAction::SET_POSITION_INTERVAL) {
      event.position_interval_sec = prospective->position_interval_sec;
    }
  }

  return irrigation_event_to_json(event, payload_json);
}

bool IrrigationModule::enqueue_command_result(const String &command_id, bool accepted, uint64_t now_ms) {
  String payload_json;
  if (!build_command_result_payload(command_id, accepted, now_ms, payload_json)) {
    LOG_WARN("Irrigation command result cannot fit radio payload");
    return false;
  }
  PendingMeshTx tx_item;
  tx_item.payload_json = payload_json;
  tx_item.hop_limit = has_sample_ ? compute_hop_limit(last_sample_) : config_.max_hop_override;
  pending_mesh_tx_.push_back(tx_item);
  return true;
}

bool IrrigationModule::enqueue_profile_status(IrrigationPublishReason publish_reason, uint64_t now_ms) {
  if (has_sample_ && sample_has_usable_fix(last_sample_)) {
    return enqueue_event_from_sample(
        publish_reason,
        last_sample_,
        false,
        true,
        last_sample_.heading_valid,
        IrrigationTimeQuality::GNSS,
        now_ms);
  }
  return enqueue_event_from_last_good_fix(publish_reason, now_ms);
}

void IrrigationModule::set_valve_output(bool open) {
  if (config_.valve_control_pin >= 0) {
    const bool output_high = open == config_.valve_active_high;
    digitalWrite(config_.valve_control_pin, output_high ? HIGH : LOW);
  }
  valve_open_ = open && config_.valve_control_pin >= 0;
}

void IrrigationModule::process_gateway_commands(size_t max_messages) {
  size_t processed_count = 0;
  String topic;
  String payload_json;
  while (processed_count < max_messages && mqtt_uplink_.pop_received_command(topic, payload_json)) {
    if (payload_json.length() > k_irrigation_mesh_payload_max) {
      LOG_WARN("Irrigation gateway command dropped: payload exceeds radio capacity");
      processed_count++;
      continue;
    }
    IrrigationCommand command{};
    const String prefix = "farm/" + config_.farm_id + "/nodes/";
    const int target_start = prefix.length();
    const int target_end = topic.lastIndexOf("/commands");
    const String topic_target = target_end > target_start ? topic.substring(target_start, target_end) : "";
    if (!irrigation_command_from_json(payload_json, command) || topic_target != command.tracker_id) {
      LOG_WARN("Irrigation gateway command dropped: invalid contract topic=%s", topic.c_str());
      processed_count++;
      continue;
    }
    if (pending_mesh_tx_.size() >= k_gateway_rx_queue_limit) {
      LOG_WARN("Irrigation gateway command dropped: mesh tx queue full command_id=%s", command.command_id.c_str());
      processed_count++;
      continue;
    }
    PendingMeshTx tx_item;
    tx_item.payload_json = payload_json;
    tx_item.hop_limit = config_.max_hop_override;
    pending_mesh_tx_.push_back(tx_item);
    LOG_INFO("Irrigation gateway command queued for mesh: command_id=%s target=%s",
             command.command_id.c_str(), command.tracker_id.c_str());
    processed_count++;
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
    LOG_WARN("Irrigation telemetry rejected: payload cannot fit radio capacity");
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

void IrrigationModule::process_gateway_ingress(uint64_t now_ms, size_t max_messages) {
  size_t processed_count = 0;
  while (processed_count < max_messages && !pending_gateway_rx_.empty()) {
    const PendingGatewayRx &next_item = pending_gateway_rx_.front();

    IrrigationTelemetryEvent event;
    if (!irrigation_event_from_json(next_item.payload_json, event)) {
      const String preview = next_item.payload_json.substring(0, 96);
      LOG_WARN("Irrigation gateway drop: invalid payload_json bytes=%u preview=%s",
               next_item.payload_json.length(), preview.c_str());
      pending_gateway_rx_.erase(pending_gateway_rx_.begin());
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
      if (mqtt_uplink_.publish_payload(topic, next_item.payload_json, 1, now_ms)) {
        LOG_WARN("Irrigation gateway fallback publish succeeded: msg_id=%s topic=%s",
                 event.msg_id.c_str(), topic.c_str());
      } else {
        LOG_WARN("Irrigation gateway fallback publish failed: msg_id=%s topic=%s",
                 event.msg_id.c_str(), topic.c_str());
        // Keep the record until a later tick can reconnect with no decoded event alive.
        break;
      }
    }

    pending_gateway_rx_.erase(pending_gateway_rx_.begin());
    processed_count++;
  }
}

void IrrigationModule::flush_gateway_backlog(uint64_t now_ms, size_t max_messages) {
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
    if (!mqtt_uplink_.publish_payload(topic, payload_json, 1, now_ms)) {
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
  event.msg_id = irrigation_message_id(config_.tracker_id, event_time_ms, now_ms, sequence_++, boot_nonce_);
  event.ts_utc_ms = event_time_ms;
  event.tracker_id = config_.tracker_id;
  event.line_id = config_.line_id;
  event.endpoint_role = config_.endpoint_role;
  event.node_profile = config_.node_profile;
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
  event.command_id = "";
  event.has_valve_state = config_.node_profile == IrrigationNodeProfile::VALVE_ACTUATOR;
  event.valve_open = valve_open_;
  event.has_position_interval = config_.node_profile == IrrigationNodeProfile::BASIC_TRACKER;
  event.position_interval_sec = config_.position_interval_sec;
  return event;
}
