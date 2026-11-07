#pragma once

#include <Arduino.h>

#include "IrrigationTypes.h"

constexpr size_t k_irrigation_mesh_payload_max = 233;

String irrigation_message_id(const String &tracker_id, uint64_t timestamp, uint64_t uptime,
                             uint32_t sequence, uint64_t boot_nonce);

String irrigation_motion_state_to_string(IrrigationMotionState motion_state);
IrrigationMotionState irrigation_motion_state_from_string(const String &value);
String irrigation_publish_reason_to_string(IrrigationPublishReason publish_reason);
IrrigationPublishReason irrigation_publish_reason_from_string(const String &value);
String irrigation_time_quality_to_string(IrrigationTimeQuality time_quality);
IrrigationTimeQuality irrigation_time_quality_from_string(const String &value);

bool irrigation_event_to_json(const IrrigationTelemetryEvent &event, String &output_json);
bool irrigation_event_from_json(const String &payload_json, IrrigationTelemetryEvent &event);
bool irrigation_command_from_json(const String &payload_json, IrrigationCommand &command);

String build_irrigation_mqtt_topic(const String &farm_id, const String &line_id, const String &tracker_id);
String build_irrigation_command_topic(const String &farm_id, const String &tracker_id);
