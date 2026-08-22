#pragma once

#include <Arduino.h>

#include "IrrigationTypes.h"

String irrigation_motion_state_to_string(IrrigationMotionState motion_state);
IrrigationMotionState irrigation_motion_state_from_string(const String &value);
String irrigation_publish_reason_to_string(IrrigationPublishReason publish_reason);
IrrigationPublishReason irrigation_publish_reason_from_string(const String &value);
String irrigation_time_quality_to_string(IrrigationTimeQuality time_quality);
IrrigationTimeQuality irrigation_time_quality_from_string(const String &value);

bool irrigation_event_to_json(const IrrigationTelemetryEvent &event, String &output_json);
bool irrigation_event_from_json(const String &payload_json, IrrigationTelemetryEvent &event);

String build_irrigation_mqtt_topic(const String &farm_id, const String &line_id, const String &tracker_id);
