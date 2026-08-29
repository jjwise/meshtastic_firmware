#pragma once

#include <Arduino.h>

#include "IrrigationTypes.h"

String irrigation_command_signature(const IrrigationCommand &command, const String &hmac_key);
bool irrigation_command_signature_is_valid(const IrrigationCommand &command, const String &hmac_key);
