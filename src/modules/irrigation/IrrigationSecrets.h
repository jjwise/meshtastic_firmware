#pragma once

/* Generated secrets are deliberately absent from source control. */
#if defined(IRRIGATION_ENABLE_MQTT_UPLINK) && __has_include("IrrigationGatewaySecrets.generated.h")
#include "IrrigationGatewaySecrets.generated.h"
#endif

#if (defined(IRRIGATION_PROFILE_BASIC_TRACKER) || defined(IRRIGATION_PROFILE_VALVE_ACTUATOR)) && \
    __has_include("IrrigationCommandSecrets.generated.h")
#include "IrrigationCommandSecrets.generated.h"
#endif

#ifndef IRRIGATION_MQTT_CA_CERT
#define IRRIGATION_MQTT_CA_CERT ""
#endif

#ifndef IRRIGATION_COMMAND_HMAC_KEY
#define IRRIGATION_COMMAND_HMAC_KEY ""
#endif

inline const char *irrigation_mqtt_ca_certificate() {
  return IRRIGATION_MQTT_CA_CERT;
}

inline const char *irrigation_command_hmac_key() {
  return IRRIGATION_COMMAND_HMAC_KEY;
}
