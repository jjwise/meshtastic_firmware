#pragma once

/*
 * The generated header is deliberately absent from source control. It contains
 * Wi-Fi/MQTT credentials and the public CA used to verify the home broker.
 */
#if __has_include("IrrigationGatewaySecrets.generated.h")
#include "IrrigationGatewaySecrets.generated.h"
#endif

#ifndef IRRIGATION_MQTT_CA_CERT
#define IRRIGATION_MQTT_CA_CERT ""
#endif

inline const char *irrigation_mqtt_ca_certificate() {
  return IRRIGATION_MQTT_CA_CERT;
}
