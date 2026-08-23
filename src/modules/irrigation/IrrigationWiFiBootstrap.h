#pragma once

#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
// Apply compiled gateway credentials before Meshtastic starts native Wi-Fi.
void applyIrrigationGatewayWiFiBootstrap();
#endif
