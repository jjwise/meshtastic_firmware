#ifdef IRRIGATION_MODULE_ENABLE

#include "IrrigationOverlayModule.h"

#include "MeshService.h"
#include "NodeDB.h"
#include "PowerStatus.h"
#include "DebugConfiguration.h"
#include "configuration.h"
#if HAS_SCREEN
#include "graphics/Screen.h"
#endif
#include "mesh/Channels.h"
#include "modules/irrigation/IrrigationConfigStore.h"
#include "modules/irrigation/IrrigationSecrets.h"
#include "gps/RTC.h"

#include <algorithm>

IrrigationOverlayModule *irrigationOverlayModule;

namespace {
constexpr uint32_t k_run_interval_ms = 1000;
constexpr uint32_t k_init_delay_ms = 15000;

String node_num_hex(NodeNum node_num) {
  String value = String(node_num, HEX);
  value.toUpperCase();
  return value;
}

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

String config_store_path() {
#ifdef IRRIGATION_CONFIG_FILE_PATH
  return IRRIGATION_CONFIG_FILE_PATH;
#else
  return "/irrigation.cfg";
#endif
}

bool should_autosave_config_on_boot() {
#ifdef IRRIGATION_CONFIG_AUTOSAVE_ON_BOOT
  return IRRIGATION_CONFIG_AUTOSAVE_ON_BOOT != 0;
#else
  return false;
#endif
}

void apply_build_overrides(IrrigationConfig &cfg) {
#ifdef IRRIGATION_FARM_ID
  cfg.farm_id = IRRIGATION_FARM_ID;
#endif
#ifdef IRRIGATION_LINE_ID
  cfg.line_id = IRRIGATION_LINE_ID;
#endif
#ifdef IRRIGATION_ENDPOINT_ROLE
  cfg.endpoint_role = IRRIGATION_ENDPOINT_ROLE;
#endif
#ifdef IRRIGATION_WIFI_SSID
  cfg.wifi_ssid = IRRIGATION_WIFI_SSID;
#endif
#ifdef IRRIGATION_WIFI_PASSWORD
  cfg.wifi_password = IRRIGATION_WIFI_PASSWORD;
#endif
#ifdef IRRIGATION_MQTT_HOST
  cfg.mqtt_host = IRRIGATION_MQTT_HOST;
#endif
#ifdef IRRIGATION_MQTT_PORT
  cfg.mqtt_port = static_cast<uint16_t>(IRRIGATION_MQTT_PORT);
#endif
#ifdef IRRIGATION_MQTT_USERNAME
  cfg.mqtt_username = IRRIGATION_MQTT_USERNAME;
#endif
#ifdef IRRIGATION_MQTT_PASSWORD
  cfg.mqtt_password = IRRIGATION_MQTT_PASSWORD;
#endif
#ifdef IRRIGATION_STATIONARY_INTERVAL_SEC
  cfg.stationary_interval_sec = static_cast<uint32_t>(IRRIGATION_STATIONARY_INTERVAL_SEC);
#endif
#ifdef IRRIGATION_MOVING_INTERVAL_SEC
  cfg.moving_interval_sec = static_cast<uint32_t>(IRRIGATION_MOVING_INTERVAL_SEC);
#endif
#ifdef IRRIGATION_HEARTBEAT_INTERVAL_SEC
  cfg.heartbeat_interval_sec = static_cast<uint32_t>(IRRIGATION_HEARTBEAT_INTERVAL_SEC);
#endif
#ifdef IRRIGATION_MOVING_HOLD_SEC
  cfg.moving_hold_sec = static_cast<uint32_t>(IRRIGATION_MOVING_HOLD_SEC);
#endif
#ifdef IRRIGATION_SETTLE_CONFIRM_DELAY_SEC
  cfg.settle_confirm_delay_sec = static_cast<uint32_t>(IRRIGATION_SETTLE_CONFIRM_DELAY_SEC);
#endif
#ifdef IRRIGATION_GNSS_FIX_TIMEOUT_SEC
  cfg.gnss_fix_timeout_sec = static_cast<uint32_t>(IRRIGATION_GNSS_FIX_TIMEOUT_SEC);
#endif
#ifdef IRRIGATION_MOTION_START_THRESHOLD_MG
  cfg.motion_start_threshold_mg = static_cast<uint16_t>(IRRIGATION_MOTION_START_THRESHOLD_MG);
#endif
#ifdef IRRIGATION_STABLE_FIX_MIN_SAMPLES
  cfg.stable_fix_min_samples = static_cast<uint8_t>(IRRIGATION_STABLE_FIX_MIN_SAMPLES);
#endif
#ifdef IRRIGATION_STABLE_FIX_MAX_HDOP
  cfg.stable_fix_max_hdop = static_cast<float>(IRRIGATION_STABLE_FIX_MAX_HDOP);
#endif
#ifdef IRRIGATION_IMU_INTERRUPT_PIN
  cfg.imu_interrupt_pin = static_cast<int32_t>(IRRIGATION_IMU_INTERRUPT_PIN);
#endif
#ifdef IRRIGATION_DEFAULT_HOP_LIMIT
  cfg.default_hop_limit = static_cast<uint8_t>(IRRIGATION_DEFAULT_HOP_LIMIT);
#endif
#ifdef IRRIGATION_MAX_HOP_OVERRIDE
  cfg.max_hop_override = static_cast<uint8_t>(IRRIGATION_MAX_HOP_OVERRIDE);
#endif
#ifdef IRRIGATION_GATEWAY_BUFFER_MAX_RECORDS
  cfg.gateway_buffer_max_records = static_cast<uint16_t>(IRRIGATION_GATEWAY_BUFFER_MAX_RECORDS);
#endif
#ifdef IRRIGATION_ENABLED
  cfg.enabled = static_cast<bool>(IRRIGATION_ENABLED);
#endif
}

IrrigationNodeProfile resolve_node_profile() {
#if defined(IRRIGATION_PROFILE_GATEWAY_CENTRAL)
  return IrrigationNodeProfile::GATEWAY_CENTRAL;
#elif defined(IRRIGATION_PROFILE_RELAY_FIXED)
  return IrrigationNodeProfile::RELAY_FIXED;
#elif defined(IRRIGATION_PROFILE_ENDPOINT_POD)
  return IrrigationNodeProfile::ENDPOINT_POD;
#else
  switch (config.device.role) {
    case meshtastic_Config_DeviceConfig_Role_TRACKER:
    case meshtastic_Config_DeviceConfig_Role_TAK_TRACKER:
      return IrrigationNodeProfile::ENDPOINT_POD;
    case meshtastic_Config_DeviceConfig_Role_ROUTER:
    case meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT:
    case meshtastic_Config_DeviceConfig_Role_REPEATER:
    case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE:
      return IrrigationNodeProfile::RELAY_FIXED;
    default:
      return IrrigationNodeProfile::ENDPOINT_POD;
  }
#endif
}
}  // namespace

IrrigationOverlayModule::IrrigationOverlayModule()
    : SinglePortModule("irrigationOverlay", meshtastic_PortNum_PRIVATE_APP),
      concurrency::OSThread("IrrigationOverlay"),
      initialized_(false) {
  setInterval(k_run_interval_ms);
}

int32_t IrrigationOverlayModule::runOnce() {
  ensureInitialized();
  if (!initialized_) {
    return k_run_interval_ms;
  }

  const uint64_t now = nowMs();
  const IrrigationSensorSample sample = readSensorSampleFromNodeDb();
  irrigation_.on_sensor_sample(sample, now);
  irrigation_.tick(now);
  trySendPendingMeshTx();
  return k_run_interval_ms;
}

void IrrigationOverlayModule::ensureInitialized() {
  if (initialized_) {
    return;
  }
  if (millis() < k_init_delay_ms) {
    return;
  }

  const IrrigationConfig config = makeRuntimeConfig();
  irrigation_.begin(config);
  initialized_ = true;
}

ProcessMessage IrrigationOverlayModule::handleReceived(const meshtastic_MeshPacket &mp) {
  if (mp.decoded.payload.size == 0) {
    return ProcessMessage::CONTINUE;
  }

  LOG_INFO("Irrigation overlay received: port=%d bytes=%u from=0x%08x",
           mp.decoded.portnum, static_cast<unsigned int>(mp.decoded.payload.size), mp.from);

  String payload;
  payload.reserve(mp.decoded.payload.size);
  for (size_t index = 0; index < mp.decoded.payload.size; ++index) {
    payload += static_cast<char>(mp.decoded.payload.bytes[index]);
  }
  irrigation_.on_packet_rx(payload);
  return ProcessMessage::CONTINUE;
}

IrrigationConfig IrrigationOverlayModule::makeRuntimeConfig() const {
  IrrigationConfig cfg = make_default_irrigation_config();

  IrrigationConfigStore config_store;
  const bool config_store_ready = config_store.begin(config_store_path());
  if (config_store_ready) {
    config_store.load(cfg);
  }

  apply_build_overrides(cfg);
  cfg.node_profile = resolve_node_profile();

#ifdef IRRIGATION_TRACKER_ID
  cfg.tracker_id = IRRIGATION_TRACKER_ID;
#else
  cfg.tracker_id = String("node_") + node_num_hex(nodeDB->getNodeNum());
#endif

#ifdef IRRIGATION_MQTT_CLIENT_ID
  cfg.mqtt_client_id = IRRIGATION_MQTT_CLIENT_ID;
#else
  cfg.mqtt_client_id = cfg.tracker_id + "_gateway";
#endif

  if (config_store_ready && should_autosave_config_on_boot()) {
    config_store.save_if_changed(cfg);
  }

  return cfg;
}

IrrigationSensorSample IrrigationOverlayModule::readSensorSampleFromNodeDb() const {
  IrrigationSensorSample sample{};
  const RTCQuality rtc_quality = getRTCQuality();
  const uint32_t epoch_seconds = getValidTime(RTCQualityDevice);
  sample.ts_utc_ms = static_cast<uint64_t>(epoch_seconds) * 1000ULL;
  sample.time_quality = rtc_quality >= RTCQualityGPS
                            ? IrrigationTimeQuality::GNSS
                            : (rtc_quality >= RTCQualityDevice ? IrrigationTimeQuality::RTC
                                                              : IrrigationTimeQuality::UNKNOWN);
  sample.gps_fix = localPosition.latitude_i != 0 && localPosition.longitude_i != 0;
  sample.lat = static_cast<double>(localPosition.latitude_i) / 10000000.0;
  sample.lon = static_cast<double>(localPosition.longitude_i) / 10000000.0;
  sample.hdop = static_cast<float>(localPosition.HDOP) / 100.0f;
  sample.speed_mps = static_cast<float>(localPosition.ground_speed) / 3.6f;
#if HAS_SCREEN
  if (screen != nullptr && screen->hasHeading()) {
    sample.heading_deg = static_cast<float>(screen->getHeading());
    sample.heading_valid = true;
  } else {
    sample.heading_deg = static_cast<float>(localPosition.ground_track);
    sample.heading_valid = false;
  }
#else
  sample.heading_deg = static_cast<float>(localPosition.ground_track);
  sample.heading_valid = false;
#endif
  sample.accel_rms_g = 0.0f;
  sample.battery_mv = powerStatus != nullptr && powerStatus->getBatteryVoltageMv() > 0
                          ? static_cast<uint16_t>(powerStatus->getBatteryVoltageMv())
                          : 0;
  return sample;
}

uint64_t IrrigationOverlayModule::nowMs() const { return static_cast<uint64_t>(millis()); }

bool IrrigationOverlayModule::trySendPendingMeshTx() {
  bool sent_any = false;
  String payload_json;
  uint8_t hop_limit = 0;
  while (irrigation_.pop_next_mesh_tx(payload_json, hop_limit)) {
    if (payload_json.isEmpty()) {
      continue;
    }

    meshtastic_MeshPacket *packet = allocDataPacket();
    const size_t max_payload = sizeof(packet->decoded.payload.bytes);
    const size_t raw_payload_len = payload_json.length();
    const bool truncated = raw_payload_len > max_payload;
    const size_t payload_len = std::min(raw_payload_len, max_payload);
    if (truncated) {
      LOG_WARN("Irrigation overlay dropping oversized payload: port=%d raw=%u max=%u truncated=1 hop_limit=%u",
               packet->decoded.portnum, static_cast<unsigned int>(raw_payload_len),
               static_cast<unsigned int>(max_payload), hop_limit);
      service->releaseToPool(packet);
      continue;
    }
    memcpy(packet->decoded.payload.bytes, payload_json.c_str(), payload_len);
    packet->decoded.payload.size = payload_len;
    packet->to = NODENUM_BROADCAST;
    packet->channel = channels.getPrimaryIndex();
    packet->decoded.want_response = false;
    packet->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
    if (hop_limit > 0) {
      packet->hop_limit = hop_limit;
    }

    LOG_INFO("Irrigation overlay sending: port=%d raw=%u bytes=%u max=%u truncated=0 hop_limit=%u",
             packet->decoded.portnum, static_cast<unsigned int>(raw_payload_len),
             static_cast<unsigned int>(packet->decoded.payload.size), static_cast<unsigned int>(max_payload),
             packet->hop_limit);
    service->sendToMesh(packet, RX_SRC_LOCAL, true);
    sent_any = true;
  }
  return sent_any;
}

#endif
