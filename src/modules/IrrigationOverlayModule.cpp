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
#include "modules/irrigation/IrrigationWiFiBootstrap.h"
#include "gps/RTC.h"

#include <algorithm>
#if defined(ARCH_ESP32)
#include <esp_timer.h>
#endif

#if defined(IRRIGATION_IMU_TEST) && defined(ARCH_ESP32)
#include "mesh/Throttle.h"

#include <ICM_20948.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#endif

IrrigationOverlayModule *irrigationOverlayModule;

namespace {
constexpr uint32_t k_run_interval_ms = 1000;
constexpr uint32_t k_init_delay_ms = 15000;

#if defined(IRRIGATION_IMU_TEST) && defined(ARCH_ESP32)
#ifndef IRRIGATION_IMU_INTERRUPT_PIN
#error "IRRIGATION_IMU_TEST requires IRRIGATION_IMU_INTERRUPT_PIN"
#endif

static_assert(IRRIGATION_IMU_INTERRUPT_PIN >= 0 && IRRIGATION_IMU_INTERRUPT_PIN <= 21,
              "IRRIGATION_IMU_INTERRUPT_PIN must be an ESP32-S3 RTC-capable GPIO (0..21)");

#ifndef IRRIGATION_IMU_TEST_WOM_THRESHOLD
#define IRRIGATION_IMU_TEST_WOM_THRESHOLD 25
#endif

#ifndef IRRIGATION_IMU_TEST_SLEEP_DELAY_MS
#define IRRIGATION_IMU_TEST_SLEEP_DELAY_MS 5000
#endif

constexpr uint32_t k_imu_test_run_interval_ms = 100;
constexpr uint32_t k_imu_test_init_delay_ms = 3000;
constexpr uint32_t k_imu_test_init_retry_ms = 5000;
constexpr gpio_num_t k_imu_test_gpio = static_cast<gpio_num_t>(IRRIGATION_IMU_INTERRUPT_PIN);

ICM_20948_I2C imu_test_sensor;
volatile bool imu_test_interrupt_pending = false;
RTC_DATA_ATTR uint32_t imu_test_deep_sleep_wake_count = 0;

void IRAM_ATTR irrigation_imu_test_isr() {
  imu_test_interrupt_pending = true;
}

bool imu_test_status_ok(ICM_20948_Status_e status, const char *operation) {
  if (status == ICM_20948_Stat_Ok) {
    return true;
  }

  LOG_WARN("[IrrigationOverlay] IMU test: %s failed: status=%d detail=%s", operation, static_cast<int>(status),
           imu_test_sensor.statusString(status));
  return false;
}
#endif

String node_num_hex(NodeNum node_num) {
  String value = String(node_num, HEX);
  value.toUpperCase();
  return value;
}

__attribute__((unused)) const char *profile_to_cstr(IrrigationNodeProfile profile) {
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
#if defined(ARCH_ESP32) && defined(IRRIGATION_ENABLE_MQTT_UPLINK)
  applyIrrigationGatewayWiFiConfig(cfg);
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
#ifdef IRRIGATION_POSITION_INTERVAL_SEC
  cfg.position_interval_sec = static_cast<uint32_t>(IRRIGATION_POSITION_INTERVAL_SEC);
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
#ifdef IRRIGATION_VALVE_CONTROL_PIN
  cfg.valve_control_pin = static_cast<int32_t>(IRRIGATION_VALVE_CONTROL_PIN);
#endif
#ifdef IRRIGATION_VALVE_MAX_OPEN_SEC
  cfg.valve_max_open_sec = static_cast<uint32_t>(IRRIGATION_VALVE_MAX_OPEN_SEC);
#endif
#ifdef IRRIGATION_VALVE_ACTIVE_HIGH
  cfg.valve_active_high = static_cast<bool>(IRRIGATION_VALVE_ACTIVE_HIGH);
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
#elif defined(IRRIGATION_PROFILE_VALVE_ACTUATOR)
  return IrrigationNodeProfile::VALVE_ACTUATOR;
#elif defined(IRRIGATION_PROFILE_BASIC_TRACKER)
  return IrrigationNodeProfile::BASIC_TRACKER;
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
#if defined(IRRIGATION_IMU_TEST) && defined(ARCH_ESP32)
      imuTestState_(ImuTestState::WAIT_FOR_INITIALIZATION),
      imuTestStateStartedMs_(0),
      imuTestLastInitAttemptMs_(0),
#endif
      initialized_(false) {
#if defined(IRRIGATION_IMU_TEST) && defined(ARCH_ESP32)
  setInterval(k_imu_test_run_interval_ms);
#else
  setInterval(k_run_interval_ms);
#endif
}

int32_t IrrigationOverlayModule::runOnce() {
#if defined(IRRIGATION_IMU_TEST) && defined(ARCH_ESP32)
  runImuTest();
  return k_imu_test_run_interval_ms;
#else
  ensureInitialized();
  if (!initialized_) {
    return k_run_interval_ms;
  }

  const uint64_t now = nowMs();
  const IrrigationSensorSample sample = readSensorSampleFromNodeDb();
  irrigation_.on_sensor_sample(sample, now);
  irrigation_.tick(now);
  trySendPendingMeshTx();
  IrrigationConfig updated_config;
  if (irrigation_.pop_config_update(updated_config)) {
    IrrigationConfigStore config_store;
    const bool saved = config_store.begin(config_store_path()) && config_store.save(updated_config);
    LOG_INFO("Irrigation runtime config persistence: saved=%d", saved ? 1 : 0);
  }
  return k_run_interval_ms;
#endif
}

#if defined(IRRIGATION_IMU_TEST) && defined(ARCH_ESP32)
void IrrigationOverlayModule::runImuTest() {
  const uint32_t now_ms = millis();

  if (imuTestState_ == ImuTestState::WAIT_FOR_INITIALIZATION) {
    const esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
    if (wake_cause == ESP_SLEEP_WAKEUP_EXT1) {
      const uint64_t wake_mask = esp_sleep_get_ext1_wakeup_status();
      ++imu_test_deep_sleep_wake_count;
      rtc_gpio_deinit(k_imu_test_gpio);
      pinMode(IRRIGATION_IMU_INTERRUPT_PIN, INPUT_PULLUP);

      if ((wake_mask & (1ULL << IRRIGATION_IMU_INTERRUPT_PIN)) != 0) {
        LOG_INFO("[IrrigationOverlay] IMU deep-sleep test PASS: GPIO%d woke the ESP32-S3 "
                 "(wake_count=%u mask=0x%llx)",
                 IRRIGATION_IMU_INTERRUPT_PIN, imu_test_deep_sleep_wake_count,
                 static_cast<unsigned long long>(wake_mask));
      } else {
        LOG_WARN("[IrrigationOverlay] IMU deep-sleep test FAIL: EXT1 wake mask does not contain "
                 "GPIO%d (mask=0x%llx)",
                 IRRIGATION_IMU_INTERRUPT_PIN, static_cast<unsigned long long>(wake_mask));
      }

      imuTestState_ = ImuTestState::COMPLETE;
      return;
    }

    if (wake_cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
      LOG_WARN("[IrrigationOverlay] IMU deep-sleep test: boot wake cause=%d, starting a new test cycle",
               static_cast<int>(wake_cause));
    }

    if (Throttle::isWithinTimespanMs(imuTestStateStartedMs_, k_imu_test_init_delay_ms)) {
      return;
    }

    if (imuTestLastInitAttemptMs_ != 0 &&
        Throttle::isWithinTimespanMs(imuTestLastInitAttemptMs_, k_imu_test_init_retry_ms)) {
      return;
    }

    imuTestLastInitAttemptMs_ = now_ms;
    if (initializeImuTest()) {
      imuTestState_ = ImuTestState::WAIT_FOR_AWAKE_INTERRUPT;
      imuTestStateStartedMs_ = now_ms;
    }
    return;
  }

  if (imuTestState_ == ImuTestState::WAIT_FOR_AWAKE_INTERRUPT) {
    if (imu_test_interrupt_pending || digitalRead(IRRIGATION_IMU_INTERRUPT_PIN) == LOW) {
      handleImuTestInterrupt();
    }
    return;
  }

  if (imuTestState_ == ImuTestState::WAIT_FOR_DEEP_SLEEP &&
      !Throttle::isWithinTimespanMs(imuTestStateStartedMs_, IRRIGATION_IMU_TEST_SLEEP_DELAY_MS)) {
    enterImuTestDeepSleep();
  }
}

bool IrrigationOverlayModule::initializeImuTest() {
  LOG_INFO("[IrrigationOverlay] IMU test 2/3: probing ICM-20948 on SDA=%d SCL=%d; "
           "INT1 must be wired to GPIO%d",
           I2C_SDA, I2C_SCL, IRRIGATION_IMU_INTERRUPT_PIN);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  bool found = false;
  for (bool ad0_high : {false, true}) {
    imu_test_sensor.begin(Wire, ad0_high);
    if (imu_test_sensor.status == ICM_20948_Stat_Ok) {
      LOG_INFO("[IrrigationOverlay] IMU test: ICM-20948 found at address 0x%02x", ad0_high ? 0x69 : 0x68);
      found = true;
      break;
    }
  }

  if (!found) {
    LOG_WARN("[IrrigationOverlay] IMU test: ICM-20948 not found; retrying in %u ms", k_imu_test_init_retry_ms);
    return false;
  }

  if (!imu_test_status_ok(imu_test_sensor.swReset(), "software reset")) {
    return false;
  }
  delay(250);

  if (!imu_test_status_ok(imu_test_sensor.sleep(false), "wake sensor") ||
      !imu_test_status_ok(imu_test_sensor.lowPower(false), "disable low power") ||
      !imu_test_status_ok(
          imu_test_sensor.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr,
                                        ICM_20948_Sample_Mode_Cycled),
          "set cycled sample mode") ||
      !imu_test_status_ok(imu_test_sensor.cfgIntActiveLow(true), "set INT active low") ||
      !imu_test_status_ok(imu_test_sensor.cfgIntOpenDrain(true), "set INT open drain") ||
      !imu_test_status_ok(imu_test_sensor.cfgIntLatch(true), "latch INT") ||
      !imu_test_status_ok(imu_test_sensor.cfgIntAnyReadToClear(false), "disable any-read clear") ||
      !imu_test_status_ok(imu_test_sensor.WOMThreshold(IRRIGATION_IMU_TEST_WOM_THRESHOLD), "set WOM threshold") ||
      !imu_test_status_ok(imu_test_sensor.WOMLogic(true, 1), "set WOM logic") ||
      !imu_test_status_ok(imu_test_sensor.clearInterrupts(), "clear pending interrupts") ||
      !imu_test_status_ok(imu_test_sensor.intEnableWOM(true), "enable WOM interrupt")) {
    return false;
  }

  pinMode(IRRIGATION_IMU_INTERRUPT_PIN, INPUT_PULLUP);
  imu_test_interrupt_pending = false;
  attachInterrupt(digitalPinToInterrupt(IRRIGATION_IMU_INTERRUPT_PIN), irrigation_imu_test_isr, FALLING);

  LOG_INFO("[IrrigationOverlay] IMU awake test READY: move the board. "
           "WOM threshold=%u (~%u mg), INT=active-low open-drain",
           IRRIGATION_IMU_TEST_WOM_THRESHOLD, IRRIGATION_IMU_TEST_WOM_THRESHOLD * 4);
  return true;
}

void IrrigationOverlayModule::handleImuTestInterrupt() {
  noInterrupts();
  imu_test_interrupt_pending = false;
  interrupts();

  const int asserted_level = digitalRead(IRRIGATION_IMU_INTERRUPT_PIN);
  const bool cleared = imu_test_status_ok(imu_test_sensor.clearInterrupts(), "clear WOM interrupt");
  delay(10);
  const int cleared_level = digitalRead(IRRIGATION_IMU_INTERRUPT_PIN);

  if (asserted_level == LOW && cleared && cleared_level == HIGH) {
    LOG_INFO("[IrrigationOverlay] IMU awake test PASS: GPIO%d asserted LOW and returned HIGH "
             "after ICM interrupt clear",
             IRRIGATION_IMU_INTERRUPT_PIN);
  } else {
    LOG_WARN("[IrrigationOverlay] IMU awake test FAIL: GPIO%d asserted_level=%d clear_ok=%d "
             "cleared_level=%d",
             IRRIGATION_IMU_INTERRUPT_PIN, asserted_level, cleared ? 1 : 0, cleared_level);
    return;
  }

  detachInterrupt(digitalPinToInterrupt(IRRIGATION_IMU_INTERRUPT_PIN));
  imuTestState_ = ImuTestState::WAIT_FOR_DEEP_SLEEP;
  imuTestStateStartedMs_ = millis();
  LOG_INFO("[IrrigationOverlay] IMU deep-sleep test ARMED: keeping the board still for %u ms, "
           "then sleeping until motion",
           IRRIGATION_IMU_TEST_SLEEP_DELAY_MS);
}

void IrrigationOverlayModule::enterImuTestDeepSleep() {
  if (!imu_test_status_ok(imu_test_sensor.clearInterrupts(), "clear interrupt before deep sleep")) {
    imuTestStateStartedMs_ = millis();
    return;
  }

  delay(10);
  if (digitalRead(IRRIGATION_IMU_INTERRUPT_PIN) == LOW) {
    LOG_WARN("[IrrigationOverlay] IMU deep-sleep test postponed: GPIO%d is still LOW; "
             "keep the board still",
             IRRIGATION_IMU_INTERRUPT_PIN);
    imuTestStateStartedMs_ = millis();
    return;
  }

  pinMode(IRRIGATION_IMU_INTERRUPT_PIN, INPUT_PULLUP);
  gpio_pullup_en(k_imu_test_gpio);
  gpio_pulldown_dis(k_imu_test_gpio);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  const esp_err_t wake_status =
      esp_sleep_enable_ext1_wakeup(1ULL << IRRIGATION_IMU_INTERRUPT_PIN, ESP_EXT1_WAKEUP_ANY_LOW);
  if (wake_status != ESP_OK) {
    LOG_WARN("[IrrigationOverlay] IMU deep-sleep test FAIL: could not arm GPIO%d, esp_err=%d",
             IRRIGATION_IMU_INTERRUPT_PIN, static_cast<int>(wake_status));
    imuTestState_ = ImuTestState::COMPLETE;
    return;
  }

  LOG_INFO("[IrrigationOverlay] IMU deep-sleep test: entering deep sleep now; "
           "move the board to wake it via GPIO%d",
           IRRIGATION_IMU_INTERRUPT_PIN);
  Serial.flush();
  delay(100);
  esp_deep_sleep_start();
}
#endif

void IrrigationOverlayModule::ensureInitialized() {
  if (initialized_) {
    return;
  }
  if (nowMs() < k_init_delay_ms) {
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
  irrigation_.on_packet_rx(payload, nowMs());
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
  cfg.command_hmac_key = irrigation_command_hmac_key();

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

uint64_t IrrigationOverlayModule::nowMs() const {
#if defined(ARCH_ESP32)
  return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
#else
  static uint32_t previous_ms = 0;
  static uint64_t epoch_ms = 0;
  const uint32_t current_ms = millis();
  if (current_ms < previous_ms) {
    epoch_ms += 1ULL << 32;
  }
  previous_ms = current_ms;
  return epoch_ms + current_ms;
#endif
}

bool IrrigationOverlayModule::trySendPendingMeshTx() {
  bool sent_any = false;
  String payload_json;
  uint8_t hop_limit = 0;
  while (irrigation_.pop_next_mesh_tx(payload_json, hop_limit)) {
    if (payload_json.isEmpty()) {
      continue;
    }

    meshtastic_MeshPacket *packet = allocDataPacket();
    static_assert(sizeof(packet->decoded.payload.bytes) >= k_irrigation_mesh_payload_max, "Irrigation radio capacity changed");
    const size_t max_payload = k_irrigation_mesh_payload_max;
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
