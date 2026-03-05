#ifdef IRRIGATION_MODULE_ENABLE

#include "IrrigationOverlayModule.h"

#include "MeshService.h"
#include "NodeDB.h"
#include "configuration.h"
#include "mesh/Channels.h"

#include <algorithm>

IrrigationOverlayModule *irrigationOverlayModule;

namespace {
constexpr uint32_t k_run_interval_ms = 1000;

String node_num_hex(NodeNum node_num)
{
    String value = String(node_num, HEX);
    value.toUpperCase();
    return value;
}
} // namespace

IrrigationOverlayModule::IrrigationOverlayModule()
    : SinglePortModule("irrigationOverlay", meshtastic_PortNum_PRIVATE_APP), concurrency::OSThread("IrrigationOverlay")
{
    setInterval(k_run_interval_ms);
    irrigation_.begin(makeRuntimeConfig());
}

int32_t IrrigationOverlayModule::runOnce()
{
    const uint64_t now = nowMs();
    const IrrigationSensorSample sample = readSensorSampleFromNodeDb();
    irrigation_.on_sensor_sample(sample, now);
    irrigation_.tick(now);
    trySendPendingMeshTx();
    return k_run_interval_ms;
}

ProcessMessage IrrigationOverlayModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.decoded.payload.size == 0) {
        return ProcessMessage::CONTINUE;
    }

    String payload;
    payload.reserve(mp.decoded.payload.size);
    for (size_t index = 0; index < mp.decoded.payload.size; ++index) {
        payload += static_cast<char>(mp.decoded.payload.bytes[index]);
    }
    irrigation_.on_packet_rx(payload);
    return ProcessMessage::CONTINUE;
}

IrrigationConfig IrrigationOverlayModule::makeRuntimeConfig() const
{
    IrrigationConfig cfg = make_default_irrigation_config();

#ifdef IRRIGATION_FARM_ID
    cfg.farm_id = IRRIGATION_FARM_ID;
#endif
#ifdef IRRIGATION_LINE_ID
    cfg.line_id = IRRIGATION_LINE_ID;
#endif
#ifdef IRRIGATION_ENDPOINT_ROLE
    cfg.endpoint_role = IRRIGATION_ENDPOINT_ROLE;
#endif

    cfg.tracker_id = String("node_") + node_num_hex(nodeDB->getNodeNum());
    cfg.mqtt_client_id = cfg.tracker_id + "_gateway";

#if defined(IRRIGATION_PROFILE_GATEWAY_CENTRAL)
    cfg.node_profile = IrrigationNodeProfile::GATEWAY_CENTRAL;
#elif defined(IRRIGATION_PROFILE_RELAY_FIXED)
    cfg.node_profile = IrrigationNodeProfile::RELAY_FIXED;
#else
    switch (config.device.role) {
    case meshtastic_Config_DeviceConfig_Role_TRACKER:
    case meshtastic_Config_DeviceConfig_Role_TAK_TRACKER:
        cfg.node_profile = IrrigationNodeProfile::ENDPOINT_POD;
        break;
    case meshtastic_Config_DeviceConfig_Role_ROUTER:
    case meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT:
    case meshtastic_Config_DeviceConfig_Role_REPEATER:
    case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE:
        cfg.node_profile = IrrigationNodeProfile::RELAY_FIXED;
        break;
    default:
        cfg.node_profile = IrrigationNodeProfile::ENDPOINT_POD;
        break;
    }
#endif

    return cfg;
}

IrrigationSensorSample IrrigationOverlayModule::readSensorSampleFromNodeDb() const
{
    IrrigationSensorSample sample{};
    sample.gps_fix = localPosition.latitude_i != 0 && localPosition.longitude_i != 0;
    sample.lat = static_cast<double>(localPosition.latitude_i) / 10000000.0;
    sample.lon = static_cast<double>(localPosition.longitude_i) / 10000000.0;
    sample.hdop = static_cast<float>(localPosition.HDOP) / 100.0f;
    sample.speed_mps = static_cast<float>(localPosition.ground_speed) / 3.6f;
    sample.heading_deg = static_cast<float>(localPosition.ground_track);
    sample.accel_rms_g = 0.0f;
    sample.battery_mv = 0;
    return sample;
}

uint64_t IrrigationOverlayModule::nowMs() const
{
    return static_cast<uint64_t>(millis());
}

bool IrrigationOverlayModule::trySendPendingMeshTx()
{
    bool sent_any = false;
    String payload_json;
    uint8_t hop_limit = 0;
    while (irrigation_.pop_next_mesh_tx(payload_json, hop_limit)) {
        if (payload_json.isEmpty()) {
            continue;
        }

        meshtastic_MeshPacket *packet = allocDataPacket();
        const size_t max_payload = sizeof(packet->decoded.payload.bytes);
        const size_t payload_len = std::min(static_cast<size_t>(payload_json.length()), max_payload);
        memcpy(packet->decoded.payload.bytes, payload_json.c_str(), payload_len);
        packet->decoded.payload.size = payload_len;
        packet->to = NODENUM_BROADCAST;
        packet->channel = channels.getPrimaryIndex();
        packet->decoded.want_response = false;
        packet->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
        if (hop_limit > 0) {
            packet->hop_limit = hop_limit;
        }

        service->sendToMesh(packet, RX_SRC_LOCAL, true);
        sent_any = true;
    }
    return sent_any;
}

#endif
