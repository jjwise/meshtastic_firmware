#!/usr/bin/env python3
"""Compile production codecs/command handling against GPIO and durable-NVS doubles."""
import argparse
import hashlib
import hmac
import json
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent

ARDUINO = r"""
#pragma once
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>
#include <type_traits>
#include <vector>
class String : public std::string {
public:
 using std::string::string;
 String() = default;
 String(const std::string &s) : std::string(s) {}
 template<class T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
 String(T n) : std::string(std::to_string(n)) {}
 bool isEmpty() const { return empty(); }
 int indexOf(char c, size_t start=0) const { auto n=find(c,start); return n==npos?-1:static_cast<int>(n); }
 int indexOf(const String &s, size_t start=0) const { auto n=find(s,start); return n==npos?-1:static_cast<int>(n); }
 int lastIndexOf(const String &s) const { auto n=rfind(s); return n==npos?-1:static_cast<int>(n); }
 String substring(size_t a, size_t b=npos) const { return substr(a,b==npos?npos:b-a); }
 void toLowerCase() { for(char &c:*this)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c))); }
 void trim() { auto a=find_first_not_of(" \r\n\t");auto b=find_last_not_of(" \r\n\t");*this=a==npos?"":substr(a,b-a+1); }
};
constexpr int LOW=0, HIGH=1, OUTPUT=1;
inline std::vector<int> gpio;
inline unsigned opens=0;
inline int active_level=HIGH;
inline bool enforce_checkpoint=false;
void digitalWrite(int pin,int level);
inline void pinMode(int,int) {}
inline uint32_t millis(){return 42;}
inline uint32_t micros(){return 42000;}
"""
NVS = r"""
#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>
using nvs_handle_t=unsigned;
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_FAIL=-1, ESP_ERR_NVS_NOT_FOUND=1, ESP_ERR_NVS_INVALID_LENGTH=2, NVS_READWRITE=1;
struct PowerCut {};
namespace flash {
inline std::vector<uint8_t> durable, pending;
inline bool fail_open=false, fail_read=false, fail_write=false, fail_commit=false, drop_commit=false, crash_after_commit=false;
inline void reset() {durable.clear();pending.clear();fail_open=fail_read=fail_write=fail_commit=drop_commit=crash_after_commit=false;}
}
inline int nvs_open(const char *,int,nvs_handle_t *h){*h=1;return flash::fail_open?ESP_FAIL:ESP_OK;}
inline void nvs_close(nvs_handle_t) {}
inline int nvs_get_blob(nvs_handle_t,const char*,void *p,size_t *n){
 if(flash::fail_read)return ESP_FAIL;
 if(flash::durable.empty())return ESP_ERR_NVS_NOT_FOUND;
 if(*n<flash::durable.size())return ESP_ERR_NVS_INVALID_LENGTH;
 *n=flash::durable.size();memcpy(p,flash::durable.data(),*n);return ESP_OK;
}
inline int nvs_set_blob(nvs_handle_t,const char*,const void *p,size_t n){
 if(flash::fail_write)return ESP_FAIL;
 auto b=static_cast<const uint8_t*>(p);flash::pending.assign(b,b+n);return ESP_OK;
}
inline int nvs_commit(nvs_handle_t){
 if(flash::fail_commit)return ESP_FAIL;
 if(!flash::drop_commit)flash::durable=flash::pending;
 if(flash::crash_after_commit){flash::crash_after_commit=false;throw PowerCut{};}
 return ESP_OK;
}
"""
TEST = r"""
#include "Arduino.h"
#include "nvs.h"
#include "IrrigationModule.h"
#include "IrrigationCommandAuth.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <set>

void digitalWrite(int pin,int level){
 assert(pin==4);
 if(enforce_checkpoint && level==active_level){assert(!flash::durable.empty());++opens;}
 gpio.push_back(level);
}
IrrigationGatewayStore::IrrigationGatewayStore():max_records_(500){}
bool IrrigationGatewayStore::begin(const String&,size_t){return true;}
bool IrrigationGatewayStore::append(const String&){return true;}
bool IrrigationGatewayStore::peek_next(String&){return false;}
bool IrrigationGatewayStore::pop_next(){return true;}
size_t IrrigationGatewayStore::size() const{return 0;}
IrrigationMqttUplink::IrrigationMqttUplink(){}
bool IrrigationMqttUplink::begin(const IrrigationConfig&){return true;}
void IrrigationMqttUplink::tick(uint64_t){}
bool IrrigationMqttUplink::is_connected(){return true;}
bool IrrigationMqttUplink::publish_payload(const String&,const String&,uint8_t,uint64_t){return true;}
bool IrrigationMqttUplink::pop_received_command(String&,String&){return false;}

const String key="UNIT_TEST_ONLY_0123456789_abcdefghijklmnopqrstuvwxyz";
constexpr uint64_t epoch=1794000000000ULL;
const String uuid="12345678-1234-4567-89ab-123456789abc";
IrrigationConfig config(bool tracker=false){
 auto c=make_default_irrigation_config();c.tracker_id="test_00000001";c.line_id="infrastructure";
 c.endpoint_role=tracker?"TRACKER":"VALVE";
 c.node_profile=tracker?IrrigationNodeProfile::BASIC_TRACKER:IrrigationNodeProfile::VALVE_ACTUATOR;
 c.valve_control_pin=4;c.command_hmac_key=key;c.valve_max_open_sec=10;return c;
}
IrrigationSensorSample sample(uint64_t utc=epoch){
 IrrigationSensorSample s{};s.ts_utc_ms=utc;s.time_quality=IrrigationTimeQuality::GNSS;
 s.gps_fix=true;s.lat=-89.99999;s.lon=-179.99999;s.hdop=1;s.battery_mv=4200;return s;
}
String command(const String &id=uuid,uint64_t issued=epoch,bool open=true,uint32_t duration=10,bool tracker=false,uint32_t interval=120){
 IrrigationCommand c{};c.command_id=id;c.tracker_id="test_00000001";c.issued_at_utc_ms=issued;
 c.expires_at_utc_ms=issued+120000;c.action=tracker?IrrigationCommandAction::SET_POSITION_INTERVAL:IrrigationCommandAction::SET_VALVE;
 c.valve_open=open;c.duration_sec=duration;c.position_interval_sec=tracker?interval:0;
 c.signature=irrigation_command_signature(c,key);
 return "{\"k\":\"C\",\"i\":\""+id+"\",\"r\":\""+c.tracker_id+"\",\"a\":\""+(tracker?"I":"V")+
        "\",\"x\":"+String(issued)+",\"z\":"+String(c.expires_at_utc_ms)+",\"w\":"+(open?"1":"0")+
        ",\"d\":"+String(duration)+",\"u\":"+String(c.position_interval_sec)+",\"s\":\""+c.signature+"\"}";
}
void drain(IrrigationModule &m){String s;uint8_t hops;while(m.pop_next_mesh_tx(s,hops)){assert(s.length()<=233);}}
void packet(IrrigationModule &m,const char *label){
 String s;uint8_t hops;assert(m.pop_next_mesh_tx(s,hops));assert(s.length()<=233);
 IrrigationTelemetryEvent e{};assert(irrigation_event_from_json(s,e));assert(e.tracker_id=="test_00000001");
 std::cout<<"PACKET|"<<label<<"|"<<s<<"\n";
}
void boot(IrrigationModule &m,const IrrigationConfig &c=config(),uint64_t utc=epoch){
 enforce_checkpoint=false;assert(m.begin(c));
 if(c.node_profile==IrrigationNodeProfile::VALVE_ACTUATOR)assert(gpio.back()==(c.valve_active_high?LOW:HIGH));
 active_level=c.valve_active_high?HIGH:LOW;enforce_checkpoint=true;m.on_sensor_sample(sample(utc),1);
}
void test_packets(){
 std::set<String> ids;
 for(auto t:{uint64_t(0),epoch,std::numeric_limits<uint64_t>::max()})
 for(auto uptime:{uint64_t(0),uint64_t(4294967296ULL),std::numeric_limits<uint64_t>::max()})
 for(auto seq:{uint32_t(0),std::numeric_limits<uint32_t>::max()}){
  auto id=irrigation_message_id(String(128,'r'),t,uptime,seq,1);assert(id.length()==32);assert(ids.insert(id).second);
  assert(id!=irrigation_message_id(String(128,'r'),t,uptime,seq,2));
  for(auto profile:{IrrigationNodeProfile::VALVE_ACTUATOR,IrrigationNodeProfile::BASIC_TRACKER,IrrigationNodeProfile::ENDPOINT_POD}){
   IrrigationTelemetryEvent e{};e.msg_id=id;e.ts_utc_ms=t;e.tracker_id="test_00000001";e.line_id="infrastructure";
   e.endpoint_role=profile==IrrigationNodeProfile::VALVE_ACTUATOR?"VALVE":"ENDPOINT_A";e.node_profile=profile;
   e.publish_reason=IrrigationPublishReason::VALVE_STATE;e.lat=-89.99999;e.lon=-179.99999;
   e.fix=e.stale=e.heading_valid=true;e.heading_deg=359.9;e.battery_mv=65535;e.hop_count=255;
   e.time_quality=IrrigationTimeQuality::GNSS;e.motion_state=IrrigationMotionState::MOVING;
   e.has_valve_state=profile==IrrigationNodeProfile::VALVE_ACTUATOR;e.valve_open=true;
   e.has_position_interval=profile==IrrigationNodeProfile::BASIC_TRACKER;e.position_interval_sec=86400;
   String s;assert(irrigation_event_to_json(e,s));assert(s.length()<=233);
   IrrigationTelemetryEvent decoded{};assert(irrigation_event_from_json(s,decoded));assert(decoded.fix && decoded.stale);
   assert(decoded.motion_state==IrrigationMotionState::MOVING);assert(decoded.lat==e.lat && decoded.lon==e.lon);
   e.publish_reason=IrrigationPublishReason::COMMAND_ACK;e.command_id=uuid;
   for(size_t n:{size_t(1),size_t(16),size_t(24)}){
    e.tracker_id=String(n,'r');e.line_id=String(n,'l');e.endpoint_role="VALVE";
    assert(irrigation_event_to_json(e,s));assert(s.length()<=233);assert(s.indexOf("\"a\"")==-1);
   }
   e.tracker_id=String(128,'r');e.line_id=String(128,'l');assert(!irrigation_event_to_json(e,s));assert(s.empty());
  }
 }
 IrrigationTelemetryEvent e{};e.msg_id=uuid;e.ts_utc_ms=epoch;e.tracker_id="test_00000001";e.line_id="infrastructure";
 e.endpoint_role="VALVE";e.node_profile=IrrigationNodeProfile::VALVE_ACTUATOR;e.publish_reason=IrrigationPublishReason::VALVE_STATE;
 e.lat=-43.12345;e.lon=170.12345;e.fix=true;e.battery_mv=4200;e.has_valve_state=true;e.valve_open=true;
 String s;assert(irrigation_event_to_json(e,s));std::cout<<"PACKET|valve-position|"<<s<<"\n";
 e.lat=std::numeric_limits<double>::quiet_NaN();assert(!irrigation_event_to_json(e,s));
 std::cout<<"PASS packet bounds: UUIDs, 128-character input identifiers, uint64 timestamps/uptime, uint32 sequence, GPS and compact ACKs\n";
}
void test_commands(){
 flash::reset();opens=0;gpio.clear();
 IrrigationModule m;boot(m);assert(m.on_packet_rx(command(),10));assert(opens==1);packet(m,"valve-ack");
 assert(m.on_packet_rx(command(),5000));assert(opens==1);drain(m);
 m.tick(10009);assert(gpio.back()==HIGH);drain(m);m.tick(10010);assert(gpio.back()==LOW);drain(m);
 assert(m.on_packet_rx(command(),11000));assert(opens==1);drain(m);
 IrrigationModule reboot;boot(reboot);assert(!reboot.on_packet_rx(command(),15));assert(opens==1);packet(reboot,"reboot-reject");
 assert(!reboot.on_packet_rx(command("same-time",epoch),20));assert(opens==1);drain(reboot);
 reboot.on_sensor_sample(sample(epoch+1),21);assert(reboot.on_packet_rx(command("new",epoch+1),22));assert(opens==2);drain(reboot);
 assert(!reboot.on_packet_rx(command("older",epoch),23));assert(opens==2);drain(reboot);
 assert(reboot.on_packet_rx(command("close",epoch+2,false,0),24));assert(gpio.back()==LOW);drain(reboot);
 assert(!reboot.on_packet_rx(command("new",epoch+1),25));assert(gpio.back()==LOW);drain(reboot);
 IrrigationModule after_close;boot(after_close,config(),epoch+3);assert(!after_close.on_packet_rx(command("new",epoch+1),26));
 assert(!after_close.on_packet_rx(command("close",epoch+2,false,0),27));drain(after_close);
 auto badsig=command("bad-signature",epoch+4);auto pos=badsig.find("\"s\":\"")+5;badsig[pos]=badsig[pos]=='a'?'b':'a';
 assert(!after_close.on_packet_rx(badsig,29));drain(after_close);
 flash::reset();IrrigationModule expired;boot(expired,config(),epoch+200000);assert(!expired.on_packet_rx(command(),30));drain(expired);
 IrrigationModule clockless;enforce_checkpoint=false;assert(clockless.begin(config()));enforce_checkpoint=true;
 assert(!clockless.on_packet_rx(command(),31));drain(clockless);
 IrrigationModule invalid_duration;boot(invalid_duration,config(),epoch+5);assert(!invalid_duration.on_packet_rx(command("duration",epoch+5,true,11),32));drain(invalid_duration);
 flash::reset();auto low=config();low.valve_active_high=false;IrrigationModule active_low;boot(active_low,low);
 assert(active_low.on_packet_rx(command(),40));assert(gpio.back()==LOW);drain(active_low);active_low.tick(10040);assert(gpio.back()==HIGH);drain(active_low);
 flash::reset();auto tracker=config(true);IrrigationModule tr;boot(tr,tracker);assert(tr.on_packet_rx(command(uuid,epoch,false,0,true),50));
 packet(tr,"tracker-ack");IrrigationConfig updated;assert(tr.pop_config_update(updated));assert(updated.position_interval_sec==120);
 IrrigationModule tr_reboot;boot(tr_reboot,tracker);assert(!tr_reboot.on_packet_rx(command(uuid,epoch,false,0,true),51));drain(tr_reboot);
 auto none=config();none.enabled=false;IrrigationModule disabled;enforce_checkpoint=false;assert(!disabled.begin(none));assert(gpio.back()==LOW);enforce_checkpoint=true;
 std::cout<<"PASS commands: durable replay rejection, duplicate no re-actuation/no timer extension, timestamp ties/order, expiry, bad signature, duration, active-low and tracker\n";
}
void test_storage_failures(){
 for(int failure=0;failure<5;++failure){
  flash::reset();IrrigationModule m;boot(m);unsigned before=opens;
  if(failure==0)flash::fail_write=true;
  if(failure==1)flash::fail_commit=true;
  if(failure==2)flash::drop_commit=true;
  if(failure==3)flash::fail_read=true;
  if(failure==4)flash::fail_open=true;
  assert(!m.on_packet_rx(command(),10));assert(opens==before);drain(m);
  flash::fail_open=flash::fail_write=flash::fail_commit=flash::drop_commit=flash::fail_read=false;
  assert(!m.on_packet_rx(command("later",epoch+1),11));assert(opens==before);drain(m);
  assert(m.on_packet_rx(command("close",epoch+2,false,0),12));assert(gpio.back()==LOW);drain(m);
 }
 flash::reset();IrrigationModule prior;boot(prior);assert(prior.on_packet_rx(command(),1));drain(prior);
 assert(prior.on_packet_rx(command("prior-close",epoch+1,false,0),2));drain(prior);
 auto baseline_record=flash::durable;auto prior_opens=opens;flash::fail_commit=true;
 assert(!prior.on_packet_rx(command("failed-new-open",epoch+2),3));assert(opens==prior_opens);assert(gpio.back()==LOW);drain(prior);
 assert(flash::durable==baseline_record);flash::fail_commit=false;IrrigationModule prior_reboot;boot(prior_reboot);
 assert(!prior_reboot.on_packet_rx(command(),4));assert(opens==prior_opens);drain(prior_reboot);
 flash::reset();flash::durable={1,2,3};IrrigationModule corrupt;boot(corrupt);auto before=opens;
 assert(!corrupt.on_packet_rx(command(),20));assert(opens==before);drain(corrupt);
 assert(corrupt.on_packet_rx(command("close",epoch,false,0),21));drain(corrupt);
 flash::reset();IrrigationModule interrupted;boot(interrupted);before=opens;flash::crash_after_commit=true;
 try{interrupted.on_packet_rx(command(),30);assert(false);}catch(const PowerCut&){}
 assert(opens==before);IrrigationModule recovered;boot(recovered);assert(!recovered.on_packet_rx(command(),31));assert(opens==before);drain(recovered);
 flash::reset();auto growing=config(true);growing.line_id=String(55,'l');IrrigationModule interval_ack;boot(interval_ack,growing);
 assert(!interval_ack.on_packet_rx(command(uuid,epoch,false,0,true,86400),38));assert(flash::durable.empty());
 IrrigationConfig unchanged;assert(!interval_ack.pop_config_update(unchanged));
 flash::reset();growing.position_interval_sec=86400;IrrigationModule rejection_ack;boot(rejection_ack,growing);
 assert(!rejection_ack.on_packet_rx(command(uuid,epoch,false,0,true,60),38));assert(flash::durable.empty());
 assert(!rejection_ack.pop_config_update(unchanged));
 flash::reset();auto long_line=config();long_line.line_id=String(128,'l');IrrigationModule oversized_ack;boot(oversized_ack,long_line);
 before=opens;assert(command().length()<=233);assert(!oversized_ack.on_packet_rx(command(),39));assert(opens==before);assert(flash::durable.empty());
 flash::reset();auto long_ids=config();long_ids.tracker_id=String(128,'r');long_ids.line_id=String(128,'l');IrrigationModule too_long;boot(too_long,long_ids);
 auto payload=command();auto at=payload.find("test_00000001");payload.replace(at,13,long_ids.tracker_id);IrrigationCommand c{};
 assert(irrigation_command_from_json(payload,c));auto sig=irrigation_command_signature(c,key);auto start=payload.find("\"s\":\"")+5;payload.replace(start,32,sig);
 before=opens;assert(!too_long.on_packet_rx(payload,40));assert(opens==before);assert(flash::durable.empty());
 uint64_t issued;String id;IrrigationCommandStore store;
 assert(store.load(issued,id));assert(!issued && id.empty());
 assert(store.save(epoch,String(128,'c')));assert(store.load(issued,id));assert(issued==epoch && id.length()==128);
 assert(!store.save(epoch,String(129,'c')));
 std::cout<<"PASS persistence faults: corrupt state, NVS open/read/write/commit/readback failures, power cut after commit before GPIO, unrepresentable ACK fails closed\n";
}
int main(){
 IrrigationCommand c{};assert(irrigation_command_from_json(command(),c));
 std::cout<<"SIGNATURE|"<<irrigation_command_signature(c,key)<<"\n";
 test_packets();test_commands();test_storage_failures();
}
"""

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--payload-output", type=pathlib.Path)
    args = parser.parse_args()
    crypto = ROOT / ".pio/libdeps/hwt_gw/Crypto"
    if not (crypto / "SHA256.cpp").exists():
        raise SystemExit("Install the existing hwt_gw dependencies first (Crypto 0.4.0 is required).")
    with tempfile.TemporaryDirectory(prefix="irrigation-radio-commands-") as temp:
        directory = pathlib.Path(temp)
        for name, content in {
            "Arduino.h": ARDUINO, "nvs.h": NVS,
            "esp_system.h": "#pragma once\n#include <cstdint>\ninline uint32_t esp_random(){ static uint32_t n=0; return ++n; }\n",
            "configuration.h": "#pragma once\n",
            "DebugConfiguration.h": "#pragma once\ntemplate<class... T> void log_ignore(T...){ }\n#define LOG_WARN(...) log_ignore(__VA_ARGS__)\n#define LOG_INFO(...) log_ignore(__VA_ARGS__)\n#define LOG_ERROR(...) log_ignore(__VA_ARGS__)\n",
            "test.cpp": TEST,
        }.items():
            (directory / name).write_text(content)
        module = ROOT / "src/modules/irrigation"
        sources = [module / (name + ".cpp") for name in (
            "IrrigationPacketCodec", "IrrigationCommandAuth", "IrrigationModule",
            "IrrigationCommandStore", "IrrigationMotionDetector")]
        sources.extend(crypto / name for name in ("SHA256.cpp", "Hash.cpp", "Crypto.cpp"))
        binary = directory / "test"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-sign-compare", "-DARCH_ESP32",
                        "-I", str(directory), "-I", str(module), "-I", str(crypto),
                        str(directory / "test.cpp"), *(str(p) for p in sources), "-o", str(binary)], check=True)
        result = subprocess.run([str(binary)], text=True, capture_output=True)
        if result.returncode:
            print(result.stdout, end="")
            print(result.stderr, end="")
            raise SystemExit(result.returncode)
        packets = []
        for line in result.stdout.splitlines():
            if line.startswith("PACKET|"):
                _, label, raw = line.split("|", 2)
                event = json.loads(raw)
                assert len(raw.encode()) <= 233
                if event["p"] in ("A", "R"):
                    assert event["c"] == "12345678-1234-4567-89ab-123456789abc"
                    assert "a" not in event and "o" not in event
                packets.append({"label": label, "bytes": len(raw.encode()), "event": event})
            elif line.startswith("SIGNATURE|"):
                canonical = "v1|36:12345678-1234-4567-89ab-123456789abc|13:test_00000001|V|1794000000000|1794000120000|1|10|0"
                expected = hmac.new(b"UNIT_TEST_ONLY_0123456789_abcdefghijklmnopqrstuvwxyz", canonical.encode(), hashlib.sha256).hexdigest()[:32]
                assert line.split("|", 1)[1] == expected, "Firmware HMAC contract changed"
            else:
                print(line)
        by_label = {p["label"]: p["event"] for p in packets}
        assert by_label["valve-ack"]["w"] == 1
        assert by_label["reboot-reject"]["w"] == 0
        assert by_label["tracker-ack"]["u"] == 120
        for label in ("valve-ack", "reboot-reject", "tracker-ack"):
            assert by_label[label]["b"] == 4200 and by_label[label]["q"] == "G"
        print("PASS unchanged HMAC signature against independent Python implementation")
        print("Actual packet sizes:", ", ".join(f"{p['label']}={p['bytes']} bytes" for p in packets))
        if args.payload_output:
            args.payload_output.write_text(json.dumps(packets, indent=2))

if __name__ == "__main__":
    main()
