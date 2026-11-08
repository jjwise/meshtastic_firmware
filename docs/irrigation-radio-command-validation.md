# Irrigation radio and command validation

Date: 8 November 2026. Firmware baseline: `92b11c4fd`.

## Wire compatibility

The JSON key names and the command HMAC canonical string/tag are unchanged. The
firmware still reads the old, full telemetry messages. The local backend codec
accepts the new packets without modification; the server deployment must be
checked separately.

Generated telemetry `i` values are now 32 hexadecimal characters: the first
128 bits of SHA-256 over tracker ID, device time, uptime, sequence and a per-boot
nonce. This avoids repeating the tracker ID and several long decimal counters
in every radio packet. Message IDs are opaque to the backend. Command IDs `c`
are preserved exactly, including UUIDs.

ACK/rejection packets contain `i,t,r,l,e,n,p,c` and the applicable current state
(`w` or `u`). Their reason remains `A`/`R`. They omit position, motion and
heading; battery and clock quality are retained when the remaining budget permits. They report firmware output state, not measured valve position or
water flow. Ordinary telemetry retains position, motion and fix/stale flags.
Battery, time quality, heading/heading-valid and hop metadata are appended, in
that order, only when they fit. Missing values use the existing backend defaults;
in particular, a missing heading is invalid and a missing time quality is UNKNOWN.
Backend state updates should not interpret missing ACK telemetry as a fresh GPS
position or a measured zero battery.

Every successfully serialized message is at most 233 bytes. IDs are never
silently truncated. Arbitrarily long combinations (for example a 128-byte tracker
ID plus a 128-byte line ID) cannot fit this radio format and are rejected. A
command whose acknowledgement cannot fit (checking both the current rejection
state and the prospective success state) is rejected before any action or
checkpoint write. Oversized MQTT commands are rejected before forwarding to
radio; their signed payload is never shortened or rewritten.

## Persistent command consumption

On ESP32, one versioned NVS blob in namespace `irrigation-cmd`, key `checkpoint`,
stores the latest consumed command timestamp and ID. No HMAC key is stored there.
The node commits and verifies this record before opening a valve or changing a
tracker interval. If the write, commit or readback fails, opening/configuration
commands remain disabled until a successful startup load. A corrupt checkpoint
also fails closed; it is not silently reset.

The output is initialized closed before loading the record, including when the
module is disabled. A reboot never resumes an opening. Retrying a consumed
command after reboot produces a rejection with the current closed state, rather
than reopening. A power loss after commit but before actuation consumes the
command without executing it; issue a new command if another action is wanted.
Within one boot, retrying the last accepted ID returns its result without
re-actuating or extending the opening timer. Different commands must have
strictly increasing `x` timestamps per device; a different ID with the same
millisecond timestamp is rejected for opening/configuration. The HMAC contract
has not changed.

The existing fail-safe exception for authenticated close commands is retained:
they can close even without valid time or a usable checkpoint. Automatic closure
at the requested duration is independent of connectivity. Factory erasing NVS
also erases this history; application-only updates should retain NVS.
The storage API and power-loss model are documented in the
[ESP-IDF NVS guide](https://docs.espressif.com/projects/esp-idf/en/v4.4.8/esp32/api-reference/storage/nvs_flash.html).

## Reproducible host tests

Run on a machine with Python 3 and g++ after installing the existing hwt_gw
PlatformIO dependencies (the harness uses its pinned Crypto library):

```sh
python3 bin/test-irrigation-radio-commands.py
python3 bin/test-irrigation-gateway-store.py
python3 bin/test-irrigation-mqtt-memory.py
```

The first harness compiles the production packet codec, command authentication,
module, motion detector and checkpoint store. Only GPIO, NVS and unused network/
queue services are doubles. It covers UUID ACKs, uint64 timestamps/uptime, uint32
sequence, long-ID rejection, active-high/active-low closure, duplicate retries,
expiry, invalid signature, ordering/ties, tracker commands, checkpoint failures
and a simulated power cut after commit before output activation. It independently
checks the real firmware HMAC against Python hashlib/hmac.

Actual C++ packets are also checked with the local backend's real mqtt_codec.py
and Pydantic 2.10.6. This is contract validation, not evidence of a deployed server
or a physical valve.

## Bench validation still required

1. Provision the existing API key into the ignored command secrets header for
   hwt_valve and hwt_tracker, then rebuild those profiles. The key remains on the
   other machine; no real command key is provisioned in this local checkout.
2. Check the driver output at startup, including active-low hardware. GPIO4 is
   the current profile default; verify the actual configured pin and driver.
3. Send a short opening command, observe physical operation and its ACK in the
   database. Check normal closure and closure at the configured maximum duration.
4. Repeat a command while open and after automatic closure. Neither retry may
   restart the opening or extend its timer.
5. Power-cycle after accepting an opening, then resend that same signed command
   while its TTL is still valid. The node must stay closed and reject it.
6. Test expired/new out-of-order commands, unavailable device time and a bad
   signature. Test a new valid command after reboot with a later timestamp.
7. Verify real serial `raw/bytes/max` sizes and end-to-end arrival through the
   gateway, MQTT, PostgreSQL and dashboard. Confirm ACKs have the exact command ID.

No hardware has been flashed or actuated by this change. The four server defects
(ACK/SENT race, stale state, initial publish timeout and worker resilience) remain
owned by the server work; no backend source is modified here.

## Build results

All three final builds succeeded; modified source/object timestamps were checked.

| Profile | Result | RAM bytes | Flash bytes |
| --- | --- | --- | --- |
| hwt_gw | SUCCESS | 117700 / 327680 (35.9%) | 1636949 / 3342336 (49.0%) |
| hwt_valve | SUCCESS | 123052 / 327680 (37.6%) | 1728381 / 3342336 (51.7%) |
| hwt_tracker | SUCCESS | 123052 / 327680 (37.6%) | 1728357 / 3342336 (51.7%) |

Host C++ fixtures: valve position 193 bytes, valve ACK/reboot rejection 206 bytes,
tracker ACK 209 bytes. Extreme-value tests enforce the 233-byte ceiling; identifier
combinations that cannot fit are explicitly rejected. The real backend normalizer
accepts all four fixtures, including UUID, profile, valve state and tracker interval.

Builds without the real command key are compilation checks only, not provisioned
images for deployment. No hardware has been flashed or actuated.

## Follow-up: real ESP32 timestamp and MQTT farm (8 November 2026)

A real valve capture contained `"t":lu`, so its JSON was rejected before the node
could be registered. The codec's `%llu` call has been replaced by a bounded,
printf-independent decimal conversion, including zero and UINT64_MAX. The same
conversion is used in the message-ID input for timestamps, uptime and boot nonce.
The command HMAC code is unchanged; the installed ESP32 Arduino String uint64
constructor uses its own division-based ulltoa rather than printf.

The host harness now wraps snprintf at link time and deliberately returns `lu`
for unsupported long-long formats. A probe verifies that the simulation is active;
production packets must still round-trip all tested uint64 timestamps and parse
as JSON integers. This closes the gap left by the desktop printf implementation,
which accepted `%llu` in the earlier tests. The simulation reproduces the reported
symptom, rather than claiming that all SDK Nano formatting details are emulated.
The local SDK configuration does not itself enable NEWLIB_NANO_FORMAT; the exact
runtime printf configuration on the remote valve has not been measured here.

The gateway profile now explicitly defines IRRIGATION_FARM_ID as farm_01, matching
the deployed profiles and inspected ACL. Build overrides run after loading the
persisted irrigation config, so a stored farm_demo does not require deleting
configuration or replay-protection state. The gateway constructs the MQTT farm
prefix; flashing only the valve will not change that prefix. Reflash hwt_gw to
apply the topic change. The generic demo defaults are retained for other uses.

This change does not manufacture a GPS fix. A valve transmitting (0, 0) without
fix may be listed by the backend, but a map marker still needs a valid position.

During this follow-up, the command secrets header is present locally. Its value
is masked and untouched; matching it to the API key remains a provisioning check.
The ESP32 on COM4 was identified by the operator as the gateway, not the valve.
No automatic flash has been performed as part of the source correction.

Follow-up builds succeeded: hwt_valve (RAM 37.6%, flash 51.7%) and hwt_gw
(RAM 35.9%, flash 49.0%). The final codec objects and gateway build flag were
checked. The snprintf regression fails with the previous formatter and passes
with the new conversion; backend normalization and the HMAC reference also pass.
The secret headers are byte-for-byte unchanged. Hardware flash/serial validation
of the new valve packet remains an operator step on the machine with that valve.


## Follow-up: ESP32 checkpoint implementation (8 November 2026)

A signed opening for 300 seconds was rejected even though the device timestamp
was within the two-minute command window. Inspection of the local ESP32 valve
object showed that IrrigationCommandStore::load/save had no NVS references and
returned false unconditionally. This translation unit included Arduino.h but
not configuration.h, which supplies the project's ARCH_ESP32 definition through
architecture.h. Its non-ESP32 fallback was compiled on the ESP32.

IrrigationCommandStore.cpp now includes configuration.h before its architecture
guards. The host harness defines only ARDUINO_ARCH_ESP32, as the platform does;
its configuration stub supplies ARCH_ESP32 only when explicitly included.
DebugConfiguration.h includes that stub just as the production header does.
This prevents the former global -DARCH_ESP32 from masking missing includes.

Failed command checks also log time/order validity, checkpoint success, configured
pin and duration limits. These diagnostics contain no HMAC key or signature.
The durable record format, signing contract and GPIO safety checks are unchanged.
No NVS erase or hardware flash is part of this correction.

Validation: the corrected host suite passes, including valid commands, persistent
replay rejection, storage faults, GPIO closure and the independent HMAC reference.
The old command-store file fails the same platform-aware test on the first valid
opening (unused-parameter warnings were disabled only in the isolated negative
control). Both hwt_valve and hwt_tracker builds succeed; their fresh ESP32 objects
reference nvs_open/get_blob/set_blob/commit/close and depend on configuration.h,
architecture.h and nvs.h. These checks establish compilation and simulated
behavior, not physical valve actuation. The generated secret headers are unchanged.
