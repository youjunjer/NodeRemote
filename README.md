# NodeRemote

ESP32 library for NodeAnywhere device claim + remote control over MQTT:
`HTTP claim -> receive per-device MQTT credentials -> MQTT up/down`.

## Features
- Wi-Fi handled by your sketch (`.ino`)
- Device claim via HTTP `POST /api/devices/claim`
- Store MQTT credential in NVS (`Preferences`)
- MQTT reconnect and auto subscribe
- Topic layout aligned with your web platform
- Periodic heartbeat JSON (default: 60s)
- Periodic status snapshot JSON (default: 1 hour)
- Remote commands (default handler):
  - `ping`
  - `info`
  - `sleep <sec>` (60~86400)
  - `reset`
- `node.println(...)` -> uplink console logs
- OTA job (download + sha256 verify + reboot)
- Serial debug logs (enabled by default)

## MQTT Topic Layout
- Uplink base: `devices/<deviceUid>/up/...`
- Downlink base: `devices/<deviceUid>/down/...`
- Heartbeat: `devices/<deviceUid>/up/heartbeat`
- Status: `devices/<deviceUid>/up/status`
- Generic uplink: `devices/<deviceUid>/up/<subTopic>`
- ACK helper: `devices/<deviceUid>/up/ack/<commandName>`
- Command subscribe filter: `devices/<deviceUid>/down/#`
- Console logs: `devices/<deviceUid>/up/console`
- OTA progress: `devices/<deviceUid>/up/ota/progress`
- OTA result: `devices/<deviceUid>/up/ota/result`

## Usage
See `examples/BasicNode/BasicNode.ino`.

Minimal setup (NodeRemote only):
```cpp
#include <WiFi.h>
#include <NodeRemote.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* CLAIM_TOKEN = "PASTE_CLAIM_TOKEN";
const char* DEVICE_UID = "PASTE_DEVICE_UID";

NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  node.begin();
}

void loop() {
  node.loop();
}
```

Command handler:
- Optional. If you don't set one, a default handler is enabled.
- Default commands (topic: `devices/<UID>/down/cmd`):
  - `ping` -> ACK ok + pong
  - `info` -> ACK with summary
  - `sleep 300` -> ACK then deep sleep
  - `reset` -> ACK ("收到") then reset soon

## OTA
- Downlink topic: `devices/<UID>/down/ota`
- Payload: JSON with `job_uid`, `url`, `sha256`, `size` (and optional `version`, `firmware_uid`)
- Behavior:
  - downloads firmware (HTTP/HTTPS)
  - validates `sha256`
  - flashes and reboots
  - reports progress/result:
    - `devices/<UID>/up/ota/progress`
    - `devices/<UID>/up/ota/result`

## Quick test with mosquitto
Subscribe:
```bash
mosquitto_sub -h node.mqttgo.io -p 8883 --cafile /etc/ssl/certs/ca-certificates.crt \
  -u <username> -P <password> -t 'devices/<UID>/up/#' -v
```

Send commands/OTA:
- Use the NodeAnywhere web UI (device page -> Remote Control / OTA).
