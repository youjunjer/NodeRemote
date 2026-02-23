# NodeRemote

ESP32 library for NodeAnywhere device claim + remote control over MQTT:
`HTTP claim -> receive per-device MQTT credentials -> MQTT up/down`.

## Required Backend
- This library is designed to work with **NodeAnywhere backend**.
- Backend URL: **https://node.mqttgo.io**
- You need to create device Token + Device UID from NodeAnywhere first, then flash your sketch.

## Version Note
- v0.5.10 includes first-stage built-in WiFi management in NodeRemote.
- Device can maintain up to 5 AP profiles in NVS and auto-retry by priority.

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
- Built-in WiFi management (optional):
  - up to 5 AP profiles
  - two rounds retry (15s per AP) then reboot
  - BOOT(IO0) clears WiFi + MQTT credentials

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

## Reclaim / Local Recovery
- At boot, NodeRemote waits 5 seconds and prints a serial hint:
  - `startup window 5s: press BOOT(IO0) to clear NVS credentials`
- If you press `BOOT (IO0)` during this window, NodeRemote clears stored NVS credentials:
  - `mqtt_user`
  - `mqtt_pass`
  - `device_uid`
- Use this when a device was removed/revoked and you need to re-register with a new Token + UID.

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

Send commands/OTA:
- Use the NodeAnywhere web UI (device page -> Remote Control / OTA).

WiFi management command payloads (MQTT `down/cmd` JSON):
- `{"cmd":"wifi_scan_start"}`
  - publishes scan result to `devices/<UID>/up/wifi/scan_result`
- `{"cmd":"wifi_apply_config","aps":[{"ssid":"A","password":"P","priority":1}, ...]}`
  - accepts 1~5 AP entries, saves to NVS, and reboots
