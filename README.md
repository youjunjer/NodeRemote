# NodeRemote

ESP32 library for NodeAnywhere device claim + remote control over MQTT:
`HTTP claim -> receive per-device MQTT credentials -> MQTT up/down`.

## Version Note
- Current release does **not** include built-in WiFi list management (scan/add/remove/reorder from cloud).
- WiFi connection is still managed by user sketch (`WiFi.begin(...)`) in this version.

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
