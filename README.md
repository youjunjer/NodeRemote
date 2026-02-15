# NodeRemote

ESP32 library for remote monitor/control with your current backend flow:
`HTTP claim -> get MQTT credential -> MQTT up/down`.

## Features
- Wi-Fi handled by your sketch (`.ino`)
- Device claim via HTTP `POST /api/devices/claim`
- Store MQTT credential in NVS (`Preferences`)
- MQTT reconnect and auto subscribe
- Topic layout aligned with your web platform
- Periodic heartbeat JSON
- Periodic status snapshot JSON (default: 1 hour)
- Command callback from downlink topic
- `node.println(...)` -> uplink console logs
- OTA via downlink job (download + sha256 verify + reboot)
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

Minimal setup:
```cpp
// Handle Wi-Fi in your setup()/loop() first
// Default server config is built in:
// - MQTT host: node.mqttgo.io
// - MQTT port: 8883 (TLS)
// - API base URL: https://node.mqttgo.io
NodeRemote node(CLAIM_TOKEN, DEVICE_UID);
// Heartbeat default is 60s. Optional override:
// node.setHeartbeatIntervalMs(60000);
// Optional: disable logs
// node.setDebugEnabled(false);
node.begin();

// Publish plain log lines to cloud console:
// node.println("boot ok");
```

Command handler:
- Optional. If you don't set one, a default handler is enabled.
- Default commands (topic: `devices/<UID>/down/cmd`):
  - `ping` -> ACK ok + pong
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

Send command:
```bash
mosquitto_pub -h <broker-ip> -t 'devices/<UID>/down/cmd' -m led_on
```
