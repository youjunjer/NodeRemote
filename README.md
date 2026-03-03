# NodeRemote

ESP32/ESP8266 library for NodeAnywhere device claim + remote control.
Use NodeAnywhere to register devices, manage remote commands, and OTA.

## NodeAnywhere 產品介紹（中文）
📢【NodeAnywhere】讓您的物聯網裝置，從此「天涯若比鄰」！

您是否曾為了修改一個 WiFi 密碼，就得前往現場？或是為了更新軟體，必須帶著電腦和 USB 線重新燒錄？  
NodeAnywhere 是專為物聯網打造的雲端管理與遠端控制平台。透過 Google 帳號登入後，即可集中管理裝置，並從後台完成遠端操作。

只要裝置可連網，您就能在後台進行：
- 遠端狀態查看
- 遠端指令控制
- WiFi 設定管理
- OTA 韌體更新

搭配 NodeRemote 程式庫，你可以快速把 ESP32/ESP8266 裝置接入 NodeAnywhere（https://node.mqttgo.io），將維運流程由「現場維護」轉成「雲端管理」。

### 重點功能列表
1. 全方位裝置管理與狀態追蹤：透過 Dashboard 即時查看裝置在線狀態、硬體資訊（晶片型號、記憶體、IP/MAC 位址）及 RSSI。
2. 遠端指令系統：內建 `Ping`、`Info`、`Sleep`、`Reset` 四大核心指令。
3. 雲端序列監控：可透過 `node.println(...)` 上傳裝置端訊息到後台即時查看。
4. WiFi 遠端管理與備援：可遠端掃描 WiFi，並設定最多 5 組 AP，支援優先序與切換策略。
5. OTA 線上更新：支援推送韌體 `.bin` 到遠端裝置，減少 USB 重複燒錄。
6. 健康狀態數據分析：可追蹤心跳與 RSSI 變化，協助觀察長期運作穩定度。
7. 安全機制：一次性 Token 註冊、NVS 儲存憑證、SHA 韌體驗證與裝置主題隔離。

## Required Backend
- This library is designed to work with **NodeAnywhere backend**.
- Backend URL: **https://node.mqttgo.io**
- You need to create device Token + Device UID from NodeAnywhere first, then flash your sketch.

## Version Note
- v1.0.0 is the first stable release.
- Device can maintain up to 5 AP profiles in NVS and auto-retry by priority.

## Features
- Wi-Fi handled by your sketch (`.ino`)
- Device claim from NodeAnywhere token + device UID
- Store device credentials in NVS (`Preferences`)
- Auto reconnect to backend service
- Periodic heartbeat JSON (default: 60s)
- Periodic status snapshot JSON (default: 1 hour)
- Remote commands (default handler):
  - `ping`
  - `info`
  - `sleep <sec>` (60~86400)
  - `reset`
- `node.println(...)` -> uplink console logs
- OTA job (download + reboot)
- Serial debug logs (enabled by default)
- Built-in WiFi management (optional):
  - up to 5 AP profiles
  - two rounds retry (15s per AP) then reboot
  - BOOT(IO0) clears WiFi + MQTT credentials

## Usage
See `examples/BasicNode/BasicNode.ino`.

Minimal setup (NodeRemote only):
```cpp
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
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

TLS CA mode (recommended for production):
```cpp
node.setMqttTlsInsecure(false);
node.setMqttCaCert(R"EOF(
-----BEGIN CERTIFICATE-----
... your CA PEM ...
-----END CERTIFICATE-----
)EOF");
```
Notes:
- Works for ESP32 and ESP8266.
- The same TLS policy is applied to MQTT, HTTPS claim, and HTTPS OTA download.

Command handler:
- Optional. If you don't set one, a default handler is enabled.
- Default commands:
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
- Payload: JSON with `job_uid`, `url`, `sha256`, `size` (and optional `version`, `firmware_uid`)
- Behavior:
  - downloads firmware (HTTP/HTTPS)
  - validates `sha256` (ESP32/ESP8266)
  - flashes and reboots
  - reports progress/result back to NodeAnywhere

Send commands/OTA:
- Use the NodeAnywhere web UI (device page -> Remote Control / OTA).

WiFi management command payloads:
- `{"cmd":"wifi_scan_start"}`
  - device reports scan result to NodeAnywhere
- `{"cmd":"wifi_apply_config","aps":[{"ssid":"A","password":"P","priority":1}, ...]}`
  - accepts 1~5 AP entries, saves to NVS, and reboots
