#include <WiFi.h>
#include <NodeRemote.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* CLAIM_TOKEN = "PASTE_CLAIM_TOKEN";
const char* DEVICE_UID = "PASTE_DEVICE_UID";

// Server defaults are built into the library:
// - MQTT host: node.mqttgo.io
// - MQTT port: 8883 (TLS)
// - API base URL: https://node.mqttgo.io
// If your API URL differs, use NodeRemote(apiBaseUrl, token, uid, mqttHost, mqttPort) instead.
NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

const int LED_PIN = 2;

void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  ensureWiFiConnected();
  // Optional: override heartbeat interval. Default is 60s.
  // node.setHeartbeatIntervalMs(60000);
  // Command handler is optional.
  // Default handler supports:
  // - payload "ping"  -> ACK ok+pong
  // - payload "reset" -> ACK ("收到") then reset soon
  // You can override by calling node.setCommandHandler(...)

  if (!node.begin()) {
    Serial.print("Node begin not ready yet: ");
    Serial.println(node.lastError());
    Serial.println("Will keep retrying in loop()...");
  }
}

void loop() {
  ensureWiFiConnected();
  node.loop();

  // Example: redirect your logs to cloud console
  // node.println("hello from device");

  delay(10);
}
