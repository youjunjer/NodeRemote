#include <WiFi.h>
#include <NodeRemote.h>

// 1) Get a one-time Token and a Device UID from NodeAnywhere (My Devices -> Add device).
// 2) Paste them below and flash this sketch.
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* CLAIM_TOKEN = "PASTE_CLAIM_TOKEN";
const char* DEVICE_UID = "PASTE_DEVICE_UID";

NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  node.begin();
}

void loop() {
  node.loop();
  delay(10);
}

