#include <WiFi.h>
#include <PubSubClient.h>
#include <NodeRemote.h>

// 1) Get a one-time Token and a Device UID from NodeAnywhere (My Devices -> Add device).
// 2) Paste them below and flash this sketch.
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* CLAIM_TOKEN = "PASTE_CLAIM_TOKEN";
const char* DEVICE_UID = "PASTE_DEVICE_UID";

NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

// Optional: demo an additional user MQTT client (your own telemetry/app traffic).
// NodeRemote is separate from this: it handles claim, command, status, console, OTA.
// Set to 1 if you want the demo enabled.
#define DEMO_USER_MQTT 0

#if DEMO_USER_MQTT
// ------ Modify the following to your own broker settings ------
const char* USER_MQTT_HOST = "your-broker-host";
const int USER_MQTT_PORT = 1883;
String USER_MQTT_CLIENT_ID = "";
const char* USER_MQTT_USER = "";
const char* USER_MQTT_PASSWORD = "";
const char* USER_MQTT_PUB_TOPIC = "YourTopic/pub";
const char* USER_MQTT_SUB_TOPIC = "YourTopic/sub";

unsigned long userMqttLastPublishMs = 0;
const unsigned long userMqttPublishIntervalMs = 10000;
WiFiClient userWiFiClient;
PubSubClient userMqtt(userWiFiClient);

void userMqttCallback(char* topic, byte* payload, unsigned int length);
void ensureUserMqttConnected();
#endif

void ensureWiFiConnected();

void setup() {
  Serial.begin(115200);

  ensureWiFiConnected();

  if (!node.begin()) {
    Serial.print("NodeRemote begin not ready yet: ");
    Serial.print(node.lastError());
    Serial.println(", will keep retrying in loop()...");
  }
}

void loop() {
  ensureWiFiConnected();
  node.loop();  // NodeRemote client loop

#if DEMO_USER_MQTT
  ensureUserMqttConnected();
  if (millis() - userMqttLastPublishMs >= userMqttPublishIntervalMs) {
    const String payload = "hello";
    userMqtt.publish(USER_MQTT_PUB_TOPIC, payload.c_str());
    Serial.println("User MQTT published: hello");
    node.println("User MQTT published: hello");
    userMqttLastPublishMs = millis();
  }
  userMqtt.loop();
#endif

  delay(10);
}

void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tryCount = 0;  // ~60 seconds
  while (WiFi.status() != WL_CONNECTED) {
    if (tryCount++ >= 60) {
      Serial.println("WiFi connect timeout, restarting...");
      ESP.restart();
    }
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.print("WiFi connected, IP=");
  Serial.println(WiFi.localIP());
}

#if DEMO_USER_MQTT
void ensureUserMqttConnected() {
  if (userMqtt.connected()) return;
  userMqtt.setServer(USER_MQTT_HOST, USER_MQTT_PORT);
  userMqtt.setCallback(userMqttCallback);

  while (!userMqtt.connected()) {
    if (USER_MQTT_CLIENT_ID.length() == 0) {
      USER_MQTT_CLIENT_ID = "esp32-" + String(random(1000000, 9999999));
    }
    if (userMqtt.connect(USER_MQTT_CLIENT_ID.c_str(), USER_MQTT_USER, USER_MQTT_PASSWORD)) {
      Serial.println("User MQTT connected");
      userMqtt.subscribe(USER_MQTT_SUB_TOPIC);
    } else {
      userMqtt.disconnect();
      Serial.print("User MQTT connect failed, state=");
      Serial.println(userMqtt.state());
      delay(5000);
    }
  }
}

void userMqttCallback(char* topic, byte* payload, unsigned int length) {
  String payloadString;
  payloadString.reserve(length);
  for (unsigned int i = 0; i < length; i++) {
    payloadString += static_cast<char>(payload[i]);
  }
  Serial.print("User MQTT received topic=");
  Serial.print(topic);
  Serial.print(" payload=");
  Serial.println(payloadString);
  node.println(String("User MQTT received: ") + topic + " " + payloadString);
}
#endif
