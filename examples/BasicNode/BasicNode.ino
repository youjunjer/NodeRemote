#include <WiFi.h>
#include <NodeRemote.h>

// Please obtain the device Token and device UID from the backend to complete the first-time registration.
const char* CLAIM_TOKEN = "your token";
const char* DEVICE_UID = "your UID";
NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

// ------ Modify the following to your settings ------
char* WIFI_SSID = "wifi ssid";
char* WIFI_PASS = "wifi password";
char* MQTTServer = "mqttgo.io";        // MQTT server with no registration required
int MQTTPort = 1883;                   // MQTT Port
String MQTTClientId = "";              // MQTT ClientID
char* MQTTUser = "";                   // MQTT account
char* MQTTPassword = "";               // MQTT Password
char* MQTTPubTopic = "YourTopic/pub";  // Publish topic: publishing
char* MQTTSubTopic = "YourTopic/sub";  // Subscribe topic: subscribing

long MQTTLastPublishTime;          // This variable is used to record the publish time
long MQTTPublishInterval = 10000;  // Publish once every 10 seconds
WiFiClient WifiClient;
PubSubClient MQTTClient(WifiClient);

void setup() {
  Serial.begin(115200);

  WiFiConnecte();
  MQTTConnect();
  if (!node.begin()) {
    Serial.print("Node Remote begin not ready yet: ");
    Serial.print(node.lastError());
    Serial.println(", Will keep retrying in loop()...");
  }
}

void loop() {
  WiFiConnecte();
  MQTTConnect();
  if ((millis() - MQTTLastPublishTime) >= MQTTPublishInterval) {

    String payload = "hello";
    MQTTClient.publish(MQTTPubTopic, payload.c_str());
    Serial.println("MQTT data has been uploaded...");
    node.println("MQTT data has been uploaded...");  // publish to NodeAnywhere conversation
    MQTTLastPublishTime = millis();     // Update LastPublishTime
  }

  MQTTClient.loop();  // your mqtt client loop
  node.loop();        // NodeRemote client loop
  delay(100);         // pause 100ms
}

void WiFiConnecte() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tryCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (tryCount++ >= 60) ESP.restart();
    Serial.println(".");
    delay(1000);
  }
}

// Start MQTT connection
void MQTTConnect() {
  if (MQTTClient.connected()) return;
  MQTTClient.setServer(MQTTServer, MQTTPort);
  MQTTClient.setCallback(MQTTCallback);
  while (!MQTTClient.connected()) {
    // randomize a client id
    if (MQTTClientId == "") MQTTClientId = "esp32-" + String(random(1000000, 9999999));
    if (MQTTClient.connect(MQTTClientId.c_str(), MQTTUser, MQTTPassword)) {
      // Connected successfully; display "Connected".
      Serial.println("MQTT connected");
      MQTTClient.subscribe(MQTTSubTopic);
    } else {
      // If connection fails, display error message and reconnect
      MQTTClient.disconnect();
      Serial.print("MQTT connection failed, status code=");
      Serial.println(MQTTClient.state());
      Serial.println("Reconnecting in five seconds");
      delay(5000);
    }
  }
}

// When a subscribed message is received
void MQTTCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print(topic);
  Serial.print(" subscription notification: ");
  String payloadString;  // Convert received payload to a string
  // Display subscription content
  for (int i = 0; i < length; i++) {
    payloadString = payloadString + (char)payload[i];
  }
  Serial.println(payloadString);
  node.println(String(topic) + " receive: " + payloadString);
}
