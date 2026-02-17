#include <WiFi.h>
#include <NodeRemote.h>

// NodeRemote + 自有 MQTT 範例：
// - NodeRemote 負責裝置註冊、心跳、遠端命令
// - MQTTClient 負責你自己的業務資料收發
// 先到後台取得一次性 Token 與 Device UID，完成首次註冊
const char* CLAIM_TOKEN = "your token";
const char* DEVICE_UID = "your UID";
NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

// ------ 使用者自行設定區 ------
char* WIFI_SSID = "wifi ssid";
char* WIFI_PASS = "wifi password";
char* MQTTServer = "mqttgo.io";        // 你的 MQTT 伺服器
int MQTTPort = 1883;                   // 你的 MQTT Port
String MQTTClientId = "";              // 你的 MQTT ClientID（空白則自動產生）
char* MQTTUser = "";                   // 你的 MQTT 帳號
char* MQTTPassword = "";               // 你的 MQTT 密碼
char* MQTTPubTopic = "YourTopic/pub";  // 發布主題
char* MQTTSubTopic = "YourTopic/sub";  // 訂閱主題

long MQTTLastPublishTime;          // 上次發布時間（毫秒）
long MQTTPublishInterval = 10000;  // 每 10 秒發布一次
WiFiClient WifiClient;
PubSubClient MQTTClient(WifiClient);

void setup() {
  Serial.begin(115200);

  // 啟動時先確保網路與自有 MQTT 可用
  WiFiConnect();
  MQTTConnect();

  // 啟動 NodeRemote（claim / MQTT 控制通道）
  if (!node.begin()) {
    Serial.print("Node Remote begin not ready yet: ");
    Serial.print(node.lastError());
    Serial.println(", Will keep retrying in loop()...");
  }
}

void loop() {
  // 斷線自動重連
  WiFiConnect();
  MQTTConnect();

  // 範例：每 10 秒送一次你的 MQTT 訊息
  if ((millis() - MQTTLastPublishTime) >= MQTTPublishInterval) {

    String payload = "hello";
    MQTTClient.publish(MQTTPubTopic, payload.c_str());
    Serial.println("MQTT data has been uploaded...");
    // 同步寫到 NodeAnywhere 對話紀錄（可選）
    node.println("MQTT data has been uploaded...");
    MQTTLastPublishTime = millis();  // 更新計時點
  }

  // 兩個 loop 都要持續執行，互不衝突
  MQTTClient.loop();  // 你的 MQTT client 維護
  node.loop();        // NodeRemote 維護（心跳/命令/重連）
  delay(100);         // 短暫延遲，避免忙迴圈
}

// Wi-Fi 重連邏輯
void WiFiConnect() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tryCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (tryCount++ >= 60) ESP.restart();  // 約 60 秒失敗則重開
    Serial.println(".");
    delay(1000);
  }
}

// 自有 MQTT 連線邏輯
void MQTTConnect() {
  if (MQTTClient.connected()) return;
  MQTTClient.setServer(MQTTServer, MQTTPort);
  MQTTClient.setCallback(MQTTCallback);
  while (!MQTTClient.connected()) {
    // 若未指定 ClientID，則隨機產生避免撞名
    if (MQTTClientId == "") MQTTClientId = "esp32-" + String(random(1000000, 9999999));
    if (MQTTClient.connect(MQTTClientId.c_str(), MQTTUser, MQTTPassword)) {
      // 連線成功後訂閱主題
      Serial.println("MQTT connected");
      MQTTClient.subscribe(MQTTSubTopic);
    } else {
      // 連線失敗：印出狀態並延遲重試
      MQTTClient.disconnect();
      Serial.print("MQTT connection failed, status code=");
      Serial.println(MQTTClient.state());
      Serial.println("Reconnecting in five seconds");
      delay(5000);
    }
  }
}

// 收到訂閱訊息時觸發
void MQTTCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print(topic);
  Serial.print(" subscription notification: ");
  String payloadString;  // 將 payload 轉為字串
  for (int i = 0; i < length; i++) {
    payloadString = payloadString + (char)payload[i];
  }
  Serial.println(payloadString);
  // 將訊息同步到 NodeAnywhere 對話紀錄（可選）
  node.println(String(topic) + " receive: " + payloadString);
}
