#include <WiFi.h>
#include <NodeRemote.h> 

// Remote-managed 最小化測試範例:
// 本程式庫需搭配 NodeAnywhere 後台使用：https://node.mqttgo.io
// 1) 在 NodeAnywhere「我的裝置 -> 新增裝置」取得一次性 Token 與 Device UID
// 2) 貼到下方後燒錄，即可完成首次註冊與連線
// 3) 首次先提供一組可上網的 WiFi（bootstrap），之後可由雲端下發 wifi_apply_config 來改
// 4) 上線後可在 NodeAnywhere 後台查看裝置狀態、WiFi 訊號、電量等資訊，並下發命令（如 OTA）
// 5) 更多細節與進階功能請參考 NodeRemote 官方網站：https://Node.MQTTgo.io
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* CLAIM_TOKEN = "PASTE_CLAIM_TOKEN";
const char* DEVICE_UID = "PASTE_DEVICE_UID";

// NodeRemote 物件，建構時提供 CLAIM_TOKEN 與 DEVICE_UID
NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

void setup() {
  // 序列埠僅供除錯使用
  Serial.begin(115200);

  // 由 NodeRemote 管理 WiFi：先加入一組 bootstrap AP。
  node.setWifiManaged(true);
  // 加入第一組 WiFi 到 NodeRemote 清單。
  // priority 數字越小優先序越高（建議用 1~5，1 最高，5 最低）。
  // NodeRemote 最多保留 5 組 AP。
  // 首次上線後，可由後台下發 wifi_apply_config 重新設定/排序。
  node.wifiAdd(WIFI_SSID, WIFI_PASS, 1);

  // 開始 claim（首次）/登入（後續）與 MQTT 控制通道
  if (!node.begin()) {
    Serial.print("NodeRemote not ready yet: ");
    Serial.println(node.lastError());
  }
}

void loop() {
  // node.loop必須呼叫，負責：
  // - WiFi 及MQTT 管理
  // - 心跳、遠端命令接收與回覆
  // - OTA 線上更新
  node.loop();
  // 短延遲避免忙迴圈，勿使用過長 delay
  delay(100);
}
