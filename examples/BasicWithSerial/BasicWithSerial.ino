#include <WiFi.h>
#include <NodeRemote.h>

// SerialInPoll 範例:
// 本程式庫需搭配 NodeAnywhere 後台使用：https://node.mqttgo.io
// 1) 在 NodeAnywhere「我的裝置 -> 新增裝置」取得一次性 Token 與 Device UID
// 2) 貼到下方後燒錄，完成首次註冊
// 3) 後台可輸入序列資料到裝置
// 4) 使用者程式不需要 callback，透過 polling 讀取輸入內容
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* CLAIM_TOKEN = "PASTE_CLAIM_TOKEN";
const char* DEVICE_UID = "PASTE_DEVICE_UID";

NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

void setup() {
  Serial.begin(115200);

  // 啟用 NodeRemote 內建 WiFi 管理，先給一組 bootstrap AP。
  node.setWifiManaged(true);
  // priority 數字越小優先序越高（建議 1~5，1 最高，5 最低）。
  // NodeRemote 最多保留 5 組 AP。
  node.wifiAdd(WIFI_SSID, WIFI_PASS, 1);

  if (!node.begin()) {
    Serial.print("NodeRemote not ready yet: ");
    Serial.println(node.lastError());
  }
}

void loop() {
  // node.loop() 必須持續呼叫，讓 NodeRemote 維持服務運作。
  node.loop();

  // 無 callback 模式：輪詢讀取後台傳來的序列輸入。
  while (node.serialInAvailable()) {
    String payload = node.serialInRead();
    Serial.print("[SerialIn] ");
    Serial.println(payload);

    // 你可以在這裡做自己的處理邏輯，例如:
    // if (payload == "led:on") { ... }
  }

  delay(100);
}
