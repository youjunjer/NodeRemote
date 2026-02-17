#include <WiFi.h>
#include <NodeRemote.h>

// Minimal setup:
// 1) 在 NodeAnywhere「我的裝置 -> 新增裝置」取得一次性 Token 與 Device UID
// 2) 貼到下方後燒錄，即可完成首次註冊與連線
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* CLAIM_TOKEN = "PASTE_CLAIM_TOKEN";
const char* DEVICE_UID = "PASTE_DEVICE_UID";

// 使用預設伺服器設定（由 NodeRemote 內建）
NodeRemote node(CLAIM_TOKEN, DEVICE_UID);

void setup() {
  // 序列埠僅供除錯使用
  Serial.begin(115200);

  // 連上 Wi-Fi 後再啟動 NodeRemote
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // 開始 claim（首次）/登入（後續）與 MQTT 控制通道
  if (!node.begin()) {
    Serial.print("NodeRemote not ready yet: ");
    Serial.println(node.lastError());
  }
}

void loop() {
  // 必須高頻呼叫，負責重連、心跳、命令接收與 ACK
  node.loop();
  // 短延遲避免忙迴圈，勿使用過長 delay
  delay(10);
}
