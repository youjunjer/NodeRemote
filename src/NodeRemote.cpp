#include "NodeRemote.h"

#include <ArduinoJson.h>
#include <ESP.h>
#if defined(ESP8266)
#include <ESP8266HTTPClient.h>
#else
#include <HTTPClient.h>
#endif
#include <Update.h>
#include <WiFiClientSecure.h>
#if defined(ESP32)
#include <mbedtls/sha256.h>
#include <esp_sleep.h>
#include <esp_system.h>
extern "C" uint32_t esp_random(void);
#elif defined(ESP8266)
#include <Hash.h>
#endif

static const char* kDefaultMqttHost = "node.mqttgo.io";
static const uint16_t kDefaultMqttPort = 8883;
// NOTE: If your API is not on this URL (or uses HTTPS), override via setClaimConfig().
static const char* kDefaultApiBaseUrl = "https://node.mqttgo.io";

static String chipModelString() {
#if defined(ESP8266)
  return "ESP8266";
#else
  // Avoid relying on IDF-only chip info headers/types (esp_chip_info_t/CHIP_*) because
  // Arduino-ESP32 exposes stable helpers via ESP.* across core versions.
  //
  // Arduino-ESP32 3.x: ESP.getChipModel() returns a String like "ESP32", "ESP32-S3".
  String model = ESP.getChipModel();
  model.trim();
  if (!model.isEmpty()) {
    return model;
  }
  return "ESP32";
#endif
}

static String resetReasonString() {
#if defined(ESP8266)
  String reason = ESP.getResetReason();
  reason.trim();
  if (!reason.isEmpty()) return reason;
  return "unknown";
#else
  const esp_reset_reason_t r = esp_reset_reason();
  switch (r) {
    case ESP_RST_POWERON:
      return "poweron";
    case ESP_RST_EXT:
      return "ext";
    case ESP_RST_SW:
      return "sw";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
      return "int_wdt";
    case ESP_RST_TASK_WDT:
      return "task_wdt";
    case ESP_RST_WDT:
      return "wdt";
    case ESP_RST_DEEPSLEEP:
      return "deepsleep";
    case ESP_RST_BROWNOUT:
      return "brownout";
    case ESP_RST_SDIO:
      return "sdio";
    default:
      return "unknown";
  }
#endif
}

static String sha256HexLower(const uint8_t* digest32) {
  static const char* kHex = "0123456789abcdef";
  char out[65];
  for (int i = 0; i < 32; i++) {
    out[i * 2] = kHex[(digest32[i] >> 4) & 0x0F];
    out[i * 2 + 1] = kHex[digest32[i] & 0x0F];
  }
  out[64] = '\0';
  return String(out);
}

#if defined(ESP8266)
bool Preferences::fsReady_ = false;

bool Preferences::begin(const char* ns, bool readOnly) {
  ns_ = String(ns ? ns : "");
  readOnly_ = readOnly;
  dirty_ = false;
  count_ = 0;
  if (!fsReady_) {
    fsReady_ = LittleFS.begin();
  }
  if (!fsReady_) {
    return false;
  }
  return load_();
}

void Preferences::end() {
  if (!readOnly_ && dirty_) {
    save_();
  }
}

String Preferences::filePath_() const {
  String p = "/nrp_";
  p += ns_;
  p += ".cfg";
  return p;
}

int Preferences::findKey_(const String& key) const {
  for (uint8_t i = 0; i < count_; i++) {
    if (keys_[i] == key) return static_cast<int>(i);
  }
  return -1;
}

bool Preferences::load_() {
  count_ = 0;
  File f = LittleFS.open(filePath_(), "r");
  if (!f) return true;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;
    const int sep = line.indexOf('\t');
    if (sep <= 0) continue;
    if (count_ >= kMaxEntries) break;
    keys_[count_] = line.substring(0, sep);
    vals_[count_] = line.substring(sep + 1);
    count_++;
  }
  f.close();
  return true;
}

bool Preferences::save_() {
  File f = LittleFS.open(filePath_(), "w");
  if (!f) return false;
  for (uint8_t i = 0; i < count_; i++) {
    f.print(keys_[i]);
    f.print('\t');
    f.print(vals_[i]);
    f.print('\n');
  }
  f.close();
  dirty_ = false;
  return true;
}

String Preferences::getString(const char* key, const String& defaultValue) const {
  const int idx = findKey_(String(key ? key : ""));
  if (idx < 0) return defaultValue;
  return vals_[idx];
}

uint32_t Preferences::getUInt(const char* key, uint32_t defaultValue) const {
  const String s = getString(key, "");
  if (s.isEmpty()) return defaultValue;
  return static_cast<uint32_t>(strtoul(s.c_str(), nullptr, 10));
}

int32_t Preferences::getInt(const char* key, int32_t defaultValue) const {
  const String s = getString(key, "");
  if (s.isEmpty()) return defaultValue;
  return static_cast<int32_t>(strtol(s.c_str(), nullptr, 10));
}

bool Preferences::getBool(const char* key, bool defaultValue) const {
  const String s = getString(key, "");
  if (s.isEmpty()) return defaultValue;
  return !(s == "0" || s == "false" || s == "False");
}

size_t Preferences::putString(const char* key, const String& value) {
  if (readOnly_) return 0;
  const String k = String(key ? key : "");
  if (k.isEmpty()) return 0;
  int idx = findKey_(k);
  if (idx < 0) {
    if (count_ >= kMaxEntries) return 0;
    idx = count_++;
    keys_[idx] = k;
  }
  vals_[idx] = value;
  dirty_ = true;
  return value.length();
}

size_t Preferences::putUInt(const char* key, uint32_t value) {
  return putString(key, String(value));
}

size_t Preferences::putInt(const char* key, int32_t value) {
  return putString(key, String(value));
}

size_t Preferences::putBool(const char* key, bool value) {
  return putString(key, value ? "1" : "0");
}

bool Preferences::remove(const char* key) {
  if (readOnly_) return false;
  const int idx = findKey_(String(key ? key : ""));
  if (idx < 0) return false;
  for (uint8_t i = static_cast<uint8_t>(idx); i + 1 < count_; i++) {
    keys_[i] = keys_[i + 1];
    vals_[i] = vals_[i + 1];
  }
  count_--;
  dirty_ = true;
  return true;
}

void Preferences::clear() {
  if (readOnly_) return;
  count_ = 0;
  dirty_ = true;
}
#endif

NodeRemote* NodeRemote::instance_ = nullptr;

NodeRemote::NodeRemote() : mqtt_(netPlain_) {
  commandHandler_ = [this](const String& subTopic, const String& payload) { handleDefaultCommand(subTopic, payload); };
}

NodeRemote::NodeRemote(const char* claimToken, const char* deviceUid) : mqtt_(netPlain_) {
  commandHandler_ = [this](const String& subTopic, const String& payload) { handleDefaultCommand(subTopic, payload); };
  setClaimConfig(String(kDefaultApiBaseUrl), String(claimToken ? claimToken : ""), String(deviceUid ? deviceUid : ""));
  setMqttServer(String(kDefaultMqttHost), kDefaultMqttPort);
}

NodeRemote::NodeRemote(
    const char* apiBaseUrl,
    const char* claimToken,
    const char* deviceUid,
    const char* mqttHost,
    uint16_t mqttPort)
    : mqtt_(netPlain_) {
  commandHandler_ = [this](const String& subTopic, const String& payload) { handleDefaultCommand(subTopic, payload); };
  setClaimConfig(String(apiBaseUrl ? apiBaseUrl : ""), String(claimToken ? claimToken : ""), String(deviceUid ? deviceUid : ""));
  setMqttServer(String(mqttHost ? mqttHost : ""), mqttPort);
}

NodeRemote::NodeRemote(PubSubClient& client) : mqtt_(client) {
  ownsClient_ = false;
  commandHandler_ = [this](const String& subTopic, const String& payload) { handleDefaultCommand(subTopic, payload); };
}

PubSubClient& NodeRemote::mqtt() {
  return mqtt_;
}

void NodeRemote::setClaimConfig(const String& apiBaseUrl, const String& claimToken, const String& deviceUid) {
  apiBaseUrl_ = apiBaseUrl;
  claimToken_ = claimToken;
  deviceUid_ = deviceUid;
  updateTopicBases();
}

void NodeRemote::setMqttServer(const String& host, uint16_t port) {
  mqttHost_ = host;
  mqttPort_ = port;
  if (ownsClient_) {
    mqtt_.setClient(mqttTlsEnabled_ ? static_cast<Client&>(netTls_) : static_cast<Client&>(netPlain_));
  }
  mqtt_.setServer(mqttHost_.c_str(), mqttPort_);
}

void NodeRemote::setMqttTlsEnabled(bool enabled) {
  mqttTlsEnabled_ = enabled;
  if (!ownsClient_) {
    return;
  }
  mqtt_.setClient(mqttTlsEnabled_ ? static_cast<Client&>(netTls_) : static_cast<Client&>(netPlain_));
  if (!mqttHost_.isEmpty() && mqttPort_ != 0) {
    mqtt_.setServer(mqttHost_.c_str(), mqttPort_);
  }
}

void NodeRemote::setMqttTlsInsecure(bool insecure) {
  mqttTlsInsecure_ = insecure;
}

void NodeRemote::setMqttCaCert(const char* caCertPem) {
  mqttCaCertPem_ = caCertPem;
#if defined(ESP8266)
  mqttCaCertList_.reset();
  if (caCertPem != nullptr && strlen(caCertPem) > 0) {
    mqttCaCertList_.reset(new BearSSL::X509List(caCertPem));
  }
#endif
}

void NodeRemote::configureTlsClient(WiFiClientSecure& client) {
  // Default remains insecure for easiest first-time onboarding.
  if (mqttTlsInsecure_) {
    client.setInsecure();
    return;
  }

#if defined(ESP32)
  if (mqttCaCertPem_ != nullptr && strlen(mqttCaCertPem_) > 0) {
    client.setCACert(mqttCaCertPem_);
    return;
  }
#elif defined(ESP8266)
  if (mqttCaCertList_) {
    client.setTrustAnchors(mqttCaCertList_.get());
    return;
  }
#endif

  // No CA configured: fallback to insecure to avoid silent connection deadlocks.
  client.setInsecure();
}

void NodeRemote::setHeartbeatIntervalMs(uint32_t intervalMs) {
  heartbeatIntervalMs_ = intervalMs;
}

void NodeRemote::setStatusIntervalMs(uint32_t intervalMs) {
  statusIntervalMs_ = intervalMs;
}

void NodeRemote::setCommandHandler(CommandHandler handler) {
  commandHandler_ = handler;
  if (!commandHandler_) {
    commandHandler_ = [this](const String& subTopic, const String& payload) { handleDefaultCommand(subTopic, payload); };
  }
}

void NodeRemote::setDebugEnabled(bool enabled) {
  debugEnabled_ = enabled;
}

void NodeRemote::setDebugOutput(Print& out) {
  logOut_ = &out;
}

void NodeRemote::setWifiManaged(bool enabled) {
  wifiManagedEnabled_ = enabled;
}

bool NodeRemote::wifiAdd(const String& ssid, const String& password, int priority) {
  String s = ssid;
  s.trim();
  if (s.isEmpty()) return false;
  if (s.length() > 32) return false;
  if (password.length() > 64) return false;

  // If stored WiFi list exists in NVS, do not let sketch bootstrap override it.
  // This keeps remote-applied wifi order/settings persistent across reboots.
  if (wifiApCount_ == 0) {
    wifiPrefs_.begin(kWifiPrefsNs, true);
    const uint32_t storedCount = wifiPrefs_.getUInt("count", 0);
    wifiPrefs_.end();
    if (storedCount > 0) {
      loadWifiConfig();
      logLine("wifi bootstrap skipped: using NVS config");
      return true;
    }
  }

  for (uint8_t i = 0; i < wifiApCount_; i++) {
    if (wifiAps_[i].ssid == s) {
      wifiAps_[i].password = password;
      wifiAps_[i].priority = priority;
      sortWifiByPriority();
      return saveWifiConfig();
    }
  }
  if (wifiApCount_ >= kWifiMaxAps) {
    return false;
  }
  wifiAps_[wifiApCount_].ssid = s;
  wifiAps_[wifiApCount_].password = password;
  wifiAps_[wifiApCount_].priority = priority;
  wifiApCount_++;
  sortWifiByPriority();
  return saveWifiConfig();
}

bool NodeRemote::wifiRemove(const String& ssid) {
  for (uint8_t i = 0; i < wifiApCount_; i++) {
    if (wifiAps_[i].ssid == ssid) {
      for (uint8_t j = i; j + 1 < wifiApCount_; j++) {
        wifiAps_[j] = wifiAps_[j + 1];
      }
      wifiApCount_--;
      return saveWifiConfig();
    }
  }
  return false;
}

bool NodeRemote::wifiClear() {
  clearWifiConfig();
  return true;
}

bool NodeRemote::wifiSetPriority(const String& ssid, int priority) {
  for (uint8_t i = 0; i < wifiApCount_; i++) {
    if (wifiAps_[i].ssid == ssid) {
      wifiAps_[i].priority = priority;
      sortWifiByPriority();
      return saveWifiConfig();
    }
  }
  return false;
}

uint8_t NodeRemote::wifiCount() const {
  return wifiApCount_;
}

String NodeRemote::wifiListJson() const {
  StaticJsonDocument<768> doc;
  doc["managed"] = wifiManagedEnabled_;
  JsonArray arr = doc.createNestedArray("aps");
  for (uint8_t i = 0; i < wifiApCount_; i++) {
    JsonObject it = arr.createNestedObject();
    it["ssid"] = wifiAps_[i].ssid;
    it["priority"] = wifiAps_[i].priority;
  }
  String out;
  serializeJson(doc, out);
  return out;
}

bool NodeRemote::wifiConnectNow() {
  return connectWifiFromManagedList();
}

bool NodeRemote::begin() {
  instance_ = this;
  logLine(String("NodeRemote version ") + kVersion);
  wifiBootStartedMs_ = millis();
  mqtt_.setCallback(staticCallback);
  // PubSubClient's default packet buffer is small (often 256 bytes). Our JSON acks (e.g. "info")
  // can exceed that and cause publish() to fail, leading to web-side ACK timeouts.
  // 1KB keeps overhead small but is enough for our command acks + summary.
  mqtt_.setBufferSize(1024);
  checkBootRevokeWindow();
  loadWifiConfig();
  loadCredentials();
  updateTopicBases();
  // Unconditional marker so users can verify they compiled the correct library.
  Serial.println("[NodeRemote] begin()");
  logLine("begin uid=" + deviceUid_);
  const bool ok = ensureConnected();
  if (!ok && lastError_.isEmpty()) {
    lastError_ = "connecting_retry";
  }
  return ok;
}

void NodeRemote::loop() {
  ensureConnected();
  mqtt_.loop();

  // Run OTA outside MQTT callback context to reduce stack pressure.
  if (otaPending_ && !otaInProgress_) {
    const String payload = otaPendingPayload_;
    otaPendingPayload_ = "";
    otaPending_ = false;
    handleOtaPayload(payload);
  }

  if (sleepPending_ && sleepAtMs_ != 0 && millis() >= sleepAtMs_) {
    sleepPending_ = false;
    sleepAtMs_ = 0;
    logLine("enter deep sleep");
    if (sleepWakeUs_ > 0) {
#if defined(ESP8266)
      ESP.deepSleep(sleepWakeUs_);
#else
      esp_sleep_enable_timer_wakeup(sleepWakeUs_);
#endif
    } else {
#if defined(ESP8266)
      ESP.deepSleep(0);
#endif
    }
    // Best-effort disconnect so broker sees clean session end.
    mqtt_.disconnect();
    delay(50);
#if defined(ESP32)
    esp_deep_sleep_start();
    // Should never return.
    ESP.restart();
#endif
  }

  if (wipeIdentityPending_ && wipeIdentityAtMs_ != 0 && millis() >= wipeIdentityAtMs_) {
    wipeIdentityPending_ = false;
    wipeIdentityAtMs_ = 0;
    logLine("wipe identity now");
    clearCredentials();
    delay(80);
    ESP.restart();
  }

  if (rebootPending_ && rebootAtMs_ != 0 && millis() >= rebootAtMs_) {
    rebootPending_ = false;
    rebootAtMs_ = 0;
    ESP.restart();
  }

  if (!mqtt_.connected()) {
    return;
  }

  const uint32_t now = millis();
  if (heartbeatIntervalMs_ > 0 && now - lastHeartbeatMs_ >= heartbeatIntervalMs_) {
    const bool ok = sendHeartbeat();
    if (ok) {
      logLine("heartbeat sent");
    } else {
      logLine("heartbeat failed");
    }
    lastHeartbeatMs_ = now;
  }

  if (statusIntervalMs_ > 0 && (lastStatusMs_ == 0 || now - lastStatusMs_ >= statusIntervalMs_)) {
    const bool ok = sendStatus();
    if (ok) {
      logLine("status sent");
    } else {
      logLine("status failed");
    }
    lastStatusMs_ = now;
  }
}

bool NodeRemote::ensureConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiManagedEnabled_) {
      const uint32_t now = millis();
      if (lastWifiManageAttemptMs_ == 0 || (now - lastWifiManageAttemptMs_) >= wifiRetryBackoffMs_) {
        lastWifiManageAttemptMs_ = now;
        if (!connectWifiFromManagedList()) {
          if (wifiAutoRebootOnFail_) {
            const uint32_t up = now - wifiBootStartedMs_;
            if (up >= wifiMinBootBeforeRebootMs_) {
              logLine("wifi all rounds failed; rebooting");
              rebootPending_ = true;
              rebootAtMs_ = millis() + 300;
            } else {
              logLine("wifi all rounds failed; waiting until min boot window before reboot");
            }
          }
          lastError_ = "wifi_not_connected";
          return false;
        }
      } else {
        lastError_ = "wifi_reconnect_backoff";
        logThrottled("wifi reconnect backoff", lastWifiLogMs_, 10000);
        return false;
      }
    } else {
      lastError_ = "wifi_not_connected";
      logThrottled("wifi not connected", lastWifiLogMs_, 10000);
      return false;
    }
  }

  if (!hasCredentials()) {
    if (claimPermanentlyFailed_) {
      // Token is invalid/expired/already used: stop spamming server.
      lastError_ = "claim_permanently_failed";
      logThrottled("Claim permanently failed; Please upload a new sketch with a new token and UID.", lastClaimLogMs_, 30000);
      return false;
    }
    if (!shouldAttemptClaimNow()) {
      lastError_ = "claim_backoff";
      logThrottled("claim backoff", lastClaimLogMs_, 15000);
      return false;
    }
    if (!claimDevice()) {
      return false;
    }
  }

  if (mqtt_.connected()) {
    return true;
  }

  const uint32_t now = millis();
  if (lastMqttReconnectAttemptMs_ != 0 && now - lastMqttReconnectAttemptMs_ < 3000) {
    lastError_ = "mqtt_reconnect_backoff";
    logThrottled("NodeAnywhere reconnect backoff", lastMqttLogMs_, 10000);
    return false;
  }
  lastMqttReconnectAttemptMs_ = now;
  return connectMqtt();
}

bool NodeRemote::connectMqtt() {
  if (mqttHost_.isEmpty()) {
    mqttHost_ = String(kDefaultMqttHost);
    mqttPort_ = kDefaultMqttPort;
  }
  if (mqttTlsEnabled_ && ownsClient_) {
    configureTlsClient(netTls_);
  }
  mqtt_.setServer(mqttHost_.c_str(), mqttPort_);

  const String clientId = defaultClientId();
  if (!mqtt_.connect(clientId.c_str(), mqttUser_.c_str(), mqttPass_.c_str())) {
    const int st = mqtt_.state();
    if (st == 5) {
      // Not authorized: clear persisted credentials and force re-claim on next loop.
      clearCredentials();
      lastError_ = "mqtt_not_authorized_reclaiming";
      logLine("NodeAnywhere auth failed (bad/removed device credentials); clearing local credentials and re-claiming");
      return false;
    }
    lastError_ = "mqtt_connect_failed_" + String(st);
    if (st == -4) {
      logLine("NodeAnywhere connect failed: connection timeout (broker reachable but no response)");
    } else if (st == -2) {
      logLine("NodeAnywhere connect failed: network/TLS connect failed (host/port/certificate/firewall)");
    } else if (st == -1) {
      logLine("NodeAnywhere connect failed: disconnected (retrying)");
    } else if (st == 2) {
      logLine("NodeAnywhere connect failed: protocol rejected");
    } else if (st == 3) {
      logLine("NodeAnywhere connect failed: broker unavailable");
    } else if (st == 4) {
      logLine("NodeAnywhere connect failed: bad username/password");
    } else {
      logLine("NodeAnywhere connect failed state=" + String(st));
    }
    return false;
  }
  logLine("NodeAnywhere connected");
  const bool okSub = subscribeDownTopics();
  if (!okSub) {
    return false;
  }
  // Send a status snapshot immediately after connect so the web UI can show
  // chip/ip/rssi/etc without waiting up to the periodic interval.
  if (mqtt_.connected()) {
    if (sendStatus()) {
      lastStatusMs_ = millis();
      logLine("status sent (on connect)");
    } else {
      logLine("status failed (on connect)");
    }
  }
  // Device-side lifecycle event for backend audit:
  // - first_online: sent exactly once after first successful registration.
  // - re_online: all subsequent online transitions (reboot/wakeup/reconnect).
  StaticJsonDocument<192> ev;
  const bool firstOnlineSent = loadFirstOnlineSent();
  const bool isFirstOnline = !firstOnlineSent;
  ev["event"] = isFirstOnline ? "first_online" : "re_online";
  ev["reason"] = "nodeanywhere_connected";
  ev["uptime_sec"] = millis() / 1000;
  ev["rssi"] = WiFi.RSSI();
  ev["ip"] = WiFi.localIP().toString();
  String evPayload;
  serializeJson(ev, evPayload);
  if (publishUp("event", evPayload, false)) {
    logLine(String("lifecycle event sent: ") + (isFirstOnline ? "first_online" : "re_online"));
    if (isFirstOnline) {
      if (!saveFirstOnlineSent(true)) {
        logLine("warning: failed to persist first_online flag");
      }
    }
  } else {
    logLine("lifecycle event send failed");
  }
  return true;
}

bool NodeRemote::subscribeDownTopics() {
  if (!mqtt_.connected()) {
    return false;
  }
  const String filter = downBaseTopic_ + "/#";
  const bool ok = mqtt_.subscribe(filter.c_str(), 0);
  if (!ok) {
    lastError_ = "mqtt_subscribe_failed";
    logLine("NodeAnywhere subscribe failed filter=" + filter);
    return false;
  }
  return true;
}

bool NodeRemote::claimDevice() {
  if (apiBaseUrl_.isEmpty() || claimToken_.isEmpty() || deviceUid_.isEmpty()) {
    lastError_ = "claim_config_missing";
    noteClaimFailure(true);
    logLine("claim config missing");
    return false;
  }

  lastClaimAttemptMs_ = millis();
  logLine("claim start uid=" + deviceUid_);

  HTTPClient http;
  const String url = apiBaseUrl_ + "/api/devices/claim";
  // Support HTTPS API endpoints (443). By default we use insecure TLS to keep the
  // out-of-box experience simple. For production, pin a CA cert and avoid insecure TLS.
  if (url.startsWith("https://")) {
    // Reuse the class-level TLS client to avoid large stack usage in this function.
    configureTlsClient(netTls_);
    if (!http.begin(netTls_, url)) {
      lastError_ = "http_begin_failed";
      noteClaimFailure(false);
      logLine("claim http begin failed url=" + url);
      return false;
    }
  } else if (!http.begin(netPlain_, url)) {
    lastError_ = "http_begin_failed";
    noteClaimFailure(false);
    logLine("claim http begin failed url=" + url);
    return false;
  }
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<256> reqDoc;
  reqDoc["claim_token"] = claimToken_;
  reqDoc["device_uid"] = deviceUid_;
  reqDoc["display_name"] = deviceUid_;

  String body;
  serializeJson(reqDoc, body);
  const int status = http.POST(body);
  const String resp = http.getString();
  http.end();

  if (status != 200) {
    // Try to parse server error to decide whether retry makes sense.
    String serverErr;
    String serverDetail;
    if (resp.startsWith("{")) {
      StaticJsonDocument<256> e;
      if (deserializeJson(e, resp) == DeserializationError::Ok) {
        serverErr = String(e["error"] | "");
        serverDetail = String(e["detail"] | "");
      }
    }
    if (serverErr == "mqtt_provision_failed") {
      lastError_ = "claim_mqtt_provision_failed";
      noteClaimFailure(false);
      if (serverDetail.length() > 0) {
        logLine("claim failed: server cannot issue NodeAnywhere credentials (" + serverDetail + ")");
      } else {
        logLine("claim failed: server cannot issue NodeAnywhere credentials");
      }
      return false;
    }
    if (serverErr == "invalid_claim_token" ||
        serverErr == "claim_token_expired" ||
        serverErr == "claim_token_already_used" ||
        serverErr == "claim_device_uid_mismatch" ||
        serverErr == "device_limit_reached") {
      claimPermanentlyFailed_ = true;
      lastError_ = "claim_failed_" + serverErr;
      noteClaimFailure(true);
      logLine("claim failed permanently error=" + serverErr);
      return false;
    }

    lastError_ = "claim_http_" + String(status);
    noteClaimFailure(false);
    logLine("claim http failed status=" + String(status));
    return false;
  }

  StaticJsonDocument<1024> doc;
  const auto err = deserializeJson(doc, resp);
  if (err) {
    lastError_ = "claim_json_parse_failed";
    noteClaimFailure(false);
    logLine("claim json parse failed");
    return false;
  }
  if (!doc["ok"].as<bool>()) {
    lastError_ = "claim_not_ok";
    noteClaimFailure(false);
    logLine("claim response not ok");
    return false;
  }

  const String user = doc["mqtt_credentials"]["username"] | "";
  const String pass = doc["mqtt_credentials"]["password"] | "";
  if (user.isEmpty() || pass.isEmpty()) {
    const String provErr = doc["mqtt_provision_error"] | "";
    if (!provErr.isEmpty()) {
      lastError_ = "claim_no_mqtt_credentials_" + provErr;
      logLine("claim failed: missing NodeAnywhere credentials from server, provision error=" + provErr);
    } else {
      lastError_ = "claim_no_mqtt_credentials";
      logLine("claim failed: missing NodeAnywhere credentials from server");
    }
    noteClaimFailure(true);
    return false;
  }

  if (!saveCredentials(user, pass, deviceUid_)) {
    lastError_ = "save_credentials_failed";
    noteClaimFailure(false);
    logLine("save credentials failed");
    return false;
  }
  mqttUser_ = user;
  mqttPass_ = pass;
  updateTopicBases();
  noteClaimSuccess();
  logLine("claim success node_user=" + mqttUser_);
  return true;
}

bool NodeRemote::shouldAttemptClaimNow() {
  const uint32_t now = millis();
  if (lastClaimAttemptMs_ == 0) {
    return true;
  }
  return (now - lastClaimAttemptMs_) >= claimRetryIntervalMs_;
}

void NodeRemote::noteClaimFailure(bool permanent) {
  if (permanent) {
    claimPermanentlyFailed_ = true;
    // Back off hard even if loop keeps calling ensureConnected.
    claimRetryIntervalMs_ = 60UL * 60UL * 1000UL;  // 1 hour
    return;
  }
  // Exponential backoff up to 5 minutes to avoid request floods.
  if (claimRetryIntervalMs_ < 5000) {
    claimRetryIntervalMs_ = 5000;
  } else if (claimRetryIntervalMs_ < 300000) {
    claimRetryIntervalMs_ = min(claimRetryIntervalMs_ * 2UL, 300000UL);
  }
}

void NodeRemote::noteClaimSuccess() {
  claimPermanentlyFailed_ = false;
  claimRetryIntervalMs_ = 5000;
}

void NodeRemote::staticCallback(char* topic, uint8_t* payload, unsigned int length) {
  if (!instance_) return;

  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; ++i) {
    msg += static_cast<char>(payload[i]);
  }

  instance_->handleMqttMessage(String(topic), msg);
}

void NodeRemote::handleMqttMessage(const String& topic, const String& payload) {
  if (!topic.startsWith(downBaseTopic_ + "/")) {
    return;
  }
  const String subTopic = topic.substring((downBaseTopic_ + "/").length());
  if (subTopic == "ota") {
    if (otaInProgress_) {
      // Parse job id best-effort for a meaningful busy response.
      String jobUid = "";
      StaticJsonDocument<256> d;
      if (deserializeJson(d, payload) == DeserializationError::Ok) {
        jobUid = String(d["job_uid"] | d["job"] | "");
      }
      logLine("ota busy");
      if (!jobUid.isEmpty()) {
        publishOtaResult(jobUid, false, "busy", "", "");
      }
      return;
    }
    otaPendingPayload_ = payload;
    otaPending_ = true;
    logLine("ota request queued");
    return;
  }
  if (subTopic == "serial" || subTopic == "serial/in") {
    String serialPayload = payload;
    if (payload.startsWith("{")) {
      StaticJsonDocument<384> doc;
      if (deserializeJson(doc, payload) == DeserializationError::Ok) {
        const char* p = doc["payload"] | doc["message"] | doc["msg"] | nullptr;
        if (p != nullptr) {
          serialPayload = String(p);
        }
      }
    }
    serialInPush(serialPayload);
    StaticJsonDocument<192> ack;
    ack["ok"] = true;
    ack["bytes"] = static_cast<int>(serialPayload.length());
    ack["queued"] = static_cast<int>(serialInSize_);
    String out;
    serializeJson(ack, out);
    publishAck("serial", out);
    logLine("serial input queued");
    return;
  }
  commandHandler_(subTopic, payload);
}

bool NodeRemote::handleWifiScanCommand(String& outAckJson) {
  StaticJsonDocument<256> ack;
  ack["cmd"] = "wifi_scan_start";

  if (WiFi.status() != WL_CONNECTED) {
    ack["ok"] = false;
    ack["error"] = "wifi_not_connected";
    serializeJson(ack, outAckJson);
    return false;
  }

  logLine("wifi scan start");
  const int found = WiFi.scanNetworks(false, true);
  DynamicJsonDocument report(2048);
  report["ts_ms"] = millis();
  report["type"] = "wifi_scan_result";
  JsonArray arr = report.createNestedArray("aps");
  const int limit = found > 15 ? 15 : found;
  for (int i = 0; i < limit; i++) {
    const String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) continue;
    JsonObject it = arr.createNestedObject();
    it["ssid"] = ssid;
    it["rssi"] = WiFi.RSSI(i);
    it["enc"] = static_cast<int>(WiFi.encryptionType(i));
  }
  String payload;
  serializeJson(report, payload);
  const bool published = publishUp("wifi/scan_result", payload, false);
  WiFi.scanDelete();

  ack["ok"] = published;
  ack["result"] = published ? "scan_published" : "scan_publish_failed";
  ack["count"] = arr.size();
  serializeJson(ack, outAckJson);
  logLine("wifi scan done count=" + String(arr.size()));
  return published;
}

bool NodeRemote::handleWifiApplyConfigCommand(const String& payload, String& outAckJson) {
  DynamicJsonDocument doc(2048);
  StaticJsonDocument<256> ack;
  ack["cmd"] = "wifi_apply_config";
  if (deserializeJson(doc, payload) != DeserializationError::Ok) {
    ack["ok"] = false;
    ack["error"] = "invalid_json";
    serializeJson(ack, outAckJson);
    return false;
  }
  JsonArray aps = doc["aps"].as<JsonArray>();
  if (aps.isNull() || aps.size() == 0 || aps.size() > kWifiMaxAps) {
    ack["ok"] = false;
    ack["error"] = "aps_required_1_to_5";
    serializeJson(ack, outAckJson);
    return false;
  }

  WifiApConfig oldAps[kWifiMaxAps];
  const uint8_t oldCount = wifiApCount_;
  for (uint8_t i = 0; i < oldCount && i < kWifiMaxAps; i++) {
    oldAps[i] = wifiAps_[i];
  }

  wifiApCount_ = 0;
  for (JsonVariant v : aps) {
    if (wifiApCount_ >= kWifiMaxAps) break;
    JsonObject o = v.as<JsonObject>();
    String ssid = String(o["ssid"] | "");
    String pass = String(o["password"] | o["pass"] | "");
    const bool keepPassword = bool(o["keep_password"] | false);
    int prio = int(o["priority"] | (100 + wifiApCount_));
    ssid.trim();
    if (ssid.isEmpty() || prio < 1 || prio > 5) continue;
    if (keepPassword && pass.isEmpty()) {
      bool foundOld = false;
      for (uint8_t j = 0; j < oldCount && j < kWifiMaxAps; j++) {
        if (oldAps[j].ssid == ssid) {
          pass = oldAps[j].password;
          foundOld = true;
          break;
        }
      }
      if (!foundOld || pass.isEmpty()) {
        ack["ok"] = false;
        ack["error"] = "keep_password_missing_old_password";
        ack["ssid"] = ssid;
        serializeJson(ack, outAckJson);
        return false;
      }
    }
    if (ssid.length() > 32 || pass.length() > 64) continue;
    wifiAps_[wifiApCount_].ssid = ssid;
    wifiAps_[wifiApCount_].password = pass;
    wifiAps_[wifiApCount_].priority = prio;
    wifiApCount_++;
  }
  if (wifiApCount_ == 0) {
    ack["ok"] = false;
    ack["error"] = "no_valid_ap";
    serializeJson(ack, outAckJson);
    return false;
  }
  sortWifiByPriority();
  wifiManagedEnabled_ = true;
  saveWifiConfig();
  // Publish newest applied profile immediately so backend UI can refresh without ambiguity.
  DynamicJsonDocument report(1024);
  report["ts_ms"] = millis();
  report["type"] = "wifi_profile";
  report["managed"] = wifiManagedEnabled_;
  report["connected_ssid"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "";
  JsonArray arr = report.createNestedArray("aps");
  for (uint8_t i = 0; i < wifiApCount_; i++) {
    JsonObject it = arr.createNestedObject();
    it["ssid"] = wifiAps_[i].ssid;
    it["priority"] = wifiAps_[i].priority;
    const size_t passLen = wifiAps_[i].password.length();
    it["has_password"] = passLen > 0;
    it["password_len"] = static_cast<int>(passLen);
  }
  String profilePayload;
  serializeJson(report, profilePayload);
  publishUp("wifi/profile", profilePayload, false);

  ack["ok"] = true;
  ack["result"] = "config_saved_rebooting";
  ack["count"] = wifiApCount_;
  serializeJson(ack, outAckJson);
  logLine("wifi config applied count=" + String(wifiApCount_));
  return true;
}

bool NodeRemote::handleWifiListCommand(String& outAckJson) {
  DynamicJsonDocument report(1024);
  report["ts_ms"] = millis();
  report["type"] = "wifi_profile";
  report["managed"] = wifiManagedEnabled_;
  report["connected_ssid"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "";
  JsonArray arr = report.createNestedArray("aps");
  for (uint8_t i = 0; i < wifiApCount_; i++) {
    JsonObject it = arr.createNestedObject();
    it["ssid"] = wifiAps_[i].ssid;
    it["priority"] = wifiAps_[i].priority;
    const size_t passLen = wifiAps_[i].password.length();
    it["has_password"] = passLen > 0;
    it["password_len"] = static_cast<int>(passLen);
  }
  String profilePayload;
  serializeJson(report, profilePayload);
  const bool published = publishUp("wifi/profile", profilePayload, false);

  StaticJsonDocument<256> ack;
  ack["cmd"] = "wifi_list";
  ack["ok"] = published;
  ack["managed"] = wifiManagedEnabled_;
  ack["count"] = wifiApCount_;
  ack["result"] = published ? "profile_published" : "profile_publish_failed";
  serializeJson(ack, outAckJson);
  return published;
}

void NodeRemote::handleDefaultCommand(const String& subTopic, const String& payload) {
  if (subTopic != "cmd" && !subTopic.startsWith("cmd/")) {
    return;
  }

  String cmd = payload;
  cmd.trim();

  int sleepSeconds = 0;

  // Text form: "sleep 300"
  {
    const int sp = cmd.indexOf(' ');
    if (sp > 0) {
      const String head = cmd.substring(0, sp);
      const String tail = cmd.substring(sp + 1);
      if (head.equalsIgnoreCase("sleep")) {
        cmd = "sleep";
        sleepSeconds = tail.toInt();
      }
    }
  }

  // Optional JSON envelope: {"cmd":"ping"} or {"cmd":"reset"} or {"cmd":"sleep","seconds":300}
  if (cmd.startsWith("{")) {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, cmd) == DeserializationError::Ok) {
      const char* c = doc["cmd"] | doc["type"] | "";
      if (c && *c) {
        cmd = String(c);
      }
      if (String(cmd).equalsIgnoreCase("sleep")) {
        sleepSeconds = int(doc["seconds"] | doc["sec"] | 0);
      }
    }
  }

  cmd.toLowerCase();
  logLine("cmd received topic=" + subTopic + " payload=" + cmd);

  StaticJsonDocument<256> ack;
  ack["cmd"] = cmd;

  if (otaInProgress_ || otaPending_) {
    ack["ok"] = false;
    ack["error"] = "ota_in_progress";
    String outOtaBusy;
    serializeJson(ack, outOtaBusy);
    publishAck("cmd", outOtaBusy);
    logLine("cmd rejected during ota");
    return;
  }

  if (cmd == "ping") {
    ack["ok"] = true;
    ack["pong"] = true;
  } else if (cmd == "wifi_scan_start") {
    String outAck;
    if (handleWifiScanCommand(outAck)) {
      const bool okAck = publishAck("cmd", outAck);
      if (okAck) logLine("cmd ack sent");
      return;
    }
    ack["ok"] = false;
    ack["error"] = "wifi_scan_failed";
  } else if (cmd == "wifi_apply_config") {
    String outAck;
    if (handleWifiApplyConfigCommand(payload, outAck)) {
      const bool okAck = publishAck("cmd", outAck);
      if (okAck) logLine("cmd ack sent");
      // Allow ACK to flush, then reboot to apply policy consistently.
      rebootPending_ = true;
      rebootAtMs_ = millis() + 500;
      return;
    }
    ack["ok"] = false;
    ack["error"] = "wifi_apply_failed";
  } else if (cmd == "wifi_list") {
    String outAck;
    if (handleWifiListCommand(outAck)) {
      const bool okAck = publishAck("cmd", outAck);
      if (okAck) logLine("cmd ack sent");
      return;
    }
    ack["ok"] = false;
    ack["error"] = "wifi_list_failed";
  } else if (cmd == "reset" || cmd == "reboot") {
    ack["ok"] = true;
    ack["result"] = "收到";
    ack["reset_soon"] = true;
    rebootPending_ = true;
    rebootAtMs_ = millis() + 250;  // allow ack to flush
  } else if (cmd == "info") {
    const uint32_t upSec = static_cast<uint32_t>(millis() / 1000);
    const uint32_t freeHeap = ESP.getFreeHeap();
    const uint32_t heapK = (freeHeap + 512) / 1024;
    const int cpuMhz = ESP.getCpuFreqMHz();
    const uint32_t flashMb = ESP.getFlashChipSize() / (1024UL * 1024UL);
    const String chip = chipModelString();
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    const String resetReason = resetReasonString();

    ack["ok"] = true;
    ack["fw"] = String("NodeRemote/") + kVersion;
    ack["ip"] = WiFi.localIP().toString();
    ack["rssi"] = WiFi.RSSI();
    ack["heap_k"] = heapK;
    ack["up_s"] = upSec;
    ack["ssid"] = WiFi.SSID();
    ack["chip"] = chip;
    ack["cpu_mhz"] = cpuMhz;
    ack["flash_mb"] = flashMb;
    ack["mac"] = mac;
    ack["reset_reason"] = resetReason;

    // Provide a human-friendly summary so the web UI doesn't need to show raw JSON.
    String summary = "ip=" + WiFi.localIP().toString();
    summary += ", rssi=" + String(WiFi.RSSI());
    summary += ", heap=" + String(heapK) + "k";
    summary += ", up=" + String(upSec) + "s";
    summary += ", chip=" + chip;
    summary += ", mac=" + mac;
    summary += ", cpu_mhz=" + String(cpuMhz);
    summary += ", flash_mb=" + String(flashMb);
    summary += ", reset_reason=" + resetReason;
    ack["summary"] = summary;
  } else if (cmd == "sleep") {
    const int minSec = 60;
    const int maxSec = 86400;
    if (sleepSeconds < minSec || sleepSeconds > maxSec) {
      ack["ok"] = false;
      ack["error"] = "sleep_seconds_out_of_range";
      ack["min_sec"] = minSec;
      ack["max_sec"] = maxSec;
    } else {
      ack["ok"] = true;
      ack["result"] = "sleeping";
      ack["wake_in_sec"] = sleepSeconds;

      sleepWakeUs_ = uint64_t(sleepSeconds) * 1000000ULL;
      sleepPending_ = true;
      sleepAtMs_ = millis() + 400;  // allow ack to flush
    }
  } else if (cmd == "wipe_identity") {
    // ACK first, then clear local identity and reboot.
    ack["ok"] = true;
    ack["result"] = "收到";
    ack["wipe_identity_soon"] = true;
    wipeIdentityPending_ = true;
    wipeIdentityAtMs_ = millis() + 500;  // allow ack to flush
  } else {
    ack["ok"] = false;
    ack["error"] = "unknown_command";
  }

  String out;
  serializeJson(ack, out);
  const bool okAck = publishAck("cmd", out);
  if (okAck) {
    logLine("cmd ack sent");
  } else {
    lastError_ = "cmd_ack_publish_failed";
    logLine("cmd ack publish failed (maybe payload too large or NodeAnywhere disconnected)");
  }
}

bool NodeRemote::publishUp(const String& subTopic, const String& payload, bool retained) {
  if (!mqtt_.connected()) {
    return false;
  }
  const String topic = upBaseTopic_ + "/" + subTopic;
  return mqtt_.publish(topic.c_str(), payload.c_str(), retained);
}

bool NodeRemote::publishAck(const String& commandName, const String& payload, bool retained) {
  return publishUp("ack/" + commandName, payload, retained);
}

void NodeRemote::publishOtaProgress(const String& jobUid, const String& state, int pct) {
  if (!mqtt_.connected()) {
    return;
  }
  StaticJsonDocument<256> doc;
  doc["job_uid"] = jobUid;
  doc["state"] = state;
  if (pct >= 0) {
    doc["pct"] = pct;
  }
  String out;
  serializeJson(doc, out);
  publishUp("ota/progress", out, false);
}

void NodeRemote::publishOtaResult(const String& jobUid, bool ok, const String& error, const String& version, const String& firmwareUid) {
  if (!mqtt_.connected()) {
    return;
  }
  StaticJsonDocument<256> doc;
  doc["job_uid"] = jobUid;
  doc["ok"] = ok;
  if (!error.isEmpty()) {
    doc["error"] = error;
  }
  if (!version.isEmpty()) {
    doc["version"] = version;
  }
  if (!firmwareUid.isEmpty()) {
    doc["firmware_uid"] = firmwareUid;
  }
  String out;
  serializeJson(doc, out);
  publishUp("ota/result", out, false);
}

bool NodeRemote::handleOtaPayload(const String& payload) {
  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, payload) != DeserializationError::Ok) {
    lastError_ = "ota_json_parse_failed";
    logLine("ota json parse failed");
    return false;
  }

  const String jobUid = String(doc["job_uid"] | doc["job"] | "");
  const String firmwareUid = String(doc["firmware_uid"] | "");
  const String version = String(doc["version"] | "");
  const String url = String(doc["url"] | "");
  const String expectSha = String(doc["sha256"] | doc["sha"] | "");
  const int expectSize = int(doc["size"] | doc["size_bytes"] | 0);

  if (jobUid.isEmpty() || url.isEmpty() || expectSha.length() < 64) {
    lastError_ = "ota_missing_fields";
    logLine("ota missing fields job/url/sha");
    if (!jobUid.isEmpty()) {
      publishOtaResult(jobUid, false, "missing_fields", version, firmwareUid);
    }
    return false;
  }

  if (otaInProgress_) {
    logLine("ota busy");
    publishOtaResult(jobUid, false, "busy", version, firmwareUid);
    return false;
  }

  if (!mqtt_.connected()) {
    lastError_ = "ota_mqtt_not_connected";
    logLine("ota NodeAnywhere not connected");
    return false;
  }

  otaInProgress_ = true;
  lastOtaProgressMs_ = 0;
  logLine("ota start job=" + jobUid + " ver=" + version);
  publishOtaProgress(jobUid, "downloading", 0);

  HTTPClient http;
  WiFiClientSecure secure;
  WiFiClient plain;

  if (url.startsWith("https://")) {
    // Mirror MQTT TLS config.
    configureTlsClient(secure);
    if (!http.begin(secure, url)) {
      lastError_ = "ota_http_begin_failed";
      logLine("ota http begin failed");
      publishOtaResult(jobUid, false, "http_begin_failed", version, firmwareUid);
      otaInProgress_ = false;
      return false;
    }
  } else {
    if (!http.begin(plain, url)) {
      lastError_ = "ota_http_begin_failed";
      logLine("ota http begin failed");
      publishOtaResult(jobUid, false, "http_begin_failed", version, firmwareUid);
      otaInProgress_ = false;
      return false;
    }
  }

  const int code = http.GET();
  if (code != 200) {
    lastError_ = "ota_http_" + String(code);
    logLine("ota http failed code=" + String(code));
    publishOtaResult(jobUid, false, "http_" + String(code), version, firmwareUid);
    http.end();
    otaInProgress_ = false;
    return false;
  }

  const int contentLen = http.getSize();
  const int totalLen = (expectSize > 0) ? expectSize : contentLen;
  if (totalLen <= 0) {
    logLine("ota unknown size");
  }

  publishOtaProgress(jobUid, "flashing", 0);
  if (!Update.begin(totalLen > 0 ? static_cast<size_t>(totalLen) : UPDATE_SIZE_UNKNOWN)) {
    lastError_ = "ota_update_begin_failed";
    logLine("ota update begin failed");
    publishOtaResult(jobUid, false, "update_begin_failed", version, firmwareUid);
    http.end();
    otaInProgress_ = false;
    return false;
  }

#if defined(ESP32)
  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  // Use non-*_ret variants for compatibility with older Arduino-ESP32 toolchains.
  mbedtls_sha256_starts(&sha, 0);
#elif defined(ESP8266)
  SHA256Builder sha;
  sha.begin();
#endif

  WiFiClient* stream = http.getStreamPtr();
  const size_t bufSize = 1024;
  uint8_t* buf = static_cast<uint8_t*>(malloc(bufSize));
  if (!buf) {
    lastError_ = "ota_no_memory";
    logLine("ota no memory for download buffer");
    publishOtaResult(jobUid, false, "no_memory", version, firmwareUid);
    Update.abort();
    http.end();
    otaInProgress_ = false;
    return false;
  }
  size_t written = 0;
  int lastPct = -1;

  while (http.connected() && (totalLen <= 0 || static_cast<int>(written) < totalLen)) {
    const size_t avail = stream->available();
    if (avail == 0) {
      mqtt_.loop();
      delay(1);
      continue;
    }

    const size_t toRead = min(avail, bufSize);
    const int n = stream->readBytes(buf, toRead);
    if (n <= 0) {
      mqtt_.loop();
      delay(1);
      continue;
    }

#if defined(ESP32)
    mbedtls_sha256_update(&sha, buf, static_cast<size_t>(n));
#elif defined(ESP8266)
    sha.add(buf, static_cast<size_t>(n));
#endif
    const size_t w = Update.write(buf, static_cast<size_t>(n));
    if (w != static_cast<size_t>(n)) {
      lastError_ = "ota_update_write_failed";
      logLine("ota update write failed");
      publishOtaResult(jobUid, false, "update_write_failed", version, firmwareUid);
      Update.abort();
      free(buf);
      http.end();
      otaInProgress_ = false;
      return false;
    }
    written += w;

    int pct = -1;
    if (totalLen > 0) {
      pct = int((written * 100UL) / static_cast<size_t>(totalLen));
      pct = max(0, min(100, pct));
    }

    const uint32_t now = millis();
    if (pct >= 0 && pct != lastPct && (pct % 5 == 0 || now - lastOtaProgressMs_ >= 1000)) {
      lastPct = pct;
      lastOtaProgressMs_ = now;
      publishOtaProgress(jobUid, "flashing", pct);
    } else if (now - lastOtaProgressMs_ >= 3000) {
      // Keep-alive progress at least every 3s.
      lastOtaProgressMs_ = now;
      publishOtaProgress(jobUid, "flashing", pct);
    }
    mqtt_.loop();
  }

  free(buf);
  http.end();

  if (totalLen > 0 && static_cast<int>(written) != totalLen) {
    lastError_ = "ota_size_mismatch";
    logLine("ota size mismatch wrote=" + String(written) + " expected=" + String(totalLen));
    publishOtaResult(jobUid, false, "size_mismatch", version, firmwareUid);
    Update.abort();
    otaInProgress_ = false;
    return false;
  }

#if defined(ESP32)
  uint8_t digest[32];
  mbedtls_sha256_finish(&sha, digest);
  mbedtls_sha256_free(&sha);
  String gotSha = sha256HexLower(digest);
  gotSha.toLowerCase();
#elif defined(ESP8266)
  sha.calculate();
  String gotSha = sha.toString();
  gotSha.toLowerCase();
#endif

  String exp = expectSha;
  exp.toLowerCase();
  if (gotSha != exp) {
    lastError_ = "ota_sha256_mismatch";
    logLine("ota sha mismatch");
    publishOtaResult(jobUid, false, "sha256_mismatch", version, firmwareUid);
    Update.abort();
    otaInProgress_ = false;
    return false;
  }

  if (!Update.end(true)) {
    lastError_ = "ota_update_end_failed";
    logLine("ota update end failed");
    publishOtaResult(jobUid, false, "update_end_failed", version, firmwareUid);
    otaInProgress_ = false;
    return false;
  }

  publishOtaProgress(jobUid, "rebooting", 100);
  publishOtaResult(jobUid, true, "", version, firmwareUid);
  logLine("ota success rebooting");

  // Delay a bit so MQTT has time to flush.
  rebootPending_ = true;
  rebootAtMs_ = millis() + 1500;
  otaInProgress_ = false;
  return true;
}

bool NodeRemote::println(const String& message) {
  if (!mqtt_.connected()) {
    lastError_ = "mqtt_not_connected";
    return false;
  }
  String msg = message;
  // Avoid pushing extremely large serial logs over MQTT.
  const size_t maxLen = 512;
  if (msg.length() > maxLen) {
    msg.remove(maxLen);
  }
  return publishUp("console", msg, false);
}

bool NodeRemote::println(const char* message) {
  return println(String(message ? message : ""));
}

bool NodeRemote::serialInAvailable() const {
  return serialInSize_ > 0;
}

uint8_t NodeRemote::serialInCount() const {
  return serialInSize_;
}

String NodeRemote::serialInRead() {
  String out;
  if (!serialInRead(out)) {
    return String("");
  }
  return out;
}

bool NodeRemote::serialInRead(String& out) {
  if (serialInSize_ == 0) {
    out = "";
    return false;
  }
  out = serialInQueue_[serialInHead_];
  serialInQueue_[serialInHead_] = "";
  serialInHead_ = (serialInHead_ + 1) % kSerialInQueueMax;
  serialInSize_--;
  return true;
}

void NodeRemote::serialInClear() {
  for (uint8_t i = 0; i < kSerialInQueueMax; i++) {
    serialInQueue_[i] = "";
  }
  serialInHead_ = 0;
  serialInTail_ = 0;
  serialInSize_ = 0;
}

void NodeRemote::serialInPush(const String& payload) {
  if (serialInSize_ >= kSerialInQueueMax) {
    // Full queue: drop oldest to keep latest backend input.
    serialInHead_ = (serialInHead_ + 1) % kSerialInQueueMax;
    serialInSize_--;
  }
  serialInQueue_[serialInTail_] = payload;
  serialInTail_ = (serialInTail_ + 1) % kSerialInQueueMax;
  serialInSize_++;
}

bool NodeRemote::sendHeartbeat() {
  StaticJsonDocument<192> doc;
  doc["uptime_sec"] = static_cast<uint32_t>(millis() / 1000);
  doc["rssi"] = WiFi.RSSI();
  doc["ip"] = WiFi.localIP().toString();
  doc["fw"] = kVersion;

  String payload;
  serializeJson(doc, payload);
  return publishUp("heartbeat", payload, false);
}

bool NodeRemote::sendStatus() {
  StaticJsonDocument<256> doc;
  const uint32_t upSec = static_cast<uint32_t>(millis() / 1000);
  const uint32_t freeHeap = ESP.getFreeHeap();
  const uint32_t heapK = (freeHeap + 512) / 1024;
  const uint32_t flashMb = ESP.getFlashChipSize() / (1024UL * 1024UL);
  const String chip = chipModelString();
  String mac = WiFi.macAddress();
  mac.replace(":", "");

  doc["up_s"] = upSec;
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();
  doc["heap_k"] = heapK;
  doc["flash_mb"] = flashMb;
  doc["chip"] = chip;
  doc["mac"] = mac;

  String payload;
  serializeJson(doc, payload);
  return publishUp("status", payload, false);
}

bool NodeRemote::hasCredentials() const {
  return !mqttUser_.isEmpty() && !mqttPass_.isEmpty() && !deviceUid_.isEmpty();
}

String NodeRemote::deviceUid() const {
  return deviceUid_;
}

String NodeRemote::mqttUsername() const {
  return mqttUser_;
}

String NodeRemote::lastError() const {
  return lastError_;
}

void NodeRemote::clearCredentials() {
  prefs_.begin(kPrefsNs, false);
  prefs_.remove(kPrefsMqttUser);
  prefs_.remove(kPrefsMqttPass);
  prefs_.remove(kPrefsDeviceUid);
  prefs_.remove(kPrefsFirstOnlineSent);
  prefs_.end();
  mqttUser_ = "";
  mqttPass_ = "";
  logLine("credentials cleared");
}

bool NodeRemote::loadWifiConfig() {
  wifiApCount_ = 0;
  wifiPrefs_.begin(kWifiPrefsNs, true);
  const uint32_t managed = wifiPrefs_.getUInt("managed", 0);
  const uint32_t count = wifiPrefs_.getUInt("count", 0);
  const uint8_t c = static_cast<uint8_t>(count > kWifiMaxAps ? kWifiMaxAps : count);
  for (uint8_t i = 0; i < c; i++) {
    const String ks = "ssid_" + String(i);
    const String kp = "pass_" + String(i);
    const String kr = "prio_" + String(i);
    const String ssid = wifiPrefs_.getString(ks.c_str(), "");
    if (ssid.isEmpty()) continue;
    wifiAps_[wifiApCount_].ssid = ssid;
    wifiAps_[wifiApCount_].password = wifiPrefs_.getString(kp.c_str(), "");
    wifiAps_[wifiApCount_].priority = static_cast<int>(wifiPrefs_.getInt(kr.c_str(), 100 + i));
    wifiApCount_++;
  }
  wifiPrefs_.end();
  sortWifiByPriority();
  if (managed == 1U || wifiApCount_ > 0) {
    wifiManagedEnabled_ = true;
  }
  return true;
}

bool NodeRemote::saveWifiConfig() {
  wifiPrefs_.begin(kWifiPrefsNs, false);
  wifiPrefs_.putUInt("managed", wifiManagedEnabled_ ? 1U : 0U);
  wifiPrefs_.putUInt("count", wifiApCount_);
  for (uint8_t i = 0; i < kWifiMaxAps; i++) {
    const String ks = "ssid_" + String(i);
    const String kp = "pass_" + String(i);
    const String kr = "prio_" + String(i);
    if (i < wifiApCount_) {
      wifiPrefs_.putString(ks.c_str(), wifiAps_[i].ssid);
      wifiPrefs_.putString(kp.c_str(), wifiAps_[i].password);
      wifiPrefs_.putInt(kr.c_str(), wifiAps_[i].priority);
    } else {
      wifiPrefs_.remove(ks.c_str());
      wifiPrefs_.remove(kp.c_str());
      wifiPrefs_.remove(kr.c_str());
    }
  }
  wifiPrefs_.end();
  return true;
}

void NodeRemote::clearWifiConfig() {
  wifiPrefs_.begin(kWifiPrefsNs, false);
  wifiPrefs_.clear();
  wifiPrefs_.end();
  wifiApCount_ = 0;
  for (uint8_t i = 0; i < kWifiMaxAps; i++) {
    wifiAps_[i].ssid = "";
    wifiAps_[i].password = "";
    wifiAps_[i].priority = 100;
  }
  wifiManagedEnabled_ = false;
  logLine("wifi config cleared");
}

void NodeRemote::sortWifiByPriority() {
  if (wifiApCount_ <= 1) return;
  for (uint8_t i = 0; i < wifiApCount_; i++) {
    for (uint8_t j = i + 1; j < wifiApCount_; j++) {
      if (wifiAps_[j].priority < wifiAps_[i].priority) {
        WifiApConfig t = wifiAps_[i];
        wifiAps_[i] = wifiAps_[j];
        wifiAps_[j] = t;
      }
    }
  }
}

bool NodeRemote::connectWifiFromManagedList() {
  if (wifiApCount_ == 0) {
    lastError_ = "wifi_list_empty";
    logThrottled("wifi managed enabled but list is empty", lastWifiLogMs_, 10000);
    return false;
  }

  WiFi.mode(WIFI_STA);
  logLine("wifi connect policy: max_ap=5 rounds=" + String(wifiRoundsPerCycle_) + " timeout_ms=" + String(wifiPerApTimeoutMs_));
  for (uint8_t round = 0; round < wifiRoundsPerCycle_; round++) {
    for (uint8_t i = 0; i < wifiApCount_; i++) {
      const String ssid = wifiAps_[i].ssid;
      const String pass = wifiAps_[i].password;
      logLine("wifi try round=" + String(round + 1) + "/" + String(wifiRoundsPerCycle_) + " ssid=" + ssid + " prio=" + String(wifiAps_[i].priority));
#if defined(ESP8266)
      WiFi.disconnect(false);
#else
      WiFi.disconnect(false, false);
#endif
      delay(50);
      if (pass.length() == 0) {
        WiFi.begin(ssid.c_str());  // Open WiFi
      } else {
        WiFi.begin(ssid.c_str(), pass.c_str());
      }
      const uint32_t startMs = millis();
      while (millis() - startMs < wifiPerApTimeoutMs_) {
        if (WiFi.status() == WL_CONNECTED) {
          logLine("wifi connected ssid=" + ssid + " ip=" + WiFi.localIP().toString());
          return true;
        }
        delay(250);
      }
      logLine("wifi timeout ssid=" + ssid);
    }
    logLine("wifi round " + String(round + 1) + " complete");
    delay(5000);
  }
  return false;
}

bool NodeRemote::checkBootRevokeWindow() {
  static constexpr int kBootButtonPin = 0;           // BOOT/FLASH button on many ESP32/ESP8266 dev boards (IO0/GPIO0)
  static constexpr uint32_t kBootWaitWindowMs = 5000;

  pinMode(kBootButtonPin, INPUT_PULLUP);
  logLine("startup window 5s: press BOOT(IO0) to clear all NVS settings (wifi + NodeAnywhere credentials)");

  const uint32_t started = millis();
  while (millis() - started < kBootWaitWindowMs) {
    // BOOT pressed => IO0 pulled LOW.
    if (digitalRead(kBootButtonPin) == LOW) {
      logLine("BOOT(IO0) detected; clearing all NVS settings...");
      clearCredentials();
      clearWifiConfig();
      logLine("all NVS settings cleared by BOOT button");
      delay(120);  // debounce / user feedback
      return true;
    }
    delay(20);
  }
  return false;
}

void NodeRemote::updateTopicBases() {
  if (deviceUid_.isEmpty()) {
    upBaseTopic_ = "";
    downBaseTopic_ = "";
    return;
  }
  upBaseTopic_ = "devices/" + deviceUid_ + "/up";
  downBaseTopic_ = "devices/" + deviceUid_ + "/down";
}

bool NodeRemote::loadCredentials() {
  prefs_.begin(kPrefsNs, true);
  mqttUser_ = prefs_.getString(kPrefsMqttUser, "");
  mqttPass_ = prefs_.getString(kPrefsMqttPass, "");
  const String savedUid = prefs_.getString(kPrefsDeviceUid, "");
  prefs_.end();

  if (!savedUid.isEmpty()) {
    // NVS identity is the source of truth after first successful claim.
    // This allows OTA images built with different placeholder UID/TOKEN
    // to keep working on already-registered devices.
    if (deviceUid_.isEmpty() || deviceUid_ != savedUid) {
      if (!deviceUid_.isEmpty() && deviceUid_ != savedUid) {
        logLine("sketch UID differs from NVS UID; using NVS UID");
      }
      deviceUid_ = savedUid;
    }
  }
  updateTopicBases();
  return hasCredentials();
}

bool NodeRemote::saveCredentials(const String& mqttUser, const String& mqttPass, const String& deviceUid) {
  prefs_.begin(kPrefsNs, false);
  const bool okUser = prefs_.putString(kPrefsMqttUser, mqttUser) > 0;
  const bool okPass = prefs_.putString(kPrefsMqttPass, mqttPass) > 0;
  const bool okUid = prefs_.putString(kPrefsDeviceUid, deviceUid) > 0;
  // New identity should emit first_online once.
  prefs_.putBool(kPrefsFirstOnlineSent, false);
  prefs_.end();
  return okUser && okPass && okUid;
}

bool NodeRemote::loadFirstOnlineSent() {
  prefs_.begin(kPrefsNs, true);
  const bool sent = prefs_.getBool(kPrefsFirstOnlineSent, false);
  prefs_.end();
  return sent;
}

bool NodeRemote::saveFirstOnlineSent(bool sent) {
  prefs_.begin(kPrefsNs, false);
  const size_t n = prefs_.putBool(kPrefsFirstOnlineSent, sent);
  prefs_.end();
  return n > 0;
}

String NodeRemote::defaultClientId() const {
  // Keep under typical MQTT client id limits (often 23).
  // Include UID suffix + chip/mac suffix + short random for collision avoidance.
#if defined(ESP8266)
  uint32_t macSuffix = static_cast<uint32_t>(ESP.getChipId() & 0xFFFF);
  uint16_t r = static_cast<uint16_t>(random(0, 0x10000));
#else
  uint64_t mac = ESP.getEfuseMac();
  uint32_t macSuffix = static_cast<uint32_t>(mac & 0xFFFF);
  uint16_t r = static_cast<uint16_t>(esp_random() & 0xFFFF);
#endif

  String uid = deviceUid_;
  uid.replace("-", "");
  uid.replace("_", "");
  uid.replace(" ", "");
  String uidSuffix = uid.length() > 4 ? uid.substring(uid.length() - 4) : uid;

  char buf[32];
  snprintf(buf, sizeof(buf), "rc-%s-%04X-%04X", uidSuffix.c_str(), (unsigned)macSuffix, (unsigned)r);
  return String(buf);
}

void NodeRemote::logLine(const String& msg) {
  if (!debugEnabled_ || logOut_ == nullptr) {
    return;
  }
  logOut_->print("[NodeRemote] ");
  logOut_->println(msg);
}

void NodeRemote::logThrottled(const String& msg, uint32_t& lastMs, uint32_t everyMs) {
  const uint32_t now = millis();
  if (lastMs == 0 || (now - lastMs) >= everyMs) {
    lastMs = now;
    logLine(msg);
  }
}
