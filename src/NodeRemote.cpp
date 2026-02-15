#include "NodeRemote.h"

#include <ArduinoJson.h>
#include <ESP.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_sleep.h>
#include <esp_system.h>
extern "C" uint32_t esp_random(void);

static const char* kDefaultMqttHost = "node.mqttgo.io";
static const uint16_t kDefaultMqttPort = 8883;
// NOTE: If your API is not on this URL (or uses HTTPS), override via setClaimConfig().
static const char* kDefaultApiBaseUrl = "https://node.mqttgo.io";

static String chipModelString() {
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
}

static String resetReasonString(esp_reset_reason_t r) {
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
}

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

bool NodeRemote::begin() {
  instance_ = this;
  mqtt_.setCallback(staticCallback);
  // PubSubClient's default packet buffer is small (often 256 bytes). Our JSON acks (e.g. "info")
  // can exceed that and cause publish() to fail, leading to web-side ACK timeouts.
  // 1KB keeps overhead small but is enough for our command acks + summary.
  mqtt_.setBufferSize(1024);
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

  if (sleepPending_ && sleepAtMs_ != 0 && millis() >= sleepAtMs_) {
    sleepPending_ = false;
    sleepAtMs_ = 0;
    logLine("enter deep sleep");
    if (sleepWakeUs_ > 0) {
      esp_sleep_enable_timer_wakeup(sleepWakeUs_);
    }
    // Best-effort disconnect so broker sees clean session end.
    mqtt_.disconnect();
    delay(50);
    esp_deep_sleep_start();
    // Should never return.
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
    lastError_ = "wifi_not_connected";
    logThrottled("wifi not connected", lastWifiLogMs_, 10000);
    return false;
  }

  if (!hasCredentials()) {
    if (claimPermanentlyFailed_) {
      // Token is invalid/expired/already used: stop spamming server.
      lastError_ = "claim_permanently_failed";
      logThrottled("claim permanently failed; waiting for new token", lastClaimLogMs_, 30000);
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
    logThrottled("mqtt reconnect backoff", lastMqttLogMs_, 10000);
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
    // Default to insecure TLS for easy first-time testing.
    if (mqttTlsInsecure_) {
      netTls_.setInsecure();
    } else if (mqttCaCertPem_ != nullptr) {
      netTls_.setCACert(mqttCaCertPem_);
    } else {
      // No CA provided; fallback to insecure rather than hard-failing silently.
      netTls_.setInsecure();
    }
  }
  mqtt_.setServer(mqttHost_.c_str(), mqttPort_);

  const String clientId = defaultClientId();
  logLine("mqtt connect host=" + mqttHost_ + " port=" + String(mqttPort_) + " clientId=" + clientId + " user=" + mqttUser_);
  if (!mqtt_.connect(clientId.c_str(), mqttUser_.c_str(), mqttPass_.c_str())) {
    const int st = mqtt_.state();
    if (st == 5) {
      // Not authorized: clear persisted credentials and force re-claim on next loop.
      clearCredentials();
      lastError_ = "mqtt_not_authorized_reclaiming";
      logLine("mqtt not authorized; clearing credentials and re-claiming");
      return false;
    }
    lastError_ = "mqtt_connect_failed_" + String(st);
    logLine("mqtt connect failed state=" + String(st));
    return false;
  }
  logLine("mqtt connected");
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
    logLine("mqtt subscribe failed filter=" + filter);
    return false;
  }
  logLine("mqtt subscribed " + filter);
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
    WiFiClientSecure secure;
    secure.setInsecure();
    if (!http.begin(secure, url)) {
      lastError_ = "http_begin_failed";
      noteClaimFailure(false);
      logLine("claim http begin failed url=" + url);
      return false;
    }
  } else if (!http.begin(url)) {
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
    if (resp.startsWith("{")) {
      StaticJsonDocument<256> e;
      if (deserializeJson(e, resp) == DeserializationError::Ok) {
        serverErr = String(e["error"] | "");
      }
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
    lastError_ = "claim_no_mqtt_credentials";
    noteClaimFailure(false);
    logLine("claim missing mqtt credentials");
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
  logLine("claim success mqtt_user=" + mqttUser_);
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
  commandHandler_(subTopic, payload);
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

  if (cmd == "ping") {
    ack["ok"] = true;
    ack["pong"] = true;
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
    const String resetReason = resetReasonString(esp_reset_reason());

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
    logLine("cmd ack publish failed (maybe payload too large or mqtt disconnected)");
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

bool NodeRemote::sendHeartbeat() {
  StaticJsonDocument<256> doc;
  doc["ts_ms"] = static_cast<uint32_t>(millis());
  doc["uptime_sec"] = static_cast<uint32_t>(millis() / 1000);
  doc["rssi"] = WiFi.RSSI();
  doc["heap"] = ESP.getFreeHeap();
  doc["ip"] = WiFi.localIP().toString();
  doc["fw"] = String("NodeRemote/") + kVersion;

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
  prefs_.end();
  mqttUser_ = "";
  mqttPass_ = "";
  logLine("credentials cleared");
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
    if (deviceUid_.isEmpty()) {
      deviceUid_ = savedUid;
    } else if (deviceUid_ != savedUid) {
      // Sketch UID differs from NVS UID: ignore stale credential set.
      mqttUser_ = "";
      mqttPass_ = "";
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
  prefs_.end();
  return okUser && okPass && okUid;
}

String NodeRemote::defaultClientId() const {
  // Keep under typical MQTT client id limits (often 23).
  // Include UID suffix + efuse suffix + short random for collision avoidance.
  uint64_t mac = ESP.getEfuseMac();
  uint32_t macSuffix = static_cast<uint32_t>(mac & 0xFFFF);
  uint16_t r = static_cast<uint16_t>(esp_random() & 0xFFFF);

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
