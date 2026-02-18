#pragma once

#include <Arduino.h>
#include <functional>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

class NodeRemote {
 public:
  static constexpr const char* kVersion = "0.4.9";
  using CommandHandler = std::function<void(const String& subTopic, const String& payload)>;

  NodeRemote();
  NodeRemote(const char* claimToken, const char* deviceUid);
  NodeRemote(
      const char* apiBaseUrl,
      const char* claimToken,
      const char* deviceUid,
      const char* mqttHost,
      uint16_t mqttPort = 1883);
  explicit NodeRemote(PubSubClient& client);  // legacy injection

  PubSubClient& mqtt();

  void setClaimConfig(const String& apiBaseUrl, const String& claimToken, const String& deviceUid);
  void setMqttServer(const String& host, uint16_t port);
  // MQTT TLS is enabled by default (port 8883). For quick testing, insecure TLS is enabled by default.
  // For production, set insecure=false and provide a CA cert via setMqttCaCert().
  void setMqttTlsEnabled(bool enabled);
  void setMqttTlsInsecure(bool insecure);
  void setMqttCaCert(const char* caCertPem);
  void setHeartbeatIntervalMs(uint32_t intervalMs);
  // Periodic status snapshot publish (defaults to 1 hour). Set to 0 to disable.
  void setStatusIntervalMs(uint32_t intervalMs);
  void setCommandHandler(CommandHandler handler);
  void setDebugEnabled(bool enabled);
  void setDebugOutput(Print& out);

  bool begin();
  void loop();

  bool ensureConnected();
  bool claimDevice();
  bool publishUp(const String& subTopic, const String& payload, bool retained = false);
  bool publishAck(const String& commandName, const String& payload, bool retained = false);
  bool sendHeartbeat();
  bool sendStatus();
  // Publish plain text to `devices/<UID>/up/console` so users can view logs in the web console.
  bool println(const String& message);
  bool println(const char* message);

  bool hasCredentials() const;
  String deviceUid() const;
  String mqttUsername() const;
  String lastError() const;
  void clearCredentials();

  bool connectMqtt();
  bool subscribeDownTopics();
  bool handleOtaPayload(const String& payload);
  void publishOtaProgress(const String& jobUid, const String& state, int pct);
  void publishOtaResult(const String& jobUid, bool ok, const String& error, const String& version, const String& firmwareUid);

 private:
  static constexpr const char* kPrefsNs = "espnode";
  static constexpr const char* kPrefsMqttUser = "mqtt_user";
  static constexpr const char* kPrefsMqttPass = "mqtt_pass";
  static constexpr const char* kPrefsDeviceUid = "device_uid";

  WiFiClient netPlain_;
  WiFiClientSecure netTls_;
  PubSubClient mqtt_;
  bool ownsClient_ = true;
  Preferences prefs_;
  CommandHandler commandHandler_;

  String apiBaseUrl_;
  String claimToken_;
  String mqttHost_;
  String mqttUser_;
  String mqttPass_;
  String deviceUid_;
  String upBaseTopic_;
  String downBaseTopic_;
  String lastError_;
  uint16_t mqttPort_ = 1883;
  uint32_t heartbeatIntervalMs_ = 60000;
  uint32_t lastHeartbeatMs_ = 0;
  uint32_t statusIntervalMs_ = 60UL * 60UL * 1000UL;  // 1 hour
  uint32_t lastStatusMs_ = 0;
  uint32_t lastMqttReconnectAttemptMs_ = 0;
  uint32_t lastClaimAttemptMs_ = 0;
  uint32_t claimRetryIntervalMs_ = 5000;
  bool claimPermanentlyFailed_ = false;
  bool rebootPending_ = false;
  uint32_t rebootAtMs_ = 0;
  bool sleepPending_ = false;
  uint32_t sleepAtMs_ = 0;
  uint64_t sleepWakeUs_ = 0;
  bool wipeIdentityPending_ = false;
  uint32_t wipeIdentityAtMs_ = 0;

  bool otaInProgress_ = false;
  uint32_t lastOtaProgressMs_ = 0;
  bool otaPending_ = false;
  String otaPendingPayload_;

  bool mqttTlsEnabled_ = true;
  bool mqttTlsInsecure_ = true;
  const char* mqttCaCertPem_ = nullptr;

  static NodeRemote* instance_;

  static void staticCallback(char* topic, uint8_t* payload, unsigned int length);
  void handleMqttMessage(const String& topic, const String& payload);
  void handleDefaultCommand(const String& subTopic, const String& payload);
  bool shouldAttemptClaimNow();
  void noteClaimFailure(bool permanent);
  void noteClaimSuccess();
  void updateTopicBases();
  bool loadCredentials();
  bool saveCredentials(const String& mqttUser, const String& mqttPass, const String& deviceUid);
  bool checkBootRevokeWindow();
  String defaultClientId() const;
  void logLine(const String& msg);
  void logThrottled(const String& msg, uint32_t& lastMs, uint32_t everyMs);

  Print* logOut_ = &Serial;
  bool debugEnabled_ = true;
  uint32_t lastWifiLogMs_ = 0;
  uint32_t lastClaimLogMs_ = 0;
  uint32_t lastMqttLogMs_ = 0;
  uint32_t lastHeartbeatLogMs_ = 0;
};
