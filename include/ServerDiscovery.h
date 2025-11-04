#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <functional>

class ServerDiscovery {
public:
  using BeaconCb     = std::function<void(const IPAddress&, uint16_t, const String&)>;
  using ConnectCb    = std::function<void(const IPAddress&, uint16_t)>;
  using DisconnectCb = std::function<void()>;
  using LineCb       = std::function<void(const String&)>;
  using ErrorCb      = std::function<void(const String&)>;

  explicit ServerDiscovery(uint16_t udpPort = 5091, uint16_t serverPort = 5090)
  : udpPort_(udpPort), serverPort_(serverPort) {}

  // --- Config / callbacks ---
  void onBeacon(BeaconCb cb)           { cb_beacon_ = std::move(cb); }
  void onConnected(ConnectCb cb)       { cb_connected_ = std::move(cb); }
  void onDisconnected(DisconnectCb cb) { cb_disconnected_ = std::move(cb); }
  void onLine(LineCb cb)               { cb_line_ = std::move(cb); }
  void onError(ErrorCb cb)             { cb_error_ = std::move(cb); }

  void setBeaconSignature(const String& sig) { beaconSignature_ = sig; }

  // --- Ciclo de vida ---
  bool begin();                // abre UDP 5091 para “beacon”
  void poll();                 // llámalo en loop()
  void tryConnect(uint32_t timeoutMs = 1500); // intenta conectar TCP a serverIp_
  void disconnect();           // cierra TCP

  // --- Utilidades ---
  void sendDiscoveryPing();                    // emite un datagrama general (opcional)
  void setServerIp(const IPAddress& ip);       // fuerza IP del servidor
  bool hasServerIp() const { return hasServerIp_; }
  bool connected()  const { return connected_; }
  IPAddress serverIp() const { return serverIp_; }
  uint16_t  serverPort() const { return serverPort_; }

  // Salida al server 5090
  bool print(const String& s)   { if (!connected_) return false; client_.print(s);   return true; }
  bool println(const String& s) { if (!connected_) return false; client_.println(s); return true; }
  bool write(const uint8_t* p, size_t n) { if (!connected_) return false; return client_.write(p, n) == n; }

  // Reintentos
  void setReconnect(uint32_t everyMs, uint8_t failThreshold = 3, uint32_t cooldownMs = 30000) {
    reconnectIntervalMs_ = everyMs;
    maxFailedAttempts_   = failThreshold;
    cooldownAfterFailMs_ = cooldownMs;
  }

private:
  void handleUdp_();
  void readTcp_();
  void failConnect_();

  // Config
  uint16_t udpPort_;
  uint16_t serverPort_;
  String   beaconSignature_ = "EASYSIM_SERVER!";

  // Estado UDP
  WiFiUDP   udp_;
  bool      udpStarted_ = false;

  // Estado servidor
  IPAddress serverIp_;
  bool      hasServerIp_ = false;

  // Estado TCP
  WiFiClient client_;
  bool       connected_ = false;
  String     inbuf_;

  // Reintentos TCP
  uint32_t lastConnectAttemptMs_   = 0;
  uint32_t reconnectIntervalMs_    = 6000;
  uint8_t  failedAttempts_         = 0;
  uint8_t  maxFailedAttempts_      = 3;
  bool     cooldownActive_         = false;
  uint32_t cooldownAfterFailMs_    = 30000;
  uint32_t cooldownStartMs_        = 0;

  // Callbacks
  BeaconCb     cb_beacon_;
  ConnectCb    cb_connected_;
  DisconnectCb cb_disconnected_;
  LineCb       cb_line_;
  ErrorCb      cb_error_;
};
