#include <Arduino.h>
#include "ServerDiscovery.h"

bool ServerDiscovery::begin() {
  if (udpStarted_) return true;

  // Equivalente a: new DatagramSocket(null); bind("0.0.0.0", 5091)
  if (!udp_.begin(udpPort_)) {
    if (cb_error_) cb_error_(String("UDP.begin failed on port ") + udpPort_);
    Serial.print(F("[ServerDiscovery] UDP.begin falló en puerto "));
    Serial.println(udpPort_);
    return false;
  }

  udpStarted_ = true;
  Serial.print(F("[ServerDiscovery] UDP.listen "));
  Serial.println(udpPort_);

  return true;
}

void ServerDiscovery::poll() {
  if (!udpStarted_) return;

  // 1) Leer cualquier datagrama UDP entrante (respuestas / beacons)
  handleUdp_();

  // 2) Si ya estamos conectados por TCP, leer datos
  if (connected_) {
    if (!client_.connected()) {
      client_.stop();
      connected_ = false;
      if (cb_disconnected_) cb_disconnected_();
      Serial.println(F("[ServerDiscovery] TCP desconectado (remote closed)"));
    } else {
      readTcp_();
    }
    return;
  }

  uint32_t now = millis();

  // 3) Si aún no sabemos IP del servidor, solo hacemos discover UDP
  if (!hasServerIp_) {
    // Reutilizamos reconnectIntervalMs_ como intervalo entre pings
    if (now - lastConnectAttemptMs_ >= reconnectIntervalMs_) {
      lastConnectAttemptMs_ = now;
      sendDiscoveryPing();
    }
    return;
  }

  // Cooldown tras demasiados fallos
  if (cooldownActive_) {
    if (now - cooldownStartMs_ < cooldownAfterFailMs_) {
      return; // aún en cooldown
    }
    cooldownActive_ = false;
    failedAttempts_ = 0;
  }

  // Respetar intervalo entre reintentos (no bloqueante)
  if (now - lastConnectAttemptMs_ < reconnectIntervalMs_) return;
  lastConnectAttemptMs_ = now;

  // Intentar conectar
  tryConnect();
}

void ServerDiscovery::tryConnect(uint32_t /*timeoutMs*/) {
  if (!hasServerIp_) {
    if (cb_error_) cb_error_(F("tryConnect() sin serverIp"));
    return;
  }
  if (connected_) return;

  Serial.print(F("[ServerDiscovery] CLIENT.CONNECT → "));
  Serial.print(serverIp_);
  Serial.print(F(":"));
  Serial.println(serverPort_);

  if (client_.connect(serverIp_, serverPort_)) {
    connected_ = true;
    failedAttempts_ = 0;
    Serial.println(F("[ServerDiscovery] CLIENT.CONNECT OK"));
    if (cb_connected_) cb_connected_(serverIp_, serverPort_);
  } else {
    Serial.println(F("[ServerDiscovery] CLIENT.CONNECT FAIL"));
    failConnect_();
  }
}

void ServerDiscovery::disconnect() {
  if (!connected_) return;
  client_.stop();
  connected_ = false;
  if (cb_disconnected_) cb_disconnected_();
  Serial.println(F("[ServerDiscovery] Desconectado por petición"));
}

void ServerDiscovery::sendDiscoveryPing() {
  // Según el protocolo: enviar desde puerto local udpPort_ (ya hecho en begin())
  // al broadcast 255.255.255.255:udpPort_ el texto "Train Simulator Client".
  IPAddress bcast(255, 255, 255, 255);
  const char* msg = "Train Simulator Client";

  udp_.beginPacket(bcast, udpPort_);
  udp_.write((const uint8_t*)msg, strlen(msg));
  udp_.endPacket();

  Serial.print(F("[ServerDiscovery] UDP broadcast '"));
  Serial.print(msg);
  Serial.print(F("' a puerto "));
  Serial.println(udpPort_);
}

void ServerDiscovery::setServerIp(const IPAddress& ip) {
  serverIp_       = ip;
  hasServerIp_    = true;
  cooldownActive_ = false;
  failedAttempts_ = 0;
  lastConnectAttemptMs_ = 0; // que el próximo poll intente conectar pronto

  Serial.print(F("[ServerDiscovery] Server IP fijada manualmente: "));
  Serial.println(serverIp_);
}

void ServerDiscovery::handleUdp_() {
  int packetSize = udp_.parsePacket();
  while (packetSize > 0) {
    IPAddress rip   = udp_.remoteIP();
    uint16_t rport  = udp_.remotePort();

    // buffer de 50 bytes como en el Java original
    char buf[50];
    int len = udp_.read((uint8_t*)buf, sizeof(buf) - 1);
    if (len < 0) len = 0;
    if (len > (int)sizeof(buf) - 1) len = sizeof(buf) - 1;
    buf[len] = 0;
    String msg(buf);  // se corta en el primer '\0', como en Python al hacer rstrip("\0")

    Serial.print(F("[ServerDiscovery] UDP RX size="));
    Serial.print(packetSize);
    Serial.print(F(" len="));
    Serial.print(len);
    Serial.print(F(" desde "));
    Serial.print(rip);
    Serial.print(F(":"));
    Serial.print(rport);
    Serial.print(F(" → '"));
    Serial.print(msg);
    Serial.println(F("'"));

    if (cb_beacon_) {
      cb_beacon_(rip, rport, msg);
    }

    // Solo tomar la primera IP detectada Y cuyo payload coincida con "Train Simulator Server"
    if (!hasServerIp_) {
      if (msg.startsWith("Train Simulator Server") || msg.startsWith("Train Simulator Socket")) {
        serverIp_        = rip;
        hasServerIp_     = true;
        cooldownActive_  = false;
        failedAttempts_  = 0;

        Serial.print(F("[ServerDiscovery] SERVER IP detectada por UDP: "));
        Serial.println(serverIp_);
      } else {
        Serial.println(F("[ServerDiscovery] UDP ignorado (payload no coincide con 'Train Simulator Server')"));
      }
    }

    // Leer siguiente datagrama, si lo hay
    packetSize = udp_.parsePacket();
  }
}

void ServerDiscovery::readTcp_() {
  while (client_.available() > 0) {
    int c = client_.read();
    if (c < 0) break;
    char ch = (char)c;

    if (ch == '\n') {
      String line = inbuf_;
      inbuf_ = "";
      line.trim();
      if (line.length() > 0) {
        Serial.print(F("[ServerDiscovery] TCP LINE: '"));
        Serial.print(line);
        Serial.println(F("'"));
        if (cb_line_) cb_line_(line);
      }
    } else if (ch != '\r') {
      inbuf_ += ch;
      if (inbuf_.length() > 256) {
        inbuf_.remove(0, inbuf_.length() - 256);
      }
    }
  }

  if (!client_.connected()) {
    Serial.println(F("[ServerDiscovery] TCP remote closed"));
    client_.stop();
    connected_ = false;
    if (cb_disconnected_) cb_disconnected_();
  }
}

void ServerDiscovery::failConnect_() {
  failedAttempts_++;
  if (cb_error_) {
    cb_error_(String("CLIENT.CONNECT FAIL (intentos=") + failedAttempts_ + ")");
  }

  if (failedAttempts_ >= maxFailedAttempts_) {
    cooldownActive_  = true;
    cooldownStartMs_ = millis();
    if (cb_error_) {
      cb_error_(String("CLIENT.COOLDOWN ") + cooldownAfterFailMs_ + " ms");
    }
  }
}
