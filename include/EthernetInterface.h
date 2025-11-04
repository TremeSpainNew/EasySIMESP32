#pragma once
#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <esp_eth.h>
#include <NetworkClient.h>
#include <NetworkServer.h>

class EthernetInterface {
  NetworkServer& server;
  NetworkClient client;
  String buffer;
  void (*handler)(const char*, const char*);
  bool useEth = false;

public:
  EthernetInterface(NetworkServer& srv, void (*cb)(const char*, const char*))
    : server(srv), handler(cb) {}

  void begin() {
    server.begin();
  }

  void setUseEth(bool ethConnected) {
    useEth = ethConnected;
  }

  void update() {
    if (!client || !client.connected()) {
      NetworkClient newClient = server.accept();
      if (newClient) {
        client = newClient;
        buffer = "";
        Serial.println(useEth ? "🌐 Cliente conectado por Ethernet." : "📶 Cliente conectado por WiFi.");
      }
    }
  
    if (client && client.connected()) {
      while (client.available()) {
        char c = client.read();
      
        // Acumular caracteres, hasta encontrar fin de línea
        if (c == '\r' || c == '\n') {
          buffer.trim(); // limpia espacios y saltos residuales
        
          if (buffer.length() > 0) {
            // 🛠 DEBUG opcional
            Serial.print(F("[TCP] Comando recibido: «"));
            Serial.print(buffer);
            Serial.println(F("»"));
          
            int sep = buffer.indexOf('=');
            if (sep != -1) {
              String cmd = buffer.substring(0, sep);
              String val = buffer.substring(sep + 1);
              handler(cmd.c_str(), val.c_str());
            } else {
              handler(buffer.c_str(), nullptr);
            }
            buffer = ""; // limpiamos para el siguiente
          }
        } else {
          buffer += c;
        
          // Seguridad: evita que se acumule basura sin \n
          if (buffer.length() > 127) buffer = "";
        }
      }
    }
  }

  void println(const String& msg) {
    if (client && client.connected()) client.println(msg);
  }

  void print(const String& msg) {
    if (client && client.connected()) client.print(msg);
  }

  NetworkClient& getClient() {
    return client;
  }

  void stopClient() {
    if (client) client.stop();
  }
};
