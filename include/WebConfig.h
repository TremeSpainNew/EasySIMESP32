#ifndef WEBCONFIG_H
#define WEBCONFIG_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

struct WiFiConfig {
  String ssid;
  String pass;
  bool enable;
};

WiFiConfig currentConfig;
AsyncWebServer server(80);
String ssidList = "[]";
bool escaneando = false;

// ---------------------- CONFIGURACIÓN ----------------------

bool cargarConfig() {
  if (!LittleFS.exists("/config.json")) {
    Serial.println("❌ Error: config.json no existe");
    return false;
  }
  File file = LittleFS.open("/config.json", "r");
  if (!file) {
    Serial.println("❌ Error abriendo config.json");
    return false;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.println("❌ Error parseando config.json");
    file.close();
    return false;
  }

  currentConfig.ssid = doc["ssid"] | "";
  currentConfig.pass = doc["pass"] | "";
  currentConfig.enable = doc["enable"] | false;
  file.close();

  Serial.println("✅ Configuración WiFi cargada de SPIFFS");
  Serial.printf("🔧 SSID: %s\n", currentConfig.ssid.c_str());
  Serial.printf("🔧 PASS: %s\n", currentConfig.pass.c_str());
  Serial.printf("🔧 Habilitado: %s\n", currentConfig.enable ? "Sí" : "No");

  return true;
}

bool guardarConfig() {
  StaticJsonDocument<256> doc;
  doc["ssid"] = currentConfig.ssid;
  doc["pass"] = currentConfig.pass;
  doc["enable"] = currentConfig.enable;

  File file = LittleFS.open("/config.json", "w");
  if (!file) {
    Serial.println("❌ Error guardando config.json");
    return false;
  }
  serializeJson(doc, file);
  file.close();
  Serial.println("✅ Configuración guardada en SPIFFS");
  return true;
}

// ---------------------- ESCANEO WiFi ----------------------

void iniciarEscaneoWiFiAsync() {
  xTaskCreate(
    [](void *parameter) {
      Serial.println("📡 Escaneando redes WiFi por solicitud web...");
      int n = WiFi.scanNetworks();
      DynamicJsonDocument doc(2048);
      JsonArray arr = doc.to<JsonArray>();
      for (int i = 0; i < n; ++i) {
        JsonObject net = arr.createNestedObject();
        net["ssid"] = WiFi.SSID(i);
        net["rssi"] = WiFi.RSSI(i);
      }
      String result;
      serializeJson(doc, result);
      ssidList = result;
      escaneando = false;
      vTaskDelete(NULL);  // termina la tarea
    },
    "ScanWiFiTask",
    4096,
    NULL,
    1,
    NULL
  );
}

// ---------------------- SERVIDOR WEB ----------------------

void iniciarServidorWeb() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/leer-config", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<256> doc;
    doc["ssid"] = currentConfig.ssid;
    doc["pass"] = currentConfig.pass;
    doc["enable"] = currentConfig.enable;

    String result;
    serializeJson(doc, result);
    request->send(200, "application/json", result);
  });

  server.on("/escanear-wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!escaneando) {
      escaneando = true;
      iniciarEscaneoWiFiAsync();
    }
    request->send(200, "application/json", ssidList);  // siempre devuelve lo último
  });

  server.on("/guardar-config", HTTP_POST, [](AsyncWebServerRequest *request){},
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String body = "";
      for (size_t i = 0; i < len; i++) {
        body += (char)data[i];
      }

      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, body);
      if (error) {
        request->send(400, "text/plain", "JSON inválido");
        return;
      }

      currentConfig.ssid = doc["ssid"] | "";
      currentConfig.pass = doc["pass"] | "";
      currentConfig.enable = doc["enable"] | false;

      guardarConfig();
      request->send(200, "application/json", R"({"ok":true})");
      Serial.println("📥 Configuración recibida:");
      Serial.println(body);
    }
  );

  // ✅ Endpoint para iniciar emparejamiento
  server.on("/iniciar-pairing", HTTP_GET, [](AsyncWebServerRequest *request){
    
    request->send(200, "application/json", R"({"ok":true})");
  });

  server.serveStatic("/", LittleFS, "/");  // para CSS, JS, favicon, etc.
  server.begin();
  Serial.println("✅ Servidor web iniciado");
}

#endif
