#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WebConfig.h>

// ==========================
// CONFIGURACIÓN
// ==========================
#define BOARD_ID 1
struct struct_message {
  uint8_t msgType;
  uint8_t id;
  String data;
  unsigned int readingId;
};
struct struct_pairing {
  uint8_t msgType;
  uint8_t id;
  uint8_t macAddr[6];
  uint8_t channel;
};

typedef enum MessageType { PAIRING = 0, DATA = 1 } MessageType;

struct_message myData;
struct_message inData;
struct_pairing pairingData;
bool paired = false;
unsigned int readingId = 0;
uint8_t lastPeer[6] = {0};

/*const char* ssid = "SSID3";
const char* password = "TremeSpain_1994";*/

const char* ssid = "";
const char* password = "";

void printMAC(const uint8_t *mac_addr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
          mac_addr[0], mac_addr[1], mac_addr[2],
          mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

bool addPeer(const uint8_t *peer_addr) {
  if (esp_now_is_peer_exist(peer_addr)) {
    //Serial.print("ℹ️ Peer ya existente: ");
    printMAC(peer_addr);
    Serial.println();
    return true;
  }
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peer_addr, 6);
  peerInfo.channel = WiFi.channel();
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("❌ Error al agregar peer");
    return false;
  }
  //Serial.print("✅ Peer agregado: ");
  printMAC(peer_addr);
  //Serial.print(" en canal ");
  Serial.println(WiFi.channel());
  return true;
}

// ==========================
// CALLBACKS
// ==========================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("📤 Envío a ");
  printMAC(mac_addr);
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? " -> Éxito" : " -> Error");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;
  uint8_t type = data[0];

  if (type == DATA) {
    memcpy(&inData, data, sizeof(inData));
    /*Serial.printf("📥 Data de %d: Temp=%.2f Hum=%.2f\n",
                  inData.id, inData.temp, inData.hum);*/
  } 
  else if (type == PAIRING) {
    memcpy(&pairingData, data, sizeof(pairingData));
    //Serial.print("🔗 Pairing solicitado por: ");
    printMAC(pairingData.macAddr);
    Serial.println();

    if (addPeer(pairingData.macAddr)) {
      struct_pairing response;
      response.msgType = PAIRING;
      response.id = 0;
      memcpy(response.macAddr, pairingData.macAddr, 6);
      response.channel = WiFi.channel();
      esp_now_send(pairingData.macAddr, (uint8_t *)&response, sizeof(response));

      memcpy(lastPeer, pairingData.macAddr, 6);
      paired = true;
      //Serial.println("✅ Emparejamiento confirmado");
    }
  }
}

// ==========================
// SETUP
// ==========================
void ESPNOW_setup() {
  Serial.begin(115200);

  // 1. Intentar conectar a WiFi (modo STA)
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(currentConfig.ssid.c_str(), currentConfig.pass.c_str());
  Serial.print("Conectando a WiFi: ");
  Serial.print(ssid);
  Serial.println();

  unsigned long startAttempt = millis();
  const unsigned long wifiTimeout = 10000; // ⏳ 10 segundos

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < wifiTimeout) {
    delay(200);
    Serial.print("...");
  }
  Serial.println();

  // 2. Si conectó, mostrar IP
  int canal = 1; // valor por defecto si no hay WiFi
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("📡 IP WiFi: ");
    Serial.println(WiFi.localIP());
    canal = WiFi.channel();
  }
  /*else{
    Serial.println("Error al conectar, activando AP...");
    //WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP("ESP_EASYSIM", "TremeSpain_1994");  // Configura el canal 1
    Serial.println("Punto de acceso iniciado.");
    Serial.print("Dirección IP del AP: ");
    Serial.println(WiFi.softAPIP());
  }*/

  // 3. Activar modo dual STA+AP (mantener WiFi si estaba conectado)
  WiFi.mode(WIFI_STA);
  Serial.println("Modo Dual (STA + AP) activado.");
  // 4. Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error al iniciar ESP-NOW");
    ESP.restart();
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("✅ Maestro listo y esperando emparejamiento");
}


// ==========================
// LOOP
// ==========================
void ESPNOW_loop() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 5000) {
    if (paired) {
      myData.msgType = DATA;
      myData.id = BOARD_ID;
      myData.data = String(random(0, 300));
      myData.readingId = readingId++;
      esp_now_send(lastPeer, (uint8_t *)&myData, sizeof(myData));
    } else {
      //Serial.println("⏳ Esperando emparejamiento...");
    }
    lastSend = millis();
  }
}
