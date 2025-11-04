// ========================== main.cpp / main.ino ==========================
#include <Arduino.h>
#include "EE.h"                 // EEPROM / configuración (NVS)
#include <ControllerMCP.h>
#include <ETH.h>
#include <WiFi.h>
#include <SPI.h>
#include <RS485.h>
#include <NetworkClient.h>
#include <MCPBusController.h>
#include <WiFiAP.h>
#include "WebConfig.h"
#include <modbus.h>             // usa EE por dentro
#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#include <ctype.h>
#include "nvs_flash.h"
#include <math.h>               // isnan, fabs, lroundf
#include <Adafruit_ADS1X15.h>   // ADS1115
#include <WiFiUdp.h>            // UDP para discover (usado dentro de ServerDiscovery normalmente)

#include "EthernetInterface.h"  // interfaz TCP puerto 5000
#include <ServerDiscovery.h>    // NUEVO: discover+cliente 5090 con callbacks

// ---------------- Wrappers para la API nueva de EE ----------------
template<typename T>
inline void EE_GET(uint32_t addr, T &dst) { EE::get(addr, &dst, sizeof(T)); }

template<typename T>
inline void EE_PUT(uint32_t addr, const T &src) { EE::put(addr, &src, sizeof(T)); }

// Asegura que el byte contador (offset 0) esté inicializado a 0 (no 0xFF)
inline void EE_ensureCountByte() {
  uint8_t c = EE::read(0);
  if (c == 0xFF) { EE::write(0, 0); EE::commit(); }
}

// ========================= Variables ========================
bool bloqueaTCP = false;

// ===================== LÍMITES/CONSTANTES EE =================
static constexpr uint32_t EE_SIZE_BYTES      = 16384;   // total NVS simulada por EE

// ======== Región NOTCH persistente (4 KB) ========
static constexpr uint32_t NOTCH_REGION_SIZE  = 4096;
static constexpr uint32_t NOTCH_REGION_BASE  = EE_SIZE_BYTES - NOTCH_REGION_SIZE;
static constexpr uint32_t NOTCH_MAGIC        = 0x4E544348; // 'N''T''C''H'
static constexpr uint8_t  NOTCH_VERSION      = 2;          // V2: guarda centros + valores

// ======== Ventana Modbus (8 KB) al final ========
static constexpr uint32_t MB_REGION_SIZE     = 8192;    // 8 KB
static constexpr uint32_t MB_REGION_BASE     = EE_SIZE_BYTES - MB_REGION_SIZE;

// ========================= AP WiFI ===========================
void handleLine(const char* command, const char* value);
void handleMbCommand(const String& cmd);
void handleAPButton();
void enableAP();
void disableAP();

// ======================= SPI/I2C comunes =======================
SPIClass SPIBUS(HSPI);
MCPBusController busMCP;

// Pines del bus HSPI (ajusta si procede)
#define PIN_SCK   11
#define PIN_MISO  13
#define PIN_MOSI  12

// I2C
#define PIN_SDA    9
#define PIN_SCL    8

// W5500
#define CS_W5500   10
#define W5500_IRQ   5
#define W5500_RST   4

#ifndef BOARD_TYPE
  #define BOARD_TYPE "BigBoardSep"
#endif
static inline const char* getBoardType() { return BOARD_TYPE; }

// ======================= Canales / Interfaces =======================
#define MODO_ETHERNET
#define MODO_SERIAL

#if defined(MODO_ETHERNET)
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  NetworkServer* confServer = nullptr;      // puerto 5000 (config)
  EthernetInterface* ethernetInterface = nullptr;
#endif

enum CanalActivo { CANAL_SERIAL, CANAL_TCP };
CanalActivo canalActivo = CANAL_SERIAL;
bool tcpSuspendidoPorSerial = false;

// ✅ Control de salida TCP y modo IP
bool ethOutEnabled = true;               // ON por defecto
bool ethStaticFallbackUsed = false;      // TRUE si usamos IP estática de emergencia

#if defined(MODO_SERIAL)
  #define SERIAL_BAUD 115200
  SerialInterface serialInterface(Serial, [](const char* c, const char* v){ handleLine(c,v); });
#endif

static void modbusInitWindowFromUsed();
static inline void modbusRefreshWindow() { modbusInitWindowFromUsed(); }

// ======================= Config de pines =======================
#if defined(PLACA_MEGA)
  #define EEPROM_MAX_ENTRIES 64
#else
  #define EEPROM_MAX_ENTRIES 132
#endif

// 💡 Convención POT-ADS: pin = 128 + canal (0..3)
static inline bool isADSIndex(uint8_t pin)   { return pin >= 128 && pin <= 131; }
static inline uint8_t adsChannel(uint8_t pin){ return (uint8_t)(pin - 128); }

struct __attribute__((packed)) PinConfig {
  uint8_t pin;
  uint8_t type;       // 0 SWITCH, 1 BUTTON, 2 OUTPUT, 3 POT
  char param[20];
  int  minIn, maxIn;
  float minOut, maxOut;
  float suavizado;
  uint8_t  modoEnvio;   // 0 CONTINUO, 1 CAMBIO, 2 INTERVALO, 3 MANUAL
  uint16_t intervalo;   // para INTERVALO
  bool enviarComoEntero;
};

// ======================= Instancias de IO =======================
SwitchMCP*        switches[EEPROM_MAX_ENTRIES];
PushButtonMCP*    buttons[EEPROM_MAX_ENTRIES];
OutputManagerMCP* outputs[EEPROM_MAX_ENTRIES];

IOPin* switchPins[EEPROM_MAX_ENTRIES];
IOPin* buttonPins[EEPROM_MAX_ENTRIES];
IOPin* outputPins[EEPROM_MAX_ENTRIES];

const char* switchParams[EEPROM_MAX_ENTRIES];
const char* buttonParams[EEPROM_MAX_ENTRIES];
const char* outputParams[EEPROM_MAX_ENTRIES];
const char* potParams[EEPROM_MAX_ENTRIES];

int switchCount = 0, buttonCount = 0, potCount = 0, outputCount = 0;

bool modoConfig = false;
bool bloqueado  = false;

// ========== NeoPixel ==========
Adafruit_NeoPixel pixels(1, 48, NEO_GRB + NEO_KHZ800);
#define APButton 0

bool apActive = false;
unsigned long buttonPressStart = 0;
const unsigned long longPressTime = 2000;

// Flush LED solo cuando cambie
static bool ledDirty = false;
static unsigned long lastLedFlush = 0;
static inline void setLed(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  ledDirty = true;
}

// ============ Helpers ============
PinConfig      potCfgs[EEPROM_MAX_ENTRIES];
float          potOutLast[EEPROM_MAX_ENTRIES];
unsigned long  potLastMs[EEPROM_MAX_ENTRIES];
static float   potEMA[EEPROM_MAX_ENTRIES];

static float   potThreshold[EEPROM_MAX_ENTRIES];   // umbral cambio
static float   potAccDelta[EEPROM_MAX_ENTRIES];
static float   potLastEmaSample[EEPROM_MAX_ENTRIES];

// ======== MUESCAS (1..30) ========
static const uint8_t MAX_NOTCHES = 30;
static uint8_t potNotchCount[EEPROM_MAX_ENTRIES];
static float   potNotchVals [EEPROM_MAX_ENTRIES][MAX_NOTCHES];
static float   potNotchHystPct[EEPROM_MAX_ENTRIES];
static int     potLastNotch[EEPROM_MAX_ENTRIES];
static float   potEmaNorm  [EEPROM_MAX_ENTRIES];

// ======== Centros crudos por muesca (V2) ========
static uint16_t potNotchCenterRaw[EEPROM_MAX_ENTRIES][MAX_NOTCHES];
static bool     potNotchHasCenters[EEPROM_MAX_ENTRIES];
static float    potEmaRaw[EEPROM_MAX_ENTRIES];

// --- Híbrido NOTCH+Lineal ---
static bool  potPartial[EEPROM_MAX_ENTRIES];   // OFF por defecto
static float potSnapWin[EEPROM_MAX_ENTRIES];   // fracción del rango crudo

// ADS1115
Adafruit_ADS1115 g_ads;
bool g_ads_ok = false;

// ------------ Helpers nombres de pin -------------
int analogPinFromString(const char* str) {
  if ((str[0] == 'A' || str[0]=='a') && isdigit((unsigned char)str[1])) {
    return A0 + atoi(str + 1);
  }
  // ADS0..ADS3
  if ((str[0]=='A'||str[0]=='a') && (str[1]=='D'||str[1]=='d') && (str[2]=='S'||str[2]=='s')) {
    int ch = atoi(str + 3);
    if (ch >= 0 && ch <= 3) return 128 + ch;
  }
  if (strncasecmp(str, "ADS", 3) == 0) {
    int ch = atoi(str + 3);
    if (ch >= 0 && ch <= 3) return 128 + ch;
  }
  return atoi(str);
}

static inline String pinToStringForDump(uint8_t pin, uint8_t type){
  if (type==3 && isADSIndex(pin)) {
    return String("ADS") + String(adsChannel(pin));
  }
  return String((int)pin);
}

static int getPotIndexByPin(uint8_t pin){
  for (int i=0;i<potCount;i++){
    if (potCfgs[i].pin == pin) return i;
  }
  return -1;
}

// ======== Cliente TCP a servidor principal (5090) manejado por ServerDiscovery ========
static const uint16_t kClientServerPort = 5090;
ServerDiscovery serverDiscovery(5091,5090);   // la clase hace discover UDP+TCP 5090

// ======== Salida unificada (Serial + servidor 5000 + cliente 5090) ========
void enviar(const String& msg) {
#if defined(MODO_SERIAL)
  Serial.println(msg);
#endif
#if defined(MODO_ETHERNET)
  if (ethernetInterface && ethOutEnabled && !tcpSuspendidoPorSerial) {
    ethernetInterface->println(msg);           // a clientes en puerto 5000
  }
  if (serverDiscovery.connected()) {
    serverDiscovery.println(msg);             // espejo al servidor 5090 si está conectado
  }
#endif
}

static inline bool isSpaceC(char c){ return c==' ' || c=='\t' || c=='\r' || c=='\n'; }
static void trimInPlace(String &s){
  int i=0; while(i<(int)s.length() && isSpaceC(s[i])) i++;
  int j=s.length()-1; while(j>=0 && isSpaceC(s[j])) j--;
  if(j<i) { s=""; return; }
  s = s.substring(i, j+1);
}
static bool parseBoolLike(const String& v, int &out){
  String t=v; t.toUpperCase(); trimInPlace(t);
  if (t=="1"||t=="ON"||t=="TRUE"||t=="HIGH")  { out=1; return true; }
  if (t=="0"||t=="OFF"||t=="FALSE"||t=="LOW") { out=0; return true; }
  bool allNum = t.length()>0;
  for (int i=0;i<t.length();++i) allNum &= isdigit((unsigned char)t[i]) || (i==0 && (t[i]=='-'||t[i]=='+'));
  if (allNum) { out = t.toInt(); return true; }
  return false;
}
static String buildModbusSetFromKV(const String& key, const String& valueCSV){
  String out = "MB.SET ";
  out += key; out += " ";
  int start = 0;
  while (start < valueCSV.length()) {
    int comma = valueCSV.indexOf(',', start);
    String token = (comma>=0) ? valueCSV.substring(start, comma) : valueCSV.substring(start);
    trimInPlace(token);
    int v;
    if (!parseBoolLike(token, v)) return String();
    out += String(v);
    if (comma>=0) { out += " "; start = comma+1; }
    else break;
  }
  return out;
}

// ===== Helpers numéricos =====
static inline float clamp01(float x){ if (x<0) return 0; if (x>1) return 1; return x; }

// Selección notch (fallback bins)
static int pickNotchWithHyst(int lastIdx, float norm01, uint8_t N, float hystPct) {
  if (N <= 1) return 0;
  if (lastIdx < 0) {
    int idx = (int)floorf(norm01 * N);
    if (idx < 0) idx = 0;
    if (idx >= (int)N) idx = (int)N - 1;
    return idx;
  }
  float cell = 1.0f / (float)N;
  float h    = hystPct * cell;
  int   i    = lastIdx;
  float lowB  = (i == 0)       ? 0.0f : (i * cell);
  float highB = (i == (int)N-1)? 1.0f : ((i+1) * cell);

  if (norm01 > (highB + h) && i < (int)N-1) return i + 1;
  if (norm01 < (lowB  - h) && i > 0)        return i - 1;
  return i;
}

// Selector por centros crudos con histéresis local
static int pickNotchByCenters(int lastIdx, float rawNow,
                              const uint16_t* centers, uint8_t N, float hystPct) {
  if (N == 0) return -1;
  if (N == 1) return 0;

  if (lastIdx < 0) {
    int best = 0;
    float bestD = fabsf(rawNow - centers[0]);
    for (uint8_t i = 1; i < N; ++i) {
      float d = fabsf(rawNow - centers[i]);
      if (d < bestD) { bestD = d; best = i; }
    }
    return best;
  }

  int i = lastIdx;

  float ci   = centers[i];
  float cim1 = (i > 0)        ? centers[i-1] : centers[0];
  float cip1 = (i < (int)N-1) ? centers[i+1] : centers[N-1];

  float midDown = (i > 0)        ? 0.5f * (cim1 + ci) : (ci - (cip1-ci));
  float midUp   = (i < (int)N-1) ? 0.5f * (ci + cip1) : (ci + (ci-cim1));

  float cellDown = (i > 0)        ? (ci - cim1) : (cip1 - ci);
  float cellUp   = (i < (int)N-1) ? (cip1 - ci) : (ci - cim1);
  if (cellDown < 0) cellDown = 0; if (cellUp < 0) cellUp = 0;

  float hDown = hystPct * cellDown;
  float hUp   = hystPct * cellUp;

  if (i < (int)N-1 && rawNow > (midUp + hUp))     return i + 1;
  if (i > 0        && rawNow < (midDown - hDown)) return i - 1;
  return i;
}

// ======================= Carga desde EE =======================
void loadConfigFromEEPROM() {
  uint8_t count = EE::read(0);
  if (count == 0xFF) { EE::write(0, 0); EE::commit(); count = 0; }

#if defined(MODO_ETHERNET)
  if (ethernetInterface) {
    ethernetInterface->print(F("Tamaño de PinConfig: "));
    ethernetInterface->println(String((int)sizeof(PinConfig)));
  }
#endif

  for (int i = 0; i < count && i < EEPROM_MAX_ENTRIES; i++) {
    PinConfig cfg;
    EE_GET(1 + i * sizeof(PinConfig), cfg);

    for (int j = 0; j < (int)sizeof(cfg.param); j++) {
      if (!isprint((unsigned char)cfg.param[j]) && cfg.param[j] != '\0') { cfg.param[j] = '\0'; break; }
    }
    cfg.param[sizeof(cfg.param) - 1] = '\0';

    char* copy = strdup(cfg.param);
    if (!copy || strlen(copy) == 0) {
      enviar(String("⚠️ Entrada con parámetro inválido ignorada en EEPROM entrada ") + i);
      if (copy) free(copy);
      continue;
    }

    IOPin* pin = nullptr;
    if (cfg.pin <= 127) pin = busMCP.getPin(cfg.pin, cfg.type != 2);  // OUTPUT es salida

    switch (cfg.type) {
      case 0: { // SWITCH
        if (!pin) { free(copy); break; }
        char val1[12], val2[12];
        dtostrf(cfg.minOut, 1, 3, val1);
        dtostrf(cfg.maxOut, 1, 3, val2);
        switchParams[switchCount] = copy;
        switchPins[switchCount] = pin;
        switches[switchCount] = new SwitchMCP(pin, strdup(copy), strdup(val1), strdup(val2));
        switches[switchCount]->begin();
        switchCount++;
        break;
      }
      case 1: { // BUTTON
        if (!pin) { free(copy); break; }
        char val1[12], val2[12];
        dtostrf(cfg.minOut, 1, 3, val1);
        dtostrf(cfg.maxOut, 1, 3, val2);
        buttonParams[buttonCount] = copy;
        buttonPins[buttonCount] = pin;
        buttons[buttonCount] = new PushButtonMCP(pin, strdup(copy), strdup(val1), strdup(val2));
        buttons[buttonCount]->begin();
        buttonCount++;
        break;
      }
      case 2: { // OUTPUT
        if (!pin) { free(copy); break; }
        outputParams[outputCount] = copy;
        outputPins[outputCount] = pin;
        outputs[outputCount] = new OutputManagerMCP(copy, pin);
        outputs[outputCount]->begin();
        outputCount++;
        break;
      }
      case 3: { // POT
        potParams[potCount]   = copy;
        potCfgs[potCount]     = cfg;
        potOutLast[potCount]  = NAN;
        potLastMs[potCount]   = 0;
        potEMA[potCount]      = NAN;
        potThreshold[potCount]= 0.5f;
        potAccDelta[potCount] = 0.0f;
        potLastEmaSample[potCount] = NAN;

        potNotchCount[potCount]    = 0;
        potNotchHystPct[potCount]  = 0.05f;
        potLastNotch[potCount]     = -1;
        potEmaNorm[potCount]       = NAN;

        potNotchHasCenters[potCount] = false;
        potEmaRaw[potCount] = NAN;

        potPartial[potCount] = false;
        potSnapWin[potCount] = 0.03f;

        potCount++;
        break;
      }
      default:
        enviar(String("⚠️ Tipo desconocido en EEPROM en entrada ") + i);
        free(copy);
        break;
    }
  }
}

void clearEEPROMIfNeeded() {
  uint8_t count = EE::read(0);
  if (count == 0xFF) {
    EE::write(0, 0);
    EE::commit();
    enviar(F("ℹ️ EEPROM normalizada (contador estaba 0xFF)."));
  } else if (count == 0) {
    enviar(F("ℹ️ EEPROM ya vacía, nada que borrar."));
  } else {
    EE::write(0, 0);
    EE::commit();
    enviar(F("🧹 EEPROM: índice puesto a 0 (sin barrer NVS)."));
  }

  for (int i = 0; i < switchCount; i++) { delete switches[i]; switches[i] = nullptr; }
  for (int i = 0; i < buttonCount; i++) { delete buttons[i]; buttons[i] = nullptr; }
  for (int i = 0; i < outputCount; i++) { delete outputs[i]; outputs[i] = nullptr; }
  switchCount = buttonCount = potCount = outputCount = 0;

  modbusRefreshWindow();

  enviar(F("✅ EEPROM borrada lógicamente y memoria liberada."));
}

void savePinConfig(const String& tipo, int pin, const char* param, const char* v1, const char* v2) {
  uint8_t count = EE::read(0);
  if (count >= EEPROM_MAX_ENTRIES) return;

  for (int i = 0; i < count; i++) {
    PinConfig cfg;
    EE_GET(1 + i * sizeof(PinConfig), cfg);
    if (cfg.pin == pin) return;
  }

  PinConfig cfg{};
  if      (tipo.equalsIgnoreCase("SWITCH")) cfg.type = 0;
  else if (tipo.equalsIgnoreCase("BUTTON")) cfg.type = 1;
  else if (tipo.equalsIgnoreCase("OUTPUT")) cfg.type = 2;
  else if (tipo.equalsIgnoreCase("POT"))    cfg.type = 3;
  else return;

  strncpy(cfg.param, param, sizeof(cfg.param));
  cfg.param[sizeof(cfg.param) - 1] = '\0';
  cfg.pin = (uint8_t)pin;
  cfg.minOut = atof(v1);
  cfg.maxOut = atof(v2);

  if (cfg.type == 3) {
    if (isADSIndex(cfg.pin)) {
      cfg.minIn  = 0;
      cfg.maxIn  = 32767;
    } else {
      cfg.minIn  = 0;
    #if defined(ESP32)
      cfg.maxIn  = 4095;
    #else
      cfg.maxIn  = 1023;
    #endif
    }
    cfg.suavizado = 0.10f;
    cfg.modoEnvio = 0;       // CONTINUO
    cfg.intervalo = 200;
    cfg.enviarComoEntero = true;
  }

  EE_PUT(1 + count * sizeof(PinConfig), cfg);
  EE::write(0, count + 1);
  EE::commit();

  enviar(String("✅ Entrada añadida a EEPROM: ") + param);
  modbusRefreshWindow();
}

// ================== PERSISTENCIA NOTCH ==================
static void notchEraseRegion() {
  uint32_t zero = 0;
  EE_PUT(NOTCH_REGION_BASE + 0, zero); // magic = 0
  EE::commit();
}

static bool notchSaveAllToEEPROM() {
  uint32_t off = NOTCH_REGION_BASE;
  EE_PUT(off, NOTCH_MAGIC);                   off += sizeof(uint32_t);
  EE::write(off, NOTCH_VERSION);              off += 1;
  EE::write(off, 0); EE::write(off+1, 0); EE::write(off+2, 0); // reservados
  off += 3;

  uint16_t numRecs = 0;
  EE_PUT(off, numRecs);                       off += sizeof(uint16_t);

  for (int i = 0; i < potCount; i++) {
    uint8_t cnt = potNotchCount[i];
    if (cnt == 0) continue;

    uint8_t pin = potCfgs[i].pin;
    bool hasC = potNotchHasCenters[i];

    size_t recSize = 1 + 1 + sizeof(float) + 1 + 1 + (cnt*sizeof(uint16_t)) + (cnt*sizeof(float));
    if (off + recSize > NOTCH_REGION_BASE + NOTCH_REGION_SIZE) {
      enviar(F("❌ NOTCH SAVEALL: sin espacio."));
      return false;
    }

    EE::write(off, pin);               off += 1;
    EE::write(off, cnt);               off += 1;
    EE_PUT(off, potNotchHystPct[i]);   off += sizeof(float);
    uint8_t flags = hasC ? 0x01 : 0x00;
    EE::write(off, flags);             off += 1;
    EE::write(off, 0);                 off += 1; // reservado

    for (uint8_t k=0;k<cnt;k++){
      uint16_t c = hasC ? potNotchCenterRaw[i][k] : 0;
      EE_PUT(off, c); off += sizeof(uint16_t);
    }
    for (uint8_t k=0;k<cnt;k++){
      EE_PUT(off, potNotchVals[i][k]); off += sizeof(float);
    }
    numRecs++;
  }

  uint32_t numRecsPos = NOTCH_REGION_BASE + 4 + 1 + 3;
  EE_PUT(numRecsPos, numRecs);

  EE::commit();
  enviar(String("💾 NOTCH SAVEALL: ") + numRecs + " registros.");
  return true;
}

static void notchLoadAllFromEEPROM() {
  uint32_t off = NOTCH_REGION_BASE;

  uint32_t magic = 0;
  EE_GET(off, magic); off += sizeof(uint32_t);
  if (magic != NOTCH_MAGIC) {
    enviar(F("ℹ️ NOTCH LOADALL: no hay tabla."));
    return;
  }

  uint8_t ver = EE::read(off); off += 1;
  off += 3; // reservados

  uint16_t numRecs = 0;
  EE_GET(off, numRecs); off += sizeof(uint16_t);

  for (int i=0;i<potCount;i++) {
    potNotchCount[i] = 0;
    potLastNotch[i]  = -1;
    potEmaNorm[i]    = NAN;
    potEmaRaw[i]     = NAN;
    potNotchHasCenters[i] = false;
  }

  for (uint16_t r=0; r<numRecs; r++) {
    if (off + 1 + 1 + sizeof(float) > NOTCH_REGION_BASE + NOTCH_REGION_SIZE) break;

    uint8_t pin  = EE::read(off); off += 1;
    uint8_t cnt  = EE::read(off); off += 1;
    float   hyst = 0.0f; EE_GET(off, hyst); off += sizeof(float);

    int idx = getPotIndexByPin(pin);

    if (ver >= 2) {
      uint8_t flags = EE::read(off); off += 1;
      off += 1; // rsv
      bool hasC = (flags & 0x01);

      uint32_t centersPos = off;
      off += cnt * sizeof(uint16_t);

      if (idx >= 0 && cnt > 0) {
        if (cnt > MAX_NOTCHES) cnt = MAX_NOTCHES;
        potNotchCount[idx]   = cnt;
        potNotchHystPct[idx] = hyst;
        potNotchHasCenters[idx] = hasC;

        for (uint8_t k=0;k<cnt;k++){
          uint16_t c=0; EE_GET(centersPos + k*sizeof(uint16_t), c);
          potNotchCenterRaw[idx][k] = c;
        }
        for (uint8_t k=0;k<cnt;k++){
          float v=0.0f; EE_GET(off + k*sizeof(float), v);
          potNotchVals[idx][k] = v;
        }
        potLastNotch[idx] = -1;
        potEmaNorm[idx]   = NAN;
        potEmaRaw[idx]    = NAN;
      }
      off += cnt * sizeof(float);
    } else {
      if (idx >= 0 && cnt > 0) {
        if (cnt > MAX_NOTCHES) cnt = MAX_NOTCHES;
        potNotchCount[idx]   = cnt;
        potNotchHystPct[idx] = hyst;
        for (uint8_t k=0;k<cnt;k++){
          float v=0.0f; EE_GET(off + k*sizeof(float), v);
          potNotchVals[idx][k] = v;
        }
        potNotchHasCenters[idx] = false;
        potLastNotch[idx] = -1;
        potEmaNorm[idx]   = NAN;
        potEmaRaw[idx]    = NAN;
      }
      off += cnt * sizeof(float);
    }
  }

  enviar(String("📥 NOTCH LOADALL: ") + numRecs + " registros cargados.");
}

static bool notchSaveOneToEEPROM(uint8_t /*pin*/) {
  return notchSaveAllToEEPROM();
}

static bool notchDeleteFromEEPROM(uint8_t pin) {
  int idx = getPotIndexByPin(pin);
  if (idx >= 0) {
    potNotchCount[idx] = 0;
    potLastNotch[idx]  = -1;
    potEmaNorm[idx]    = NAN;
    potEmaRaw[idx]     = NAN;
    potNotchHasCenters[idx] = false;
  }
  return notchSaveAllToEEPROM();
}

void updatePotParam(String cmd) {
  char p1[10], field[10], a[16], b[16], c[16], d[16];
  int args = sscanf(cmd.c_str() + 4, "%9s %9s %15s %15s %15s %15s", p1, field, a, b, c, d);
  int pin = analogPinFromString(p1);
  uint8_t count = EE::read(0);

  for (int i = 0; i < count; i++) {
    PinConfig cfg;
    EE_GET(1 + i * sizeof(PinConfig), cfg);
    if ((int)cfg.pin == pin && cfg.type == 3) {
      PinConfig original = cfg;

      String f = String(field);
      if (f == "SCALE" && args >= 6) {
        cfg.minIn = atoi(a); cfg.maxIn = atoi(b);
        cfg.minOut = atof(c); cfg.maxOut = atof(d);
      } else if (f == "SMOOTH" && args >= 3) {
        cfg.suavizado = atof(a);
      } else if (f == "MODE" && args >= 3) {
        if (String(a) == "CONTINUO") cfg.modoEnvio = 0;
        else if (String(a) == "CAMBIO") cfg.modoEnvio = 1;
        else if (String(a) == "INTERVALO") { cfg.modoEnvio = 2; cfg.intervalo = atoi(b); }
        else if (String(a) == "MANUAL") cfg.modoEnvio = 3;
      } else if (f == "FORMAT" && args >= 3) {
        cfg.enviarComoEntero = (String(a) == "INT");
      } else if (f == "THRESH" && args >= 3) {
        float th = atof(a);
        int idx = getPotIndexByPin(cfg.pin);
        if (idx >= 0) {
          potThreshold[idx] = th;
          enviar(String("THRESH OK en ") + pinToStringForDump(cfg.pin, cfg.type) + " = " + String(th, 4));
        } else {
          enviar(F("THRESH: pot aún no inicializado en RAM (reinicia o #END/#CONFIG)."));
        }
        return;
      } else {
        enviar(F("ERROR: CFG inválido"));
        return;
      }

      if (memcmp(&cfg, &original, sizeof(PinConfig)) != 0) {
        EE_PUT(1 + i * sizeof(PinConfig), cfg);
        EE::commit();
        enviar("CFG ACTUALIZADO EN PIN " + String(pinToStringForDump(cfg.pin, cfg.type)));
      } else {
        enviar("CFG SIN CAMBIOS EN PIN " + String(pinToStringForDump(cfg.pin, cfg.type)));
      }
      return;
    }
  }
}

static void nvsWipeAllAndReboot() {
  enviar("⚠️ Borrando NVS completa…");
  nvs_flash_deinit();
  nvs_flash_erase();
  nvs_flash_init();
  enviar("✅ NVS borrada. Reiniciando…");
  delay(300);
  ESP.restart();
}

// ========= Ventana Modbus con EE =========
static void modbusInitWindowFromUsed() {
  uint8_t count = EE::read(0);
  if (count == 0xFF) { count = 0; EE::write(0, 0); EE::commit(); }

  modbusSetEepromWindow(MB_REGION_BASE, MB_REGION_SIZE);

  enviar("MB.WINDOW base=" + String(MB_REGION_BASE) +
          " size=" + String(MB_REGION_SIZE) +
          " total=" + String(EE_SIZE_BYTES));
}

// ========= Networking =========
bool eth_connected = false;
void onEthEvent(arduino_event_id_t event, arduino_event_info_t /*info*/) {
  if (event == ARDUINO_EVENT_ETH_GOT_IP) {
    eth_connected = true;
    Serial.println("🌐 Ethernet conectado");
  } else if (event == ARDUINO_EVENT_ETH_DISCONNECTED ||
              event == ARDUINO_EVENT_ETH_LOST_IP ||
              event == ARDUINO_EVENT_ETH_STOP) {
    eth_connected = false;
    Serial.println("📴 Ethernet desconectado");
  }
}

// ===== Helpers POT =====
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ===================== Lectura/envío de POTs =====================
void tickPots() {
  for (int i = 0; i < potCount; i++) {
    const PinConfig &cfg = potCfgs[i];

    int raw = 0;
    if (isADSIndex(cfg.pin)) {
      if (!g_ads_ok) continue;
      uint8_t ch = adsChannel(cfg.pin);
      int16_t r = g_ads.readADC_SingleEnded(ch);
      if (r < 0) r = 0;
      raw = (int)r; // 0..~32767 según PGA
    } else {
      raw = analogRead(cfg.pin);
    }

    float val = mapf((float)raw, (float)cfg.minIn, (float)cfg.maxIn, cfg.minOut, cfg.maxOut);
    if (val < cfg.minOut) val = cfg.minOut;
    if (val > cfg.maxOut) val = cfg.maxOut;

    bool first = isnan(potEMA[i]);
    if (first) potEMA[i] = val;
    else       potEMA[i] = (1.0f - cfg.suavizado) * potEMA[i] + cfg.suavizado * val;

    float outVal = potEMA[i];
    if (outVal < cfg.minOut) outVal = cfg.minOut;
    if (outVal > cfg.maxOut) outVal = cfg.maxOut;

    bool debeEnviar = false;
    unsigned long now = millis();

    switch (cfg.modoEnvio) {
      case 0: // CONTINUO
        debeEnviar = true;
        break;

      case 1: { // CAMBIO (+ NOTCH/HÍBRIDO)
        const bool hasNotches = (potNotchCount[i] > 0);

        if (hasNotches) {
          float rawNow = (float)raw;
          if (isnan(potEmaRaw[i])) potEmaRaw[i] = rawNow;
          else                     potEmaRaw[i] = (1.0f - cfg.suavizado) * potEmaRaw[i] + cfg.suavizado * rawNow;

          int currByCenters = -1;
          if (potNotchHasCenters[i]) {
            currByCenters = pickNotchByCenters(potLastNotch[i], potEmaRaw[i],
                                               potNotchCenterRaw[i], potNotchCount[i], potNotchHystPct[i]);
          } else {
            float norm = clamp01((float)(raw - potCfgs[i].minIn) / (float)(potCfgs[i].maxIn - potCfgs[i].minIn));
            if (isnan(potEmaNorm[i])) potEmaNorm[i] = norm;
            else                      potEmaNorm[i] = (1.0f - cfg.suavizado) * potEmaNorm[i] + cfg.suavizado * norm;
            currByCenters = pickNotchWithHyst(potLastNotch[i], potEmaNorm[i],
                                              potNotchCount[i], potNotchHystPct[i]);
          }

          bool near = false;
          int  nearIdx = currByCenters;

          float snapAbs = potSnapWin[i] * (float)(potCfgs[i].maxIn - potCfgs[i].minIn);
          if (snapAbs < 0) snapAbs = 0;

          if (potNotchHasCenters[i]) {
            float bestD = 1e9f; int bestI = -1;
            for (uint8_t k=0;k<potNotchCount[i];k++){
              float d = fabsf(potEmaRaw[i] - (float)potNotchCenterRaw[i][k]);
              if (d < bestD) { bestD = d; bestI = (int)k; }
            }
            near    = (bestD <= snapAbs);
            nearIdx = bestI;
          } else {
            float minR = (float)potCfgs[i].minIn, maxR = (float)potCfgs[i].maxIn;
            float cell = (potNotchCount[i] > 0) ? (maxR - minR)/ (float)potNotchCount[i] : (maxR - minR);
            float center = minR + (nearIdx + 0.5f) * cell;
            near  = (fabsf(potEmaRaw[i] - center) <= snapAbs);
          }

          if (potPartial[i] && !near) {
            float prev = potOutLast[i];
            float thr  = potThreshold[i];

            if (isnan(prev) || fabs(outVal - prev) >= thr) {
              if (cfg.enviarComoEntero) {
                enviar(String(potParams[i]) + "=" + String((int)lroundf(outVal)));
              } else {
                char f[24]; dtostrf(outVal, 0, 3, f); char* p = f; while (*p==' ') ++p;
                enviar(String(potParams[i]) + "=" + String(p));
              }
              potOutLast[i] = outVal;
              potLastMs[i]  = now;
            }
          } else {
            int curr = (nearIdx < 0) ? 0 : nearIdx;

            if (potLastNotch[i] < 0) {
              potLastNotch[i] = curr;
              debeEnviar = true;
            } else if (curr != potLastNotch[i]) {
              potLastNotch[i] = curr;
              debeEnviar = true;
            }

            if (debeEnviar) {
              float notchVal = potNotchVals[i][potLastNotch[i]];
              if (cfg.enviarComoEntero) {
                enviar(String(potParams[i]) + "=" + String((int)lroundf(notchVal)));
              } else {
                char f[24]; dtostrf(notchVal, 0, 3, f); char* p = f; while (*p==' ') ++p;
                enviar(String(potParams[i]) + "=" + String(p));
              }
              potOutLast[i] = notchVal;
              potLastMs[i]  = now;
            }
          }
        } else {
          float prev = potOutLast[i];
          float thr  = potThreshold[i];

          if (isnan(prev)) {
            debeEnviar = true;
          } else if (fabs(outVal - prev) >= thr) {
            debeEnviar = true;
          }

          if (debeEnviar) {
            if (cfg.enviarComoEntero) {
              enviar(String(potParams[i]) + "=" + String((int)lroundf(outVal)));
            } else {
              char f[24]; dtostrf(outVal, 0, 3, f);
              char* p = f; while (*p == ' ') ++p;
              enviar(String(potParams[i]) + "=" + String(p));
            }
            potOutLast[i] = outVal;
            potLastMs[i]  = now;
          }
        }
        break;
      }

      case 2: // INTERVALO
        if ((now - potLastMs[i]) >= cfg.intervalo) debeEnviar = true;
        break;

      case 3: // MANUAL
      default:
        debeEnviar = false;
        break;
    }

    if (debeEnviar) {
      if (potNotchCount[i] > 0) {
        int idxN = (potLastNotch[i] < 0) ? 0 : potLastNotch[i];
        float notchVal = potNotchVals[i][idxN];
        if (cfg.enviarComoEntero) {
          enviar(potParams[i] + String("=") + String((int)lroundf(notchVal)));
        } else {
          char f[24]; dtostrf(notchVal, 0, 3, f);
          char* p = f; while (*p == ' ') ++p;
          enviar(String(potParams[i]) + "=" + String(p));
        }
        potOutLast[i] = notchVal;
      } else {
        if (cfg.enviarComoEntero) {
          enviar(potParams[i] + String("=") + String((int)lroundf(outVal)));
        } else {
          char f[24]; dtostrf(outVal, 0, 3, f);
          char* p = f; while (*p == ' ') ++p;
          enviar(String(potParams[i]) + "=" + String(p));
        }
        potOutLast[i] = outVal;
      }
      potLastMs[i] = now;
    }
  }
}

// ========= handleLine (comandos) =========
void handleLine(const char* command, const char* value) {
  if (!command || strlen(command) == 0) return;

#if defined(MODO_ETHERNET)
  bloqueaTCP = true;
  tcpSuspendidoPorSerial = true;
#endif

  String cmd = String(command);
  String val = value ? String(value) : "";

  if (cmd.startsWith("MB.") || cmd.startsWith("MODBUS.")) { handleMbCommand(cmd); return; }

#if defined(MODO_ETHERNET)
  if (cmd.equalsIgnoreCase("ETH.STATUS")) {
    IPAddress ip = ETH.localIP();
    bool linkUp = ETH.linkUp();
    bool hasIp  = (ip != IPAddress((uint32_t)0));
    String mode = ethStaticFallbackUsed ? "STATIC" : (hasIp ? "DHCP" : "UNKNOWN");

    IPAddress srvIp = serverDiscovery.serverIp();
    bool hasSrv = serverDiscovery.hasServerIp();

    String s = "ETH.STATUS OUT=";
    s += (ethOutEnabled ? "ON" : "OFF");
    s += " LINK="; s += (linkUp ? "UP" : "DOWN");
    s += " MODE="; s += mode;
    s += " IP=";   s += ip.toString();
    s += " CLIENT="; s += (serverDiscovery.connected() ? "UP" : "DOWN");
    s += " SERVER.IP="; s += (hasSrv ? srvIp.toString() : "0.0.0.0");
    s += " SERVER.PORT="; s += String(kClientServerPort);
    enviar(s);
    return;
  }

  if (cmd.startsWith("ETH.OUT")) {
    String arg = val;
    if (!arg.length()) {
      int sp = cmd.indexOf(' ');
      if (sp > 0) arg = cmd.substring(sp + 1);
    }
    arg.trim(); arg.toUpperCase();
    if (arg == "ON" || arg == "1" || arg == "TRUE") {
      ethOutEnabled = true;
      enviar(F("OK ETH.OUT ON"));
    } else if (arg == "OFF" || arg == "0" || arg == "FALSE") {
      ethOutEnabled = false;
      enviar(F("OK ETH.OUT OFF"));
    } else {
      enviar(F("ERR ETH.OUT (usa ON|OFF)"));
    }
    return;
  }
#endif

  // 🔧 Set IP manual del servidor por discover (compatible con lo que tenías)
  if (cmd.startsWith("DISCOVER.SETIP")) {
    String ipS = val;
    if (!ipS.length()) {
      int sp = cmd.indexOf(' ');
      if (sp > 0) ipS = cmd.substring(sp + 1);
    }
    ipS.trim();
    IPAddress ip;
    if (ip.fromString(ipS)) {
      serverDiscovery.setServerIp(ip);
      enviar(String("📡 DISCOVERY[MANUAL]: server=") + ip.toString() + ":" + String(kClientServerPort));
    } else {
      enviar(String("❌ DISCOVER.SETIP inválida: ") + ipS);
    }
    return;
  }

  // 🔧 Set IP manual del servidor (ETH.SERVER <ip>)
  if (cmd.startsWith("ETH.SERVER")) {
    String ipS = val;
    if (!ipS.length()) {
      int sp = cmd.indexOf(' ');
      if (sp > 0) ipS = cmd.substring(sp + 1);
    }
    ipS.trim();

    IPAddress ip;
    if (ip.fromString(ipS)) {
      serverDiscovery.setServerIp(ip);
      enviar(String("OK ETH.SERVER ") + ip.toString());
    } else {
      enviar(F("ERR ETH.SERVER ip inválida (usa p.ej. 192.168.0.10)"));
    }
    return;
  }

  // 2) Modo configuración ON/OFF
  if (cmd == "#CONFIG") {
    modoConfig = true;
    enviar(F("✅ MODO CONFIG ACTIVADO"));
    bloqueado = false;
    return;
  }

  if (cmd == "#END") {
    modoConfig = false;
    enviar(F("✅ MODO CONFIG DESACTIVADO"));
    delay(100);
    enviar(F("#READY"));
    delay(300);
    if (!apActive) {
  #if defined(__AVR__)
      wdt_enable(WDTO_15MS); while (1);
  #else
      ESP.restart();
  #endif
    } else {
      enviar(F("⚠️ AP activo, no se reinicia"));
      loadConfigFromEEPROM();
      notchLoadAllFromEEPROM();
    }
    bloqueado = false;
    return;
  }

  if (cmd == "#CLEAR") {
    clearEEPROMIfNeeded();
    notchEraseRegion();
    enviar(F("✅ EEPROM borrada correctamente."));
    delay(100);
    enviar(F("#READY"));
    modbusRefreshWindow();
    return;
  }

  if (cmd == "#NVSWIPE") {
    nvsWipeAllAndReboot();
    return;
  }

  if (modoConfig && cmd.startsWith("#DELETEPIN")) {
    int pinToDelete = analogPinFromString(cmd.c_str() + 10);
    int count = EE::read(0);
    bool eliminado = false;

    for (int i = 0; i < count; i++) {
      PinConfig cfg;
      EE_GET(1 + i * sizeof(PinConfig), cfg);
      if ((int)cfg.pin == pinToDelete) {

        switch (cfg.type) {
          case 0: {
            for (int j = 0; j < switchCount; j++) {
              if (switches[j] && switchPins[j] && switchPins[j]->getPinNumber() == pinToDelete) {
                delete switches[j]; switches[j]=nullptr; switchPins[j]=nullptr;
                free((void*)switchParams[j]); switchParams[j]=nullptr;
                break;
              }
            }
            break;
          }
          case 1: {
            for (int j = 0; j < buttonCount; j++) {
              if (buttons[j] && buttonPins[j] && buttonPins[j]->getPinNumber() == pinToDelete) {
                delete buttons[j]; buttons[j]=nullptr; buttonPins[j]=nullptr;
                free((void*)buttonParams[j]); buttonParams[j]=nullptr;
                break;
              }
            }
            break;
          }
          case 2: {
            for (int j = 0; j < outputCount; j++) {
              if (outputs[j] && outputPins[j] && outputPins[j]->getPinNumber() == pinToDelete) {
                delete outputs[j]; outputs[j]=nullptr; outputPins[j]=nullptr;
                free((void*)outputParams[j]); outputParams[j]=nullptr;
                break;
              }
            }
            break;
          }
          case 3: {
            for (int j = 0; j < potCount; j++) {
              if (potParams[j] && (int)potCfgs[j].pin == pinToDelete) {
                free((void*)potParams[j]); potParams[j]=nullptr;
                potOutLast[j]=NAN; potEMA[j]=NAN; potLastMs[j]=0;
                potNotchCount[j]=0; potLastNotch[j]=-1; potEmaNorm[j]=NAN;
                potEmaRaw[j]=NAN; potNotchHasCenters[j]=false;
                break;
              }
            }
            notchDeleteFromEEPROM((uint8_t)pinToDelete);
            break;
          }
        }

        for (int j = i; j < count - 1; j++) {
          PinConfig next;
          EE_GET(1 + (j + 1) * sizeof(PinConfig), next);
          EE_PUT(1 + j * sizeof(PinConfig), next);
        }

        EE::write(0, count - 1);
        EE::commit();
        eliminado = true;
        modbusRefreshWindow();
        break;
      }
    }

    if (eliminado) {
      enviar(String("🗑️ Configuración eliminada del pin ") + pinToStringForDump((uint8_t)pinToDelete, 3));
      enviar(String("DELETED_PIN ") + pinToStringForDump((uint8_t)pinToDelete, 3));
    } else {
      enviar(String("❌ No se encontró configuración para el pin ") + pinToStringForDump((uint8_t)pinToDelete, 3));
      enviar(String("NOT_FOUND_PIN ") + pinToStringForDump((uint8_t)pinToDelete, 3));
    }

    bloqueado = false;
    return;
  }

  if (cmd == "#BOARD?") {
    enviar(String("BOARD ") + getBoardType());
    return;
  }

  // ===== NOTCH commands =====
  if (modoConfig && cmd.startsWith("NOTCH")) {
    String s = cmd; s.trim();
    int sp1 = s.indexOf(' ');
    if (sp1 < 0) { enviar(F("ERR NOTCH")); bloqueado=false; return; }
    String sub = s.substring(sp1+1); sub.trim();
    int sp2 = sub.indexOf(' ');
    String op = (sp2<0) ? sub : sub.substring(0, sp2);
    op.trim();

    if (op.equalsIgnoreCase("DUMPALL")) {
      for (int i=0;i<potCount;i++){
        const PinConfig &cfg = potCfgs[i];
        String p = pinToStringForDump(cfg.pin, 3);
        enviar("NOTCH PIN " + p + " COUNT " + String(potNotchCount[i]) +
               " HYST " + String(potNotchHystPct[i], 3) +
               " CENTERS " + String(potNotchHasCenters[i] ? "YES" : "NO"));
        if (potNotchCount[i] > 0) {
          String lv = "NOTCH LIST.VALS " + p + " ";
          String lc = "NOTCH LIST.CENT " + p + " ";
          for (uint8_t k=0;k<potNotchCount[i];k++){
            if (k) { lv += ","; lc += ","; }
            lv += String(potNotchVals[i][k], 3);
            lc += String((unsigned)potNotchCenterRaw[i][k]);
          }
          enviar(lv); enviar(lc);
        }
        int idx = getPotIndexByPin(cfg.pin);
        if (idx >= 0) {
          enviar(String("NOTCH PARTIAL ") + p + " " + (potPartial[idx] ? "ON":"OFF"));
          enviar(String("NOTCH SNAPWIN ") + p + " " + String(potSnapWin[idx],3));
        }
      }
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("SAVEALL")) {
      if (notchSaveAllToEEPROM()) enviar(F("OK NOTCH SAVEALL"));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("LOADALL")) {
      notchLoadAllFromEEPROM();
      enviar(F("OK NOTCH LOADALL"));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("PARTIAL")) {
      int sp = sub.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH PARTIAL")); bloqueado=false; return; }
      String rest = sub.substring(sp+1); rest.trim();
      int sp2b = rest.indexOf(' ');
      if (sp2b < 0) { enviar(F("ERR NOTCH PARTIAL")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp2b); pinStr.trim();
      String onoff  = rest.substring(sp2b+1);  onoff.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH PARTIAL pin")); bloqueado=false; return; }

      bool on = onoff.equalsIgnoreCase("ON") || onoff == "1" || onoff.equalsIgnoreCase("TRUE");
      potPartial[idx] = on;
      enviar(String("OK NOTCH PARTIAL ") + pinToStringForDump((uint8_t)pin,3) + " " + (on? "ON":"OFF"));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("SNAPWIN")) {
      int sp = sub.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH SNAPWIN")); bloqueado=false; return; }
      String rest = sub.substring(sp+1); rest.trim();
      int sp2b = rest.indexOf(' ');
      if (sp2b < 0) { enviar(F("ERR NOTCH SNAPWIN")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp2b); pinStr.trim();
      String pctStr = rest.substring(sp2b+1);  pctStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH SNAPWIN pin")); bloqueado=false; return; }

      float pct = pctStr.toFloat();
      if (pct < 0.0f) pct = 0.0f; if (pct > 0.3f) pct = 0.3f;
      potSnapWin[idx] = pct;
      enviar(String("OK NOTCH SNAPWIN ") + pinToStringForDump((uint8_t)pin,3) + " " + String(pct,3));
      bloqueado=false; return;
    }

    if (sp2 < 0) { enviar(F("ERR NOTCH params")); bloqueado=false; return; }
    String rest = sub.substring(sp2+1); rest.trim();

    if (op.equalsIgnoreCase("CLEAR")) {
      int pin = analogPinFromString(rest.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH CLEAR pin")); bloqueado=false; return; }
      potNotchCount[idx]=0; potLastNotch[idx]=-1; potEmaNorm[idx]=NAN; potEmaRaw[idx]=NAN; potNotchHasCenters[idx]=false;
      enviar("OK NOTCH CLEAR " + pinToStringForDump((uint8_t)pin,3));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("ADD")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH ADD")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp); pinStr.trim();
      String valStr = rest.substring(sp+1); valStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH ADD pin")); bloqueado=false; return; }
      if (potNotchCount[idx] >= MAX_NOTCHES) { enviar(F("ERR NOTCH ADD max 30")); bloqueado=false; return; }
      float v = valStr.toFloat();
      potNotchVals[idx][ potNotchCount[idx]++ ] = v;
      enviar("OK NOTCH ADD " + pinToStringForDump((uint8_t)pin,3) + " " + String(v,3));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("ADDHERE")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH ADDHERE")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp); pinStr.trim();
      String valStr = rest.substring(sp+1);  valStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH ADDHERE pin")); bloqueado=false; return; }
      if (potNotchCount[idx] >= MAX_NOTCHES) { enviar(F("ERR NOTCH ADDHERE max 30")); bloqueado=false; return; }

      int raw = 0;
      if (isADSIndex(potCfgs[idx].pin)) {
        if (!g_ads_ok) { enviar(F("ERR NOTCH ADDHERE ADS no listo")); bloqueado=false; return; }
        uint8_t ch = adsChannel(potCfgs[idx].pin);
        int16_t r = g_ads.readADC_SingleEnded(ch);
        if (r < 0) r = 0;
        raw = (int)r;
      } else {
        raw = analogRead(potCfgs[idx].pin);
      }

      uint8_t k = potNotchCount[idx]++;
      potNotchCenterRaw[idx][k] = (uint16_t)raw;
      potNotchVals[idx][k]      = valStr.toFloat();
      potNotchHasCenters[idx]   = true;

      enviar("OK NOTCH ADDHERE " + pinToStringForDump((uint8_t)pin,3) +
             " RAW " + String(raw) + " VAL " + String(potNotchVals[idx][k],3));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("CAP")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH CAP")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp); pinStr.trim();
      String idxStr = rest.substring(sp+1);  idxStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) { enviar(F("ERR NOTCH CAP pin")); bloqueado=false; return; }

      int k = idxStr.toInt();
      if (k < 0 || k >= potNotchCount[poti]) { enviar(F("ERR NOTCH CAP idx")); bloqueado=false; return; }

      int raw = 0;
      if (isADSIndex(potCfgs[poti].pin)) {
        if (!g_ads_ok) { enviar(F("ERR NOTCH CAP ADS no listo")); bloqueado=false; return; }
        uint8_t ch = adsChannel(potCfgs[poti].pin);
        int16_t r = g_ads.readADC_SingleEnded(ch);
        if (r < 0) r = 0;
        raw = (int)r;
      } else {
        raw = analogRead(potCfgs[poti].pin);
      }

      potNotchCenterRaw[poti][k] = (uint16_t)raw;
      potNotchHasCenters[poti]   = true;

      enviar("OK NOTCH CAP " + pinToStringForDump((uint8_t)pin,3) +
             " IDX " + String(k) + " RAW " + String(raw));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("VAL")) {
      int spA = rest.indexOf(' ');
      int spB = rest.indexOf(' ', spA+1);
      if (spA < 0 || spB < 0) { enviar(F("ERR NOTCH VAL")); bloqueado=false; return; }

      String pinStr = rest.substring(0, spA);       pinStr.trim();
      String idxStr = rest.substring(spA+1, spB);   idxStr.trim();
      String valStr = rest.substring(spB+1);        valStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) { enviar(F("ERR NOTCH VAL pin")); bloqueado=false; return; }

      int k = idxStr.toInt();
      if (k < 0 || k >= potNotchCount[poti]) { enviar(F("ERR NOTCH VAL idx")); bloqueado=false; return; }

      potNotchVals[poti][k] = valStr.toFloat();
      enviar("OK NOTCH VAL " + pinToStringForDump((uint8_t)pin,3) +
             " IDX " + String(k) + " VAL " + String(potNotchVals[poti][k],3));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("HYST")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH HYST")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp); pinStr.trim();
      String valStr = rest.substring(sp+1); valStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH HYST pin")); bloqueado=false; return; }
      float pct = valStr.toFloat();
      if (pct < 0) pct=0; if (pct>0.3f) pct=0.3f;
      potNotchHystPct[idx] = pct;
      enviar("OK NOTCH HYST " + pinToStringForDump((uint8_t)pin,3) + " " + String(pct,3));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("DUMP")) {
      String pinStr = rest;
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH DUMP pin")); bloqueado=false; return; }
      enviar("NOTCH PIN " + pinToStringForDump((uint8_t)pin,3) + " COUNT " + String(potNotchCount[idx]) +
             " HYST " + String(potNotchHystPct[idx],3) +
             " CENTERS " + String(potNotchHasCenters[idx] ? "YES" : "NO"));
      if (potNotchCount[idx] > 0) {
        String lv = "NOTCH LIST.VALS " + pinToStringForDump((uint8_t)pin,3) + " ";
        String lc = "NOTCH LIST.CENT " + pinToStringForDump((uint8_t)pin,3) + " ";
        for (uint8_t k=0;k<potNotchCount[idx];k++){
          if (k) { lv += ","; lc += ","; }
          lv += String(potNotchVals[idx][k],3);
          lc += String((unsigned)potNotchCenterRaw[idx][k]);
        }
        enviar(lv);
        enviar(lc);
      }
      enviar(String("NOTCH PARTIAL ") + pinToStringForDump((uint8_t)pin,3) + " " + (potPartial[idx] ? "ON":"OFF"));
      enviar(String("NOTCH SNAPWIN ") + pinToStringForDump((uint8_t)pin,3) + " " + String(potSnapWin[idx],3));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("SAVE")) {
      int pin = analogPinFromString(rest.c_str());
      (void)pin;
      if (notchSaveOneToEEPROM((uint8_t)pin)) enviar("OK NOTCH SAVE " + pinToStringForDump((uint8_t)pin,3));
      else enviar(F("ERR NOTCH SAVE"));
      bloqueado=false; return;
    }

    if (op.equalsIgnoreCase("DEL")) {
      int pin = analogPinFromString(rest.c_str());
      if (notchDeleteFromEEPROM((uint8_t)pin)) enviar("OK NOTCH DEL " + pinToStringForDump((uint8_t)pin,3));
      else enviar(F("ERR NOTCH DEL"));
      bloqueado=false; return;
    }

    enviar(F("ERR NOTCH ?"));
    bloqueado=false; return;
  }

  if (cmd == "#DUMP") {
    tcpSuspendidoPorSerial = true;

    enviar(F("✅ DUMP COMPLETO INICIADO"));
    enviar(String("BOARD ") + getBoardType());
    enviar(F("BEGIN CONFIG"));

    uint8_t count = EE::read(0);
    for (int i = 0; i < count; i++) {
      PinConfig cfg;
      EE_GET(1 + i * sizeof(PinConfig), cfg);

      String linea = "ADD ";
      linea += (cfg.type == 0 ? "SWITCH " :
                cfg.type == 1 ? "BUTTON " :
                cfg.type == 2 ? "OUTPUT " : "POT ");

      linea += pinToStringForDump(cfg.pin, cfg.type);
      linea += " ";
      linea += String(cfg.param) + " " +
               String(cfg.minOut, 3) + " " + String(cfg.maxOut, 3);
      enviar(linea);

      if (cfg.type == 3) {
        String scale = "CFG " + pinToStringForDump(cfg.pin, cfg.type) + " SCALE " +
                      String(cfg.minIn) + " " + String(cfg.maxIn) + " " +
                      String(cfg.minOut, 3) + " " + String(cfg.maxOut, 3);
        String format = String("CFG ") + pinToStringForDump(cfg.pin, cfg.type) + " FORMAT " + (cfg.enviarComoEntero ? "INT" : "FLOAT");
        String smooth = String("CFG ") + pinToStringForDump(cfg.pin, cfg.type) + " SMOOTH " + String(cfg.suavizado, 3);
        String modo = String("CFG ") + pinToStringForDump(cfg.pin, cfg.type) + " MODE ";
        switch (cfg.modoEnvio) {
          case 0: modo += "CONTINUO"; break;
          case 1: modo += "CAMBIO"; break;
          case 2: modo += "INTERVALO " + String(cfg.intervalo); break;
          case 3: modo += "MANUAL"; break;
          default: modo += "CONTINUO";
        }
        enviar(scale); enviar(format); enviar(smooth); enviar(modo);

        int idx = getPotIndexByPin(cfg.pin);
        if (idx >= 0) {
          String thr = "CFG " + pinToStringForDump(cfg.pin, cfg.type) + " THRESH " + String(potThreshold[idx], 4);
          enviar(thr);

          if (potNotchCount[idx] > 0) {
            enviar("NOTCH PIN " + pinToStringForDump(cfg.pin, cfg.type) +
                   " COUNT " + String(potNotchCount[idx]) +
                   " HYST " + String(potNotchHystPct[idx],3) +
                   " CENTERS " + String(potNotchHasCenters[idx] ? "YES" : "NO"));
            String lv = "NOTCH LIST.VALS " + pinToStringForDump(cfg.pin, cfg.type) + " ";
            String lc = "NOTCH LIST.CENT " + pinToStringForDump(cfg.pin, cfg.type) + " ";
            for (uint8_t k=0;k<potNotchCount[idx];k++){
              if (k) { lv += ","; lc += ","; }
              lv += String(potNotchVals[idx][k],3);
              lc += String((unsigned)potNotchCenterRaw[idx][k]);
            }
            enviar(lv); enviar(lc);
          }

          enviar(String("NOTCH PARTIAL ") + pinToStringForDump(cfg.pin, cfg.type) + " " + (potPartial[idx] ? "ON":"OFF"));
          enviar(String("NOTCH SNAPWIN ") + pinToStringForDump(cfg.pin, cfg.type) + " " + String(potSnapWin[idx],3));
        }
      }
    }

    enviar(F("#END"));

    String resumen = String("{\"board\":\"") + getBoardType() + "\"," +
                    "\"switches\":" + switchCount +
                    ",\"buttons\":" + buttonCount +
                    ",\"outputs\":" + outputCount +
                    ",\"pots\":" + potCount + "}";
    enviar(resumen);

    enviar("MB.WINDOW base/size ya informada");

    enviar("BEGIN MB");
    handleMbCommand("MB.DUMP");
    enviar("END MB");

    enviar(F("✅ DUMP COMPLETO FIN"));

    delay(300);
    tcpSuspendidoPorSerial = false;
    bloqueado = false;
    return;
  }

  if (modoConfig && cmd.startsWith("#SCANPINS")) {
    int chipsI2C = busMCP.getDetectedChips();
    int totalPins = chipsI2C * 16;
    enviar(String("🔎 MCP detectados: ") + chipsI2C);
    enviar(String("🔌 Pines totales disponibles: ") + totalPins);
    bloqueado = false;
    return;
  }

  if (modoConfig && cmd.startsWith("ADD")) {
    char tipo[20], pinStr[16], param[20], v1[10], v2[10];
    int args = sscanf(cmd.c_str() + 4, "%19s %15s %19s %9s %9s", tipo, pinStr, param, v1, v2);
    if (args == 5) {
      int pin = analogPinFromString(pinStr);
      if (strcasecmp(tipo, "OUTPUT") == 0) {
        if (outputCount < EEPROM_MAX_ENTRIES) {
          IOPin* pinObj = busMCP.getPin(pin, false);
          outputs[outputCount] = new OutputManagerMCP(param, pinObj);
          outputs[outputCount]->begin();
          outputParams[outputCount] = strdup(param);
          outputCount++;
        }
        savePinConfig(tipo, pin, param, v1, v2);
      } else if (strcasecmp(tipo, "SWITCH") == 0) {
        if (switchCount < EEPROM_MAX_ENTRIES) {
          IOPin* pinObj = busMCP.getPin(pin, true);
          switches[switchCount] = new SwitchMCP(pinObj, param, v1, v2);
          switches[switchCount]->begin();
          switchParams[switchCount] = strdup(param);
          switchCount++;
        }
        savePinConfig(tipo, pin, param, v1, v2);
      } else if (strcasecmp(tipo, "BUTTON") == 0) {
        if (buttonCount < EEPROM_MAX_ENTRIES) {
          IOPin* pinObj = busMCP.getPin(pin, true);
          buttons[buttonCount] = new PushButtonMCP(pinObj, param, v1, v2);
          buttons[buttonCount]->begin();
          buttonParams[buttonCount] = strdup(param);
          buttonCount++;
        }
        savePinConfig(tipo, pin, param, v1, v2);
      } else if (strcasecmp(tipo, "POT") == 0) {
        savePinConfig(tipo, pin, param, v1, v2);
        potCount++; // contabilidad lógica
      } else {
        enviar(String("❌ ERROR: Tipo desconocido en ADD → ") + tipo);
      }
    } else {
      enviar(String("❌ ERROR: Formato ADD inválido → ") + cmd);
    }
    bloqueado = false;
    return;
  }

  if (modoConfig && cmd.startsWith("CFG")) {
    updatePotParam(cmd);
    bloqueado = false;
    return;
  }

  // 3) Entrada genérica: soportar tanto command+value como "clave=valor"
  {
    String key, valEq;

    if (value && *value) {
      key   = cmd;
      valEq = val;
    } else {
      int eqPos = cmd.indexOf('=');
      if (eqPos >= 0) {
        key   = cmd.substring(0, eqPos);
        valEq = cmd.substring(eqPos + 1);
      }
    }

    if (key.length()) {
      key.trim(); valEq.trim();

      String mb = buildModbusSetFromKV(key, valEq);
      if (mb.length()) {
        handleMbCommand(mb);
        return;
      }

      bool handledKV = false;
      for (int i = 0; i < outputCount; i++) {
        if (outputs[i] && outputParams[i] &&
            strcasecmp(outputParams[i], key.c_str()) == 0) {
          handledKV = outputs[i]->outputDigital(key.c_str(), valEq.c_str(), 1);
          break;
        }
      }
      if (handledKV) enviar("✅ ACK: " + key + " = " + valEq);
      else           enviar("❌ NACK: " + key + " = " + valEq);
      return;
    }
  }

  bool handled = false;
  for (int i = 0; i < outputCount; i++) {
    if (outputs[i]->outputDigital(command, value, 1)) handled = true;
  }
  if (handled) enviar("✅ ACK: " + String(command) + " = " + (value?String(value):""));
  else         enviar("❌ NACK: " + String(command) + " = " + (value?String(value):""));

  bloqueado = false;
}

// ========= AP / LED =========
void enableAP() {
  if (!apActive) {
    WiFi.softAP("EASYSIMBigBoard");
    Serial.println("🌐 AP activado: 192.168.4.1");
    apActive = true;
    setLed(0,255,0);
#if defined(MODO_ETHERNET)
    if (ethernetInterface) ethernetInterface->setUseEth(true);
#endif
  }
}

void disableAP() {
  if (apActive) {
    WiFi.softAPdisconnect(true);
    Serial.println("📴 AP desactivado");
    apActive = false;
    setLed(0,0,0);
  }
}

void handleAPButton() {
  static bool armed = false;
  static unsigned long stableSince = 0;
  static int lastStable = HIGH;
  const unsigned long DEBOUNCE_MS = 50;
  const unsigned long LONG_MS = 2000;

  int raw = digitalRead(APButton);

  if (raw != lastStable) {
    stableSince = millis();
    lastStable = raw;
  }
  if (millis() - stableSince < DEBOUNCE_MS) return;

  if (lastStable == LOW) {
    if (!armed) {
      armed = true;
      buttonPressStart = millis();
    } else if (millis() - buttonPressStart >= LONG_MS) {
      if (!apActive) enableAP(); else disableAP();
      armed = false;
      buttonPressStart = 0;
      while (digitalRead(APButton) == LOW) { delay(1); }
      stableSince = millis();
      lastStable = HIGH;
    }
  } else {
    armed = false;
    buttonPressStart = 0;
  }
}

// ======================= setup/loop =======================
void setup() {
#if defined(MODO_SERIAL)
  Serial.begin(SERIAL_BAUD);
#endif

  EE::begin(16384);
  EE_ensureCountByte();

  for (int i=0;i<EEPROM_MAX_ENTRIES;i++){
    potEMA[i]=NAN; potOutLast[i]=NAN; potLastMs[i]=0; potThreshold[i]=0.5f;
    potAccDelta[i]=0.0f; potLastEmaSample[i]=NAN;
    potNotchCount[i]=0; potNotchHystPct[i]=0.05f; potLastNotch[i] = -1; potEmaNorm[i] = NAN;
    potNotchHasCenters[i] = false; potEmaRaw[i] = NAN;
    potPartial[i] = false; potSnapWin[i] = 0.03f;
  }

  // === SPI/I2C ===
  SPIBUS.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  pinMode(CS_W5500, OUTPUT);
  digitalWrite(CS_W5500, HIGH);
  Wire.begin(PIN_SDA, PIN_SCL);

  // === ADS1115 ===
  g_ads_ok = g_ads.begin(0x48);
  if (g_ads_ok) {
    g_ads.setGain(GAIN_ONE); // ±4.096V
    Serial.println(F("✅ ADS1115 OK en 0x48"));
  } else {
    Serial.println(F("⚠️ ADS1115 no detectado (0x48)."));
  }

  pinMode(APButton, INPUT_PULLUP);
  pixels.begin();
  pixels.setBrightness(30);
  setLed(0,0,0);

#if defined(MODO_ETHERNET) && defined(ESP32)
  WiFi.mode(WIFI_OFF);
  delay(50);

  Network.onEvent(onEthEvent);

  if (!ETH.begin(ETH_PHY_W5500, 1, CS_W5500, W5500_IRQ, W5500_RST, SPIBUS)) {
    Serial.println("❌ ETH.begin() falló");
  } else {
    Serial.println("✅ ETH.begin() ok, esperando IP...");
  }

  // Espera corta a DHCP y fallback a IP estática si no hay servidor
  {
    ethStaticFallbackUsed = false;
    unsigned long t0 = millis();
    while (ETH.localIP() == IPAddress(0,0,0,0) && (millis() - t0) < 5000) {
      delay(100);
    }
    if (ETH.localIP() == IPAddress(0,0,0,0)) {
      ETH.config(IPAddress(192,168,0,177),
                IPAddress(192,168,0,1),
                IPAddress(255,255,255,0));
      ethStaticFallbackUsed = true;
      Serial.println("ℹ️ No DHCP → IP fija 192.168.0.177/24 (GW 192.168.0.1)");
    } else {
      Serial.print("🌐 DHCP OK → IP ");
      Serial.println(ETH.localIP().toString());
    }
  }

  confServer = new NetworkServer(5000);
  ethernetInterface = new EthernetInterface(
      *confServer,
      [](const char* c, const char* v){ handleLine(c,v); }
  );
  ethernetInterface->setUseEth(true);
  ethernetInterface->begin();

  // === ServerDiscovery: escucha beacons UDP 5091 + cliente TCP 5090 ===
  serverDiscovery.begin();

  // Callbacks del discover/cliente
  serverDiscovery.onBeacon([](const IPAddress& ip, uint16_t port, const String& msg){
    enviar(String("📡 DISCOVERY: beacon desde ") + ip.toString() + ":" + String(port) + " → " + msg);
  });
  serverDiscovery.onConnected([](const IPAddress& ip, uint16_t port){
    enviar(String("✅ CLIENT.CONNECT OK → ") + ip.toString() + ":" + String(port));
  });
  serverDiscovery.onDisconnected([](){
    enviar("⚠️ CLIENT.DISCONNECTED");
  });
  
  serverDiscovery.onLine([](const String& line){
    // Mensajes que vienen del servidor 5090
    Serial.print(F("[Server] LINE: "));
    Serial.println(line);

    // 1) Manejar handshakes / estados especiales del servidor
    if (line.startsWith("connected=")) {
      // Ejemplo: solo loguear y NO pasarlo a handleLine (para que no intente MB.SET)
      enviar(String("📡 SERVER STATUS: ") + line);
      return;
    }

    // 2) El resto de líneas se tratan como comandos normales
    handleLine(line.c_str(), nullptr);
  });
  
  serverDiscovery.onError([](const String& err){
    enviar(String("❌ CLIENT.ERROR ") + err);
  });
#endif

  // Ventana Modbus
  modbusInitWindowFromUsed();
  initModbusTCP();

  // Autoload Modbus
  {
    uint16_t nDev = 0, nTag = 0;
    uint32_t crc  = 0;
    if (mbLoadFromEEPROM(nDev, nTag, crc)) {
      enviar("✅ MB.AUTOLOAD dev=" + String(nDev) +
            " tag=" + String(nTag) +
            " crc=" + String((unsigned long)crc, 16));
    } else {
      enviar("ℹ️ MB.AUTOLOAD: no hay paquete válido (aún). Usa MB.SAVE tras configurar.");
    }
  }

  // MCP por I2C (ajusta direcciones)
  if (!busMCP.beginI2C({0x20, 0x21}, &Wire)) {
    Serial.println("❌ Error al iniciar el bus MCP");
  }

#if defined(MODO_SERIAL)
  serialInterface.begin();
#endif

  // Cargar configuración y NOTCH
  loadConfigFromEEPROM();
  notchLoadAllFromEEPROM();

  // Inicializa IO
  for (int i = 0; i < switchCount; i++) switches[i]->begin();
  for (int i = 0; i < buttonCount; i++) buttons[i]->begin();
  for (int i = 0; i < outputCount; i++) {
    outputs[i]->begin();
    enviar(String("register(") + outputParams[i] + ")");
  }
}

void loop() {
  if (ledDirty && millis() - lastLedFlush > 50) {
    pixels.show();
    lastLedFlush = millis();
    ledDirty = false;
  }

#if defined(MODO_ETHERNET)
  if (ethernetInterface) ethernetInterface->update();
  if (eth_connected) {
    // ServerDiscovery se encarga de leer UDP 5091 y mantener el cliente TCP 5090
    serverDiscovery.poll();
  }
#endif

#if defined(MODO_SERIAL)
  serialInterface.update();
#endif

  tickModbus();

  // Log “esperando beacon” cada 3s (si aún no sabemos IP del server)
  static unsigned long lastWaitLog = 0;
  if (eth_connected && !serverDiscovery.hasServerIp() && millis() - lastWaitLog > 3000) {
    enviar("📡 Esperando beacon UDP en 5091…");
    lastWaitLog = millis();
  }

  if (!modoConfig && !bloqueado) {
    for (int i = 0; i < switchCount; i++) switches[i]->update();
    for (int i = 0; i < buttonCount; i++) buttons[i]->update();
    tickPots();
  }
}
