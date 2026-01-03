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
//#include <Menu.h>
//#include <ElegantOTA.h>      // NUEVO: ElegantOTA para actualizaciones vía web
#include <math.h>               // isnan, fabs, lroundf
#include <Adafruit_ADS1X15.h>   // ADS1115
#include <WiFiUdp.h>            // UDP para discover (usado dentro de ServerDiscovery normalmente)
#include "EthernetInterface.h"  // interfaz TCP puerto 5000
#include <ServerDiscovery.h>    // NUEVO: discover+cliente 5090 con callbacks
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
//#include <Fonts/FreeSerif9pt7b.h>
//#include <Adafruit_SH110X.h>
#include <U8g2lib.h>


// ---------------- Wrappers para la API nueva de EE ----------------
template<typename T>
inline void EE_GET(uint32_t addr, T &dst) { EE::get(addr, &dst, sizeof(T)); }

template<typename T>
inline void EE_PUT(uint32_t addr, const T &src) { EE::put(addr, &src, sizeof(T)); }

// Asegura que el byte contador (offset 0) esté inicializado a 0 (no 0xFF)
inline void EE_ensureCountByte() {
  uint8_t c = EE::read(0);
  if (c == 0xFF) {
    EE::write(0, 0);
    EE::commit();
  }
}

// ========================= Variables ========================
bool bloqueaTCP = false;

// ===================== LÍMITES/CONSTANTES EE =================
static constexpr uint32_t EE_SIZE_BYTES = 16384; // total NVS simulada por EE

// ======== Región NOTCH persistente (4 KB) ========
static constexpr uint32_t NOTCH_REGION_SIZE = 4096;
static constexpr uint32_t NOTCH_REGION_BASE = EE_SIZE_BYTES - NOTCH_REGION_SIZE;
static constexpr uint32_t NOTCH_MAGIC = 0x4E544348; // 'N''T''C''H'
static constexpr uint8_t  NOTCH_VERSION = 3;        // V3: partial+snapwin

// ======== Ventana Modbus (8 KB) al final ========
static constexpr uint32_t MB_REGION_SIZE = 8192; // 8 KB
static constexpr uint32_t MB_REGION_BASE = EE_SIZE_BYTES - MB_REGION_SIZE;

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
#define PIN_SCK  11
#define PIN_MISO 13
#define PIN_MOSI 12

// I2C
#define PIN_SDA 9
#define PIN_SCL 8
// I2C Bus Oled
#define OLED_SDA 21
#define OLED_SCL 14

TwoWire I2CBUS1 = TwoWire(1); // para OLED

// W5500
#define CS_W5500   10
#define W5500_IRQ  5
#define W5500_RST  4

#ifndef BOARD_TYPE
#define BOARD_TYPE "BigBoardSep"
#endif

static inline const char* getBoardType() { return BOARD_TYPE; }

// ======================= Canales / Interfaces =======================
#define MODO_ETHERNET
#define MODO_SERIAL
#define MODO_WIFI_AP      // ✅ comandos también por WiFi cuando está en AP

#if defined(MODO_ETHERNET)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
NetworkServer*      confServer        = nullptr; // puerto 5000 (config)
EthernetInterface*  ethernetInterface = nullptr;
#endif

#if defined(MODO_WIFI_AP)
WiFiServer wifiServer(5000); // servidor TCP en el AP, puerto 5000
WiFiClient wifiClient;
bool wifiServerStarted = false;
#endif

enum CanalActivo { CANAL_SERIAL, CANAL_TCP };
CanalActivo canalActivo = CANAL_SERIAL;

bool tcpSuspendidoPorSerial = false; // ya no se usa para bloquear, pero se deja declarado por compatibilidad

// ✅ Control de salida TCP y modo IP
bool ethOutEnabled         = true;  // ON por defecto
bool ethStaticFallbackUsed = false; // TRUE si usamos IP estática de emergencia

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

// ======================= Estructura de configuración de pin =======================

struct __attribute__((packed)) PinConfig {
  uint8_t  pin;
  uint8_t  type;            // 0 SWITCH, 1 BUTTON, 2 OUTPUT, 3 POT, 4 SELECTOR
  char     param[40];
  int      minIn, maxIn;
  float    minOut, maxOut;
  float    suavizado;
  uint8_t  modoEnvio;       // 0 CONTINUO, 1 CAMBIO, 2 INTERVALO, 3 MANUAL
  uint16_t intervalo;       // para INTERVALO o fixed-point THRESH en CAMBIO
  bool     enviarComoEntero;
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

uint8_t switchPinNum[EEPROM_MAX_ENTRIES];
uint8_t buttonPinNum[EEPROM_MAX_ENTRIES];
uint8_t outputPinNum[EEPROM_MAX_ENTRIES];

int switchCount = 0, buttonCount = 0, potCount = 0, outputCount = 0;

// ======================= AP/LED (MOVIDO ARRIBA para oledDrawStatus) =======================
// ===== Botones ESP32 =====
#define PIN_BTN_OLED  0   // cambia pantalla
#define PIN_BTN_AP    15  // AP (long press)

// páginas OLED
enum OledPage : uint8_t { OLED_PAGE_NET = 0, OLED_PAGE_IO = 1 };
static OledPage oledPage = OLED_PAGE_NET;

// debounce
static int lastOledBtn = HIGH;
static int lastApBtn   = HIGH;
static unsigned long oledBtnStableMs = 0;
static unsigned long apBtnStableMs   = 0;
static unsigned long apPressStart    = 0;

bool apActive = false;
unsigned long buttonPressStart = 0;
const unsigned long longPressTime = 2000;

// ======================= Cliente TCP a servidor principal (5090) (MOVIDO ARRIBA para oledDrawStatus) =======================
static const uint16_t kClientServerPort = 5090;
ServerDiscovery serverDiscovery(5091,5090); // la clase hace discover UDP+TCP 5090

// ======================= OLED SSD1306 =======================
#define OLED_ADDR   0x78
#define OLED_W      128
#define OLED_H      64
#define OLED_RESET  -1   // sin pin reset

//Adafruit_SSD1306 display(OLED_W, OLED_H, &I2CBUS1, OLED_RESET);
U8G2_SSD1309_128X64_NONAME2_F_SW_I2C display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ U8X8_PIN_NONE);
//Adafruit_SH1106G display(128, 64, &Wire, -1);

static bool oledOK = false;
static unsigned long lastOledMs = 0;
static String oledLast;
static unsigned long lastOledDraw = 0;
static const unsigned long OLED_MIN_MS = 300; // 300..1000 típico
static volatile bool oledPending = true;   // true al arranque para dibujar 1ª vez
static String oledLastFrame;
static unsigned long lastPotsMs = 0;
static const unsigned long POTS_PERIOD_MS = 20;

static inline void oledRequest() {
  oledPending = true;
}

static void pollButtonsESP32() {
  const unsigned long now = millis();
  const unsigned long DEBOUNCE_MS = 50;

  // Ventana de doble click (ajusta a gusto)
  const unsigned long DC_WINDOW_MS = 350;

  // ---- BTN OLED (BOOT): 1 click cambia pantalla, doble click toggle AP ----
  {
    static int stable = HIGH;
    static unsigned long lastChangeMs = 0;

    static uint8_t clickCount = 0;
    static unsigned long firstClickMs = 0;

    int raw = digitalRead(PIN_BTN_OLED); // BOOT normalmente GPIO0 (LOW=pressed)

    // Debounce básico
    if (raw != stable && (now - lastChangeMs) >= DEBOUNCE_MS) {
      stable = raw;
      lastChangeMs = now;

      if (stable == LOW) {
        // Se ha pulsado (flanco)
        if (clickCount == 0) {
          clickCount = 1;
          firstClickMs = now;
        } else {
          // Segundo click dentro de ventana => doble click
          if ((now - firstClickMs) <= DC_WINDOW_MS) {
            clickCount = 0;

            // ✅ DOBLE CLICK: toggle AP
            if (!apActive) enableAP();
            else           disableAP();

            oledRequest();
          } else {
            // Demasiado tarde: esto cuenta como “nuevo primer click”
            clickCount = 1;
            firstClickMs = now;
          }
        }
      }
    }

    // Si ha pasado la ventana sin segundo click => es single click
    if (clickCount == 1 && (now - firstClickMs) > DC_WINDOW_MS) {
      clickCount = 0;

      // ✅ SINGLE CLICK: cambia página
      oledPage = (oledPage == OLED_PAGE_NET) ? OLED_PAGE_IO : OLED_PAGE_NET;
      oledRequest();
    }
  }

  // ---- BTN AP: si lo vas a dejar, quítalo o ignóralo (porque ahora AP va por doble click) ----
  // Si quieres, puedes comentar/eliminar el bloque de PIN_BTN_AP para evitar confusión.
}

enum IoKind : uint8_t { IO_SW=0, IO_BTN=1, IO_OUT=2 };

struct IoItem {
  IoKind kind;
  uint8_t pin;          // pin global 0..127
  const char* name;     // param (texto)
  IOPin* io;            // puntero pin
};

// Pagina interna de IO (si hay muchos)
static uint8_t oledIoSubPage = 0;

static bool ioIsActive(const IoItem& it) {
  if (!it.io) return false;

  // Entradas con pullup: activo cuando LOW
  if (it.kind == IO_SW || it.kind == IO_BTN) {
    return it.io->digitalRead() == LOW;
  }

  // Salidas: si tu MCPPin soporta leer el estado real, esto sirve.
  // Si no, te digo abajo cómo hacerlo con “shadow”.
  return it.io->digitalRead() == HIGH;
}

static int buildIoItems(IoItem* out, int maxItems) {
  int n = 0;

  for (int i=0; i<switchCount && n<maxItems; ++i) {
    out[n++] = { IO_SW, switchPinNum[i], switchParams[i], switchPins[i] };
  }
  for (int i=0; i<buttonCount && n<maxItems; ++i) {
    out[n++] = { IO_BTN, buttonPinNum[i], buttonParams[i], buttonPins[i] };
  }
  for (int i=0; i<outputCount && n<maxItems; ++i) {
    out[n++] = { IO_OUT, outputPinNum[i], outputParams[i], outputPins[i] };
  }

  return n;
}

static void oledDrawIOPage() {
  if (!oledOK) return;

  static IoItem items[256];
  const int total = buildIoItems(items, 256);

  // 16 por página (4x4)
  const int PER_PAGE = 16;
  int pages = (total + PER_PAGE - 1) / PER_PAGE;
  if (pages < 1) pages = 1;

  if (oledIoSubPage >= (uint8_t)pages) oledIoSubPage = 0;

  int start = oledIoSubPage * PER_PAGE;
  int end   = start + PER_PAGE;
  if (end > total) end = total;

  display.clearBuffer();
  display.setFont(u8g2_font_5x8_tf);

  // Encabezado
  char hdr[32];
  snprintf(hdr, sizeof(hdr), "IO %d/%d  (tot:%d)", (int)oledIoSubPage+1, pages, total);
  display.drawStr(0, 7, hdr);

  // Grid: 4 columnas x 4 filas
  const int COLS = 4;
  const int ROWS = 4;
  const int cellW = 128 / COLS;   // 32
  const int cellH = 14;           // 14px por fila (cabe justo)
  const int y0 = 12;

  int idx = start;
  for (int r=0; r<ROWS; r++) {
    for (int c=0; c<COLS; c++) {
      if (idx >= end) break;

      const IoItem& it = items[idx];
      bool on = ioIsActive(it);

      int x = c * cellW;
      int y = y0 + r * cellH;

      // círculo en (x+4, y+5)
      int cx = x + 6;
      int cy = y + 6;
      int rad = 4;

      display.drawCircle(cx, cy, rad);
      if (on) display.drawDisc(cx, cy, rad-1);

      // etiqueta tipo + pin (ej: S12, B07, O45)
      char tag[10];
      char k = (it.kind==IO_SW) ? 'S' : (it.kind==IO_BTN) ? 'B' : 'O';

      // si no tienes pin global aquí, pon el índice o el param corto
      uint8_t p = it.pin;
      if (p == 255) snprintf(tag, sizeof(tag), "%c??", k);
      else          snprintf(tag, sizeof(tag), "%c%u", k, (unsigned)p);

      display.drawStr(x + 14, y + 8, tag);

      idx++;
    }
  }
  display.sendBuffer();
}

static String buildOledFrame() {
  bool linkUp = ETH.linkUp();
  IPAddress ip = ETH.localIP();
  bool hasIp = (ip != IPAddress((uint32_t)0));

  const char* mode;
  if (!linkUp) mode = "NO LINK";
  else if (!hasIp)  mode = "NO IP";
  else              mode = (ethStaticFallbackUsed ? "STATIC" : "DHCP");

  bool cliUp = serverDiscovery.connected();
  bool hasSrv = serverDiscovery.hasServerIp();
  IPAddress srvIp = serverDiscovery.serverIp();

  String s;
  s.reserve(200);
  s += "BOARD:"; s += getBoardType(); s += "\n";
  s += "ETH:";   s += mode;          s += "\n";
  s += "IP:";    s += (hasIp ? ip.toString() : "0.0.0.0"); s += "\n";
  s += "CLI5090:"; s += (cliUp ? "UP" : "DOWN"); s += "\n";
  s += "SRV:";     s += (hasSrv ? srvIp.toString() : "0.0.0.0"); s += "\n";
  s += "AP:";      s += (apActive ? WiFi.softAPIP().toString() : "OFF");
  return s;
}

static const unsigned long OLED_KEEPALIVE_MS = 1500; // 1.5s

static void scanBus(TwoWire &bus, const char* name) {
  Serial.printf("\n--- I2C scan en %s ---\n", name);

  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission(true);

    if (err == 0) {
      Serial.printf("✅ Found: 0x%02X\n", addr);
      found++;
    } else if (err == 4) {
      Serial.printf("⚠️  Unknown error at 0x%02X\n", addr);
    }
    delay(2);
  }
  if (!found) Serial.println("❌ No se encontraron dispositivos en este bus.");
}

/*static void oledMaybeDrawChangeOnly() {
  if (!oledOK) return;

  const unsigned long now = millis();
  if (now - lastOledDraw < OLED_MIN_MS) return;

  String frame = buildOledFrame();
  const bool changed   = (frame != oledLastFrame);
  const bool keepalive = (now - lastOledDraw) >= OLED_KEEPALIVE_MS;

  if (!changed && !keepalive) return;

  oledLastFrame = frame;
  lastOledDraw  = now;

  // ---- DIBUJO ----
  display.clearBuffer ();
  display.setCursor(0,0);

  int start = 0;
  int y = 0;

  while (start < frame.length()) {
    int nl = frame.indexOf('\n', start);
    String line = (nl >= 0)
                    ? frame.substring(start, nl)
                    : frame.substring(start);

    display.setCursor(0, y);
    display.print(line);

    y += 8;                 // 8px por línea (font 5x7)
    if (y > 56) break;      // evita salir del display

    if (nl < 0) break;
    start = nl + 1;
  }

  display.display();
}*/

static void oledDrawNetPage_U8G2(const String& frame) {
  display.clearBuffer();
  display.setFont(u8g2_font_5x8_tf);

  int y = 8;
  int start = 0;
  while (start < (int)frame.length()) {
    int nl = frame.indexOf('\n', start);
    String line = (nl >= 0) ? frame.substring(start, nl) : frame.substring(start);
    display.drawStr(0, y, line.c_str());
    y += 9;
    if (y > 63) break;
    if (nl < 0) break;
    start = nl + 1;
  }

  display.sendBuffer();
}

static void oledMaybeDrawChangeOnly() {
  if (!oledOK) return;

  const unsigned long now = millis();
  if (now - lastOledDraw < OLED_MIN_MS) return;

  if (oledPage == OLED_PAGE_IO) {
    oledDrawIOPage();           // <-- tu página de círculos
    lastOledDraw = now;
    return;
  }

  String frame = buildOledFrame();
  const bool changed   = (frame != oledLastFrame);
  const bool keepalive = (now - lastOledDraw) >= OLED_KEEPALIVE_MS;
  if (!changed && !keepalive) return;

  oledLastFrame = frame;
  lastOledDraw  = now;

  oledDrawNetPage_U8G2(frame);
}


// 💡 Convención POT-ADS: pin = 128 + canal (0..3)
static inline bool isADSIndex(uint8_t pin) { return pin >= 128 && pin <= 131; }
static inline uint8_t adsChannel(uint8_t pin){ return (uint8_t)(pin - 128); }
// ===== I2C Mutex ya lo tienes =====
// g_i2cMutex, i2cTryLock(), i2cUnlock()

// ===== ADS cache =====
static unsigned long lastAdsReadMs = 0;
static const unsigned long ADS_READ_PERIOD_MS = 30; // 20..50ms típico

static int  adsRawCache[4] = {0,0,0,0};
static bool adsCacheOk[4]  = {false,false,false,false};

extern Adafruit_ADS1115 g_ads;
extern bool g_ads_ok;

void adsPollCache() {
  if (!g_ads_ok) return;

  unsigned long now = millis();
  if (now - lastAdsReadMs < ADS_READ_PERIOD_MS) return;
  lastAdsReadMs = now;

  for (uint8_t ch = 0; ch < 4; ch++) {
    int16_t r = g_ads.readADC_SingleEnded(ch);
    if (r >= 0) {
      adsRawCache[ch] = r;
      adsCacheOk[ch] = true;
    }
  }
}

// ======================= SELECTORS (como POT: múltiples filas agrupadas) =======================
// ControllerMCP.h ya trae SelectorMCP; aquí solo mantenemos el runtime.
static const int MAX_SELECTORS = 24;
SelectorMCP* selectors[MAX_SELECTORS];
const char*  selectorNames[MAX_SELECTORS];
int selectorCount = 0;

static int findSelectorByName(const char* name) {
  for (int i=0;i<selectorCount;i++){
    if (selectorNames[i] && strcasecmp(selectorNames[i], name)==0) return i;
  }
  return -1;
}

static int getOrCreateSelector(const char* name) {
  int idx = findSelectorByName(name);
  if (idx >= 0) return idx;
  if (selectorCount >= MAX_SELECTORS) return -1;

  selectorNames[selectorCount] = strdup(name);
  selectors[selectorCount] = new SelectorMCP(selectorNames[selectorCount]);
  selectors[selectorCount]->setDebounceMs(20);
  selectors[selectorCount]->begin();
  return selectorCount++;
}

static void clearSelectorsRAM() {
  for (int i=0;i<selectorCount;i++){
    if (selectors[i]) { delete selectors[i]; selectors[i]=nullptr; }
    if (selectorNames[i]) { free((void*)selectorNames[i]); selectorNames[i]=nullptr; }
  }
  selectorCount = 0;
}

bool modoConfig = false;
bool bloqueado  = false;

// ========== NeoPixel ==========
Adafruit_NeoPixel pixels(1, 48, NEO_GRB + NEO_KHZ800);

// Flush LED solo cuando cambie
static bool ledDirty = false;
static unsigned long lastLedFlush = 0;

static inline void setLed(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  ledDirty = true;
}

// ============ Helpers ============
PinConfig potCfgs[EEPROM_MAX_ENTRIES];
float potOutLast[EEPROM_MAX_ENTRIES];
unsigned long potLastMs[EEPROM_MAX_ENTRIES];
static float potEMA[EEPROM_MAX_ENTRIES];
static float potThreshold[EEPROM_MAX_ENTRIES];      // umbral cambio
static float potAccDelta[EEPROM_MAX_ENTRIES];
static float potLastEmaSample[EEPROM_MAX_ENTRIES];

// ======== MUESCAS (1..30) ========
static const uint8_t MAX_NOTCHES = 30;
static uint8_t potNotchCount[EEPROM_MAX_ENTRIES];
static float   potNotchVals [EEPROM_MAX_ENTRIES][MAX_NOTCHES];
static float   potNotchHystPct[EEPROM_MAX_ENTRIES];
static int     potLastNotch[EEPROM_MAX_ENTRIES];
static float   potEmaNorm [EEPROM_MAX_ENTRIES];

// ======== Centros crudos por muesca (V2+) ========
static uint16_t potNotchCenterRaw[EEPROM_MAX_ENTRIES][MAX_NOTCHES];
static bool     potNotchHasCenters[EEPROM_MAX_ENTRIES];
static float    potEmaRaw[EEPROM_MAX_ENTRIES];

// --- Híbrido NOTCH+Lineal ---
static bool  potPartial[EEPROM_MAX_ENTRIES]; // OFF por defecto
static float potSnapWin[EEPROM_MAX_ENTRIES]; // fracción del rango crudo

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

// ======== Salida unificada (Serial + servidor 5000 + cliente 5090 + WiFi AP) ========
void enviar(const String& msg) {
#if defined(MODO_SERIAL)
  Serial.println(msg);
#endif

#if defined(MODO_ETHERNET)
  if (ethernetInterface && ethOutEnabled && !tcpSuspendidoPorSerial) {
    ethernetInterface->println(msg); // a clientes en puerto 5000
  }
  if (serverDiscovery.connected()) {
    serverDiscovery.println(msg);    // espejo al servidor 5090 si está conectado
  }
#endif

#if defined(MODO_WIFI_AP)
  if (wifiClient && wifiClient.connected()) {
    wifiClient.println(msg);         // espejo al cliente WiFi del AP
  }
#endif
}

static inline bool isSpaceC(char c){ return c==' ' || c=='\t' || c=='\r' || c=='\n'; }

static void trimInPlace(String &s){
  int i=0;
  while(i<(int)s.length() && isSpaceC(s[i])) i++;
  int j=s.length()-1;
  while(j>=0 && isSpaceC(s[j])) j--;
  if(j<i) { s=""; return; }
  s = s.substring(i, j+1);
}

static bool parseBoolLike(const String& v, int &out){
  String t=v;
  t.toUpperCase();
  trimInPlace(t);
  if (t=="1"||t=="ON"||t=="TRUE"||t=="HIGH") { out=1; return true; }
  if (t=="0"||t=="OFF"||t=="FALSE"||t=="LOW") { out=0; return true; }

  bool allNum = t.length()>0;
  for (int i=0;i<t.length();++i)
    allNum &= isdigit((unsigned char)t[i]) || (i==0 && (t[i]=='-'||t[i]=='+'));

  if (allNum) {
    out = t.toInt();
    return true;
  }
  return false;
}

static String buildModbusSetFromKV(const String& key, const String& valueCSV){
  String out = "MB.SET ";
  out += key;
  out += " ";
  int start = 0;
  while (start < valueCSV.length()) {
    int comma = valueCSV.indexOf(',', start);
    String token = (comma>=0) ? valueCSV.substring(start, comma) : valueCSV.substring(start);
    trimInPlace(token);
    int v;
    if (!parseBoolLike(token, v)) return String();
    out += String(v);
    if (comma>=0) {
      out += " ";
      start = comma+1;
    } else break;
  }
  return out;
}

// ===== Helpers numéricos =====
static inline float clamp01(float x){
  if (x<0) return 0;
  if (x>1) return 1;
  return x;
}

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
  int i      = lastIdx;
  float lowB  = (i == 0) ? 0.0f : (i * cell);
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
    int   best  = 0;
    float bestD = fabsf(rawNow - centers[0]);
    for (uint8_t i = 1; i < N; ++i) {
      float d = fabsf(rawNow - centers[i]);
      if (d < bestD) {
        bestD = d;
        best  = i;
      }
    }
    return best;
  }

  int   i  = lastIdx;
  float ci = centers[i];
  float cim1 = (i > 0)           ? centers[i-1] : centers[0];
  float cip1 = (i < (int)N-1)    ? centers[i+1] : centers[N-1];

  float midDown = (i > 0) ?
                  0.5f * (cim1 + ci) :
                  (ci - (cip1-ci));
  float midUp   = (i < (int)N-1) ?
                  0.5f * (ci + cip1) :
                  (ci + (ci-cim1));

  float cellDown = (i > 0)           ? (ci - cim1) : (cip1 - ci);
  float cellUp   = (i < (int)N-1)    ? (cip1 - ci) : (ci - cim1);
  if (cellDown < 0) cellDown = 0;
  if (cellUp   < 0) cellUp   = 0;

  float hDown = hystPct * cellDown;
  float hUp   = hystPct * cellUp;

  if (i < (int)N-1 && rawNow > (midUp + hUp))     return i + 1;
  if (i > 0        && rawNow < (midDown - hDown)) return i - 1;
  return i;
}

// ======================= SEL helpers (EEPROM type=4) =======================
static bool eepromDeleteAllByType(uint8_t typeToDelete) {
  int count = (int)EE::read(0);
  if (count <= 0) return true;

  int writeIdx = 0;
  for (int readIdx = 0; readIdx < count; readIdx++) {
    PinConfig cfg;
    EE_GET(1 + readIdx * sizeof(PinConfig), cfg);

    if (cfg.type == typeToDelete) {
      // skip (delete)
      continue;
    }
    if (writeIdx != readIdx) {
      EE_PUT(1 + writeIdx * sizeof(PinConfig), cfg);
    }
    writeIdx++;
  }

  if (writeIdx != count) {
    EE::write(0, (uint8_t)writeIdx);
    EE::commit();
  }
  return true;
}

static bool eepromDeletePinIfType(uint8_t pin, uint8_t typeMatch) {
  int count = (int)EE::read(0);
  if (count <= 0) return false;

  for (int i = 0; i < count; i++) {
    PinConfig cfg;
    EE_GET(1 + i * sizeof(PinConfig), cfg);
    if (cfg.pin == pin && cfg.type == typeMatch) {
      // compactar
      for (int j = i; j < count - 1; j++) {
        PinConfig next;
        EE_GET(1 + (j + 1) * sizeof(PinConfig), next);
        EE_PUT(1 + j * sizeof(PinConfig), next);
      }
      EE::write(0, (uint8_t)(count - 1));
      EE::commit();
      return true;
    }
  }
  return false;
}

static void dumpSelectorsAsSEL_ADD() {
  uint8_t count = EE::read(0);
  for (int i = 0; i < count; i++) {
    PinConfig cfg;
    EE_GET(1 + i * sizeof(PinConfig), cfg);
    if (cfg.type != 4) continue;

    // SEL.ADD <name> <pin> <val>
    String name = String(cfg.param);
    int16_t v = (int16_t)lroundf(cfg.minOut);
    enviar(String("SEL.ADD ") + name + " " + String((int)cfg.pin) + " " + String((int)v));
  }
}

// ======================= Carga desde EE =======================
void loadConfigFromEEPROM() {
  // reset RAM containers
  clearSelectorsRAM();
  switchCount = buttonCount = potCount = outputCount = 0;

  uint8_t count = EE::read(0);
  if (count == 0xFF) {
    EE::write(0, 0);
    EE::commit();
    count = 0;
  }

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
      if (!isprint((unsigned char)cfg.param[j]) && cfg.param[j] != '\0') {
        cfg.param[j] = '\0';
        break;
      }
    }
    cfg.param[sizeof(cfg.param) - 1] = '\0';

    // En SELECTOR no “dependemos” de copy como persistencia, pero normalizamos igual
    char* copy = strdup(cfg.param);
    if (!copy || strlen(copy) == 0) {
      enviar(String("⚠️ Entrada con parámetro inválido ignorada en EEPROM entrada ") + i);
      if (copy) free(copy);
      continue;
    }

    IOPin* pin = nullptr;
    if (cfg.pin <= 127) {
      // OUTPUT => salida, resto entrada
      pin = busMCP.getPin(cfg.pin, cfg.type != 2);
    }

    switch (cfg.type) {
      case 0: { // SWITCH
        if (!pin) { free(copy); break; }
        char val1[12], val2[12];
        dtostrf(cfg.minOut, 1, 3, val1);
        dtostrf(cfg.maxOut, 1, 3, val2);
        switchPinNum[switchCount] = cfg.pin;
        switchParams[switchCount] = copy;
        switchPins[switchCount]   = pin;
        switches[switchCount]     = new SwitchMCP(pin, strdup(copy), strdup(val1), strdup(val2));
        switches[switchCount]->begin();
        switchCount++;
        break;
      }

      case 1: { // BUTTON
        if (!pin) { free(copy); break; }
        char val1[12], val2[12];
        dtostrf(cfg.minOut, 1, 3, val1);
        dtostrf(cfg.maxOut, 1, 3, val2);
        buttonPinNum[buttonCount] = cfg.pin;
        buttonParams[buttonCount] = copy;
        buttonPins[buttonCount]   = pin;
        buttons[buttonCount]      = new PushButtonMCP(pin, strdup(copy), strdup(val1), strdup(val2));
        buttons[buttonCount]->begin();
        buttonCount++;
        break;
      }

      case 2: { // OUTPUT
        if (!pin) { free(copy); break; }
        outputPinNum[outputCount] = cfg.pin;
        outputParams[outputCount] = copy;
        outputPins[outputCount]   = pin;
        outputs[outputCount]      = new OutputManagerMCP(copy, pin);
        outputs[outputCount]->begin();
        // ✅ Por seguridad: todo LOW al arrancar
        pin->digitalWrite(LOW);
        outputCount++;
        break;
      }

      case 3: { // POT
        potParams[potCount]       = copy;
        potCfgs[potCount]         = cfg;
        potOutLast[potCount]      = NAN;
        potLastMs[potCount]       = 0;
        potEMA[potCount]          = NAN;

        // ==== THRESH persistente ====
        float thr = 0.5f;
        if (cfg.modoEnvio == 1) {          // 1 = CAMBIO
          thr = (float)cfg.intervalo / 10000.0f;
          if (thr <= 0.0f) thr = 0.01f;
          if (thr > 1.0f)  thr = 1.0f;
        }
        potThreshold[potCount]    = thr;
        // ============================

        potAccDelta[potCount]     = 0.0f;
        potLastEmaSample[potCount]= NAN;
        potNotchCount[potCount]   = 0;
        potNotchHystPct[potCount] = 0.05f;
        potLastNotch[potCount]    = -1;
        potEmaNorm[potCount]      = NAN;
        potNotchHasCenters[potCount] = false;
        potEmaRaw[potCount]       = NAN;
        potPartial[potCount]      = false;
        potSnapWin[potCount]      = 0.03f;
        potCount++;
        break;
      }

      case 4: { // SELECTOR (como POT: varias filas agrupadas por nombre)
        if (!pin) { free(copy); break; }

        // cfg.param = nombre selector
        // cfg.minOut = valor asociado a este pin
        int sidx = getOrCreateSelector(copy);
        if (sidx < 0) {
          enviar(F("❌ SELECTOR: sin espacio en selectors[]"));
          free(copy);
          break;
        }

        pin->pinMode(INPUT_PULLUP);
        int16_t v = (int16_t)lroundf(cfg.minOut);
        selectors[sidx]->add(pin, v);

        // ya duplicamos nombre internamente, liberamos copy
        free(copy);
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

  for (int i = 0; i < switchCount; i++) {
    delete switches[i];
    switches[i] = nullptr;
  }
  for (int i = 0; i < buttonCount; i++) {
    delete buttons[i];
    buttons[i] = nullptr;
  }
  for (int i = 0; i < outputCount; i++) {
    delete outputs[i];
    outputs[i] = nullptr;
  }
  switchCount = buttonCount = potCount = outputCount = 0;

  clearSelectorsRAM();

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
  if (tipo.equalsIgnoreCase("SWITCH"))        cfg.type = 0;
  else if (tipo.equalsIgnoreCase("BUTTON"))  cfg.type = 1;
  else if (tipo.equalsIgnoreCase("OUTPUT"))  cfg.type = 2;
  else if (tipo.equalsIgnoreCase("POT"))     cfg.type = 3;
  else if (tipo.equalsIgnoreCase("SELECTOR")) cfg.type = 4;
  else return;

  strncpy(cfg.param, param, sizeof(cfg.param));
  cfg.param[sizeof(cfg.param) - 1] = '\0';
  cfg.pin   = (uint8_t)pin;

  // En SELECTOR: minOut = valor asociado a este pin
  cfg.minOut = atof(v1);
  cfg.maxOut = atof(v2);

  if (cfg.type == 3) {
    if (isADSIndex(cfg.pin)) {
      cfg.minIn = 0;
      cfg.maxIn = 32767;
    } else {
      cfg.minIn = 0;
      #if defined(ESP32)
        cfg.maxIn = 4095;
      #else
        cfg.maxIn = 1023;
      #endif
    }
    cfg.suavizado        = 0.10f;
    cfg.modoEnvio        = 0;    // CONTINUO
    cfg.intervalo        = 200;
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

  EE_PUT(off, NOTCH_MAGIC);
  off += sizeof(uint32_t);

  EE::write(off, NOTCH_VERSION);
  off += 1;

  EE::write(off, 0);
  EE::write(off+1, 0);
  EE::write(off+2, 0);
  off += 3;

  uint16_t numRecs = 0;
  EE_PUT(off, numRecs);
  off += sizeof(uint16_t);

  for (int i = 0; i < potCount; i++) {
    uint8_t cnt = potNotchCount[i];
    if (cnt == 0) continue;
    uint8_t pin = potCfgs[i].pin;
    bool hasC   = potNotchHasCenters[i];

    size_t recSize = 1 + 1 + sizeof(float) + 1 + 1 +
                    (cnt*sizeof(uint16_t)) + (cnt*sizeof(float)) +
                     sizeof(float); // snapWin
    if (off + recSize > NOTCH_REGION_BASE + NOTCH_REGION_SIZE) {
      enviar(F("❌ NOTCH SAVEALL: sin espacio."));
      return false;
    }

    EE::write(off, pin); off += 1;
    EE::write(off, cnt); off += 1;
    EE_PUT(off, potNotchHystPct[i]);
    off += sizeof(float);

    uint8_t flags = 0;
    if (hasC)          flags |= 0x01;
    if (potPartial[i]) flags |= 0x02;
    EE::write(off, flags); off += 1;
    EE::write(off, 0);     off += 1; // reservado

    for (uint8_t k=0;k<cnt;k++){
      uint16_t c = hasC ? potNotchCenterRaw[i][k] : 0;
      EE_PUT(off, c);
      off += sizeof(uint16_t);
    }

    for (uint8_t k=0;k<cnt;k++){
      EE_PUT(off, potNotchVals[i][k]);
      off += sizeof(float);
    }

    EE_PUT(off, potSnapWin[i]);
    off += sizeof(float);

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
  EE_GET(off, magic);
  off += sizeof(uint32_t);

  if (magic != NOTCH_MAGIC) {
    enviar(F("ℹ️ NOTCH LOADALL: no hay tabla."));
    return;
  }

  uint8_t ver = EE::read(off);
  off += 1;

  off += 3;

  uint16_t numRecs = 0;
  EE_GET(off, numRecs);
  off += sizeof(uint16_t);

  const uint32_t regionEnd = NOTCH_REGION_BASE + NOTCH_REGION_SIZE;

  // Reset completo en RAM
  for (int i = 0; i < potCount; i++) {
    potNotchCount[i]      = 0;
    potLastNotch[i]       = -1;
    potEmaNorm[i]         = NAN;
    potEmaRaw[i]          = NAN;
    potNotchHasCenters[i] = false;
    potPartial[i]         = false;
    potSnapWin[i]         = 0.03f;
  }

  for (uint16_t r = 0; r < numRecs; r++) {
    if (off + 1 + 1 + sizeof(float) > regionEnd) break;

    uint8_t pin = EE::read(off); off += 1;
    uint8_t cnt = EE::read(off); off += 1;

    float hyst = 0.0f;
    EE_GET(off, hyst);
    off += sizeof(float);

    bool    hasC    = false;
    bool    partial = false;
    float   snap    = 0.03f;

    uint32_t centersPos = 0;
    uint32_t valsPos    = 0;

    if (ver >= 2) {
      uint8_t flags = EE::read(off); off += 1;
      off += 1;

      hasC    = (flags & 0x01) != 0;
      partial = (flags & 0x02) != 0;

      centersPos = off;
      off += cnt * sizeof(uint16_t);

      valsPos = off;
      off += cnt * sizeof(float);

      if (ver >= 3) {
        if (off + sizeof(float) > regionEnd) break;
        EE_GET(off, snap);
        off += sizeof(float);
      }
    } else {
      centersPos = 0;
      valsPos    = off;
      off += cnt * sizeof(float);
      hasC       = false;
      partial    = false;
      snap       = 0.03f;
    }

    int idx = getPotIndexByPin(pin);
    if (idx >= 0 && cnt > 0) {
      if (cnt > MAX_NOTCHES) cnt = MAX_NOTCHES;

      potNotchCount[idx]      = cnt;
      potNotchHystPct[idx]    = hyst;
      potNotchHasCenters[idx] = (ver >= 2) ? hasC : false;
      potPartial[idx]         = (ver >= 3) ? partial : false;
      potSnapWin[idx]         = (ver >= 3) ? snap : 0.03f;

      if (ver >= 2) {
        for (uint8_t k = 0; k < cnt; k++) {
          uint16_t c = 0;
          EE_GET(centersPos + k * sizeof(uint16_t), c);
          potNotchCenterRaw[idx][k] = c;
        }
      }

      for (uint8_t k = 0; k < cnt; k++) {
        float v = 0.0f;
        EE_GET(valsPos + k * sizeof(float), v);
        potNotchVals[idx][k] = v;
      }

      potLastNotch[idx] = -1;
      potEmaNorm[idx]   = NAN;
      potEmaRaw[idx]    = NAN;
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
    potNotchCount[idx]      = 0;
    potLastNotch[idx]       = -1;
    potEmaNorm[idx]         = NAN;
    potEmaRaw[idx]          = NAN;
    potNotchHasCenters[idx] = false;
    potPartial[idx]         = false;
    potSnapWin[idx]         = 0.03f;
  }
  return notchSaveAllToEEPROM();
}

// ==================== updatePotParam ====================
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
        cfg.minIn  = atoi(a);
        cfg.maxIn  = atoi(b);
        cfg.minOut = atof(c);
        cfg.maxOut = atof(d);
      } else if (f == "SMOOTH" && args >= 3) {
        cfg.suavizado = atof(a);
      } else if (f == "MODE" && args >= 3) {
        if      (String(a) == "CONTINUO")  cfg.modoEnvio = 0;
        else if (String(a) == "CAMBIO")    cfg.modoEnvio = 1;
        else if (String(a) == "INTERVALO") {
          cfg.modoEnvio = 2;
          cfg.intervalo = atoi(b);
        } else if (String(a) == "MANUAL")  cfg.modoEnvio = 3;
      } else if (f == "FORMAT" && args >= 3) {
        cfg.enviarComoEntero = (String(a) == "INT");
      }
      // ---- THRESH / HYST: ajustan potThreshold[idx] ----
      else if (f == "THRESH" && args >= 3) {
        float th = atof(a);
        if (th < 0.0f) th = 0.0f;
        if (th > 1.0f) th = 1.0f;

        int idx = getPotIndexByPin(cfg.pin);
        if (idx >= 0) {
          potThreshold[idx] = th;
        } else {
          enviar(F("THRESH: pot aún no inicializado en RAM (pero se guardará en EEPROM)."));
        }

        if (cfg.modoEnvio == 1) {
          uint16_t fp = (uint16_t)(th * 10000.0f + 0.5f);
          cfg.intervalo = fp;
        }

        EE_PUT(1 + i * sizeof(PinConfig), cfg);
        EE::commit();

        enviar(String("THRESH OK en ") + pinToStringForDump(cfg.pin, cfg.type) +
              " = " + String(th, 4));
        return;
      }
      else {
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
  if (count == 0xFF) {
    count = 0;
    EE::write(0, 0);
    EE::commit();
  }
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
            event == ARDUINO_EVENT_ETH_LOST_IP      ||
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
// (SIN CAMBIOS: tu tickPots completo)
void tickPots() {
  const unsigned long now = millis();

  for (int i = 0; i < potCount; i++) {
    const PinConfig &cfg = potCfgs[i];
    int raw = 0;

    // =======================
    // 1) LECTURA RAW (SIN I2C directo)  ✅
    // =======================
    if (isADSIndex(cfg.pin)) {
      if (!g_ads_ok) continue;
      uint8_t ch = adsChannel(cfg.pin);
      if (ch > 3) continue;
    
      // ✅ leer del cache (ya refrescado en adsPollCache())
      if (!adsCacheOk[ch]) continue;
      raw = adsRawCache[ch];
    
    } else {
      raw = analogRead(cfg.pin);
    }


    // =======================
    // 2) MAP + EMA
    // =======================
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

    // =======================
    // 3) MODOENVIO
    // =======================
    switch (cfg.modoEnvio) {
      case 0: // CONTINUO
        debeEnviar = true;
        break;

      case 1: { // CAMBIO (+ NOTCH/HÍBRIDO universal)
        const bool hasNotches = (potNotchCount[i] > 0);

        if (hasNotches) {
          // ====== con muescas (con o sin PARTIAL) ======
          float rawNow = (float)raw;

          // Filtrado crudo para usar con centros / decisions
          if (isnan(potEmaRaw[i])) potEmaRaw[i] = rawNow;
          else                     potEmaRaw[i] = (1.0f - cfg.suavizado) * potEmaRaw[i] + cfg.suavizado * rawNow;

          // --- 1) Índice de notch según histéresis ---
          int currIdx = -1;
          float rawForIdx = potEmaRaw[i];

          if (potNotchHasCenters[i]) {
            currIdx = pickNotchByCenters(
                        potLastNotch[i],
                        rawForIdx,
                        potNotchCenterRaw[i],
                        potNotchCount[i],
                        potNotchHystPct[i]
                      );
          } else {
            float norm = clamp01(
              (float)(raw - potCfgs[i].minIn) /
              (float)(potCfgs[i].maxIn - potCfgs[i].minIn)
            );
            if (isnan(potEmaNorm[i])) potEmaNorm[i] = norm;
            else                      potEmaNorm[i] = (1.0f - cfg.suavizado) * potEmaNorm[i] + cfg.suavizado * norm;

            currIdx = pickNotchWithHyst(
                        potLastNotch[i],
                        potEmaNorm[i],
                        potNotchCount[i],
                        potNotchHystPct[i]
                      );
          }

          if (currIdx < 0) currIdx = 0;
          if (currIdx >= potNotchCount[i]) currIdx = potNotchCount[i] - 1;

          // --- 2) SNAPWIN: ¿estamos cerca de alguna muesca? ---
          bool  near    = false;
          int   nearIdx = currIdx;
          float snapAbs = potSnapWin[i] * (float)(potCfgs[i].maxIn - potCfgs[i].minIn);
          if (snapAbs < 0) snapAbs = 0;

          if (potNotchHasCenters[i]) {
            float bestD = 1e9f;
            int   bestI = -1;
            for (uint8_t k = 0; k < potNotchCount[i]; k++) {
              float d = fabsf(rawForIdx - (float)potNotchCenterRaw[i][k]);
              if (d < bestD) { bestD = d; bestI = (int)k; }
            }
            if (bestI >= 0) {
              near    = (bestD <= snapAbs);
              nearIdx = bestI;
            }
          } else {
            float minR = (float)potCfgs[i].minIn;
            float maxR = (float)potCfgs[i].maxIn;
            float cell = (potNotchCount[i] > 0) ?
                         (maxR - minR) / (float)potNotchCount[i] :
                         (maxR - minR);
            float center = minR + (nearIdx + 0.5f) * cell;
            near = (fabsf(rawForIdx - center) <= snapAbs);
          }

          // --- 3) Índices mínimo y máximo de muesca ---
          uint8_t minIdx = 0, maxIdx = potNotchCount[i] - 1;
          if (potNotchHasCenters[i]) {
            uint16_t minC = potNotchCenterRaw[i][0];
            uint16_t maxC = potNotchCenterRaw[i][0];
            for (uint8_t k = 1; k < potNotchCount[i]; k++) {
              uint16_t c = potNotchCenterRaw[i][k];
              if (c < minC) { minC = c; minIdx = k; }
              if (c > maxC) { maxC = c; maxIdx = k; }
            }
          }

          // --- 4) Valor híbrido (colas lineales + muescas internas) ---
          float outHybrid   = 0.0f;
          bool  useLinear   = false;
          float minInF      = (float)cfg.minIn;
          float maxInF      = (float)cfg.maxIn;
          float minCenterF  = potNotchHasCenters[i] ? (float)potNotchCenterRaw[i][minIdx] : minInF;
          float maxCenterF  = potNotchHasCenters[i] ? (float)potNotchCenterRaw[i][maxIdx] : maxInF;
          float valMinNotch = potNotchVals[i][minIdx];
          float valMaxNotch = potNotchVals[i][maxIdx];

          if (potPartial[i]) {
            if (rawForIdx < minCenterF) {
              outHybrid = mapf(rawForIdx, minInF, minCenterF,
                               cfg.minOut, valMinNotch);
              useLinear = true;
            } else if (rawForIdx > maxCenterF) {
              outHybrid = mapf(rawForIdx, maxCenterF, maxInF,
                               valMaxNotch, cfg.maxOut);
              useLinear = true;
            }
          }

          if (!useLinear) {
            int idxFinal = currIdx;
            if (near && nearIdx >= 0 && nearIdx < potNotchCount[i]) {
              idxFinal = nearIdx;
            }

            if (potLastNotch[i] < 0) {
              potLastNotch[i] = idxFinal;
            } else if (idxFinal != potLastNotch[i]) {
              potLastNotch[i] = idxFinal;
            }

            outHybrid = potNotchVals[i][potLastNotch[i]];
          }

          // --- 5) THRESH y envío ---
          float prev = potOutLast[i];
          float thr  = potThreshold[i];
          bool enviarAhora = false;

          if (isnan(prev)) {
            enviarAhora = true;
          } else if (fabsf(outHybrid - prev) >= thr) {
            enviarAhora = true;
          }

          if (enviarAhora) {
            if (cfg.enviarComoEntero) {
              enviar(String(potParams[i]) + "=" + String((int)lroundf(outHybrid)));
            } else {
              char f[24];
              dtostrf(outHybrid, 0, 3, f);
              char* p = f; while (*p==' ') ++p;
              enviar(String(potParams[i]) + "=" + String(p));
            }
            potOutLast[i] = outHybrid;
            potLastMs[i]  = now;
          }

          debeEnviar = false;

        } else {
          bool enviarCambio = false;
          float prev = potOutLast[i];
          float thr  = potThreshold[i];

          if (isnan(prev)) {
            enviarCambio = true;
          } else if (fabs(outVal - prev) >= thr) {
            enviarCambio = true;
          }

          if (enviarCambio) {
            if (cfg.enviarComoEntero) {
              enviar(String(potParams[i]) + "=" + String((int)lroundf(outVal)));
            } else {
              char f[24];
              dtostrf(outVal, 0, 3, f);
              char* p = f; while (*p == ' ') ++p;
              enviar(String(potParams[i]) + "=" + String(p));
            }
            potOutLast[i] = outVal;
            potLastMs[i]  = now;
          }

          debeEnviar = false;
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

    // =======================
    // 4) ENVÍO FINAL
    // =======================
    if (debeEnviar) {
      if (potNotchCount[i] > 0) {
        int idxN = (potLastNotch[i] < 0) ? 0 : potLastNotch[i];
        float notchVal = potNotchVals[i][idxN];

        if (cfg.enviarComoEntero) {
          enviar(String(potParams[i]) + "=" + String((int)lroundf(notchVal)));
        } else {
          char f[24];
          dtostrf(notchVal, 0, 3, f);
          char* p = f; while (*p == ' ') ++p;
          enviar(String(potParams[i]) + "=" + String(p));
        }
        potOutLast[i] = notchVal;

      } else {
        if (cfg.enviarComoEntero) {
          enviar(String(potParams[i]) + "=" + String((int)lroundf(outVal)));
        } else {
          char f[24];
          dtostrf(outVal, 0, 3, f);
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
  if (!command || !*command) return;

#if defined(MODO_ETHERNET)
  bloqueaTCP = true;
#endif

  String cmd = String(command);
  String val = value ? String(value) : "";

  cmd.trim();
  val.trim();

  // ===================== MODBUS passthrough =====================
  if (cmd.startsWith("MB.") || cmd.startsWith("MODBUS.")) {
    handleMbCommand(cmd);
    return;
  }

  // ===================== ETH helpers =====================
#if defined(MODO_ETHERNET)
  if (cmd.equalsIgnoreCase("ETH.STATUS")) {
    IPAddress ip = ETH.localIP();
    bool linkUp  = ETH.linkUp();
    bool hasIp   = (ip != IPAddress((uint32_t)0));

    String mode = ethStaticFallbackUsed ? "STATIC" : (hasIp ? "DHCP" : "UNKNOWN");

    IPAddress srvIp = serverDiscovery.serverIp();
    bool hasSrv     = serverDiscovery.hasServerIp();

    String s = "ETH.STATUS OUT=";
    s += (ethOutEnabled ? "ON" : "OFF");
    s += " LINK=";
    s += (linkUp ? "UP" : "DOWN");
    s += " MODE=";
    s += mode;
    s += " IP=";
    s += ip.toString();
    s += " CLIENT=";
    s += (serverDiscovery.connected() ? "UP" : "DOWN");
    s += " SERVER.IP=";
    s += (hasSrv ? srvIp.toString() : "0.0.0.0");
    s += " SERVER.PORT=";
    s += String(5090); // o kClientServerPort si lo prefieres
    enviar(s);
    return;
  }

  if (cmd.startsWith("ETH.OUT")) {
    String arg = val;
    if (!arg.length()) {
      int sp = cmd.indexOf(' ');
      if (sp > 0) arg = cmd.substring(sp + 1);
    }
    arg.trim();
    arg.toUpperCase();
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

  // ===================== DISCOVER / SERVER IP =====================
  if (cmd.startsWith("DISCOVER.SETIP") || cmd.startsWith("ETH.SERVER")) {
    String ipS = val;
    if (!ipS.length()) {
      int sp = cmd.indexOf(' ');
      if (sp > 0) ipS = cmd.substring(sp + 1);
    }
    ipS.trim();
    IPAddress ip;
    if (ip.fromString(ipS)) {
      serverDiscovery.setServerIp(ip);
      enviar(String("OK SERVER.IP ") + ip.toString());
    } else {
      enviar(String("ERR SERVER.IP inválida: ") + ipS);
    }
    return;
  }

  // ===================== Modo CONFIG =====================
  if (cmd.equalsIgnoreCase("#CONFIG")) {
    modoConfig = true;
    bloqueado  = false;
    enviar(F("✅ MODO CONFIG ACTIVADO"));
    return;
  }

  if (cmd.equalsIgnoreCase("#END")) {
    modoConfig = false;
    enviar(F("✅ MODO CONFIG DESACTIVADO"));
    delay(100);
    enviar(F("#READY"));
    delay(300);

    if (!apActive) {
#if defined(__AVR__)
      wdt_enable(WDTO_15MS);
      while (1);
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

  if (cmd.equalsIgnoreCase("#CLEAR")) {
    clearEEPROMIfNeeded();
    notchEraseRegion();
    enviar(F("✅ EEPROM borrada correctamente."));
    delay(100);
    enviar(F("#READY"));
    modbusRefreshWindow();
    return;
  }

  // ✅ FALTABA: NVSWIPE (injertado del otro handleLine)
  if (cmd.equalsIgnoreCase("#NVSWIPE")) {
    nvsWipeAllAndReboot();
    return;
  }

  if (cmd.equalsIgnoreCase("#BOARD?")) {
    enviar(String("BOARD ") + getBoardType());
    return;
  }

  // ===================== SEL.* (nuevo API) =====================
  // Formatos:
  //   SEL.ADD <name> <pin> <val>
  //   SEL.CLEAR
  //   SEL.DELPIN <pin>
  //   SEL.DUMP
  if (cmd.startsWith("SEL.")) {
    if (cmd.equalsIgnoreCase("SEL.CLEAR")) {
      if (!modoConfig) { enviar(F("❌ SEL.CLEAR requiere #CONFIG")); return; }

      eepromDeleteAllByType(4);
      clearSelectorsRAM();
      loadConfigFromEEPROM();
      notchLoadAllFromEEPROM();
      modbusRefreshWindow();

      enviar(F("✅ OK SEL.CLEAR"));
      return;
    }

    if (cmd.equalsIgnoreCase("SEL.DUMP")) {
      enviar(F("BEGIN SEL"));
      dumpSelectorsAsSEL_ADD();
      enviar(F("END SEL"));
      return;
    }

    if (cmd.startsWith("SEL.DELPIN")) {
      if (!modoConfig) { enviar(F("❌ SEL.DELPIN requiere #CONFIG")); return; }
      char pinStr[16];
      if (sscanf(cmd.c_str(), "SEL.DELPIN %15s", pinStr) == 1) {
        int pin = analogPinFromString(pinStr);
        if (pin < 0 || pin > 127) { enviar(F("❌ SEL.DELPIN: pin inválido")); return; }

        bool ok = eepromDeletePinIfType((uint8_t)pin, 4);
        clearSelectorsRAM();
        loadConfigFromEEPROM();
        notchLoadAllFromEEPROM();
        modbusRefreshWindow();

        enviar(ok ? F("✅ OK SEL.DELPIN") : F("❌ SEL.DELPIN: pin no encontrado"));
        return;
      }
      enviar(F("❌ Formato: SEL.DELPIN <pin>"));
      return;
    }

    if (cmd.startsWith("SEL.ADD")) {
      if (!modoConfig) { enviar(F("❌ SEL.ADD requiere #CONFIG")); return; }

      char name[40], pinStr[16], valStr[16];
      int n = sscanf(cmd.c_str(), "SEL.ADD %39s %15s %15s", name, pinStr, valStr);
      if (n == 3) {
        int pin = analogPinFromString(pinStr);
        int v   = atoi(valStr);

        if (pin < 0 || pin > 127) { enviar(F("❌ SEL.ADD: pin inválido (solo MCP 0..127)")); return; }

        int sidx = getOrCreateSelector(name);
        if (sidx < 0) { enviar(F("❌ SEL.ADD: sin espacio en selectors[]")); return; }

        IOPin* pinObj = busMCP.getPin(pin, true);
        if (!pinObj) { enviar(F("❌ SEL.ADD: pinObj nulo")); return; }

        pinObj->pinMode(INPUT_PULLUP);
        selectors[sidx]->add(pinObj, (int16_t)v);

        char v1[16]; snprintf(v1, sizeof(v1), "%d", v);
        savePinConfig("SELECTOR", pin, name, v1, "0");

        enviar(String("✅ OK SEL.ADD ") + name + " PIN=" + String(pin) + " VAL=" + String(v));
        return;
      }

      enviar(F("❌ Formato: SEL.ADD <name> <pin> <val>"));
      return;
    }

    enviar(F("❌ SEL.* desconocido (usa SEL.ADD / SEL.CLEAR / SEL.DELPIN / SEL.DUMP)"));
    return;
  }

  // ===================== #DELETEPIN =====================
  if (modoConfig && cmd.startsWith("#DELETEPIN")) {
    int pinToDelete = analogPinFromString(cmd.c_str() + 10);
    int count = EE::read(0);
    bool eliminado = false;
    PinConfig deletedCfg{};

    for (int i = 0; i < count; i++) {
      PinConfig cfg;
      EE_GET(1 + i * sizeof(PinConfig), cfg);
      if ((int)cfg.pin == pinToDelete) {
        deletedCfg = cfg;

        for (int j = i; j < count - 1; j++) {
          PinConfig next;
          EE_GET(1 + (j + 1) * sizeof(PinConfig), next);
          EE_PUT(1 + j * sizeof(PinConfig), next);
        }
        EE::write(0, count - 1);
        EE::commit();

        eliminado = true;
        break;
      }
    }

    if (eliminado) {
      if (deletedCfg.type == 3) {
        notchDeleteFromEEPROM((uint8_t)pinToDelete);
      }

      enviar(String("🗑️ Configuración eliminada del pin ") +
            pinToStringForDump((uint8_t)pinToDelete, deletedCfg.type));
      enviar(String("DELETED_PIN ") +
            pinToStringForDump((uint8_t)pinToDelete, deletedCfg.type));

      for (int i=0;i<switchCount;i++){
        if (switches[i]) { delete switches[i]; switches[i]=nullptr; }
        if (switchParams[i]) { free((void*)switchParams[i]); switchParams[i]=nullptr; }
      }
      for (int i=0;i<buttonCount;i++){
        if (buttons[i]) { delete buttons[i]; buttons[i]=nullptr; }
        if (buttonParams[i]) { free((void*)buttonParams[i]); buttonParams[i]=nullptr; }
      }
      for (int i=0;i<outputCount;i++){
        if (outputs[i]) { delete outputs[i]; outputs[i]=nullptr; }
        if (outputParams[i]) { free((void*)outputParams[i]); outputParams[i]=nullptr; }
      }
      switchCount = buttonCount = potCount = outputCount = 0;

      clearSelectorsRAM();

      loadConfigFromEEPROM();
      notchLoadAllFromEEPROM();
      modbusRefreshWindow();

      for (int i=0;i<outputCount;i++){
        if (outputPins[i]) outputPins[i]->digitalWrite(LOW);
      }

    } else {
      enviar(String("❌ No se encontró configuración para el pin ") +
             pinToStringForDump((uint8_t)pinToDelete, 3));
      enviar(String("NOT_FOUND_PIN ") +
             pinToStringForDump((uint8_t)pinToDelete, 3));
    }
    bloqueado = false;
    return;
  }

  // ===================== #SCANPINS =====================
  if (modoConfig && cmd.startsWith("#SCANPINS")) {
    int chips = busMCP.getDetectedChips();
    int totalPins = chips * 16;
    enviar(String("🔎 MCP detectados: ") + chips);
    enviar(String("🔌 Pines totales disponibles: ") + totalPins);
    bloqueado = false;
    return;
  }

  // ===================== NOTCH commands =====================
  // ✅ FALTABA: aquí va el bloque NOTCH COMPLETO (injertado del handleLine viejo)
  if (modoConfig && cmd.startsWith("NOTCH")) {
    String s = cmd;
    s.trim();
    int sp1 = s.indexOf(' ');
    if (sp1 < 0) {
      enviar(F("ERR NOTCH"));
      bloqueado=false;
      return;
    }
    String sub = s.substring(sp1+1);
    sub.trim();
    int sp2 = sub.indexOf(' ');
    String op = (sp2<0) ? sub : sub.substring(0, sp2);
    op.trim();

    if (op.equalsIgnoreCase("DUMPALL")) {
      for (int i=0;i<potCount;i++){
        const PinConfig &cfg = potCfgs[i];
        String p = pinToStringForDump(cfg.pin, 3);
        enviar("NOTCH PIN " + p +
                " COUNT " + String(potNotchCount[i]) +
                " HYST " + String(potNotchHystPct[i], 3) +
                " CENTERS " +
                String(potNotchHasCenters[i] ? "YES" : "NO"));
        if (potNotchCount[i] > 0) {
          String lv = "NOTCH LIST.VALS " + p + " ";
          String lc = "NOTCH LIST.CENT " + p + " ";
          for (uint8_t k=0;k<potNotchCount[i];k++){
            if (k) { lv += ","; lc += ","; }
            lv += String(potNotchVals[i][k],3);
            lc += String((unsigned)potNotchCenterRaw[i][k]);
          }
          enviar(lv);
          enviar(lc);
        }
        int idx = getPotIndexByPin(cfg.pin);
        if (idx >= 0) {
          enviar(String("NOTCH PARTIAL ") + p + " " + (potPartial[idx] ? "ON":"OFF"));
          enviar(String("NOTCH SNAPWIN ") + p + " " + String(potSnapWin[idx],3));
        }
      }
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("SAVEALL")) {
      if (notchSaveAllToEEPROM()) enviar(F("OK NOTCH SAVEALL"));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("LOADALL")) {
      notchLoadAllFromEEPROM();
      enviar(F("OK NOTCH LOADALL"));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("PARTIAL")) {
      int sp = sub.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH PARTIAL")); bloqueado=false; return; }
      String rest = sub.substring(sp+1);
      rest.trim();
      int sp2b = rest.indexOf(' ');
      if (sp2b < 0) { enviar(F("ERR NOTCH PARTIAL")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp2b);
      pinStr.trim();
      String onoff = rest.substring(sp2b+1);
      onoff.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH PARTIAL pin")); bloqueado=false; return; }
      bool on = onoff.equalsIgnoreCase("ON") || onoff == "1" || onoff.equalsIgnoreCase("TRUE");
      potPartial[idx] = on;
      enviar(String("OK NOTCH PARTIAL ") +
              pinToStringForDump((uint8_t)pin,3) +
              " " + (on? "ON":"OFF"));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("SNAPWIN")) {
      int sp = sub.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH SNAPWIN")); bloqueado=false; return; }
      String rest = sub.substring(sp+1);
      rest.trim();
      int sp2b = rest.indexOf(' ');
      if (sp2b < 0) { enviar(F("ERR NOTCH SNAPWIN")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp2b);
      pinStr.trim();
      String pctStr = rest.substring(sp2b+1);
      pctStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH SNAPWIN pin")); bloqueado=false; return; }
      float pct = pctStr.toFloat();
      if (pct < 0.0f) pct = 0.0f;
      if (pct > 0.3f) pct = 0.3f;
      potSnapWin[idx] = pct;
      enviar(String("OK NOTCH SNAPWIN ") +
              pinToStringForDump((uint8_t)pin,3) +
              " " + String(pct,3));
      bloqueado=false;
      return;
    }

    if (sp2 < 0) {
      enviar(F("ERR NOTCH params"));
      bloqueado=false;
      return;
    }

    String rest = sub.substring(sp2+1);
    rest.trim();

    if (op.equalsIgnoreCase("RAW")) {
      String pinStr = rest;
      pinStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) {
        enviar(F("ERR NOTCH RAW pin"));
        bloqueado=false;
        return;
      }
      int raw = 0;
      if (isADSIndex(potCfgs[poti].pin)) {
        if (!g_ads_ok) { enviar(F("ERR NOTCH RAW ADS no listo")); bloqueado=false; return; }

        uint8_t ch = adsChannel(potCfgs[poti].pin);

        int16_t r = g_ads.readADC_SingleEnded(ch);

        if (r < 0) r = 0;
        raw = (int)r;
      } else {
        raw = analogRead(potCfgs[poti].pin);
      }

      enviar("OK NOTCH RAW " +
              pinToStringForDump((uint8_t)pin,3) +
              " RAW " + String(raw));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("CLEAR")) {
      int pin = analogPinFromString(rest.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH CLEAR pin")); bloqueado=false; return; }
      potNotchCount[idx]=0;
      potLastNotch[idx]=-1;
      potEmaNorm[idx]=NAN;
      potEmaRaw[idx]=NAN;
      potNotchHasCenters[idx]=false;
      enviar("OK NOTCH CLEAR " + pinToStringForDump((uint8_t)pin,3));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("ADD")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH ADD")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp);
      pinStr.trim();
      String valStr = rest.substring(sp+1);
      valStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH ADD pin")); bloqueado=false; return; }
      if (potNotchCount[idx] >= MAX_NOTCHES) {
        enviar(F("ERR NOTCH ADD max 30"));
        bloqueado=false;
        return;
      }
      float v = valStr.toFloat();
      potNotchVals[idx][ potNotchCount[idx]++ ] = v;
      enviar("OK NOTCH ADD " +
              pinToStringForDump((uint8_t)pin,3) +
              " " + String(v,3));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("ADDHERE")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH ADDHERE")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp);
      pinStr.trim();
      String valStr = rest.substring(sp+1);
      valStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH ADDHERE pin")); bloqueado=false; return; }
      if (potNotchCount[idx] >= MAX_NOTCHES) {
        enviar(F("ERR NOTCH ADDHERE max 30"));
        bloqueado=false;
        return;
      }
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
      enviar("OK NOTCH ADDHERE " +
              pinToStringForDump((uint8_t)pin,3) +
              " RAW " + String(raw) +
              " VAL " + String(potNotchVals[idx][k],3));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("CAP")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH CAP")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp);
      pinStr.trim();
      String idxStr = rest.substring(sp+1);
      idxStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) { enviar(F("ERR NOTCH CAP pin")); bloqueado=false; return; }
      int k = idxStr.toInt();
      if (k < 0 || k >= potNotchCount[poti]) {
        enviar(F("ERR NOTCH CAP idx"));
        bloqueado=false;
        return;
      }
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
      enviar("OK NOTCH CAP " +
              pinToStringForDump((uint8_t)pin,3) +
              " IDX " + String(k) +
              " RAW " + String(raw));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("VAL")) {
      int spA = rest.indexOf(' ');
      int spB = rest.indexOf(' ', spA+1);
      if (spA < 0 || spB < 0) { enviar(F("ERR NOTCH VAL")); bloqueado=false; return; }
      String pinStr = rest.substring(0, spA);
      pinStr.trim();
      String idxStr = rest.substring(spA+1, spB);
      idxStr.trim();
      String valStr = rest.substring(spB+1);
      valStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) { enviar(F("ERR NOTCH VAL pin")); bloqueado=false; return; }
      int k = idxStr.toInt();
      if (k < 0 || k >= potNotchCount[poti]) { enviar(F("ERR NOTCH VAL idx")); bloqueado=false; return; }
      potNotchVals[poti][k] = valStr.toFloat();
      enviar("OK NOTCH VAL " +
              pinToStringForDump((uint8_t)pin,3) +
              " IDX " + String(k) +
              " VAL " + String(potNotchVals[poti][k],3));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("HYST")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH HYST")); bloqueado=false; return; }
      String pinStr = rest.substring(0, sp);
      pinStr.trim();
      String valStr = rest.substring(sp+1);
      valStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH HYST pin")); bloqueado=false; return; }
      float pct = valStr.toFloat();
      if (pct < 0)    pct=0;
      if (pct>0.3f) pct=0.3f;
      potNotchHystPct[idx] = pct;
      enviar("OK NOTCH HYST " +
              pinToStringForDump((uint8_t)pin,3) +
              " " + String(pct,3));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("DUMP")) {
      String pinStr = rest;
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH DUMP pin")); bloqueado=false; return; }
      enviar("NOTCH PIN " +
              pinToStringForDump((uint8_t)pin,3) +
              " COUNT " + String(potNotchCount[idx]) +
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
      enviar(String("NOTCH PARTIAL ") +
              pinToStringForDump((uint8_t)pin,3) +
              " " + (potPartial[idx] ? "ON":"OFF"));
      enviar(String("NOTCH SNAPWIN ") +
              pinToStringForDump((uint8_t)pin,3) +
              " " + String(potSnapWin[idx],3));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("SAVE")) {
      int pin = analogPinFromString(rest.c_str());
      (void)pin;
      if (notchSaveOneToEEPROM((uint8_t)pin))
        enviar("OK NOTCH SAVE " + pinToStringForDump((uint8_t)pin,3));
      else
        enviar(F("ERR NOTCH SAVE"));
      bloqueado=false;
      return;
    }

    if (op.equalsIgnoreCase("DEL")) {
      int pin = analogPinFromString(rest.c_str());
      if (notchDeleteFromEEPROM((uint8_t)pin))
        enviar("OK NOTCH DEL " + pinToStringForDump((uint8_t)pin,3));
      else
        enviar(F("ERR NOTCH DEL"));
      bloqueado=false;
      return;
    }

    enviar(F("ERR NOTCH ?"));
    bloqueado=false;
    return;
  }

  // ===================== #DUMP =====================
  if (cmd.equalsIgnoreCase("#DUMP")) {
    enviar(F("✅ DUMP COMPLETO INICIADO"));
    enviar(String("BOARD ") + getBoardType());
    enviar(F("BEGIN CONFIG"));

    uint8_t count = EE::read(0);
    for (int i = 0; i < count; i++) {
      PinConfig cfg;
      EE_GET(1 + i * sizeof(PinConfig), cfg);

      if (cfg.type == 4) {
        int16_t v = (int16_t)lroundf(cfg.minOut);
        enviar(String("SEL.ADD ") + String(cfg.param) + " " + String((int)cfg.pin) + " " + String((int)v));
        continue;
      }

      String linea = "ADD ";
      linea += (cfg.type == 0 ? "SWITCH "  :
                cfg.type == 1 ? "BUTTON "  :
                cfg.type == 2 ? "OUTPUT "  :
                cfg.type == 3 ? "POT "     : "POT ");

      linea += pinToStringForDump(cfg.pin, cfg.type);
      linea += " ";
      linea +=  String(cfg.param) + " " +
                String(cfg.minOut, 3) + " " +
                String(cfg.maxOut, 3);
      enviar(linea);

      if (cfg.type == 3) {
        String scale = "CFG " + pinToStringForDump(cfg.pin, cfg.type) +
                       " SCALE " + String(cfg.minIn) + " " +
                       String(cfg.maxIn) + " " +
                       String(cfg.minOut, 3) + " " +
                       String(cfg.maxOut, 3);
        String format = String("CFG ") + pinToStringForDump(cfg.pin, cfg.type) +
                        " FORMAT " + (cfg.enviarComoEntero ? "INT" : "FLOAT");
        String smooth = String("CFG ") + pinToStringForDump(cfg.pin, cfg.type) +
                        " SMOOTH " + String(cfg.suavizado, 3);

        String modo = String("CFG ") + pinToStringForDump(cfg.pin, cfg.type) + " MODE ";
        switch (cfg.modoEnvio) {
          case 0: modo += "CONTINUO"; break;
          case 1: modo += "CAMBIO";   break;
          case 2: modo += "INTERVALO " + String(cfg.intervalo); break;
          case 3: modo += "MANUAL";   break;
          default: modo += "CONTINUO";
        }

        enviar(scale);
        enviar(format);
        enviar(smooth);
        enviar(modo);

        int idx = getPotIndexByPin(cfg.pin);
        if (idx >= 0) {
          enviar(String("CFG ") + pinToStringForDump(cfg.pin,cfg.type) +
                 " THRESH " + String(potThreshold[idx], 4));

          if (potNotchCount[idx] > 0) {
            enviar("NOTCH COUNT " + pinToStringForDump(cfg.pin,cfg.type) + " " + String(potNotchCount[idx]));
            enviar("NOTCH HYST "  + pinToStringForDump(cfg.pin,cfg.type) + " " + String(potNotchHystPct[idx],3));

            String lv = "NOTCH LIST.VALS " + pinToStringForDump(cfg.pin,cfg.type) + " ";
            String lc = "NOTCH LIST.CENT " + pinToStringForDump(cfg.pin,cfg.type) + " ";
            for (uint8_t k=0;k<potNotchCount[idx];k++){
              if (k) { lv += ","; lc += ","; }
              lv += String(potNotchVals[idx][k],3);
              lc += String((unsigned)potNotchCenterRaw[idx][k]);
            }
            enviar(lv);
            if (potNotchHasCenters[idx]) enviar(lc);
            enviar(String("NOTCH PARTIAL ") + pinToStringForDump(cfg.pin,cfg.type) + " " + (potPartial[idx] ? "ON":"OFF"));
            enviar(String("NOTCH SNAPWIN ") + pinToStringForDump(cfg.pin,cfg.type) + " " + String(potSnapWin[idx],3));
          }
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

    // opcional, venía en el viejo
    enviar("MB.WINDOW base/size ya informada");

    enviar("BEGIN MB");
    handleMbCommand("MB.DUMP");
    enviar("END MB");

    enviar(F("✅ DUMP COMPLETO FIN"));
    bloqueado = false;
    return;
  }

  // ===================== ADD (legacy) =====================
  if (modoConfig && cmd.startsWith("ADD")) {
    char tipo[20], pinStr[16], param[40], v1[16], v2[16];
    int args = sscanf(cmd.c_str() + 4, "%19s %15s %39s %15s %15s", tipo, pinStr, param, v1, v2);

    if (args == 5) {
      int pin = analogPinFromString(pinStr);

      if (strcasecmp(tipo, "OUTPUT") == 0) {
        if (outputCount < EEPROM_MAX_ENTRIES) {
          IOPin* pinObj = busMCP.getPin(pin, false);
          outputs[outputCount] = new OutputManagerMCP(param, pinObj);
          outputs[outputCount]->begin();
          if (pinObj) pinObj->digitalWrite(LOW);
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

      } else if (strcasecmp(tipo, "SELECTOR") == 0) {
        int sidx = getOrCreateSelector(param);
        if (sidx < 0) {
          enviar(F("❌ ADD SELECTOR: sin espacio"));
        } else {
          IOPin* pinObj = busMCP.getPin(pin, true);
          if (pinObj) {
            pinObj->pinMode(INPUT_PULLUP);
            int16_t vv = (int16_t)atoi(v1);
            selectors[sidx]->add(pinObj, vv);
            enviar(String("✅ SELECTOR ADD ") + param + " PIN=" + String(pin) + " VAL=" + String((int)vv));
          }
        }
        savePinConfig("SELECTOR", pin, param, v1, v2);

      } else if (strcasecmp(tipo, "POT") == 0) {
        savePinConfig(tipo, pin, param, v1, v2);

      } else {
        enviar(String("❌ ERROR: Tipo desconocido en ADD → ") + tipo);
      }

    } else {
      enviar(String("❌ ERROR: Formato ADD inválido → ") + cmd);
    }

    bloqueado = false;
    return;
  }

  // ===================== CFG (POT params) =====================
  if (modoConfig && cmd.startsWith("CFG")) {
    updatePotParam(cmd);
    bloqueado = false;
    return;
  }

  // ===================== Entrada clave=valor (outputs + MB.SET) =====================
  {
    String key, v;

    if (value && *value) {
      key = cmd;
      v   = val;
    } else {
      int eq = cmd.indexOf('=');
      if (eq >= 0) {
        key = cmd.substring(0, eq);
        v   = cmd.substring(eq + 1);
      }
    }

    if (key.length()) {
      key.trim(); v.trim();

      for (int i=0;i<outputCount;i++){
        if (outputs[i] && outputParams[i] && strcasecmp(outputParams[i], key.c_str()) == 0) {
          outputs[i]->outputDigital(key.c_str(), v.c_str(), 1);
          enviar("✅ ACK: " + key + "=" + v);
          return;
        }
      }

      String mb = buildModbusSetFromKV(key, v);
      if (mb.length()) {
        handleMbCommand(mb);
        return;
      }

      enviar("❌ NACK: " + key + "=" + v);
      return;
    }
  }

  // ===================== Fallback outputs por "command value" =====================
  bool handled = false;
  for (int i = 0; i < outputCount; i++) {
    if (outputs[i] && outputs[i]->outputDigital(command, value ? value : "", 1)) handled = true;
  }
  if (handled) enviar("✅ ACK: " + String(command));
  else         enviar("❌ NACK: " + String(command));

  bloqueado = false;
}


// ========= AP / LED =========
void enableAP() {
  if (apActive) return;

#if defined(MODO_WIFI_AP)
  // Fuerza modo AP (o AP+STA si quieres mantener compatibilidad)
  WiFi.mode(WIFI_AP);          // <- clave
  delay(100);

  const char* ssid = "EASYSIMBigBoard";
  const char* pass = nullptr;  // abierto (más fácil de ver)
  int channel = 1;             // 1..13
  bool hidden = false;
  int maxConn = 1;

  bool ok = WiFi.softAP(ssid, pass, channel, hidden, maxConn);
  delay(200);

  IPAddress ip = WiFi.softAPIP();
  Serial.printf("🌐 AP enable: ok=%d mode=%d ssid=%s ip=%s\n",
                ok ? 1 : 0, (int)WiFi.getMode(), WiFi.softAPSSID().c_str(), ip.toString().c_str());

  if (!wifiServerStarted) {
    wifiServer.begin();
    wifiServerStarted = true;
    Serial.println("📥 WiFiServer (AP) escuchando en puerto 5000 (TCP)");
  }

  apActive = ok;
  if (apActive) setLed(0,255,0);
#endif
}

void disableAP() {
  if (!apActive) return;

#if defined(MODO_WIFI_AP)
  Serial.printf("📴 AP disable: stations=%d\n", WiFi.softAPgetStationNum());
  WiFi.softAPdisconnect(true);
  delay(100);

  // Si quieres dejar WiFi totalmente apagado al salir:
  WiFi.mode(WIFI_OFF);
  delay(100);
#endif

  apActive = false;
  setLed(0,0,0);
}

// ======================= setup/loop =======================
void setup() {
  #if defined(MODO_SERIAL)
    Serial.begin(SERIAL_BAUD);
  #endif

  EE::begin(16384);
  EE_ensureCountByte();

  for (int i=0;i<EEPROM_MAX_ENTRIES;i++){
    potEMA[i]=NAN;
    potOutLast[i]=NAN;
    potLastMs[i]=0;
    potThreshold[i]=0.5f;
    potAccDelta[i]=0.0f;
    potLastEmaSample[i]=NAN;
    potNotchCount[i]=0;
    potNotchHystPct[i]=0.05f;
    potLastNotch[i] = -1;
    potEmaNorm[i]   = NAN;
    potNotchHasCenters[i] = false;
    potEmaRaw[i]    = NAN;
    potPartial[i]   = false;
    potSnapWin[i]   = 0.03f;
  }

  for (int i=0;i<MAX_SELECTORS;i++){ selectors[i]=nullptr; selectorNames[i]=nullptr; }
  selectorCount = 0;

  // SPI/I2C
  SPIBUS.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  pinMode(CS_W5500, OUTPUT);
  digitalWrite(CS_W5500, HIGH);

  pinMode(PIN_BTN_OLED, INPUT_PULLUP);
  pinMode(PIN_BTN_AP,   INPUT_PULLUP);

  // ===== OLED SSD1306 =====
  Wire.begin(PIN_SDA, PIN_SCL);
  I2CBUS1.begin(21, 14);

  scanBus(I2CBUS1, "OLED");
  oledOK = true;  
  display.begin();
  display.setI2CAddress(0x3C << 1);
  Serial.printf("oledOK=%d\n", oledOK ? 1 : 0);
  if (oledOK) {
    display.clearBuffer();
    display.setFont(u8g2_font_5x8_tf);
    display.drawStr(0, 10, "OLED OK");
    display.drawStr(0, 20, "INICIANDO..");
    display.sendBuffer();
  }

  // ADS1115
  g_ads_ok = g_ads.begin(0x48);
  if (g_ads_ok) {
    g_ads.setGain(GAIN_ONE);
    Serial.println(F("✅ ADS1115 OK en 0x48"));
  } else {
    Serial.println(F("⚠️ ADS1115 no detectado (0x48)."));
  }


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

  // ServerDiscovery
  serverDiscovery.begin();

  serverDiscovery.onBeacon([](const IPAddress& ip, uint16_t port, const String& msg){
    enviar(String("📡 DISCOVERY: beacon desde ") +
          ip.toString() + ":" + String(port) +
          " → " + msg);
  });
  serverDiscovery.onConnected([](const IPAddress& ip, uint16_t port){
    enviar(String("✅ CLIENT.CONNECT OK → ") +
          ip.toString() + ":" + String(port));

    for (int i = 0; i < outputCount; i++) {
      if (outputParams[i]) {
        enviar(String("register(") + outputParams[i] + ")");
      }
    }
  });

  serverDiscovery.onDisconnected([](){
    enviar("⚠️ CLIENT.DISCONNECTED");
  });
  serverDiscovery.onLine([](const String& line){
    Serial.print(F("[Server] LINE: "));
    Serial.println(line);

    if (line.startsWith("connected=")) {
      enviar(String("📡 SERVER STATUS: ") + line);
      return;
    }

    handleLine(line.c_str(), nullptr);
  });
  serverDiscovery.onError([](const String& err){
    enviar(String("❌ CLIENT.ERROR ") + err);
  });
#endif

  // Ventana Modbus y autoload
  modbusInitWindowFromUsed();
  initModbusTCP();

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

  // MCP por I2C
  if (!busMCP.beginI2C_Auto8(&Wire)) {
    Serial.println("❌ No se detectó ningún MCP23017 (0x20..0x27)");
  } else {
    Serial.printf("✅ MCP detectados: %d  mask=0b", busMCP.getDetectedChips());
    Serial.println(busMCP.presentMask(), BIN);
  }


  #if defined(MODO_SERIAL)
    serialInterface.begin();
  #endif

  // Cargar configuración y NOTCH (y selectors)
  loadConfigFromEEPROM();
  notchLoadAllFromEEPROM();

  // Inicializa IO
  for (int i = 0; i < switchCount; i++) switches[i]->begin();
  for (int i = 0; i < buttonCount; i++) buttons[i]->begin();
  for (int i = 0; i < outputCount; i++) {
    outputs[i]->begin();
    if (outputPins[i]) outputPins[i]->digitalWrite(LOW); // ✅ LOW al arranque
    enviar(String("register(") + outputParams[i] + ")");
  }
  for (int i = 0; i < selectorCount; i++) {
    if (selectors[i]) selectors[i]->begin();
  }

  Serial.println("TEST: levantando AP 5s...");
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP("EASYSIM_TEST");
  Serial.printf("TEST softAP=%d IP=%s\n", ok ? 1 : 0, WiFi.softAPIP().toString().c_str());
  Serial.printf("AP IP=%s  SSID=%s  mode=%d\n",
  WiFi.softAPIP().toString().c_str(),
  WiFi.softAPSSID().c_str(),
  (int)WiFi.getMode()
  );
}

void loop() {
  // 1) refresca cache ADS (I2C corto y no bloqueante)
  adsPollCache();

  // 2) refresca OLED (I2C, no bloqueante)
  oledMaybeDrawChangeOnly();
  pollButtonsESP32();  

  // 3) networking / serial
  #if defined(MODO_ETHERNET)
    if (ethernetInterface) ethernetInterface->update();
    if (eth_connected) serverDiscovery.poll();
  #endif

  #if defined(MODO_SERIAL)
    serialInterface.update();
  #endif

  #if defined(MODO_WIFI_AP)
    if (!wifiClient || !wifiClient.connected()) {
      // ✅ FIX: available() está deprecated → accept()
      WiFiClient newClient = wifiServerStarted ? wifiServer.accept() : WiFiClient();
      if (newClient) {
        if (wifiClient && wifiClient.connected()) {
          wifiClient.stop();
        }
        wifiClient = newClient;
        Serial.println("📶 Cliente WiFi (AP) conectado");
      }
    } else {
      while (wifiClient.available()) {
        static String wifiLine;
        char ch = wifiClient.read();
        if (ch == '\r' || ch == '\n') {
          if (wifiLine.length() > 0) {
            int sp = wifiLine.indexOf(' ');
            String cmd, val;
            if (sp > 0) {
              cmd = wifiLine.substring(0, sp);
              val = wifiLine.substring(sp + 1);
            } else {
              cmd = wifiLine;
              val = "";
            }
            cmd.trim();
            val.trim();
            handleLine(cmd.c_str(), val.length() ? val.c_str() : nullptr);
            wifiLine = "";
          }
        } else {
          wifiLine += ch;
          if (wifiLine.length() > 200) {
            wifiLine.remove(0);
          }
        }
      }
    }
  #endif

  tickModbus();

  // 4) IO (MCP) + POTS
  if (!modoConfig && !bloqueado) {

    for (int i = 0; i < switchCount; i++) switches[i]->update();
    for (int i = 0; i < buttonCount; i++) buttons[i]->update();
    for (int i = 0; i < selectorCount; i++) if (selectors[i]) selectors[i]->update();
    tickPots();
  }
}


