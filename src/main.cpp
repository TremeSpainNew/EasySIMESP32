// ========================== main.cpp / main.ino ==========================
#include <Arduino.h>
#include "EE.h"                 // EEPROM / configuración (NVS)
#include <ControllerMCP.h>
#include <ETH.h>
#include <WiFi.h>
#include <SPI.h>
#include <RS485.h>
#include <NetworkClient.h>
#include <NetworkServer.h>
#include <MCPBusController.h>
#include <WiFiAP.h>
#include <modbus.h>             // usa EE por dentro
#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#include <ctype.h>
#include "nvs_flash.h"
#include <io/Menu.h>
#include <math.h>               // isnan, fabs, lroundf
#include <Adafruit_ADS1X15.h>   // ADS1115<<
#include <Update.h>
#include <WiFiUdp.h>            // UDP para discover (usado dentro de ServerDiscovery normalmente)
#include "EthernetInterface.h"  // interfaz TCP puerto 5000
#include <ServerDiscovery.h>    // NUEVO: discover+cliente 5090 con callbacks

#include "CanManager.h"
#include <Profiles/ASFADigitalProfile.h>

#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2

#define PIN_TYPE_CAN_BUTTON 5
#define PIN_TYPE_CAN_SWITCH 6
#define PIN_TYPE_CAN_OUTPUT 7

#include <PCF8574.h>

//#define SSD1306
#define SSD1309

#if defined(SSD1306)
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#elif defined(SSD1309)
  #include <U8g2lib.h>
#endif

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
static constexpr uint32_t EE_SIZE_BYTES = 32768; // 32 KB // total NVS simulada por EE

static constexpr uint32_t MB_REGION_SIZE = 8192;
static constexpr uint32_t MB_REGION_BASE = EE_SIZE_BYTES - MB_REGION_SIZE;

static constexpr uint32_t NOTCH_REGION_SIZE = 4096;
static constexpr uint32_t NOTCH_REGION_BASE = MB_REGION_BASE - NOTCH_REGION_SIZE;

static constexpr uint32_t NET_REGION_SIZE = 256;
static constexpr uint32_t NET_REGION_BASE = NOTCH_REGION_BASE - NET_REGION_SIZE;

static constexpr uint32_t NOTCH_MAGIC = 0x4E544348; // 'N''T''C''H'
static constexpr uint8_t  NOTCH_VERSION = 5;        // V4: partial+snapwin

// ========================= AP WiFI ===========================
void handleLine(const char* command, const char* value);
void handleMbCommand(const String& cmd);
void handleAPButton();
void enableAP();
void disableAP();
static void startDirectOtaServer();
static void pollDirectOtaServer();
static bool parseCanRef(const char* s, uint8_t& node, uint8_t& channel);
static String canRefToString(uint8_t node, uint8_t channel);

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
#define OLED_SDA 14
#define OLED_SCL 21

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

static constexpr uint16_t kOtaTcpPort = 3232;
NetworkServer otaServer(kOtaTcpPort);
bool otaServerStarted = false;
bool otaRestartPending = false;
unsigned long otaRestartAtMs = 0;

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
bool outputInvertedFlags[EEPROM_MAX_ENTRIES];

static bool g_can_started = false;
static bool g_can_seen_hello = false;
static bool g_can_seen_heartbeat = false;
static bool g_can_seen_ack = false;
static uint8_t g_can_last_hello_node = 0;
static uint8_t g_can_last_hb_node = 0;
static uint8_t g_can_last_ack_node = 0;
static uint8_t g_can_last_ack_channel = 0;
static uint8_t g_can_last_ack_value = 0;
static unsigned long g_can_last_hello_ms = 0;
static unsigned long g_can_last_hb_ms = 0;
static unsigned long g_can_last_ack_ms = 0;
static ASFADigitalProfile g_can_profile_asfad;

bool ioWatchEnabled = false;
uint8_t ioWatchPin = 0;
char ioWatchKind[16] = "";
int ioWatchLastValue = -9999;
unsigned long ioWatchLastMs = 0;

static int lastSwitchState[EEPROM_MAX_ENTRIES];
static int lastButtonState[EEPROM_MAX_ENTRIES];
static int lastOutputState[EEPROM_MAX_ENTRIES];
static int lastPotState[EEPROM_MAX_ENTRIES];

static unsigned long lastIoStateMs = 0;

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
const unsigned long longPressTime = 1000;

// ======================= Cliente TCP a servidor principal (5090) (MOVIDO ARRIBA para oledDrawStatus) =======================
static const uint16_t kClientServerPort = 5090;
ServerDiscovery serverDiscovery(5091,5090); // la clase hace discover UDP+TCP 5090

// ======================= OLED SSD1306 =======================
#define OLED_ADDR   0x3C
#define OLED_W      128
#define OLED_H      64
#define OLED_RESET  -1   // sin pin reset

#if defined(SSD1306)
  Adafruit_SSD1306 display(OLED_W, OLED_H, &I2CBUS1, OLED_RESET);
#elif defined(SSD1309)
  U8G2_SSD1309_128X64_NONAME2_F_SW_I2C display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ U8X8_PIN_NONE);
  Menu menu(display);
  PCF8574 pcf(0x38, &Wire);
#endif
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1309_128X64_NONAME2_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
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
static bool menuPcfReady = false;
static const char* apSsid = "EASYSIMBigBoard";
static const char* apPassword = "easysim123";

#if defined(SSD1309)
static constexpr uint8_t MENU_PCF_PIN_ENT = 4;
static constexpr unsigned long MENU_ENTER_HOLD_MS = 5000;
#endif

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
#if defined(SSD1306)
static void oledDrawNetPage_Adafruit(const String& frame) {
  if (!oledOK) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  int start = 0;
  int lines = 0;
  while (start < (int)frame.length() && lines < 8) { // 8 líneas aprox en 64px
    int nl = frame.indexOf('\n', start);
    String line = (nl >= 0) ? frame.substring(start, nl) : frame.substring(start);
    display.println(line);
    lines++;
    if (nl < 0) break;
    start = nl + 1;
  }

  display.display();
}
#endif
#if defined(SSD1309)
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
#endif

static String buildOledFrame() {
  bool linkUp = ETH.linkUp();
  IPAddress ip = ETH.localIP();
  bool hasIp = (ip != IPAddress((uint32_t)0));

  const char* mode;
  if (!linkUp) mode = "NO LINK";
  else if (!hasIp)  mode = "NO IP";
  else              mode = (ethStaticFallbackUsed ? "STATIC" : "DHCP");

  bool cliUp  = linkUp && serverDiscovery.connected();   // ✅ AQUÍ
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

#if defined(SSD1309)
static String menuStatusProvider() {
  bool linkUp = ETH.linkUp();
  IPAddress ip = ETH.localIP();
  if (!linkUp) return "ETH DOWN";
  if (ip == IPAddress((uint32_t)0)) return "IP 0.0.0.0";
  return String("IP ") + ip.toString();
}

static void menuApplyNet(const Menu::NetCfg& cfg) {
  IPAddress ip(cfg.ip[0], cfg.ip[1], cfg.ip[2], cfg.ip[3]);
  IPAddress gw(cfg.gw[0], cfg.gw[1], cfg.gw[2], cfg.gw[3]);
  IPAddress mask(cfg.mask[0], cfg.mask[1], cfg.mask[2], cfg.mask[3]);

  EE::setDhcpEnabled(cfg.dhcp);
  EE::setStaticIP(ip);
  EE::setStaticGateway(gw);
  EE::setStaticMask(mask);
  EE::commit();

  Serial.printf("📡 MENU NET APPLY mode=%s ip=%s gw=%s mask=%s\n",
                cfg.dhcp ? "DHCP" : "STATIC",
                ip.toString().c_str(),
                gw.toString().c_str(),
                mask.toString().c_str());
  Serial.println("🔄 Reiniciando para aplicar configuración de red...");
  delay(300);
  ESP.restart();
}

static void menuToolDump() {
  handleLine("#DUMP", "");
}

static void menuToolScanI2C() {
  scanBus(Wire, "MAIN");
  scanBus(I2CBUS1, "OLED");
}

static void menuClosed() {
  oledRequest();
}

static bool otaReadLine(NetworkClient& client, String& line, unsigned long timeoutMs) {
  line = "";
  unsigned long startMs = millis();

  while ((millis() - startMs) < timeoutMs) {
    while (client.available()) {
      char ch = (char)client.read();

      if (ch == '\n') {
        line.trim();
        return true;
      }

      if (ch != '\r') {
        line += ch;
        if (line.length() > 160) {
          return false;
        }
      }
    }

    if (!client.connected()) {
      return false;
    }

    delay(1);
    yield();
  }

  return false;
}

static void startDirectOtaServer() {
  if (otaServerStarted) return;

  otaServer.begin();
  otaServerStarted = true;
  Serial.printf("🚀 OTA TCP escuchando en puerto %u\n", kOtaTcpPort);
}

static void handleDirectOtaClient(NetworkClient& client) {
  String header;
  if (!otaReadLine(client, header, 5000)) {
    client.println("ERR HEADER");
    return;
  }

  header.trim();
  if (!header.length()) {
    client.println("ERR EMPTY");
    return;
  }

  if (header == "PING") {
    client.println("PONG OTA");
    return;
  }

  int sp1 = header.indexOf(' ');
  int sp2 = (sp1 >= 0) ? header.indexOf(' ', sp1 + 1) : -1;

  String cmd = (sp1 >= 0) ? header.substring(0, sp1) : header;
  String sizeText = (sp1 >= 0 && sp2 > sp1) ? header.substring(sp1 + 1, sp2) : "";
  String md5Text = (sp2 > sp1) ? header.substring(sp2 + 1) : "";

  cmd.trim();
  sizeText.trim();
  md5Text.trim();

  if (cmd != "EASYSIM_OTA") {
    client.println("ERR PROTOCOL");
    return;
  }

  size_t firmwareSize = (size_t)strtoull(sizeText.c_str(), nullptr, 10);
  if (!firmwareSize) {
    client.println("ERR SIZE");
    return;
  }

  if (!Update.begin(firmwareSize, U_FLASH)) {
    client.print("ERR BEGIN ");
    client.println(Update.errorString());
    return;
  }

  if (md5Text.length() == 32 && !Update.setMD5(md5Text.c_str())) {
    client.println("ERR MD5");
    Update.abort();
    return;
  }

  client.println("OK READY");
  Serial.printf("🚀 OTA TCP inicio: %u bytes\n", (unsigned)firmwareSize);

  uint8_t buffer[1024];
  size_t received = 0;
  size_t nextProgressMark = 65536;
  unsigned long lastDataMs = millis();

  while (received < firmwareSize) {
    int availableBytes = client.available();

    if (availableBytes > 0) {
      size_t missing = firmwareSize - received;
      size_t toRead = missing;
      if (toRead > sizeof(buffer)) toRead = sizeof(buffer);
      if (toRead > (size_t)availableBytes) toRead = (size_t)availableBytes;

      int justRead = client.read(buffer, toRead);
      if (justRead <= 0) {
        client.println("ERR READ");
        Update.abort();
        return;
      }

      size_t written = Update.write(buffer, (size_t)justRead);
      if (written != (size_t)justRead) {
        client.print("ERR WRITE ");
        client.println(Update.errorString());
        Update.abort();
        return;
      }

      received += written;
      lastDataMs = millis();

      if (received >= nextProgressMark || received == firmwareSize) {
        Serial.printf("🚀 OTA TCP progreso: %u / %u\n", (unsigned)received, (unsigned)firmwareSize);
        nextProgressMark += 65536;
      }

      continue;
    }

    if (!client.connected()) {
      client.println("ERR DISCONNECTED");
      Update.abort();
      return;
    }

    if ((millis() - lastDataMs) > 10000) {
      client.println("ERR TIMEOUT");
      Update.abort();
      return;
    }

    delay(1);
    yield();
  }

  if (!Update.end(true)) {
    client.print("ERR END ");
    client.println(Update.errorString());
    Update.abort();
    return;
  }

  client.println("OK DONE");
  delay(20);

  Serial.println("✅ OTA TCP completada, reiniciando...");
  otaRestartPending = true;
  otaRestartAtMs = millis() + 500;
}

static void pollDirectOtaServer() {
  if (otaRestartPending && millis() >= otaRestartAtMs) {
    ESP.restart();
  }

  if (!otaServerStarted) return;

  NetworkClient client = otaServer.accept();
  if (!client) return;

  Serial.println("📦 Cliente OTA TCP conectado");
  handleDirectOtaClient(client);
  delay(20);
  client.stop();
}

static void pollMenuEnterHold() {
  if (!menuPcfReady || menu.isActive()) return;

  static bool holdConsumed = false;
  static unsigned long pressedSinceMs = 0;

  uint8_t raw = pcf.read8();
  bool entPressed = (raw & (1u << MENU_PCF_PIN_ENT)) == 0;
  unsigned long now = millis();

  if (!entPressed) {
    pressedSinceMs = 0;
    holdConsumed = false;
    return;
  }

  if (holdConsumed) return;

  if (pressedSinceMs == 0) {
    pressedSinceMs = now;
    return;
  }

  if ((now - pressedSinceMs) >= MENU_ENTER_HOLD_MS) {
    holdConsumed = true;
    menu.open();
    oledRequest();
    Serial.println("✅ Menu abierto por pulsacion larga ENT");
  }
}

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
#endif
static void oledMaybeDrawChangeOnly() {
  if (!oledOK) return;

  const unsigned long now = millis();
  if (now - lastOledDraw < OLED_MIN_MS) return;

  if (oledPage == OLED_PAGE_IO) {
    //oledDrawIOPage();           // <-- tu página de círculos
    lastOledDraw = now;
    return;
  }

  String frame = buildOledFrame();
  const bool changed   = (frame != oledLastFrame);
  const bool keepalive = (now - lastOledDraw) >= OLED_KEEPALIVE_MS;
  if (!changed && !keepalive) return;

  oledLastFrame = frame;
  lastOledDraw  = now;

  #if defined(SSD1306)
    oledDrawNetPage_Adafruit(frame);
  #elif defined(SSD1309)  
    oledDrawNetPage_U8G2(frame);
  #endif
  //oledDrawNetPage_Adafruit(frame);
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

// ✅ NUEVO: HYBRIDSEG (lineal solo entre 2 muescas internas)
static bool    potHybridSegEn[EEPROM_MAX_ENTRIES];   // ON/OFF
static uint8_t potHybridSegA[EEPROM_MAX_ENTRIES];    // índice notch A (0-based)
static uint8_t potHybridSegB[EEPROM_MAX_ENTRIES];    // índice notch B (0-based)

// ===================== POT SPLIT (persistente en NOTCH v4) =====================
enum PotSplitMode : uint8_t {
  SPLIT_OFF = 0,
  SPLIT_DUAL = 1,           // tagFwd + tagBack
  SPLIT_SINGLE_SIGNED = 2,  // tagSingle = -..+
  SPLIT_SINGLE_CENTERED = 3 // tagSingle = 0..(2*center)
};

enum NotchChan : uint8_t {
  NCH_BASE   = 0, // canal único (sin split)
  NCH_FWD    = 1, // split dual forward
  NCH_BACK   = 2, // split dual back
  NCH_SINGLE = 3  // split single (signed/centered)
};

static inline const char* notchChanToStr(NotchChan ch){
  switch(ch){
    case NCH_BASE:   return "BASE";
    case NCH_FWD:    return "FWD";
    case NCH_BACK:   return "BACK";
    case NCH_SINGLE: return "SINGLE";
    default:         return "BASE";
  }
}

static bool parseNotchChanToken(const String& tok, NotchChan &out){
  String t = tok; t.trim(); t.toUpperCase();
  if (t=="BASE")   { out=NCH_BASE; return true; }
  if (t=="FWD")    { out=NCH_FWD; return true; }
  if (t=="BACK")   { out=NCH_BACK; return true; }
  if (t=="SINGLE") { out=NCH_SINGLE; return true; }
  return false;
}

static PotSplitMode potSplitMode[EEPROM_MAX_ENTRIES];
static float potSplitDeadband[EEPROM_MAX_ENTRIES];   // fracción 0..0.3 del rango raw
static float potSplitCenterBias[EEPROM_MAX_ENTRIES]; // 0..1 (0.5 típico)

static char* potTagFwd[EEPROM_MAX_ENTRIES];     // throttle (adelante)
static char* potTagBack[EEPROM_MAX_ENTRIES];    // brake (atrás)
static char* potTagSingle[EEPROM_MAX_ENTRIES];  // palanca única

static void potSplitResetRAM(int i){
  potSplitMode[i] = SPLIT_OFF;
  potSplitDeadband[i] = 0.02f;     // 2% por defecto
  potSplitCenterBias[i] = 0.5f;    // centro por defecto
  if (potTagFwd[i])    { free(potTagFwd[i]); potTagFwd[i]=nullptr; }
  if (potTagBack[i])   { free(potTagBack[i]); potTagBack[i]=nullptr; }
  if (potTagSingle[i]) { free(potTagSingle[i]); potTagSingle[i]=nullptr; }
}

// Helpers numéricos SPLIT
static inline float norm01_from_raw(int raw, int rmin, int rmax){
  if (rmax == rmin) return 0.0f;
  float n = (float)(raw - rmin) / (float)(rmax - rmin);
  if (n < 0) n = 0;
  if (n > 1) n = 1;
  return n;
}
static inline bool potSplitEnabled(int i){
  return potSplitMode[i] != SPLIT_OFF;
}


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

static inline String pinToStringForIo(uint8_t pin, const char* kind) {
  if (kind && strcasecmp(kind, "POT") == 0 && isADSIndex(pin)) {
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
    ethernetInterface->println(msg); // puerto 5000 configuración/log
  }
#endif

#if defined(MODO_WIFI_AP)
  if (wifiClient && wifiClient.connected()) {
    wifiClient.println(msg);         // AP configuración/log
  }
#endif
}

void enviarServidor(const String& msg) {
#if defined(MODO_ETHERNET)
  if (serverDiscovery.connected()) {
    serverDiscovery.println(msg);
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

static bool parseValueToInt(const String& v, int &out) {
  String t = v;
  t.trim();
  t.toUpperCase();

  if (t == "1" || t == "ON" || t == "TRUE" || t == "HIGH") {
    out = 1;
    return true;
  }

  if (t == "0" || t == "OFF" || t == "FALSE" || t == "LOW") {
    out = 0;
    return true;
  }

  // Acepta enteros y decimales: 67.47 -> 67 o 67.47 -> 67 redondeado
  char *endptr = nullptr;
  float f = strtof(t.c_str(), &endptr);

  if (endptr == t.c_str()) {
    return false;
  }

  out = (int)lroundf(f);   // redondea: 67.47 -> 67, 67.50 -> 68
  // out = (int)f;         // usa esta línea si prefieres truncar siempre

  return true;
}

static inline bool isOutputConfigType(uint8_t type) {
  return type == 2 || type == PIN_TYPE_CAN_OUTPUT;
}

static inline bool pinConfigOutputInverted(const PinConfig& cfg) {
  return isOutputConfigType(cfg.type) && cfg.intervalo != 0;
}

static inline void pinConfigSetOutputInverted(PinConfig& cfg, bool inverted) {
  if (isOutputConfigType(cfg.type)) {
    cfg.intervalo = inverted ? 1 : 0;
  }
}

static inline int logicalToPhysicalOutputValue(int logicalValue, bool inverted) {
  int value = logicalValue ? 1 : 0;
  return inverted ? (1 - value) : value;
}

static inline int physicalToLogicalOutputValue(int physicalValue, bool inverted) {
  int value = physicalValue ? 1 : 0;
  return inverted ? (1 - value) : value;
}

static int getLocalOutputIndexByPin(uint8_t pin) {
  for (int i = 0; i < outputCount; i++) {
    if (outputPinNum[i] == pin) return i;
  }
  return -1;
}

static bool writeLocalOutputByIndex(int index, int logicalValue) {
  if (index < 0 || index >= outputCount) return false;

  IOPin* pin = outputPins[index];
  if (!pin) return false;

  pin->pinMode(OUTPUT);

  int physicalValue = logicalToPhysicalOutputValue(
    logicalValue,
    outputInvertedFlags[index]
  );

  pin->digitalWrite(physicalValue ? HIGH : LOW);
  return true;
}

static bool writeLocalOutputByName(const char* name, int logicalValue) {
  if (!name || !name[0]) return false;

  for (int i = 0; i < outputCount; i++) {
    if (outputParams[i] && strcasecmp(outputParams[i], name) == 0) {
      return writeLocalOutputByIndex(i, logicalValue);
    }
  }

  return false;
}

static bool applyLocalOutputInversionByPin(uint8_t pin, bool inverted) {
  int index = getLocalOutputIndexByPin(pin);
  if (index < 0 || !outputPins[index]) return false;

  int oldPhysical = outputPins[index]->digitalRead() == HIGH ? 1 : 0;
  int logicalValue = physicalToLogicalOutputValue(oldPhysical, outputInvertedFlags[index]);

  outputInvertedFlags[index] = inverted;
  writeLocalOutputByIndex(index, logicalValue);
  return true;
}

static bool areAllOutputsInverted() {
  uint8_t count = EE::read(0);
  bool found = false;

  for (int i = 0; i < count && i < EEPROM_MAX_ENTRIES; i++) {
    PinConfig cfg{};
    EE_GET(1 + i * sizeof(PinConfig), cfg);
    if (!isOutputConfigType(cfg.type)) continue;
    found = true;
    if (!pinConfigOutputInverted(cfg)) return false;
  }

  return found;
}

static int setAllOutputsInvertedStored(bool inverted) {
  uint8_t count = EE::read(0);
  int changed = 0;

  for (int i = 0; i < count && i < EEPROM_MAX_ENTRIES; i++) {
    PinConfig cfg{};
    EE_GET(1 + i * sizeof(PinConfig), cfg);
    if (!isOutputConfigType(cfg.type)) continue;

    bool oldInv = pinConfigOutputInverted(cfg);
    if (oldInv != inverted) {
      pinConfigSetOutputInverted(cfg, inverted);
      EE_PUT(1 + i * sizeof(PinConfig), cfg);
      changed++;
    }

    if (cfg.type == 2) {
      applyLocalOutputInversionByPin(cfg.pin, inverted);
    }
  }

  if (changed > 0) EE::commit();
  return changed;
}

static uint8_t menuOutputCountProvider() {
  return (uint8_t)outputCount;
}

static String menuOutputLabelProvider(uint8_t index) {
  if (index >= outputCount) return String("OUT ?");
  String label = outputParams[index] ? String(outputParams[index]) : String("OUT");
  label += " P";
  label += String(outputPinNum[index]);
  return label;
}

static int menuOutputStateProvider(uint8_t index) {
  if (index >= outputCount || !outputPins[index]) return 0;
  int physical = outputPins[index]->digitalRead() == HIGH ? 1 : 0;
  return physicalToLogicalOutputValue(physical, outputInvertedFlags[index]);
}

static void menuOutputSet(uint8_t index, bool on) {
  writeLocalOutputByIndex((int)index, on ? 1 : 0);
}

static bool menuAllOutInvGetter() {
  return areAllOutputsInverted();
}

static void menuAllOutInvSetter(bool enabled) {
  int changed = setAllOutputsInvertedStored(enabled);
  Serial.printf("MENU OUTINV ALL %s changed=%d\n", enabled ? "ON" : "OFF", changed);
}

static String menuEthStatusDetails() {
  bool linkUp = ETH.linkUp();
  IPAddress ip = ETH.localIP();
  bool hasIp = (ip != IPAddress((uint32_t)0));
  IPAddress srvIp = serverDiscovery.serverIp();

  String s;
  s.reserve(96);
  s += "LINK:";
  s += linkUp ? "UP" : "DOWN";
  s += "\nIP:";
  s += hasIp ? ip.toString() : "0.0.0.0";
  s += "\nCLI5090:";
  s += serverDiscovery.connected() ? "UP" : "DOWN";
  s += "\nSRV:";
  s += serverDiscovery.hasServerIp() ? srvIp.toString() : "0.0.0.0";
  return s;
}

static String menuCanStatusDetails() {
  String s;
  s.reserve(96);
  s += "CAN:";
  s += g_can_started ? "ON" : "OFF";
  s += "\nHELLO:";
  if (g_can_seen_hello) {
    s += "N";
    s += String(g_can_last_hello_node);
    s += " ";
    s += String((millis() - g_can_last_hello_ms) / 1000);
    s += "s";
  } else {
    s += "NONE";
  }
  s += "\nHB:";
  if (g_can_seen_heartbeat) {
    s += "N";
    s += String(g_can_last_hb_node);
    s += " ";
    s += String((millis() - g_can_last_hb_ms) / 1000);
    s += "s";
  } else {
    s += "NONE";
  }
  s += "\nACK:";
  if (g_can_seen_ack) {
    s += "N";
    s += String(g_can_last_ack_node);
    s += ":";
    s += String(g_can_last_ack_channel);
    s += "=";
    s += String(g_can_last_ack_value);
  } else {
    s += "NONE";
  }
  return s;
}

static String menuIoStatusDetails() {
  int activeButtons = 0;
  int activeSwitches = 0;
  int activeOutputs = 0;

  for (int i = 0; i < buttonCount; i++) {
    if (buttonPins[i] && buttonPins[i]->digitalRead() == LOW) activeButtons++;
  }
  for (int i = 0; i < switchCount; i++) {
    if (switchPins[i] && switchPins[i]->digitalRead() == LOW) activeSwitches++;
  }
  for (int i = 0; i < outputCount; i++) {
    if (!outputPins[i]) continue;
    int physical = outputPins[i]->digitalRead() == HIGH ? 1 : 0;
    if (physicalToLogicalOutputValue(physical, outputInvertedFlags[i])) activeOutputs++;
  }

  String s;
  s.reserve(96);
  s += "BTN ";
  s += String(activeButtons);
  s += "/";
  s += String(buttonCount);
  s += " SW ";
  s += String(activeSwitches);
  s += "/";
  s += String(switchCount);
  s += "\nOUT ";
  s += String(activeOutputs);
  s += "/";
  s += String(outputCount);
  s += " SEL ";
  s += String(selectorCount);
  if (outputCount > 0 && outputParams[0]) {
    s += "\nOUT0:";
    s += String(outputParams[0]).substring(0, 12);
    s += "\nINVALL:";
    s += areAllOutputsInverted() ? "ON" : "OFF";
  } else {
    s += "\nNO LOCAL OUT";
  }
  return s;
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
    if (!parseValueToInt(token, v)) return String();
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

static void sortNotchCentersAndVals(int idx) {
  if (idx < 0) return;
  uint8_t n = potNotchCount[idx];
  if (n < 2) return;
  if (!potNotchHasCenters[idx]) return;

  // Ordena por centro RAW ascendente, arrastrando el valor asociado
  for (uint8_t i = 0; i < n - 1; i++) {
    for (uint8_t j = i + 1; j < n; j++) {
      if (potNotchCenterRaw[idx][j] < potNotchCenterRaw[idx][i]) {
        uint16_t tc = potNotchCenterRaw[idx][i];
        potNotchCenterRaw[idx][i] = potNotchCenterRaw[idx][j];
        potNotchCenterRaw[idx][j] = tc;

        float tv = potNotchVals[idx][i];
        potNotchVals[idx][i] = potNotchVals[idx][j];
        potNotchVals[idx][j] = tv;
      }
    }
  }
}

// ======================= Carga desde EE =======================
void loadConfigFromEEPROM() {
  // reset RAM containers
  clearSelectorsRAM();
  switchCount = buttonCount = potCount = outputCount = 0;
  memset(outputInvertedFlags, 0, sizeof(outputInvertedFlags));

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
        outputInvertedFlags[outputCount] = pinConfigOutputInverted(cfg);
        outputs[outputCount]      = new OutputManagerMCP(copy, pin);
        outputs[outputCount]->begin();
        // Mantiene la salida en OFF lógico respetando inversión configurada.
        writeLocalOutputByIndex(outputCount, 0);
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
  loadConfigFromEEPROM();
  enviar(String("✅ Entrada añadida a EEPROM: ") + param);
  modbusRefreshWindow();
}

// ================== PERSISTENCIA NOTCH ==================
static void notchEraseRegion() {
  uint32_t zero = 0;
  EE_PUT(NOTCH_REGION_BASE + 0, zero); // magic = 0
  EE::commit();
}

// ===== Helpers EEPROM para NOTCH v4 (strings compactas) =====
static void eeWriteStr(uint32_t &off, const char* s, uint8_t maxLen=63){
  uint8_t len = 0;
  if (s) len = (uint8_t)strnlen(s, maxLen);
  EE::write(off++, len);
  for (uint8_t i=0;i<len;i++) EE::write(off++, (uint8_t)s[i]);
}

static void eeReadStr(uint32_t &off, char* &dst, uint32_t regionEnd){
  uint8_t len = 0;
  if (off >= regionEnd) { dst=nullptr; return; }
  len = EE::read(off++);
  if (dst) { free(dst); dst=nullptr; }
  if (!len) return;
  if (off + len > regionEnd) { return; }
  dst = (char*)malloc(len+1);
  for (uint8_t i=0;i<len;i++) dst[i] = (char)EE::read(off++);
  dst[len] = 0;
}

static inline size_t eeStrSize(const char* s){
  return 1 + (s ? strnlen(s, 63) : 0);
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
    uint8_t pin = potCfgs[i].pin;
    bool hasC   = potNotchHasCenters[i];

    // ✅ Guardar también si hay SPLIT/tags aunque no haya NOTCH
    const bool hasAnySplitTag =
      (potTagSingle[i] && potTagSingle[i][0]) ||
      (potTagFwd[i]    && potTagFwd[i][0])    ||
      (potTagBack[i]   && potTagBack[i][0]);

    const bool hasSplit = (potSplitMode[i] != SPLIT_OFF) || hasAnySplitTag;

    // ✅ HYBRIDSEG cuenta como “extras que merecen guardado”
    const bool hasHybrid = potHybridSegEn[i];

    // Si cnt==0 y no hay split ni extras que merezcan guardado, saltar
    if (cnt == 0 && !hasSplit && !potPartial[i] && !hasHybrid) {
      // (si también quieres que snapWin se guarde siempre, quita este if)
      continue;
    }

    // ---- v4 añade SPLIT: mode + deadband + centerBias + 3 strings ----
    size_t splitSize = 1 + sizeof(float) + sizeof(float)
                     + eeStrSize(potTagSingle[i])
                     + eeStrSize(potTagFwd[i])
                     + eeStrSize(potTagBack[i]);

    size_t recSize = 1 + 1 + sizeof(float) + 1 + 1 +
                 (cnt*sizeof(uint16_t)) + (cnt*sizeof(float)) +
                 sizeof(float) + // snapWin
                 splitSize +
                 3;              // ✅ HYBRIDSEG: en + a + b
    (void)recSize; // por si no lo usas para checks

    EE::write(off, pin); off += 1;
    EE::write(off, cnt); off += 1;
    EE_PUT(off, potNotchHystPct[i]);
    off += sizeof(float);

    uint8_t flags = 0;
    if (hasC)          flags |= 0x01;
    if (potPartial[i]) flags |= 0x02;
    EE::write(off, flags); off += 1;
    EE::write(off, 0);     off += 1; // reservado

    // centros (si cnt==0, no entra)
    for (uint8_t k=0;k<cnt;k++){
      uint16_t c = hasC ? potNotchCenterRaw[i][k] : 0;
      EE_PUT(off, c);
      off += sizeof(uint16_t);
    }

    // vals (si cnt==0, no entra)
    for (uint8_t k=0;k<cnt;k++){
      EE_PUT(off, potNotchVals[i][k]);
      off += sizeof(float);
    }

    // snapWin se guarda siempre si el registro existe
    EE_PUT(off, potSnapWin[i]);
    off += sizeof(float);

    // ---- SPLIT v4 ----
    EE::write(off, (uint8_t)potSplitMode[i]); off += 1;
    EE_PUT(off, potSplitDeadband[i]);         off += sizeof(float);
    EE_PUT(off, potSplitCenterBias[i]);       off += sizeof(float);

    // orden fijo: single, fwd, back
    eeWriteStr(off, potTagSingle[i], 63);
    eeWriteStr(off, potTagFwd[i], 63);
    eeWriteStr(off, potTagBack[i], 63);

    // ✅ v5: HYBRIDSEG (en + a + b)
    EE::write(off++, potHybridSegEn[i] ? 1 : 0);
    EE::write(off++, potHybridSegA[i]);
    EE::write(off++, potHybridSegB[i]);

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

  // reservado (3 bytes)
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

    // ✅ HYBRIDSEG reset
    potHybridSegEn[i] = false;
    potHybridSegA[i]  = 0;
    potHybridSegB[i]  = 0;

    // ✅ SPLIT reset
    potSplitResetRAM(i);
  }

  for (uint16_t r = 0; r < numRecs; r++) {
    if (off + 1 + 1 + sizeof(float) > regionEnd) break;

    uint8_t pin = EE::read(off); off += 1;
    uint8_t cnt = EE::read(off); off += 1;

    float hyst = 0.0f;
    EE_GET(off, hyst);
    off += sizeof(float);

    bool  hasC    = false;
    bool  partial = false;
    float snap    = 0.03f;

    // ---- SPLIT v4 defaults ----
    PotSplitMode sm = SPLIT_OFF;
    float db = 0.02f;
    float cb = 0.5f;
    char* sSingle = nullptr;
    char* sFwd    = nullptr;
    char* sBack   = nullptr;

    // ✅ HYBRIDSEG v5 defaults
    bool    hsegEn = false;
    uint8_t hsegA  = 0;
    uint8_t hsegB  = 0;

    uint32_t centersPos = 0;
    uint32_t valsPos    = 0;

    if (ver >= 2) {
      uint8_t flags = EE::read(off); off += 1;
      off += 1; // reservado

      hasC    = (flags & 0x01) != 0;
      partial = (flags & 0x02) != 0;

      centersPos = off;
      if (off + cnt * sizeof(uint16_t) > regionEnd) break;
      off += cnt * sizeof(uint16_t);

      valsPos = off;
      if (off + cnt * sizeof(float) > regionEnd) break;
      off += cnt * sizeof(float);

      if (ver >= 3) {
        if (off + sizeof(float) > regionEnd) break;
        EE_GET(off, snap);
        off += sizeof(float);
      }

      // ✅ v4: SPLIT (mode + deadband + centerBias + strings)
      if (ver >= 4) {
        if (off + 1 + sizeof(float) + sizeof(float) > regionEnd) break;

        sm = (PotSplitMode)EE::read(off); off += 1;
        EE_GET(off, db);                 off += sizeof(float);
        EE_GET(off, cb);                 off += sizeof(float);

        eeReadStr(off, sSingle, regionEnd);
        eeReadStr(off, sFwd,    regionEnd);
        eeReadStr(off, sBack,   regionEnd);

        // ✅ v5: HYBRIDSEG (en + a + b) después de las 3 strings
        if (ver >= 5) {
          if (off + 3 > regionEnd) break;
          hsegEn = (EE::read(off++) != 0);
          hsegA  = EE::read(off++);
          hsegB  = EE::read(off++);
        }
      }

    } else {
      // v1: solo vals
      centersPos = 0;
      valsPos    = off;
      if (off + cnt * sizeof(float) > regionEnd) break;
      off += cnt * sizeof(float);
      hasC       = false;
      partial    = false;
      snap       = 0.03f;
      // (no split ni hybrid en v1)
    }

    int idx = getPotIndexByPin(pin);
    if (idx >= 0) {
      // Si cnt > 0 cargamos NOTCH, si cnt==0 dejamos notchCount=0 pero aplicamos flags/split/hybrid igualmente
      uint8_t cntClamped = cnt;
      if (cntClamped > MAX_NOTCHES) cntClamped = MAX_NOTCHES;

      potNotchHystPct[idx]    = hyst;
      potPartial[idx]         = (ver >= 3) ? partial : false;
      potSnapWin[idx]         = (ver >= 3) ? snap : 0.03f;

      if (cntClamped > 0) {
        potNotchCount[idx]      = cntClamped;
        potNotchHasCenters[idx] = (ver >= 2) ? hasC : false;

        if (ver >= 2 && hasC) {
          for (uint8_t k = 0; k < cntClamped; k++) {
            uint16_t c = 0;
            EE_GET(centersPos + k * sizeof(uint16_t), c);
            potNotchCenterRaw[idx][k] = c;
          }
          potNotchHasCenters[idx] = true;
        } else {
          potNotchHasCenters[idx] = false;
        }

        for (uint8_t k = 0; k < cntClamped; k++) {
          float v = 0.0f;
          EE_GET(valsPos + k * sizeof(float), v);
          potNotchVals[idx][k] = v;
        }
      } else {
        potNotchCount[idx] = 0;
        potNotchHasCenters[idx] = false;
      }

      potLastNotch[idx] = -1;
      potEmaNorm[idx]   = NAN;
      potEmaRaw[idx]    = NAN;

      // ✅ aplicar SPLIT v4 también cuando cnt==0
      if (ver >= 4) {
        potSplitMode[idx]       = sm;
        potSplitDeadband[idx]   = db;
        potSplitCenterBias[idx] = cb;

        if (potTagSingle[idx]) { free(potTagSingle[idx]); potTagSingle[idx] = nullptr; }
        if (potTagFwd[idx])    { free(potTagFwd[idx]);    potTagFwd[idx]    = nullptr; }
        if (potTagBack[idx])   { free(potTagBack[idx]);   potTagBack[idx]   = nullptr; }

        potTagSingle[idx] = sSingle; sSingle = nullptr;
        potTagFwd[idx]    = sFwd;    sFwd    = nullptr;
        potTagBack[idx]   = sBack;   sBack   = nullptr;
      }

      // ✅ aplicar HYBRIDSEG v5 también cuando cnt==0
      if (ver >= 5) {
        potHybridSegEn[idx] = hsegEn;
        potHybridSegA[idx]  = hsegA;
        potHybridSegB[idx]  = hsegB;
      }

      // ✅ IMPORTANTÍSIMO: normaliza el orden por centro (solo si idx válido)
      sortNotchCentersAndVals(idx);
    }

    // Limpieza si no se asignaron (por ejemplo pin no existe)
    if (sSingle) { free(sSingle); sSingle = nullptr; }
    if (sFwd)    { free(sFwd);    sFwd    = nullptr; }
    if (sBack)   { free(sBack);   sBack   = nullptr; }
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

    // ✅ SPLIT reset también al borrar notch del pin
    potSplitResetRAM(idx);
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

static bool handleOutputInvertConfig(String cmd) {
  char target[24], field[16], state[16];
  int args = sscanf(cmd.c_str() + 4, "%23s %15s %15s", target, field, state);
  if (args < 3) return false;
  if (strcasecmp(field, "OUTINV") != 0) return false;

  int invInt = 0;
  if (!parseValueToInt(String(state), invInt)) {
    enviar(F("ERR CFG OUTINV (usa ON|OFF)"));
    return true;
  }

  bool inverted = invInt != 0;
  uint8_t count = EE::read(0);
  int matched = 0;
  int changed = 0;
  uint8_t canNode = 0, canChannel = 0;
  bool isCanTarget = parseCanRef(target, canNode, canChannel);

  bool isPinTarget = isdigit((unsigned char)target[0]) != 0;
  if (!isPinTarget && (target[0] == 'A' || target[0] == 'a')) {
    isPinTarget =
      isdigit((unsigned char)target[1]) != 0 ||
      strncasecmp(target, "ADS", 3) == 0;
  }

  if (strcasecmp(target, "ALL") == 0) {
    for (int i = 0; i < count; i++) {
      PinConfig cfg{};
      EE_GET(1 + i * sizeof(PinConfig), cfg);
      if (!isOutputConfigType(cfg.type)) continue;

      matched++;
      bool oldInv = pinConfigOutputInverted(cfg);
      if (oldInv == inverted) continue;

      pinConfigSetOutputInverted(cfg, inverted);
      EE_PUT(1 + i * sizeof(PinConfig), cfg);
      changed++;

      if (cfg.type == 2) {
        applyLocalOutputInversionByPin(cfg.pin, inverted);
      }
    }

    if (changed > 0) EE::commit();

    if (matched == 0) {
      enviar(F("ERR CFG ALL OUTINV (sin salidas)"));
    } else {
      enviar(String("OK CFG ALL OUTINV ") + (inverted ? "ON" : "OFF") +
             " CHANGED=" + String(changed));
    }
    return true;
  }

  if (!isCanTarget && !isPinTarget) {
    enviar(String("ERR CFG OUTINV destino invalido: ") + target);
    return true;
  }

  int pinTarget = isCanTarget ? -1 : analogPinFromString(target);
  for (int i = 0; i < count; i++) {
    PinConfig cfg{};
    EE_GET(1 + i * sizeof(PinConfig), cfg);

    bool match = false;
    if (isCanTarget) {
      match = (cfg.type == PIN_TYPE_CAN_OUTPUT &&
               cfg.minIn == canNode &&
               cfg.pin == canChannel);
    } else {
      match = (cfg.type == 2 && cfg.pin == (uint8_t)pinTarget);
    }

    if (!match) continue;

    matched++;
    bool oldInv = pinConfigOutputInverted(cfg);
    if (oldInv != inverted) {
      pinConfigSetOutputInverted(cfg, inverted);
      EE_PUT(1 + i * sizeof(PinConfig), cfg);
      EE::commit();
      changed++;
    }

    if (cfg.type == 2) {
      applyLocalOutputInversionByPin(cfg.pin, inverted);
    }

    enviar(String("OK CFG ") +
           (isCanTarget
              ? canRefToString(canNode, canChannel)
              : pinToStringForDump(cfg.pin, cfg.type)) +
           " OUTINV " + (inverted ? "ON" : "OFF"));
    return true;
  }

  enviar(String("ERR CFG OUTINV no encontrado: ") + target);
  return true;
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

int readIoLiveValue(uint8_t pin, const char* kind) {
  if (strcasecmp(kind, "BUTTON") == 0 ||
      strcasecmp(kind, "SWITCH") == 0 ||
      strcasecmp(kind, "SELECTOR") == 0) {

    IOPin* p = busMCP.getPin(pin, true);
    if (!p) return 0;

    p->pinMode(INPUT_PULLUP);
    return (p->digitalRead() == LOW) ? 1 : 0;
  }

  if (strcasecmp(kind, "OUTPUT") == 0) {
    IOPin* p = busMCP.getPin(pin, false);
    if (!p) return 0;

    return p->digitalRead() == HIGH ? 1 : 0;
  }

  if (strcasecmp(kind, "POT") == 0) {
    if (isADSIndex(pin)) {
      uint8_t ch = adsChannel(pin);
      if (!g_ads_ok || ch > 3) return 0;

      if (adsCacheOk[ch]) return adsRawCache[ch];

      int16_t r = g_ads.readADC_SingleEnded(ch);
      return r < 0 ? 0 : (int)r;
    }

    return analogRead(pin);
  }

  return 0;
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
    startDirectOtaServer();
    Serial.printf("🚀 OTA TCP disponible en %s:%u\n", ETH.localIP().toString().c_str(), kOtaTcpPort);
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
// ✅ tickPots() COMPLETO (corregido: SPLIT sin SPLIT_UNIPOLAR, y con “last” por tag)
// - ADS lee desde cache (adsPollCache())
// - NOTCH + PARTIAL + SNAPWIN igual que lo tenías
// - SPLIT:
//    * DUAL: tagFwd / tagBack (y opcional tagSingle para zona muerta)
//    * SIGNED: un solo tag (-..+)
//    * CENTERED: un solo tag (0..2 con centro=1)
// - THRESH se aplica por “canal/tag” para que al cambiar de lado no se pierdan envíos
void tickPots() {
  const unsigned long now = millis();

  // “Last” independientes para SPLIT (por POT i)
  static float lastFwd[EEPROM_MAX_ENTRIES];
  static float lastBack[EEPROM_MAX_ENTRIES];
  static float lastSingle[EEPROM_MAX_ENTRIES];
  static bool  lastInit[EEPROM_MAX_ENTRIES] = {false};

  // EMA opcional SOLO para split (por POT i)
  static float splitEMA[EEPROM_MAX_ENTRIES];
  static bool  splitEMAInit[EEPROM_MAX_ENTRIES] = {false};

  for (int i = 0; i < potCount; i++) {
    const PinConfig &cfg = potCfgs[i];
    int raw = 0;

    // =======================
    // 1) LECTURA RAW (sin I2C directo) ✅
    // =======================
    if (isADSIndex(cfg.pin)) {
      if (!g_ads_ok) continue;
      uint8_t ch = adsChannel(cfg.pin);
      if (ch > 3) continue;
      if (!adsCacheOk[ch]) continue;      // cache no listo
      raw = adsRawCache[ch];
    } else {
      raw = analogRead(cfg.pin);
    }

    // =======================
    // 2) MAP + EMA (SCALE)
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

      case 1: { // CAMBIO  (+ NOTCH / SPLIT / HYBRIDSEG)
        const bool hasNotches = (potNotchCount[i] > 0);

        // ============================================================
        // A) NOTCH / HÍBRIDO (tu lógica, con HYBRIDSEG añadido)
        // ============================================================
        if (hasNotches) {
          float rawNow = (float)raw;

          // Filtrado crudo para centros / decisions
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
            float denom = (float)(potCfgs[i].maxIn - potCfgs[i].minIn);
            if (denom == 0) denom = 1.0f;

            float norm = clamp01(((float)raw - (float)potCfgs[i].minIn) / denom);

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
            float cell = (potNotchCount[i] > 0) ? (maxR - minR) / (float)potNotchCount[i] : (maxR - minR);
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

          // ✅ HYBRIDSEG: segmento interno lineal entre A..B (índices de notch)
          // - Prioridad: si HYBRIDSEG está activo y raw está entre centros A..B -> lineal entre vals[A]..vals[B]
          // - Luego ya entra tu PARTIAL (colas) y por último el snap/notches
          if (potHybridSegEn[i] && potNotchHasCenters[i] && potNotchCount[i] >= 2) {
            uint8_t a = potHybridSegA[i];
            uint8_t b = potHybridSegB[i];
            if (a >= potNotchCount[i]) a = potNotchCount[i] - 1;
            if (b >= potNotchCount[i]) b = potNotchCount[i] - 1;
            if (a != b) {
              float ca = (float)potNotchCenterRaw[i][a];
              float cb = (float)potNotchCenterRaw[i][b];
              float va = potNotchVals[i][a];
              float vb = potNotchVals[i][b];

              float loC = ca, hiC = cb, loV = va, hiV = vb;
              if (hiC < loC) { float t=loC; loC=hiC; hiC=t; t=loV; loV=hiV; hiV=t; }

              if (rawForIdx >= loC && rawForIdx <= hiC && (hiC - loC) > 0.0001f) {
                float t = (rawForIdx - loC) / (hiC - loC);
                t = clamp01(t);
                outHybrid = mapf(t, 0.0f, 1.0f, loV, hiV);
                useLinear = true; // ya tenemos salida lineal interna
              }
            }
          }

          // PARTIAL (colas) solo si no estamos usando el segmento híbrido interno
          if (!useLinear && potPartial[i]) {
            if (rawForIdx < minCenterF) {
              outHybrid = mapf(rawForIdx, minInF, minCenterF, cfg.minOut, valMinNotch);
              useLinear = true;
            } else if (rawForIdx > maxCenterF) {
              outHybrid = mapf(rawForIdx, maxCenterF, maxInF, valMaxNotch, cfg.maxOut);
              useLinear = true;
            }
          }

          if (!useLinear) {
            int idxFinal = currIdx;
            if (near && nearIdx >= 0 && nearIdx < potNotchCount[i]) idxFinal = nearIdx;
            potLastNotch[i] = (potLastNotch[i] < 0) ? idxFinal : idxFinal;
            outHybrid = potNotchVals[i][potLastNotch[i]];
          }

          // --- 5) THRESH y envío ---
          float prev = potOutLast[i];
          float thr  = potThreshold[i];
          bool enviarAhora = false;

          if (isnan(prev)) enviarAhora = true;
          else if (fabsf(outHybrid - prev) >= thr) enviarAhora = true;

          if (enviarAhora) {
            if (cfg.enviarComoEntero) {
              enviarServidor(String(potParams[i]) + "=" + String((int)lroundf(outHybrid)));
            } else {
              char f[24];
              dtostrf(outHybrid, 0, 3, f);
              char* p = f; while (*p==' ') ++p;
              enviarServidor(String(potParams[i]) + "=" + String(p));
            }
            potOutLast[i] = outHybrid;
            potLastMs[i]  = now;
          }

          debeEnviar = false;
          break;
        }

        // ============================================================
        // B) SIN NOTCH: SPLIT o CAMBIO normal
        // ============================================================
        if (!lastInit[i]) {
          lastFwd[i] = NAN;
          lastBack[i] = NAN;
          lastSingle[i] = NAN;
          lastInit[i] = true;
        }

        const PotSplitMode sm = potSplitMode[i];
        const bool splitOn =
          (sm != SPLIT_OFF) &&
          (
            (potTagSingle[i] && potTagSingle[i][0]) ||
            (potTagFwd[i]    && potTagFwd[i][0])    ||
            (potTagBack[i]   && potTagBack[i][0])
          );

        if (splitOn) {
          // 1) normalizar raw a 0..1 con cfg.minIn/maxIn (crudo)
          float denom = (float)(cfg.maxIn - cfg.minIn);
          if (denom == 0.0f) denom = 1.0f;
          float normRaw = ((float)raw - (float)cfg.minIn) / denom;
          normRaw = clamp01(normRaw);

          // 2) centro + deadband
          float center = potSplitCenterBias[i];
          if (center < 0.0f) center = 0.0f;
          if (center > 1.0f) center = 1.0f;

          float db = potSplitDeadband[i];
          if (db < 0.0f) db = 0.0f;
          if (db > 0.4f) db = 0.4f;

          float lo = center - db;
          float hi = center + db;
          if (lo < 0.0f) lo = 0.0f;
          if (hi > 1.0f) hi = 1.0f;

          auto sendValueIfThresh = [&](const char* t, float x, float &lastRef) {
            const float thr = potThreshold[i];
            bool doSend = false;
            if (isnan(lastRef)) doSend = true;
            else if (fabsf(x - lastRef) >= thr) doSend = true;
            if (!doSend) return;

            if (cfg.enviarComoEntero) {
              enviar(String(t) + "=" + String((int)lroundf(x)));
            } else {
              char f[24];
              dtostrf(x, 0, 3, f);
              char* p = f; while (*p==' ') ++p;
              enviar(String(t) + "=" + String(p));
            }
            lastRef = x;
            potLastMs[i] = now;
          };

          const bool inDead = (normRaw >= lo && normRaw <= hi);

          if (sm == SPLIT_DUAL) {
            if (inDead) {
              // zona muerta: preferimos tagSingle si existe; si no, mandamos 0 a ambos
              if (potTagSingle[i] && potTagSingle[i][0]) {
                const char* tag = potTagSingle[i];
                float outSend = mapf(0.0f, 0.0f, 1.0f, cfg.minOut, cfg.maxOut);

                if (!splitEMAInit[i]) { splitEMA[i] = outSend; splitEMAInit[i] = true; }
                else splitEMA[i] = (1.0f - cfg.suavizado) * splitEMA[i] + cfg.suavizado * outSend;
                outSend = splitEMA[i];

                sendValueIfThresh(tag, outSend, lastSingle[i]);
              } else {
                // parar ambos
                const char* tf = (potTagFwd[i]  && potTagFwd[i][0])  ? potTagFwd[i]  : potParams[i];
                const char* tb = (potTagBack[i] && potTagBack[i][0]) ? potTagBack[i] : potParams[i];
                float z = mapf(0.0f, 0.0f, 1.0f, cfg.minOut, cfg.maxOut);

                sendValueIfThresh(tf, z, lastFwd[i]);
                sendValueIfThresh(tb, z, lastBack[i]);
              }

            } else if (normRaw > hi) {
              // adelante
              const char* tag = (potTagFwd[i] && potTagFwd[i][0]) ? potTagFwd[i] : potParams[i];

              float t = (normRaw - hi) / (1.0f - hi);
              t = clamp01(t);
              float outSend = mapf(t, 0.0f, 1.0f, cfg.minOut, cfg.maxOut);

              if (!splitEMAInit[i]) { splitEMA[i] = outSend; splitEMAInit[i] = true; }
              else splitEMA[i] = (1.0f - cfg.suavizado) * splitEMA[i] + cfg.suavizado * outSend;
              outSend = splitEMA[i];

              sendValueIfThresh(tag, outSend, lastFwd[i]);

              // opcional: “apaga” back cuando vas forward
              if (potTagBack[i] && potTagBack[i][0]) {
                float z = mapf(0.0f, 0.0f, 1.0f, cfg.minOut, cfg.maxOut);
                sendValueIfThresh(potTagBack[i], z, lastBack[i]);
              }

            } else { // normRaw < lo
              // atrás
              const char* tag = (potTagBack[i] && potTagBack[i][0]) ? potTagBack[i] : potParams[i];

              float t = (lo - normRaw) / (lo - 0.0f);
              t = clamp01(t);
              float outSend = mapf(t, 0.0f, 1.0f, cfg.minOut, cfg.maxOut);

              if (!splitEMAInit[i]) { splitEMA[i] = outSend; splitEMAInit[i] = true; }
              else splitEMA[i] = (1.0f - cfg.suavizado) * splitEMA[i] + cfg.suavizado * outSend;
              outSend = splitEMA[i];

              sendValueIfThresh(tag, outSend, lastBack[i]);

              // opcional: “apaga” fwd cuando vas back
              if (potTagFwd[i] && potTagFwd[i][0]) {
                float z = mapf(0.0f, 0.0f, 1.0f, cfg.minOut, cfg.maxOut);
                sendValueIfThresh(potTagFwd[i], z, lastFwd[i]);
              }
            }

            debeEnviar = false;
            break;
          }

          // ---- SINGLE SIGNED / CENTERED (usa potTagSingle o potParams) ----
          const char* tag = (potTagSingle[i] && potTagSingle[i][0]) ? potTagSingle[i] : potParams[i];

          float outSend = 0.0f;

          if (sm == SPLIT_SINGLE_SIGNED) {
            // signed in [-1..+1], deadband => 0
            float signed01;
            if (inDead) {
              signed01 = 0.0f;
            } else if (normRaw > hi) {
              float t = (normRaw - hi) / (1.0f - hi);
              t = clamp01(t);
              signed01 = +t;
            } else { // normRaw < lo
              float t = (lo - normRaw) / (lo - 0.0f);
              t = clamp01(t);
              signed01 = -t;
            }

            outSend = mapf(signed01, -1.0f, 1.0f, cfg.minOut, cfg.maxOut);

          } else { // SPLIT_SINGLE_CENTERED
            // centered in [0..2], center=1, deadband => 1
            float centered;
            if (inDead) {
              centered = 1.0f;
            } else if (normRaw >= hi) {
              float t = (normRaw - hi) / (1.0f - hi);
              t = clamp01(t);
              centered = 1.0f + t;      // 1..2
            } else { // normRaw <= lo
              float t = (lo - normRaw) / (lo - 0.0f);
              t = clamp01(t);
              centered = 1.0f - t;      // 0..1
            }

            outSend = mapf(centered, 0.0f, 2.0f, cfg.minOut, cfg.maxOut);
          }

          // EMA opcional del split single
          if (!splitEMAInit[i]) { splitEMA[i] = outSend; splitEMAInit[i] = true; }
          else splitEMA[i] = (1.0f - cfg.suavizado) * splitEMA[i] + cfg.suavizado * outSend;
          outSend = splitEMA[i];

          // THRESH + envío
          {
            const float thr = potThreshold[i];
            bool doSend = false;
            if (isnan(lastSingle[i])) doSend = true;
            else if (fabsf(outSend - lastSingle[i]) >= thr) doSend = true;

            if (doSend) {
              if (cfg.enviarComoEntero) {
                enviar(String(tag) + "=" + String((int)lroundf(outSend)));
              } else {
                char f[24];
                dtostrf(outSend, 0, 3, f);
                char* p = f; while (*p==' ') ++p;
                enviar(String(tag) + "=" + String(p));
              }
              lastSingle[i] = outSend;
              potLastMs[i] = now;
            }
          }

          debeEnviar = false;
          break;
        }

        // ============================================================
        // C) CAMBIO normal (sin NOTCH, sin SPLIT)
        // ============================================================
        {
          bool enviarCambio = false;
          float prev = potOutLast[i];
          float thr  = potThreshold[i];

          if (isnan(prev)) enviarCambio = true;
          else if (fabsf(outVal - prev) >= thr) enviarCambio = true;

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
          break;
        }
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
    // 4) ENVÍO FINAL (CONTINUO / INTERVALO)
    // =======================
    if (debeEnviar) {
      if (potNotchCount[i] > 0) {
        int idxN = (potLastNotch[i] < 0) ? 0 : potLastNotch[i];
        if (idxN < 0) idxN = 0;
        if (idxN >= potNotchCount[i]) idxN = potNotchCount[i] - 1;

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


// ===================== RENAME helpers =====================

static bool renamePinParam(uint8_t pin, const char* newName) {
  uint8_t count = EE::read(0);
  if (count == 0xFF) return false;

  for (int i = 0; i < count; i++) {
    PinConfig cfg;
    EE_GET(1 + i * sizeof(PinConfig), cfg);
    if (cfg.pin == pin) {
      strncpy(cfg.param, newName, sizeof(cfg.param));
      cfg.param[sizeof(cfg.param) - 1] = '\0';
      EE_PUT(1 + i * sizeof(PinConfig), cfg);
      EE::commit();
      return true;
    }
  }
  return false;
}

static int renameAllByName(const char* oldName, const char* newName) {
  uint8_t count = EE::read(0);
  if (count == 0xFF) return 0;

  int changed = 0;
  for (int i = 0; i < count; i++) {
    PinConfig cfg;
    EE_GET(1 + i * sizeof(PinConfig), cfg);

    if (strcasecmp(cfg.param, oldName) == 0) {
      strncpy(cfg.param, newName, sizeof(cfg.param));
      cfg.param[sizeof(cfg.param) - 1] = '\0';
      EE_PUT(1 + i * sizeof(PinConfig), cfg);
      changed++;
    }
  }
  if (changed) EE::commit();
  return changed;
}

void tickIoStates() {
  unsigned long now = millis();

  // Frecuencia máxima de comprobación
  if (now - lastIoStateMs < 30) return;
  lastIoStateMs = now;

  // ===================== BUTTONS =====================
  for (int i = 0; i < buttonCount; i++) {
    if (!buttonPins[i]) continue;

    int value = buttonPins[i]->digitalRead() == LOW ? 1 : 0;

    if (value != lastButtonState[i]) {
      lastButtonState[i] = value;

      enviar(
        String("IO.STATE ") +
        String(buttonPinNum[i]) +
        " BUTTON " +
        String(value)
      );
    }
  }

  // ===================== SWITCHES =====================
  for (int i = 0; i < switchCount; i++) {
    if (!switchPins[i]) continue;

    int value = switchPins[i]->digitalRead() == LOW ? 1 : 0;

    if (value != lastSwitchState[i]) {
      lastSwitchState[i] = value;

      enviar(
        String("IO.STATE ") +
        String(switchPinNum[i]) +
        " SWITCH " +
        String(value)
      );
    }
  }

  // ===================== OUTPUTS =====================
  for (int i = 0; i < outputCount; i++) {
    if (!outputPins[i]) continue;

    int value = outputPins[i]->digitalRead() == HIGH ? 1 : 0;

    if (value != lastOutputState[i]) {
      lastOutputState[i] = value;

      enviar(
        String("IO.STATE ") +
        String(outputPinNum[i]) +
        " OUTPUT " +
        String(value)
      );
    }
  }

  // ===================== POTS =====================
  for (int i = 0; i < potCount; i++) {
    const PinConfig& cfg = potCfgs[i];

    int value = 0;

    if (isADSIndex(cfg.pin)) {
      uint8_t ch = adsChannel(cfg.pin);

      if (!g_ads_ok || ch > 3) continue;

      if (adsCacheOk[ch]) {
        value = adsRawCache[ch];
      } else {
        int16_t r = g_ads.readADC_SingleEnded(ch);
        value = r < 0 ? 0 : (int)r;
      }
    } else {
      value = analogRead(cfg.pin);
    }

    // Evita ruido analógico
    if (lastPotState[i] == -9999 || abs(value - lastPotState[i]) > 8) {
      lastPotState[i] = value;

      enviar(
        String("IO.STATE ") +
        pinToStringForDump(cfg.pin, 3) +
        " POT " +
        String(value)
      );
    }
  }
}

void handleIoCommand(const String& cmd) {
  char pinStr[16];
  char kind[16];
  char state[8];
  int pin = -1;
  int value = 0;

  if (sscanf(cmd.c_str(), "IO.WATCH %15s %15s %7s", pinStr, kind, state) == 3) {
    pin = analogPinFromString(pinStr);

    if (pin < 0 || pin > 255) {
      enviar(String("IO.ERROR PIN ") + pinStr);
      return;
    }

    if (strcasecmp(state, "ON") == 0 || strcmp(state, "1") == 0) {
      ioWatchEnabled = true;
      ioWatchPin = (uint8_t)pin;

      strncpy(ioWatchKind, kind, sizeof(ioWatchKind));
      ioWatchKind[sizeof(ioWatchKind) - 1] = '\0';

      ioWatchLastValue = -9999;

      int result = readIoLiveValue(ioWatchPin, ioWatchKind);
      ioWatchLastValue = result;

      String pinText = pinToStringForIo(ioWatchPin, ioWatchKind);
      enviar(String("IO.WATCH OK ") + pinText + " " + ioWatchKind + " ON");
      enviar(String("IO.STATE ") + pinText + " " + ioWatchKind + " " + result);
      return;
    }

    if (strcasecmp(state, "OFF") == 0 || strcmp(state, "0") == 0) {
      ioWatchEnabled = false;
      ioWatchLastValue = -9999;

      String pinText = pinToStringForIo((uint8_t)pin, kind);
      enviar(String("IO.WATCH OK ") + pinText + " " + kind + " OFF");
      return;
    }

    enviar(F("IO.ERROR WATCH"));
    return;
  }

  if (sscanf(cmd.c_str(), "IO.READ %15s %15s", pinStr, kind) == 2) {
    pin = analogPinFromString(pinStr);

    if (pin < 0 || pin > 255) {
      enviar(String("IO.ERROR PIN ") + pinStr);
      return;
    }

    int result = readIoLiveValue((uint8_t)pin, kind);
    String pinText = pinToStringForIo((uint8_t)pin, kind);
    enviar(String("IO.STATE ") + pinText + " " + kind + " " + result);
    return;
  }

  if (sscanf(cmd.c_str(), "IO.WRITE %d %d", &pin, &value) == 2) {
    if (pin < 0 || pin > 127) {
      enviar(String("IO.ERROR PIN ") + pin);
      return;
    }

    IOPin* p = busMCP.getPin(pin, false);
    if (!p) {
      enviar(String("IO.ERROR PIN ") + pin);
      return;
    }

    p->pinMode(OUTPUT);
    p->digitalWrite(value ? HIGH : LOW);

    enviar(String("IO.STATE ") + pin + " OUTPUT " + (value ? 1 : 0));
    return;
  }

  enviar(String("IO.ERROR CMD ") + cmd);
}

void tickIoWatch() {
  if (!ioWatchEnabled) return;

  unsigned long now = millis();
  if (now - ioWatchLastMs < 20) return;
  ioWatchLastMs = now;

  int value = readIoLiveValue(ioWatchPin, ioWatchKind);

  if (value != ioWatchLastValue) {
    ioWatchLastValue = value;
    String pinText = pinToStringForIo(ioWatchPin, ioWatchKind);
    enviar(String("IO.STATE ") + pinText + " " + ioWatchKind + " " + value);
  }
}

static bool parseCanRef(const char* s, uint8_t& node, uint8_t& channel) {
  if (!s) return false;

  if (strncasecmp(s, "CAN", 3) != 0) return false;

  const char* p = s + 3;
  if (!isdigit((unsigned char)*p)) return false;

  int n = atoi(p);

  const char* colon = strchr(p, ':');
  if (!colon) return false;

  int ch = atoi(colon + 1);

  if (n < 0 || n > 255 || ch < 0 || ch > 255) return false;

  node = (uint8_t)n;
  channel = (uint8_t)ch;
  return true;
}

static String canRefToString(uint8_t node, uint8_t channel) {
  return String("CAN") + String(node) + ":" + String(channel);
}

static bool saveCanConfig(const String& tipo, uint8_t node, uint8_t channel, const char* param) {
  uint8_t count = EE::read(0);
  if (count >= EEPROM_MAX_ENTRIES) {
    enviar(F("❌ EEPROM llena"));
    return false;
  }

  for (int i = 0; i < count; i++) {
    PinConfig cfg{};
    EE_GET(1 + i * sizeof(PinConfig), cfg);

    if ((cfg.type == PIN_TYPE_CAN_BUTTON ||
         cfg.type == PIN_TYPE_CAN_SWITCH ||
         cfg.type == PIN_TYPE_CAN_OUTPUT) &&
        cfg.minIn == node &&
        cfg.pin == channel) {
      enviar(F("❌ CAN ya configurado en ese nodo/canal"));
      return false;
    }
  }

  PinConfig cfg{};

  if (tipo.equalsIgnoreCase("BUTTON")) {
    cfg.type = PIN_TYPE_CAN_BUTTON;
  } else if (tipo.equalsIgnoreCase("SWITCH")) {
    cfg.type = PIN_TYPE_CAN_SWITCH;
  } else if (tipo.equalsIgnoreCase("OUTPUT")) {
    cfg.type = PIN_TYPE_CAN_OUTPUT;
  } else {
    enviar(F("❌ Tipo CAN no soportado"));
    return false;
  }

  cfg.pin = channel;
  cfg.minIn = node;
  cfg.maxIn = channel;
  cfg.minOut = 0;
  cfg.maxOut = 1;
  cfg.suavizado = 0;
  cfg.modoEnvio = 1;
  cfg.intervalo = 0;
  cfg.enviarComoEntero = true;

  strncpy(cfg.param, param, sizeof(cfg.param));
  cfg.param[sizeof(cfg.param) - 1] = '\0';

  EE_PUT(1 + count * sizeof(PinConfig), cfg);
  EE::write(0, count + 1);
  EE::commit();

  enviar(String("✅ CAN añadido: ") +
         tipo + " CAN" + String(node) + ":" + String(channel) +
         " " + String(param));

  return true;
}

static void handleCanInput(uint8_t node, uint8_t channel, uint8_t value, const char* resolvedCommand = nullptr) {
  uint8_t count = EE::read(0);

  for (int i = 0; i < count && i < EEPROM_MAX_ENTRIES; i++) {
    PinConfig cfg{};
    EE_GET(1 + i * sizeof(PinConfig), cfg);

    if ((cfg.type == PIN_TYPE_CAN_BUTTON || cfg.type == PIN_TYPE_CAN_SWITCH) &&
        cfg.minIn == node &&
        cfg.pin == channel) {

      String kv = String(cfg.param) + "=" + String(value ? 1 : 0);

      enviar(kv);
      enviarServidor(kv);

      return;
    }
  }

  if (resolvedCommand && resolvedCommand[0]) {
    String kv = String(resolvedCommand) + "=" + String(value ? 1 : 0);
    enviar(kv);
    enviarServidor(kv);
    return;
  }

  enviar(String("⚠️ CAN INPUT sin asignar: CAN") +
         String(node) + ":" + String(channel) +
         "=" + String(value));
}

static bool tryCanOutputByName(const char* name, int value) {
  if (!name || !name[0]) return false;

  uint8_t count = EE::read(0);

  for (int i = 0; i < count && i < EEPROM_MAX_ENTRIES; i++) {
    PinConfig cfg{};
    EE_GET(1 + i * sizeof(PinConfig), cfg);

    if (cfg.type == PIN_TYPE_CAN_OUTPUT &&
        strcasecmp(cfg.param, name) == 0) {

      uint8_t node = (uint8_t)cfg.minIn;
      uint8_t ch   = cfg.pin;
      uint8_t val  = (uint8_t)logicalToPhysicalOutputValue(
        value ? 1 : 0,
        pinConfigOutputInverted(cfg)
      );

      canManager.sendOutputSet(node, ch, val);

      enviar(String("📤 CAN OUTPUT: CAN") +
             String(node) + ":" + String(ch) +
             "=" + String(val));

      return true;
    }
  }

  bool usedProfileInvert = false;
  if (canManager.sendOutputByCommand(name, (uint8_t)(value ? 1 : 0), &usedProfileInvert)) {
    enviar(String("📤 CAN PROFILE OUTPUT: ") +
           String(name) +
           "=" + String(value ? 1 : 0) +
           (usedProfileInvert ? " INV" : ""));
    return true;
  }

  return false;
}

// ========= handleLine (comandos) =========
void handleLine(const char* command, const char* value) {
  if (!command || !*command) return;

  // --- cleanup helpers (para no dejar flags colgados en returns tempranos) ---
  auto _hend = [&]() {
    bloqueado = false;
  #if defined(MODO_ETHERNET)
    bloqueaTCP = false;
  #endif
  };

#define HRET() do { _hend(); return; } while (0)

#if defined(MODO_ETHERNET)
  // Mientras procesamos un comando, suspendemos el procesamiento TCP (evita “interferencias”)
  bloqueaTCP = true;
#endif

  String cmd = String(command);
  String val = value ? String(value) : "";
  String fullCmd;

  cmd.trim();
  val.trim();
  fullCmd = val.length() ? (cmd + " " + val) : cmd;

  // ===================== IO live wizard =====================
  if (cmd.startsWith("IO.")) {
    handleIoCommand(fullCmd);
    HRET();
  }

  // ===================== MODBUS passthrough =====================
  if (cmd.startsWith("MB.") || cmd.startsWith("MODBUS.")) {
    handleMbCommand(fullCmd);
    HRET();
  }

  // ===================== ETH helpers =====================
#if defined(MODO_ETHERNET)
  if (cmd.equalsIgnoreCase("ETH.STATUS")) {
    IPAddress ip = ETH.localIP();
    bool linkUp  = ETH.linkUp();
    bool hasIp   = (ip != IPAddress((uint32_t)0));
    bool dhcpConfigured = EE::getDhcpEnabled();

    String configuredMode = dhcpConfigured ? "DHCP" : "STATIC";
    String mode = ethStaticFallbackUsed ? "STATIC" : (hasIp ? "DHCP" : "UNKNOWN");

    IPAddress srvIp = serverDiscovery.serverIp();
    bool hasSrv     = serverDiscovery.hasServerIp();

    String s = "ETH.STATUS OUT=";
    s += (ethOutEnabled ? "ON" : "OFF");
    s += " LINK=";
    s += (linkUp ? "UP" : "DOWN");
    s += " CFG=";
    s += configuredMode;
    s += " MODE=";
    s += mode;
    s += " IP=";
    s += ip.toString();
    s += " CLIENT=";
    s += (serverDiscovery.connected() ? "UP" : "DOWN");
    s += " SERVER.IP=";
    s += (hasSrv ? srvIp.toString() : "0.0.0.0");
    s += " SERVER.PORT=";
    s += String(5090); // o kClientServerPort
    enviar(s);
    HRET();
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
    HRET();
  }

  if (cmd.startsWith("ETH.MODE")) {
    String arg = val;
    if (!arg.length()) {
      int sp = cmd.indexOf(' ');
      if (sp > 0) arg = cmd.substring(sp + 1);
    }
    arg.trim();
    arg.toUpperCase();

    if (arg == "DHCP") {
      EE::setDhcpEnabled(true);
      EE::commit();

      enviar(F("OK ETH.MODE DHCP"));
      enviar(F("🔄 Reiniciando para aplicar configuración Ethernet..."));

      delay(500);
      ESP.restart();
      HRET();
    }

    if (arg == "STATIC") {
      IPAddress savedIp = EE::getStaticIP();
      if (savedIp == IPAddress(0, 0, 0, 0)) {
        enviar(F("ERR ETH.MODE STATIC (usa antes ETH.SETIP <ip>)"));
        HRET();
      }

      EE::setDhcpEnabled(false);
      EE::commit();

      enviar(String("OK ETH.MODE STATIC IP=") + savedIp.toString());
      enviar(F("🔄 Reiniciando para aplicar configuración Ethernet..."));

      delay(500);
      ESP.restart();
      HRET();
    }

    enviar(F("ERR ETH.MODE (usa DHCP|STATIC)"));
    HRET();
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
    HRET();
  }

  // 🚀 NUEVO COMANDO PARA CAMBIAR LA IP ESTÁTICA
  if (cmd.startsWith("ETH.SETIP")) {
    String val = "";
    int sp = cmd.indexOf(' ');
    if (sp > 0) val = cmd.substring(sp + 1);
    val.trim();
    if (val.length() == 0) {
      enviar(F("❌ Uso: ETH.SETIP 192.168.1.100"));
      HRET();
    }
  
    IPAddress newIp;
  
    if (!newIp.fromString(val)) {
      enviar(F("❌ Error: IP no válida. Ejemplo: ETH.SETIP 192.168.1.50"));
      HRET();
    }
  
    EE::setStaticIP(newIp);
    EE::setDhcpEnabled(false);
    EE::commit();
  
    enviar(String("💾 IP estática guardada: ") + newIp.toString());
    enviar(F("🔄 Reiniciando para aplicar configuración Ethernet..."));
  
    delay(500);
    ESP.restart();
    HRET();
  }

    // ===================== Modo CONFIG =====================
    if (cmd.equalsIgnoreCase("#CONFIG")) {
      modoConfig = true;
      bloqueado  = false;
      enviar(F("✅ MODO CONFIG ACTIVADO"));
      HRET();
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
        while (1) {}
  #else
        ESP.restart();
  #endif
      } else {
        enviar(F("⚠️ AP activo, no se reinicia"));
        loadConfigFromEEPROM();
        notchLoadAllFromEEPROM();
      }
      HRET();
  }

  //PING

  if (cmd.equalsIgnoreCase("PING")) {
    enviar("PONG");
    return;
  }

  // ===================== POT.SPLIT.* (dual throttle/brake o palanca única) =====================
  // Requiere #CONFIG
  if (cmd.startsWith("POT.SPLIT")) {
    if (!modoConfig) { enviar(F("❌ POT.SPLIT requiere #CONFIG")); HRET(); }

    String full = cmd;
    if (val.length()) { full += " "; full += val; }
    full.trim();

    auto needIdx = [&](const char* pinS)->int{
      int pin = analogPinFromString(pinS);
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) enviar(F("❌ POT.SPLIT: pin no es POT"));
      return idx;
    };

    if (full.startsWith("POT.SPLIT.TAGS")) {
      char pinS[16], t1[64], t2[64];
      if (sscanf(full.c_str(), "POT.SPLIT.TAGS %15s %63s %63s", pinS, t1, t2) == 3) {
        int idx = needIdx(pinS);
        if (idx < 0) HRET();

        if (potTagFwd[idx])  free(potTagFwd[idx]);
        if (potTagBack[idx]) free(potTagBack[idx]);
        potTagFwd[idx]  = strdup(t1);
        potTagBack[idx] = strdup(t2);

        potSplitMode[idx] = SPLIT_DUAL;
        enviar(String("✅ OK POT.SPLIT.TAGS ") + pinS + " FWD=" + t1 + " BACK=" + t2);
        notchSaveAllToEEPROM();
        HRET();
      }
      enviar(F("❌ Formato: POT.SPLIT.TAGS <pin> <tagFwd> <tagBack>"));
      HRET();
    }

    if (full.startsWith("POT.SPLIT.TAG")) {
      char pinS[16], t[64];
      if (sscanf(full.c_str(), "POT.SPLIT.TAG %15s %63s", pinS, t) == 2) {
        int idx = needIdx(pinS);
        if (idx < 0) HRET();

        if (potTagSingle[idx]) free(potTagSingle[idx]);
        potTagSingle[idx] = strdup(t);

        if (potSplitMode[idx] == SPLIT_OFF) potSplitMode[idx] = SPLIT_SINGLE_SIGNED;

        enviar(String("✅ OK POT.SPLIT.TAG ") + pinS + " TAG=" + t);
        notchSaveAllToEEPROM();
        HRET();
      }
      enviar(F("❌ Formato: POT.SPLIT.TAG <pin> <tagSingle>"));
      HRET();
    }

    if (full.startsWith("POT.SPLIT.DB")) {
      char pinS[16]; float db=0;
      if (sscanf(full.c_str(), "POT.SPLIT.DB %15s %f", pinS, &db) == 2) {
        int idx = needIdx(pinS);
        if (idx < 0) HRET();
        if (db < 0) db = 0;
        if (db > 0.3f) db = 0.3f;
        potSplitDeadband[idx] = db;
        enviar(String("✅ OK POT.SPLIT.DB ") + pinS + " " + String(db,3));
        notchSaveAllToEEPROM();
        HRET();
      }
      enviar(F("❌ Formato: POT.SPLIT.DB <pin> <0..0.3>"));
      HRET();
    }

    if (full.startsWith("POT.SPLIT.CBIAS")) {
      char pinS[16]; float cb=0.5f;
      if (sscanf(full.c_str(), "POT.SPLIT.CBIAS %15s %f", pinS, &cb) == 2) {
        int idx = needIdx(pinS);
        if (idx < 0) HRET();
        if (cb < 0) cb = 0;
        if (cb > 1.0f) cb = 1.0f;
        potSplitCenterBias[idx] = cb;
        enviar(String("✅ OK POT.SPLIT.CBIAS ") + pinS + " " + String(cb,3));
        notchSaveAllToEEPROM();
        HRET();
      }
      enviar(F("❌ Formato: POT.SPLIT.CBIAS <pin> <0..1>"));
      HRET();
    }

    // POT.SPLIT <pin> OFF|DUAL|SIGNED|CENTERED
    {
      char pinS[16], modeS[16];
      if (sscanf(full.c_str(), "POT.SPLIT %15s %15s", pinS, modeS) == 2) {
        int idx = needIdx(pinS);
        if (idx < 0) HRET();

        String ms = String(modeS); ms.toUpperCase();
        if (ms=="OFF") {
          potSplitMode[idx] = SPLIT_OFF;
          enviar(String("✅ OK POT.SPLIT ") + pinS + " OFF");
          notchSaveAllToEEPROM();
          HRET();
        }
        if (ms=="DUAL") {
          potSplitMode[idx] = SPLIT_DUAL;
          enviar(String("✅ OK POT.SPLIT ") + pinS + " DUAL");
          notchSaveAllToEEPROM();
          HRET();
        }
        if (ms=="SIGNED") {
          potSplitMode[idx] = SPLIT_SINGLE_SIGNED;
          enviar(String("✅ OK POT.SPLIT ") + pinS + " SIGNED");
          notchSaveAllToEEPROM();
          HRET();
        }
        if (ms=="CENTERED") {
          potSplitMode[idx] = SPLIT_SINGLE_CENTERED;
          enviar(String("✅ OK POT.SPLIT ") + pinS + " CENTERED");
          notchSaveAllToEEPROM();
          HRET();
        }

        enviar(F("❌ POT.SPLIT: modo inválido (OFF|DUAL|SIGNED|CENTERED)"));
        HRET();
      }
    }

    enviar(F("❌ POT.SPLIT: comando inválido"));
    HRET();
  }

  // ===================== RENAME commands =====================
  if (modoConfig && cmd.startsWith("RENAME.")) {

    if (cmd.startsWith("RENAME.PIN")) {
      char pinStr[16], newName[40];

      if (sscanf(cmd.c_str(), "RENAME.PIN %15s %39s", pinStr, newName) == 2) {
        int pin = analogPinFromString(pinStr);
        if (pin < 0 || pin > 255) {
          enviar(F("❌ RENAME.PIN: pin inválido"));
          HRET();
        }

        if (renamePinParam((uint8_t)pin, newName)) {
          loadConfigFromEEPROM();
          notchLoadAllFromEEPROM();
          modbusRefreshWindow();
          enviar(String("✅ OK RENAME.PIN ") + pin + " -> " + newName);
        } else {
          enviar(F("❌ RENAME.PIN: pin no encontrado"));
        }
        HRET();
      }

      enviar(F("❌ Formato: RENAME.PIN <pin> <newName>"));
      HRET();
    }

    if (cmd.startsWith("RENAME.NAME")) {
      char oldName[40], newName[40];

      if (sscanf(cmd.c_str(), "RENAME.NAME %39s %39s", oldName, newName) == 2) {
        int n = renameAllByName(oldName, newName);

        if (n > 0) {
          loadConfigFromEEPROM();
          notchLoadAllFromEEPROM();
          modbusRefreshWindow();
          enviar(String("✅ OK RENAME.NAME '") + oldName +
                 "' -> '" + newName + "' (" + n + " cambios)");
        } else {
          enviar(F("❌ RENAME.NAME: no hubo coincidencias"));
        }
        HRET();
      }

      enviar(F("❌ Formato: RENAME.NAME <oldName> <newName>"));
      HRET();
    }

    enviar(F("❌ RENAME.* desconocido"));
    HRET();
  }

  if (cmd.equalsIgnoreCase("#CLEAR")) {
    clearEEPROMIfNeeded();
    notchEraseRegion();
    mbClearEEPROM();
    enviar(F("✅ EEPROM borrada correctamente."));
    delay(100);
    enviar(F("#READY"));
    modbusRefreshWindow();
    HRET();
  }

  if (cmd.equalsIgnoreCase("#NVSWIPE")) {
    nvsWipeAllAndReboot();
    HRET();
  }

  if (cmd.equalsIgnoreCase("#BOARD?")) {
    enviar(String("BOARD ") + getBoardType());
    HRET();
  }

  // ===================== SEL.* (nuevo API) =====================
  if (cmd.startsWith("SEL.")) {
    if (cmd.equalsIgnoreCase("SEL.CLEAR")) {
      if (!modoConfig) { enviar(F("❌ SEL.CLEAR requiere #CONFIG")); HRET(); }

      eepromDeleteAllByType(4);
      clearSelectorsRAM();
      loadConfigFromEEPROM();
      notchLoadAllFromEEPROM();
      modbusRefreshWindow();

      enviar(F("✅ OK SEL.CLEAR"));
      HRET();
    }

    if (cmd.equalsIgnoreCase("SEL.DUMP")) {
      enviar(F("BEGIN SEL"));
      dumpSelectorsAsSEL_ADD();
      enviar(F("END SEL"));
      HRET();
    }

    if (cmd.startsWith("SEL.DELPIN")) {
      if (!modoConfig) { enviar(F("❌ SEL.DELPIN requiere #CONFIG")); HRET(); }
      char pinStr[16];
      if (sscanf(cmd.c_str(), "SEL.DELPIN %15s", pinStr) == 1) {
        int pin = analogPinFromString(pinStr);
        if (pin < 0 || pin > 127) { enviar(F("❌ SEL.DELPIN: pin inválido")); HRET(); }

        bool ok = eepromDeletePinIfType((uint8_t)pin, 4);
        clearSelectorsRAM();
        loadConfigFromEEPROM();
        notchLoadAllFromEEPROM();
        modbusRefreshWindow();

        enviar(ok ? F("✅ OK SEL.DELPIN") : F("❌ SEL.DELPIN: pin no encontrado"));
        HRET();
      }
      enviar(F("❌ Formato: SEL.DELPIN <pin>"));
      HRET();
    }

    if (cmd.startsWith("SEL.ADD")) {
      if (!modoConfig) { enviar(F("❌ SEL.ADD requiere #CONFIG")); HRET(); }

      char name[40], pinStr[16], valStr[16];
      int n = sscanf(cmd.c_str(), "SEL.ADD %39s %15s %15s", name, pinStr, valStr);
      if (n == 3) {
        int pin = analogPinFromString(pinStr);
        int v   = atoi(valStr);

        if (pin < 0 || pin > 127) { enviar(F("❌ SEL.ADD: pin inválido (solo MCP 0..127)")); HRET(); }

        int sidx = getOrCreateSelector(name);
        if (sidx < 0) { enviar(F("❌ SEL.ADD: sin espacio en selectors[]")); HRET(); }

        IOPin* pinObj = busMCP.getPin(pin, true);
        if (!pinObj) { enviar(F("❌ SEL.ADD: pinObj nulo")); HRET(); }

        pinObj->pinMode(INPUT_PULLUP);
        selectors[sidx]->add(pinObj, (int16_t)v);

        char v1[16]; snprintf(v1, sizeof(v1), "%d", v);
        savePinConfig("SELECTOR", pin, name, v1, "0");

        enviar(String("✅ OK SEL.ADD ") + name + " PIN=" + String(pin) + " VAL=" + String(v));
        HRET();
      }

      enviar(F("❌ Formato: SEL.ADD <name> <pin> <val>"));
      HRET();
    }

    enviar(F("❌ SEL.* desconocido (usa SEL.ADD / SEL.CLEAR / SEL.DELPIN / SEL.DUMP)"));
    HRET();
  }

  // ===================== #DELETEPIN =====================
  if (modoConfig && cmd.startsWith("#DELETEPIN")) {
    String pinArg = cmd.substring(10);
    pinArg.trim();

    uint8_t canNode = 0;
    uint8_t canChannel = 0;
    bool deleteCan = parseCanRef(pinArg.c_str(), canNode, canChannel);
    int pinToDelete = deleteCan ? (int)canChannel : analogPinFromString(pinArg.c_str());
    int count = EE::read(0);
    bool eliminado = false;
    PinConfig deletedCfg{};

    for (int i = 0; i < count; i++) {
      PinConfig cfg;
      EE_GET(1 + i * sizeof(PinConfig), cfg);
      bool matches = false;

      if (deleteCan) {
        matches =
          (cfg.type == PIN_TYPE_CAN_BUTTON ||
           cfg.type == PIN_TYPE_CAN_SWITCH ||
           cfg.type == PIN_TYPE_CAN_OUTPUT) &&
          cfg.minIn == canNode &&
          cfg.pin == canChannel;
      } else {
        matches = (int)cfg.pin == pinToDelete;
      }

      if (matches) {
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

      String deletedRef =
        deleteCan ?
        canRefToString(canNode, canChannel) :
        pinToStringForDump((uint8_t)pinToDelete, deletedCfg.type);

      enviar(String("🗑️ Configuración eliminada del pin ") +
            deletedRef);
      enviar(String("DELETED_PIN ") +
            deletedRef);

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
        writeLocalOutputByIndex(i, 0);
      }

    } else {
      enviar(String("❌ No se encontró configuración para el pin ") +
              pinToStringForDump((uint8_t)pinToDelete, 3));
      enviar(String("NOT_FOUND_PIN ") +
              pinToStringForDump((uint8_t)pinToDelete, 3));
    }

    HRET();
  }

  // ===================== #SCANPINS =====================
  if (modoConfig && cmd.startsWith("#SCANPINS")) {
    int chips = busMCP.getDetectedChips();
    int totalPins = chips * 16;
    enviar(String("🔎 MCP detectados: ") + chips);
    enviar(String("🔌 Pines totales disponibles: ") + totalPins);
    HRET();
  }

  // ===================== NOTCH commands =====================
  if (modoConfig && cmd.startsWith("NOTCH")) {
    String s = cmd;
    s.trim();
    int sp1 = s.indexOf(' ');
    if (sp1 < 0) { enviar(F("ERR NOTCH")); HRET(); }

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
                " CENTERS " + String(potNotchHasCenters[i] ? "YES" : "NO"));

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
      HRET();
    }

    if (op.equalsIgnoreCase("SAVEALL")) {
      if (notchSaveAllToEEPROM()) enviar(F("OK NOTCH SAVEALL"));
      HRET();
    }

    if (op.equalsIgnoreCase("LOADALL")) {
      notchLoadAllFromEEPROM();
      enviar(F("OK NOTCH LOADALL"));
      HRET();
    }

    if (op.equalsIgnoreCase("PARTIAL")) {
      int sp = sub.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH PARTIAL")); HRET(); }
      String rest = sub.substring(sp+1);
      rest.trim();
      int sp2b = rest.indexOf(' ');
      if (sp2b < 0) { enviar(F("ERR NOTCH PARTIAL")); HRET(); }
      String pinStr = rest.substring(0, sp2b);
      pinStr.trim();
      String onoff = rest.substring(sp2b+1);
      onoff.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH PARTIAL pin")); HRET(); }

      bool on = onoff.equalsIgnoreCase("ON") || onoff == "1" || onoff.equalsIgnoreCase("TRUE");
      potPartial[idx] = on;
      enviar(String("OK NOTCH PARTIAL ") +
              pinToStringForDump((uint8_t)pin,3) +
              " " + (on? "ON":"OFF"));
      HRET();
    }

    // ===================== NOTCH HYBRIDSEG =====================
    if (op.equalsIgnoreCase("HYBRIDSEG")) {
    
      int sp = sub.indexOf(' ');
      if (sp < 0) {
        enviar(F("ERR NOTCH HYBRIDSEG"));
        HRET();
      }
    
      String rest = sub.substring(sp + 1);
      rest.trim();
    
      int sp2 = rest.indexOf(' ');
      if (sp2 < 0) {
        enviar(F("ERR NOTCH HYBRIDSEG"));
        HRET();
      }
    
      String pinStr = rest.substring(0, sp2);
      pinStr.trim();
    
      String rest2 = rest.substring(sp2 + 1);
      rest2.trim();
    
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) {
        enviar(F("ERR NOTCH HYBRIDSEG pin"));
        HRET();
      }
    
      // ---------- OFF ----------
      if (rest2.equalsIgnoreCase("OFF")) {
        potHybridSegEn[idx] = false;
      
        enviar(String("OK NOTCH HYBRIDSEG ") +
               pinToStringForDump((uint8_t)pin, 3) +
               " OFF");
      
        notchSaveAllToEEPROM();
        HRET();
      }
    
      // ---------- ON: A B ----------
      int a = -1, b = -1;
      if (sscanf(rest2.c_str(), "%d %d", &a, &b) != 2) {
        enviar(F("ERR NOTCH HYBRIDSEG formato (usa OFF o <A> <B>)"));
        HRET();
      }
    
      if (a < 0 || b < 0 || a >= potNotchCount[idx] || b >= potNotchCount[idx] || a == b) {
        enviar(F("ERR NOTCH HYBRIDSEG índices inválidos"));
        HRET();
      }
    
      potHybridSegEn[idx] = true;
      potHybridSegA[idx]  = (uint8_t)a;
      potHybridSegB[idx]  = (uint8_t)b;
    
      enviar(String("OK NOTCH HYBRIDSEG ") +
             pinToStringForDump((uint8_t)pin, 3) +
             " " + String(a) + " " + String(b));
    
      notchSaveAllToEEPROM();
      HRET();
    }

    if (op.equalsIgnoreCase("SNAPWIN")) {
      int sp = sub.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH SNAPWIN")); HRET(); }
      String rest = sub.substring(sp+1);
      rest.trim();
      int sp2b = rest.indexOf(' ');
      if (sp2b < 0) { enviar(F("ERR NOTCH SNAPWIN")); HRET(); }
      String pinStr = rest.substring(0, sp2b);
      pinStr.trim();
      String pctStr = rest.substring(sp2b+1);
      pctStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH SNAPWIN pin")); HRET(); }

      float pct = pctStr.toFloat();
      if (pct < 0.0f) pct = 0.0f;
      if (pct > 0.3f) pct = 0.3f;
      potSnapWin[idx] = pct;

      enviar(String("OK NOTCH SNAPWIN ") +
              pinToStringForDump((uint8_t)pin,3) +
              " " + String(pct,3));
      HRET();
    }

    if (sp2 < 0) { enviar(F("ERR NOTCH params")); HRET(); }

    String rest = sub.substring(sp2+1);
    rest.trim();

    if (op.equalsIgnoreCase("RAW")) {
      String pinStr = rest;
      pinStr.trim();
      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) { enviar(F("ERR NOTCH RAW pin")); HRET(); }

      int raw = 0;
      if (isADSIndex(potCfgs[poti].pin)) {
        if (!g_ads_ok) { enviar(F("ERR NOTCH RAW ADS no listo")); HRET(); }
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
      HRET();
    }

    if (op.equalsIgnoreCase("CLEAR")) {
      int pin = analogPinFromString(rest.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH CLEAR pin")); HRET(); }

      potNotchCount[idx]=0;
      potLastNotch[idx]=-1;
      potEmaNorm[idx]=NAN;
      potEmaRaw[idx]=NAN;
      potNotchHasCenters[idx]=false;

      enviar("OK NOTCH CLEAR " + pinToStringForDump((uint8_t)pin,3));
      HRET();
    }

    if (op.equalsIgnoreCase("ADD")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH ADD")); HRET(); }
      String pinStr = rest.substring(0, sp);
      pinStr.trim();
      String valStr = rest.substring(sp+1);
      valStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH ADD pin")); HRET(); }
      if (potNotchCount[idx] >= MAX_NOTCHES) { enviar(F("ERR NOTCH ADD max 30")); HRET(); }

      float v = valStr.toFloat();
      potNotchVals[idx][ potNotchCount[idx]++ ] = v;

      enviar("OK NOTCH ADD " +
              pinToStringForDump((uint8_t)pin,3) +
              " " + String(v,3));
      HRET();
    }

    if (op.equalsIgnoreCase("ADDHERE")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH ADDHERE")); HRET(); }
      String pinStr = rest.substring(0, sp);
      pinStr.trim();
      String valStr = rest.substring(sp+1);
      valStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH ADDHERE pin")); HRET(); }
      if (potNotchCount[idx] >= MAX_NOTCHES) { enviar(F("ERR NOTCH ADDHERE max 30")); HRET(); }

      int raw = 0;
      if (isADSIndex(potCfgs[idx].pin)) {
        if (!g_ads_ok) { enviar(F("ERR NOTCH ADDHERE ADS no listo")); HRET(); }
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
      HRET();
    }

    if (op.equalsIgnoreCase("CAP")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH CAP")); HRET(); }
      String pinStr = rest.substring(0, sp);
      pinStr.trim();
      String idxStr = rest.substring(sp+1);
      idxStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) { enviar(F("ERR NOTCH CAP pin")); HRET(); }

      int k = idxStr.toInt();
      if (k < 0 || k >= potNotchCount[poti]) { enviar(F("ERR NOTCH CAP idx")); HRET(); }

      int raw = 0;
      if (isADSIndex(potCfgs[poti].pin)) {
        if (!g_ads_ok) { enviar(F("ERR NOTCH CAP ADS no listo")); HRET(); }
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
      HRET();
    }

    if (op.equalsIgnoreCase("CENT")) {
      int spA = rest.indexOf(' ');
      int spB = rest.indexOf(' ', spA+1);
      if (spA < 0 || spB < 0) { enviar(F("ERR NOTCH CENT")); HRET(); }

      String pinStr = rest.substring(0, spA);
      pinStr.trim();
      String idxStr = rest.substring(spA+1, spB);
      idxStr.trim();
      String rawStr = rest.substring(spB+1);
      rawStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) { enviar(F("ERR NOTCH CENT pin")); HRET(); }

      int k = idxStr.toInt();
      if (k < 0 || k >= potNotchCount[poti]) { enviar(F("ERR NOTCH CENT idx")); HRET(); }

      int raw = rawStr.toInt();
      if (raw < 0) raw = 0;
      if (raw > 65535) raw = 65535;

      potNotchCenterRaw[poti][k] = (uint16_t)raw;
      potNotchHasCenters[poti]   = true;

      enviar("OK NOTCH CENT " +
              pinToStringForDump((uint8_t)pin,3) +
              " IDX " + String(k) +
              " RAW " + String(raw));
      HRET();
    }

    if (op.equalsIgnoreCase("VAL")) {
      int spA = rest.indexOf(' ');
      int spB = rest.indexOf(' ', spA+1);
      if (spA < 0 || spB < 0) { enviar(F("ERR NOTCH VAL")); HRET(); }

      String pinStr = rest.substring(0, spA);
      pinStr.trim();
      String idxStr = rest.substring(spA+1, spB);
      idxStr.trim();
      String valStr = rest.substring(spB+1);
      valStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int poti = getPotIndexByPin((uint8_t)pin);
      if (poti < 0) { enviar(F("ERR NOTCH VAL pin")); HRET(); }

      int k = idxStr.toInt();
      if (k < 0 || k >= potNotchCount[poti]) { enviar(F("ERR NOTCH VAL idx")); HRET(); }

      potNotchVals[poti][k] = valStr.toFloat();

      enviar("OK NOTCH VAL " +
              pinToStringForDump((uint8_t)pin,3) +
              " IDX " + String(k) +
              " VAL " + String(potNotchVals[poti][k],3));
      HRET();
    }

    if (op.equalsIgnoreCase("HYST")) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { enviar(F("ERR NOTCH HYST")); HRET(); }

      String pinStr = rest.substring(0, sp);
      pinStr.trim();
      String valStr = rest.substring(sp+1);
      valStr.trim();

      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH HYST pin")); HRET(); }

      float pct = valStr.toFloat();
      if (pct < 0)    pct=0;
      if (pct>0.3f) pct=0.3f;
      potNotchHystPct[idx] = pct;

      enviar("OK NOTCH HYST " +
              pinToStringForDump((uint8_t)pin,3) +
              " " + String(pct,3));
      HRET();
    }

    if (op.equalsIgnoreCase("DUMP")) {
      String pinStr = rest;
      int pin = analogPinFromString(pinStr.c_str());
      int idx = getPotIndexByPin((uint8_t)pin);
      if (idx < 0) { enviar(F("ERR NOTCH DUMP pin")); HRET(); }

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

      HRET();
    }

    if (op.equalsIgnoreCase("SAVE")) {
      int pin = analogPinFromString(rest.c_str());
      if (notchSaveOneToEEPROM((uint8_t)pin))
        enviar("OK NOTCH SAVE " + pinToStringForDump((uint8_t)pin,3));
      else
        enviar(F("ERR NOTCH SAVE"));
      HRET();
    }

    if (op.equalsIgnoreCase("DEL")) {
      int pin = analogPinFromString(rest.c_str());
      if (notchDeleteFromEEPROM((uint8_t)pin))
        enviar("OK NOTCH DEL " + pinToStringForDump((uint8_t)pin,3));
      else
        enviar(F("ERR NOTCH DEL"));
      HRET();
    }

    enviar(F("ERR NOTCH ?"));
    HRET();
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

      if (cfg.type == PIN_TYPE_CAN_BUTTON ||
          cfg.type == PIN_TYPE_CAN_SWITCH ||
          cfg.type == PIN_TYPE_CAN_OUTPUT) {
        String linea = "ADD ";
        linea += (cfg.type == PIN_TYPE_CAN_BUTTON ? "BUTTON " :
                  cfg.type == PIN_TYPE_CAN_SWITCH ? "SWITCH " :
                  "OUTPUT ");
        linea += canRefToString((uint8_t)cfg.minIn, cfg.pin);
        linea += " ";
        linea += String(cfg.param) + " 0 1";
        enviar(linea);
        if (cfg.type == PIN_TYPE_CAN_OUTPUT) {
          enviar(String("CFG ") +
                 canRefToString((uint8_t)cfg.minIn, cfg.pin) +
                 " OUTINV " +
                 (pinConfigOutputInverted(cfg) ? "ON" : "OFF"));
        }
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

      if (cfg.type == 2) {
        enviar(String("CFG ") +
               pinToStringForDump(cfg.pin, cfg.type) +
               " OUTINV " +
               (pinConfigOutputInverted(cfg) ? "ON" : "OFF"));
      }

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

          if (potSplitMode[idx] != SPLIT_OFF) {
            String p = pinToStringForDump(cfg.pin, cfg.type);

            String mode;
            if      (potSplitMode[idx] == SPLIT_DUAL)            mode = "DUAL";
            else if (potSplitMode[idx] == SPLIT_SINGLE_SIGNED)   mode = "SIGNED";
            else if (potSplitMode[idx] == SPLIT_SINGLE_CENTERED) mode = "CENTERED";
            else                                                 mode = "OFF";

            enviar("POT.SPLIT " + p + " " + mode);
            enviar("POT.SPLIT.DB " + p + " " + String(potSplitDeadband[idx], 3));
            enviar("POT.SPLIT.CBIAS " + p + " " + String(potSplitCenterBias[idx], 3));

            if (potSplitMode[idx] == SPLIT_DUAL) {
              if (potTagFwd[idx] && potTagBack[idx]) {
                enviar(String("POT.SPLIT.TAGS ") + p + " " + potTagFwd[idx] + " " + potTagBack[idx]);
              }
              if (potTagSingle[idx] && potTagSingle[idx][0]) {
                enviar(String("POT.SPLIT.TAG ") + p + " " + potTagSingle[idx]);
              }
            } else {
              if (potTagSingle[idx] && potTagSingle[idx][0]) {
                enviar(String("POT.SPLIT.TAG ") + p + " " + potTagSingle[idx]);
              }
            }
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

    enviar("MB.WINDOW base/size ya informada");

    enviar("BEGIN MB");
    handleMbCommand("MB.DUMP");
    enviar("END MB");

    enviar(F("✅ DUMP COMPLETO FIN"));
    HRET();
  }

  // ===================== ADD (legacy) =====================
  if (modoConfig && cmd.startsWith("ADD")) {
    char tipo[20], pinStr[16], param[40], v1[16], v2[16];
    int args = sscanf(cmd.c_str() + 4, "%19s %15s %39s %15s %15s", tipo, pinStr, param, v1, v2);

    if (args == 5) {
      // ---- ADD CAN ----
      // Ejemplos:
      // ADD BUTTON CAN1:0 PZB_WACHSAM 0 1
      // ADD SWITCH CAN1:1 LZB_ON 0 1
      // ADD OUTPUT CAN1:0 PZB_LED 0 1
      uint8_t canNode = 0;
      uint8_t canChannel = 0;

      if (parseCanRef(pinStr, canNode, canChannel)) {
        if (strcasecmp(tipo, "BUTTON") == 0 ||
            strcasecmp(tipo, "SWITCH") == 0 ||
            strcasecmp(tipo, "OUTPUT") == 0) {

          saveCanConfig(String(tipo), canNode, canChannel, param);

          enviar(String("✅ CAN ") + tipo +
                 " CAN" + String(canNode) + ":" + String(canChannel) +
                 " -> " + String(param));
          HRET();
        }

        enviar(String("❌ Tipo CAN no soportado en ADD → ") + tipo);
        HRET();
      }

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

    HRET();
  }

  // ===================== CFG (POT params) =====================
  if (modoConfig && cmd.startsWith("CFG")) {
    if (handleOutputInvertConfig(cmd)) {
      HRET();
    }
    updatePotParam(cmd);
    HRET();
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

      int outVal = 0;
      if (parseValueToInt(v, outVal) && writeLocalOutputByName(key.c_str(), outVal)) {
          enviar("✅ ACK: " + key + "=" + v);
          HRET();
      }

      // ---- Salidas CAN por nombre ----
      // Ejemplo: PZB_LED=1 -> CAN OUTPUT configurado como PZB_LED
      if (parseValueToInt(v, outVal)) {
        if (tryCanOutputByName(key.c_str(), outVal)) {
          enviar("✅ ACK CAN: " + key + "=" + v);
          HRET();
        }
      }

      String mb = buildModbusSetFromKV(key, v);
      if (mb.length()) {
        handleMbCommand(mb);
        HRET();
      }

      enviar("❌ NACK: " + key + "=" + v);
      HRET();
    }
  }

  // ===================== Fallback outputs por "command value" =====================
  bool handled = false;
  int outVal = 0;
  if (value && parseValueToInt(String(value), outVal)) {
    if (writeLocalOutputByName(command, outVal)) handled = true;
    else if (tryCanOutputByName(command, outVal)) handled = true;
  }
  if (handled) enviar("✅ ACK: " + String(command));
  else         enviar("❌ NACK: " + String(command));

  HRET();

#undef HRET
}

// ========= AP / LED =========
void handleAPButton() {
  const unsigned long now = millis();
  const unsigned long DEBOUNCE_MS = 40;
  const unsigned long LONG_MS     = longPressTime; // 2000

  static int stable = HIGH;
  static int lastRaw = HIGH;
  static unsigned long lastEdgeMs = 0;
  static bool longFired = false;
  static unsigned long pressStartMs = 0;

  int raw = digitalRead(PIN_BTN_AP); // GPIO15 (LOW = pressed)

  // debounce
  if (raw != lastRaw) {
    lastRaw = raw;
    lastEdgeMs = now;
  }
  if ((now - lastEdgeMs) > DEBOUNCE_MS && raw != stable) {
    stable = raw;

    if (stable == LOW) {
      // pressed
      pressStartMs = now;
      longFired = false;
    } else {
      // released
      longFired = false;
    }
  }

  // long press
  if (stable == LOW && !longFired && (now - pressStartMs) >= LONG_MS) {
    longFired = true;

    // Toggle AP
    if (!apActive) enableAP();
    else           disableAP();

    oledRequest();
  }
}

static void flushLedIfDirty() {
  if (!ledDirty) return;

  const unsigned long now = millis();
  if (now - lastLedFlush < 15) return; // limita refresco

  pixels.show();
  ledDirty = false;
  lastLedFlush = now;
}

void enableAP() {
  if (apActive) return;

#if defined(MODO_WIFI_AP)
  // Fuerza modo AP (o AP+STA si quieres mantener compatibilidad)
  WiFi.mode(WIFI_AP);          // <- clave
  delay(100);

  int channel = 1;             // 1..13
  bool hidden = false;
  int maxConn = 4;

  bool ok = WiFi.softAP(apSsid, apPassword, channel, hidden, maxConn);
  delay(200);

  IPAddress ip = WiFi.softAPIP();
  Serial.printf("🌐 AP enable: ok=%d mode=%d ssid=%s ip=%s\n",
                ok ? 1 : 0, (int)WiFi.getMode(), WiFi.softAPSSID().c_str(), ip.toString().c_str());

  if (ok) {
    startDirectOtaServer();
    Serial.printf("🚀 OTA TCP: %s:%u\n", ip.toString().c_str(), kOtaTcpPort);
    Serial.printf("🔐 AP password: %s\n", apPassword);
  }

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

static void toolReboot() {
#ifdef ESP32
  ESP.restart();
#endif
}

// ======================= setup/loop =======================
void setup() {
  #if defined(MODO_SERIAL)
    Serial.begin(SERIAL_BAUD);
  #endif

  EE::begin(EE_SIZE_BYTES);
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
    potTagFwd[i] = nullptr;
    potTagBack[i] = nullptr;
    potTagSingle[i] = nullptr;
    potHybridSegEn[i] = false;
    potHybridSegA[i]  = 0;
    potHybridSegB[i]  = 0;
    potSplitResetRAM(i);
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
  I2CBUS1.begin(OLED_SDA, OLED_SCL);

  //scanBus(I2CBUS1, "OLED");
  #if defined(SSD1306)
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Serial.printf("oledOK=%d\n", oledOK ? 1 : 0);
  if (oledOK) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("OLED OK");
    display.println("INICIANDO..");
    display.display();
  }
  #elif defined(SSD1309)
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
  #endif
  
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
  pixels.show();
  ETH.setHostname("EASYSIM_ETH");

#if defined(MODO_ETHERNET) && defined(ESP32)
  WiFi.mode(WIFI_OFF);
  delay(50);

  Network.onEvent(onEthEvent);

  bool dhcpEnabled = EE::getDhcpEnabled();
  IPAddress savedIp = EE::getStaticIP();

  Serial.println("🌐 Iniciando Ethernet...");

  if (!ETH.begin(ETH_PHY_W5500, 1, CS_W5500, W5500_IRQ, W5500_RST, SPIBUS)) {
    Serial.println("❌ ETH.begin() falló");
  } else {
    Serial.println("✅ ETH.begin() ok");

    if (!dhcpEnabled && savedIp != IPAddress(0, 0, 0, 0)) {
      IPAddress gateway(savedIp[0], savedIp[1], savedIp[2], 1);
      IPAddress subnet(255, 255, 255, 0);
      IPAddress dns(8, 8, 8, 8);

      Serial.print("📥 IP estática guardada encontrada: ");
      Serial.println(savedIp);

      if (ETH.config(savedIp, gateway, subnet, dns)) {
        ethStaticFallbackUsed = true;
        Serial.print("✅ IP estática aplicada → ");
        Serial.println(ETH.localIP());
      } else {
        ethStaticFallbackUsed = false;
        Serial.println("❌ No se pudo aplicar la IP estática guardada");
      }
    } else {
      ethStaticFallbackUsed = false;
      Serial.println("🌐 DHCP habilitado, esperando IP...");
    }
  }

  {
    unsigned long t0 = millis();

    while (ETH.localIP() == IPAddress(0, 0, 0, 0) && (millis() - t0) < 5000) {
      delay(100);
    }

    if (ETH.localIP() == IPAddress(0, 0, 0, 0)) {
      Serial.println("⚠️ No se obtuvo IP por DHCP ni estática guardada");

      IPAddress fallbackIp(192, 168, 1, 177);
      IPAddress fallbackGw(192, 168, 1, 1);
      IPAddress fallbackMask(255, 255, 255, 0);
      IPAddress fallbackDns(8, 8, 8, 8);

      ETH.config(fallbackIp, fallbackGw, fallbackMask, fallbackDns);
      ethStaticFallbackUsed = true;

      Serial.print("ℹ️ IP fallback aplicada → ");
      Serial.println(ETH.localIP());
    } else {
      Serial.print("🌐 IP final → ");
      Serial.println(ETH.localIP());

      if (ethStaticFallbackUsed) {
        Serial.println("📌 Modo IP: ESTÁTICA");
      } else {
        Serial.println("📌 Modo IP: DHCP");
      }
    }
  }

  confServer = new NetworkServer(5000);
  ethernetInterface = new EthernetInterface(
      *confServer,
      [](const char* c, const char* v){ handleLine(c,v); }
  );
  ethernetInterface->setUseEth(true);
  ethernetInterface->begin();

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

    mbRegisterAllTags();
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
  Serial.printf("EE_SIZE=%u\n", EE_SIZE_BYTES);
  Serial.printf("PINCFG_END=%u\n", (unsigned)(1 + EEPROM_MAX_ENTRIES * sizeof(PinConfig)));
  Serial.printf("NOTCH: base=%u size=%u end=%u\n", NOTCH_REGION_BASE, NOTCH_REGION_SIZE, NOTCH_REGION_BASE+NOTCH_REGION_SIZE);
  Serial.printf("MODBUS: base=%u size=%u end=%u\n", MB_REGION_BASE, MB_REGION_SIZE, MB_REGION_BASE+MB_REGION_SIZE);

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

  //CANBUS Devices

  canManager.begin(CAN_TX_PIN, CAN_RX_PIN, 500000);
  g_can_started = true;
  canManager.registerProfile(&g_can_profile_asfad);

  canManager.onInput([](uint8_t node, uint8_t channel, uint8_t value, const char* command) {
    handleCanInput(node, channel, value, command);
  });

  canManager.onHello([](uint8_t node) {
    g_can_seen_hello = true;
    g_can_last_hello_node = node;
    g_can_last_hello_ms = millis();
    enviar(String("✅ CAN HELLO node=") + String(node));
  });

  canManager.onHeartbeat([](uint8_t node) {
    g_can_seen_heartbeat = true;
    g_can_last_hb_node = node;
    g_can_last_hb_ms = millis();
  });

  canManager.onOutputAck([](uint8_t node, uint8_t channel, uint8_t value) {
    g_can_seen_ack = true;
    g_can_last_ack_node = node;
    g_can_last_ack_channel = channel;
    g_can_last_ack_value = value;
    g_can_last_ack_ms = millis();
    enviar(String("✅ CAN ACK CAN") +
            String(node) + ":" + String(channel) +
            "=" + String(value));
  });

  #if defined(SSD1309)
  IPAddress menuIp = EE::getStaticIP();
  IPAddress menuGw = EE::getStaticGateway();
  IPAddress menuMask = EE::getStaticMask();

  if (menuIp == IPAddress(0, 0, 0, 0)) {
    menuIp = IPAddress(192, 168, 1, 177);
  }
  if (menuGw == IPAddress(0, 0, 0, 0)) {
    menuGw = IPAddress(menuIp[0], menuIp[1], menuIp[2], 1);
  }
  if (menuMask == IPAddress(0, 0, 0, 0)) {
    menuMask = IPAddress(255, 255, 255, 0);
  }

  menu.begin();
  menu.setNetCurrent(EE::getDhcpEnabled(), menuIp, menuGw, menuMask);
  menu.setOnApply(menuApplyNet);
  menu.setOnExit(menuClosed);
  menu.setStatusProvider(menuStatusProvider);
  menu.setEthStatusProvider(menuEthStatusDetails);
  menu.setCanStatusProvider(menuCanStatusDetails);
  menu.setIoStatusProvider(menuIoStatusDetails);
  menu.setOutputCountProvider(menuOutputCountProvider);
  menu.setOutputLabelProvider(menuOutputLabelProvider);
  menu.setOutputStateProvider(menuOutputStateProvider);
  menu.setOutputSetFn(menuOutputSet);
  menu.setAllOutInvGetter(menuAllOutInvGetter);
  menu.setAllOutInvSetter(menuAllOutInvSetter);
  menu.setOnToolDump(menuToolDump);
  menu.setOnToolScanI2C(menuToolScanI2C);
  menu.setOnToolReboot(toolReboot);

  bool pcfOk = pcf.begin();
  Serial.printf("pcfOK=%d\n", pcfOk ? 1 : 0);
  menuPcfReady = pcfOk;
  if (pcfOk) {
    menu.beginPCF(pcf, 0, 1, 2, 3, 4, 5, true);
    Serial.println("✅ Menu SSD1309 listo (mantener ENT 5s para abrir)");
  } else {
    Serial.println("⚠️ PCF8574 no detectado, menu no activado");
  }
  #endif

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
    writeLocalOutputByIndex(i, 0);
    enviar(String("register(") + outputParams[i] + ")");
  }
  for (int i = 0; i < selectorCount; i++) {
    if (selectors[i]) selectors[i]->begin();
  }
}

void loop() {
  // 1) refresca cache ADS (I2C corto y no bloqueante)
  adsPollCache();
  handleAPButton();

  // 2) refresca OLED (I2C, no bloqueante)
  pollButtonsESP32();
  #if defined(SSD1309)
  pollMenuEnterHold();
  if (!menu.isActive()) {
    oledMaybeDrawChangeOnly();
  }
  #else
  oledMaybeDrawChangeOnly();
  #endif
  tickIoStates();
  tickIoWatch();
  canManager.loop();

  // 3) networking / serial
  #if defined(MODO_ETHERNET)
    if (ethernetInterface) ethernetInterface->update();
    if (ETH.linkUp()) serverDiscovery.poll();
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
  pollDirectOtaServer();

  #if defined(SSD1309)
  if (menu.isActive()) {
    menu.tick();
    return;
  }
  #endif

  // 4) IO (MCP) + POTS
  if (!modoConfig && !bloqueado) {

    for (int i = 0; i < switchCount; i++) switches[i]->update();
    for (int i = 0; i < buttonCount; i++) buttons[i]->update();
    for (int i = 0; i < selectorCount; i++) if (selectors[i]) selectors[i]->update();
    tickPots();
  }

  flushLedIfDirty();
}
