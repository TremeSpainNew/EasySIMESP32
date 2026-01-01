/*#include <io/OledStatus.h>
#include <core/I2CGuard.h>
#include <core/AppState.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_ADDR   0x3C
#define OLED_W      128
#define OLED_H      64
#define OLED_RESET  -1

static Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

static bool oledOK = false;
static String oledLast;
static unsigned long lastOledDraw = 0;
static const unsigned long OLED_MIN_MS = 300;
static const unsigned long OLED_KEEPALIVE_MS = 1500;

static String buildOledFrame() {
  bool linkUp = ETH.linkUp();
  IPAddress ip = ETH.localIP();
  bool hasIp = (ip != IPAddress((uint32_t)0));

  const char* mode;
  if (!linkUp) mode = "NO LINK";
  else if (!hasIp)  mode = "NO IP";
  else              mode = (gApp.ethStaticFallbackUsed ? "STATIC" : "DHCP");

  String s;
  s.reserve(200);
  s += "BOARD:"; s += gApp.boardType; s += "\n";
  s += "ETH:";   s += mode;           s += "\n";
  s += "IP:";    s += (hasIp ? ip.toString() : "0.0.0.0"); s += "\n";
  s += "AP:";    s += (gApp.apActive ? WiFi.softAPIP().toString() : "OFF");
  return s;
}

void oledInit() {
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOK) return;

  if (i2cTryLock(50)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("OLED OK");
    display.println(gApp.boardType);
    display.display();
    i2cUnlock();
  }
}

bool oledIsOk() { return oledOK; }

void oledMaybeDraw() {
  if (!oledOK) return;

  unsigned long now = millis();
  if (now - lastOledDraw < OLED_MIN_MS) return;

  String frame = buildOledFrame();
  bool changed = (frame != oledLast);
  bool keepalive = (now - lastOledDraw) >= OLED_KEEPALIVE_MS;
  if (!changed && !keepalive) return;

  if (!i2cTryLock(0)) return; // no bloquear

  oledLast = frame;
  lastOledDraw = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print(frame);
  display.display();

  i2cUnlock();
}
*/