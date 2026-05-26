#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <PCF8574.h>   // RobTillaart/PCF8574

#ifdef ESP32
  #include <Preferences.h>
#endif

class Menu {
public:
  // ========= Callbacks =========
  struct NetCfg {
    bool dhcp = true;
    uint8_t ip[4]   = {192,168,0,50};
    uint8_t gw[4]   = {192,168,0,1};
    uint8_t mask[4] = {255,255,255,0};
  };

  using OnApplyFn        = void (*)(const NetCfg& cfg);
  using OnExitFn         = void (*)();
  using StatusProviderFn = String (*)();

  // NUEVO: Tools callbacks
  using OnToolFn = void (*)();

  explicit Menu(U8G2& d);

  // ========= Lifecycle =========
  void begin();
  void open();
  void close();
  void tick();

  bool isActive() const { return active_; }

  // ========= Inputs via PCF8574 =========
  void beginPCF(PCF8574& pcf,
                uint8_t up, uint8_t down,
                uint8_t left, uint8_t right,
                uint8_t ok, uint8_t back,
                bool activeLow);

  void beginPCF(TwoWire& wire, uint8_t addr,
                uint8_t up, uint8_t down,
                uint8_t left, uint8_t right,
                uint8_t ok, uint8_t back,
                bool activeLow);

  // ========= Data =========
  void setNetCurrent(bool dhcp, const IPAddress& ip, const IPAddress& gw, const IPAddress& mask);

  // ========= Hooks =========
  void setOnApply(OnApplyFn fn) { onApply_ = fn; }
  void setOnExit(OnExitFn fn)   { onExit_  = fn; }
  void setStatusProvider(StatusProviderFn fn) { statusProvider_ = fn; }

  // NUEVO: Tools hooks
  void setOnToolDump(OnToolFn fn)    { onToolDump_ = fn; }
  void setOnToolScanI2C(OnToolFn fn) { onToolScanI2C_ = fn; }
  void setOnToolReboot(OnToolFn fn)  { onToolReboot_ = fn; }

private:
  // ========= Pages =========
  enum Page : uint8_t { PAGE_STATUS=0, PAGE_NET=1, PAGE_TOOLS=2 };
  enum NetSub : uint8_t { NET_SUB_0=0, NET_SUB_1=1 };
  enum NetField : uint8_t { F_MODE=0, F_IP=1, F_GW=2, F_MASK=3, F_APPLY=4, F_BACK=5 };

  enum KeyIdx : uint8_t { K_UP=0, K_DOWN=1, K_LEFT=2, K_RIGHT=3, K_OK=4, K_BACK=5 };

  struct KeyState {
    bool stable = false;          // estado estable (debounced)
    unsigned long lastChangeMs = 0;
    unsigned long pressedMs = 0;
    unsigned long lastRepeatMs = 0;
  };

  // ========= UI =========
  void handleStatus(bool evUp, bool evDown, bool evLeft, bool evRight, bool evOk, bool evBack);
  void handleNet(bool evUp, bool evDown, bool evLeft, bool evRight, bool evOk, bool evBack);
  void handleTools(bool evUp, bool evDown, bool evLeft, bool evRight, bool evOk, bool evBack);

  void drawStatus();
  void drawNet();
  void drawTools();

  void drawHeader(const char* title);

  // ========= NET helpers =========
  void netSetSub(NetSub s);
  uint8_t incWrap(uint8_t v, int delta);
  bool isIpField(NetField f) const { return (f==F_IP || f==F_GW || f==F_MASK); }

  // campos disponibles según DHCP/STATIC y subpágina
  int  netFieldCount(NetSub sub) const;
  NetField netFieldAt(NetSub sub, int idx) const;
  int  netFieldIndex(NetSub sub, NetField f) const;
  void netSelectFirstValidField();

  // ========= Keys =========
  void readKeys(bool& evUp, bool& evDown, bool& evLeft, bool& evRight, bool& evOk, bool& evBack);
  bool keyPressedFromRaw(uint8_t raw, uint8_t idx) const;

  // ========= IP draw helper =========
  void drawIp4(int x, int yBase, const uint8_t a[4], bool editing, uint8_t activeOctet);

  // ========= Storage =========
  void loadNetFromStorage();
  void saveNetToStorage();

private:
  U8G2& d_;

  bool active_ = false;
  Page page_ = PAGE_STATUS;
  bool g_confirmReboot = false;
  bool g_confirmYes    = false;

  // STATUS menu
  uint8_t statusSel_ = 0; // 0=NETWORK,1=TOOLS,2=EXIT

  // NET
  NetCfg cfg_;
  NetSub netSub_ = NET_SUB_0;
  NetField netField_ = F_MODE;
  uint8_t octet_ = 0;
  bool editing_ = false;  // modo edición de octetos (IP/GW/MASK)

  // TOOLS
  uint8_t toolsSel_ = 0;

  // Draw flag
  bool dirty_ = true;

  // Callbacks
  OnApplyFn onApply_ = nullptr;
  OnExitFn onExit_ = nullptr;
  StatusProviderFn statusProvider_ = nullptr;

  // Tools callbacks
  OnToolFn onToolDump_ = nullptr;
  OnToolFn onToolScanI2C_ = nullptr;
  OnToolFn onToolReboot_ = nullptr;

  // PCF
  bool pcfEnabled_ = false;
  bool pcfActiveLow_ = true;
  uint8_t pcfAddr_ = 0x20;
  uint8_t pcfPins_[6] = {0,1,2,3,4,5};

  // Instancia PCF:
  PCF8574* pcfExternal_ = nullptr;              // puntero al PCF activo
  PCF8574  pcfOwned_    = PCF8574(0x20, &Wire); // si lo construye Menu

  KeyState keys_[6];
  uint8_t lastRaw_ = 0xFF;

#ifdef ESP32
  Preferences prefs_;
#endif
};
