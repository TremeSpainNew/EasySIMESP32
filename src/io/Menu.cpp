#include "io/Menu.h"

// ---------------- util ----------------
static void ipToStr(char* out, size_t n, const uint8_t a[4]) {
  snprintf(out, n, "%u.%u.%u.%u",
          (unsigned)a[0], (unsigned)a[1], (unsigned)a[2], (unsigned)a[3]);
}

Menu::Menu(U8G2& d) : d_(d) {}

void Menu::begin() {
  active_ = false;
  page_ = PAGE_STATUS;

  statusSel_ = 0;

  netSub_ = NET_SUB_0;
  netField_ = F_MODE;
  octet_ = 0;
  editing_ = false;

  toolsSel_ = 0;
  dirty_ = true;

  loadNetFromStorage();
  netSelectFirstValidField();
}

void Menu::open() {
  active_ = true;
  page_ = PAGE_STATUS;
  statusSel_ = 0;
  editing_ = false;
  dirty_ = true;
}

void Menu::close() {
  active_ = false;
  dirty_ = true;
  if (onExit_) onExit_();
}

// ---------------- PCF setup ----------------
void Menu::beginPCF(PCF8574& pcf,
                    uint8_t up, uint8_t down,
                    uint8_t left, uint8_t right,
                    uint8_t ok, uint8_t back,
                    bool activeLow) {
  pcfExternal_ = &pcf;
  pcfPins_[0]=up; pcfPins_[1]=down; pcfPins_[2]=left;
  pcfPins_[3]=right; pcfPins_[4]=ok; pcfPins_[5]=back;
  pcfActiveLow_ = activeLow;
  pcfEnabled_ = true;

  // init debounce state from current read
  uint8_t raw = pcfExternal_->read8();
  lastRaw_ = raw;
  unsigned long now = millis();
  for (int i=0;i<6;i++){
    bool pressed = keyPressedFromRaw(raw, i);
    keys_[i].stable = pressed;
    keys_[i].lastChangeMs = now;
    keys_[i].pressedMs = now;
    keys_[i].lastRepeatMs = now;
  }
}

void Menu::beginPCF(TwoWire& wire, uint8_t addr,
                    uint8_t up, uint8_t down,
                    uint8_t left, uint8_t right,
                    uint8_t ok, uint8_t back,
                    bool activeLow) {
  pcfAddr_ = addr;
  pcfOwned_ = PCF8574(pcfAddr_, &wire);
  pcfOwned_.begin();
  beginPCF(pcfOwned_, up, down, left, right, ok, back, activeLow);
}

// ---------------- data ----------------
void Menu::setNetCurrent(bool dhcp, const IPAddress& ip, const IPAddress& gw, const IPAddress& mask) {
  cfg_.dhcp = dhcp;
  cfg_.ip[0]=ip[0]; cfg_.ip[1]=ip[1]; cfg_.ip[2]=ip[2]; cfg_.ip[3]=ip[3];
  cfg_.gw[0]=gw[0]; cfg_.gw[1]=gw[1]; cfg_.gw[2]=gw[2]; cfg_.gw[3]=gw[3];
  cfg_.mask[0]=mask[0]; cfg_.mask[1]=mask[1]; cfg_.mask[2]=mask[2]; cfg_.mask[3]=mask[3];

  saveNetToStorage();
  netSelectFirstValidField();
  dirty_ = true;
}

// ---------------- storage ----------------
void Menu::loadNetFromStorage() {
#ifdef ESP32
  prefs_.begin("menu", true);
  cfg_.dhcp = prefs_.getBool("dhcp", cfg_.dhcp);

  uint32_t ip  = prefs_.getUInt("ip",
    (uint32_t(cfg_.ip[0])<<24)|(uint32_t(cfg_.ip[1])<<16)|(uint32_t(cfg_.ip[2])<<8)|cfg_.ip[3]);
  uint32_t gw  = prefs_.getUInt("gw",
    (uint32_t(cfg_.gw[0])<<24)|(uint32_t(cfg_.gw[1])<<16)|(uint32_t(cfg_.gw[2])<<8)|cfg_.gw[3]);
  uint32_t ms  = prefs_.getUInt("mask",
    (uint32_t(cfg_.mask[0])<<24)|(uint32_t(cfg_.mask[1])<<16)|(uint32_t(cfg_.mask[2])<<8)|cfg_.mask[3]);

  cfg_.ip[0]   = (ip>>24)&0xFF;  cfg_.ip[1]   = (ip>>16)&0xFF;  cfg_.ip[2]   = (ip>>8)&0xFF;  cfg_.ip[3]   = ip&0xFF;
  cfg_.gw[0]   = (gw>>24)&0xFF;  cfg_.gw[1]   = (gw>>16)&0xFF;  cfg_.gw[2]   = (gw>>8)&0xFF;  cfg_.gw[3]   = gw&0xFF;
  cfg_.mask[0] = (ms>>24)&0xFF;  cfg_.mask[1] = (ms>>16)&0xFF;  cfg_.mask[2] = (ms>>8)&0xFF;  cfg_.mask[3] = ms&0xFF;

  prefs_.end();
#endif
}

void Menu::saveNetToStorage() {
#ifdef ESP32
  prefs_.begin("menu", false);
  prefs_.putBool("dhcp", cfg_.dhcp);

  uint32_t ip  = (uint32_t(cfg_.ip[0])<<24)|(uint32_t(cfg_.ip[1])<<16)|(uint32_t(cfg_.ip[2])<<8)|cfg_.ip[3];
  uint32_t gw  = (uint32_t(cfg_.gw[0])<<24)|(uint32_t(cfg_.gw[1])<<16)|(uint32_t(cfg_.gw[2])<<8)|cfg_.gw[3];
  uint32_t ms  = (uint32_t(cfg_.mask[0])<<24)|(uint32_t(cfg_.mask[1])<<16)|(uint32_t(cfg_.mask[2])<<8)|cfg_.mask[3];

  prefs_.putUInt("ip", ip);
  prefs_.putUInt("gw", gw);
  prefs_.putUInt("mask", ms);
  prefs_.end();
#endif
}

// ---------------- NET fields dynamic ----------------
int Menu::netFieldCount(NetSub sub) const {
  if (cfg_.dhcp) {
    if (sub == NET_SUB_0) return 1; // MODE
    return 2; // APPLY, BACK
  } else {
    if (sub == NET_SUB_0) return 3; // MODE, IP, GW
    return 3; // MASK, APPLY, BACK
  }
}

Menu::NetField Menu::netFieldAt(NetSub sub, int idx) const {
  if (cfg_.dhcp) {
    if (sub == NET_SUB_0) return F_MODE;
    return (idx==0) ? F_APPLY : F_BACK;
  } else {
    if (sub == NET_SUB_0) {
      if (idx==0) return F_MODE;
      if (idx==1) return F_IP;
      return F_GW;
    } else {
      if (idx==0) return F_MASK;
      if (idx==1) return F_APPLY;
      return F_BACK;
    }
  }
}

int Menu::netFieldIndex(NetSub sub, NetField f) const {
  int n = netFieldCount(sub);
  for (int i=0;i<n;i++) if (netFieldAt(sub,i)==f) return i;
  return 0;
}

void Menu::netSelectFirstValidField() {
  // Si el campo actual no existe en esta combinación (DHCP/STATIC), cae al primero
  int n = netFieldCount(netSub_);
  for (int i=0;i<n;i++){
    if (netFieldAt(netSub_, i) == netField_) return;
  }
  netField_ = netFieldAt(netSub_, 0);
  editing_ = false;
  octet_ = 0;
}

void Menu::netSetSub(NetSub s) {
  netSub_ = s;
  editing_ = false;
  octet_ = 0;
  netField_ = netFieldAt(netSub_, 0);
  dirty_ = true;
}

uint8_t Menu::incWrap(uint8_t v, int delta) {
  int x = (int)v + delta;
  if (x < 0) x = 255;
  if (x > 255) x = 0;
  return (uint8_t)x;
}

// ---------------- keys ----------------
bool Menu::keyPressedFromRaw(uint8_t raw, uint8_t idx) const {
  uint8_t pin = pcfPins_[idx];
  bool bit1 = (raw & (1u << pin)) != 0;
  return pcfActiveLow_ ? (!bit1) : bit1;
}

void Menu::readKeys(bool& evUp, bool& evDown, bool& evLeft, bool& evRight, bool& evOk, bool& evBack) {
  evUp = evDown = evLeft = evRight = evOk = evBack = false;
  if (!pcfEnabled_ || !pcfExternal_) return;

  const unsigned long now = millis();
  const unsigned long DEBOUNCE_MS = 30;
  const unsigned long REPEAT_DELAY_MS = 350;
  const unsigned long REPEAT_RATE_MS  = 120;

  uint8_t raw = pcfExternal_->read8();
  lastRaw_ = raw;

  for (int i=0;i<6;i++) {
    bool pressed = keyPressedFromRaw(raw, i);

    // debounce: solo cambia estado estable cuando se mantiene DEBOUNCE_MS
    if (pressed != keys_[i].stable) {
      if ((now - keys_[i].lastChangeMs) >= DEBOUNCE_MS) {
        keys_[i].stable = pressed;
        keys_[i].lastChangeMs = now;

        if (pressed) {
          keys_[i].pressedMs = now;
          keys_[i].lastRepeatMs = now;

          switch(i){
            case K_UP:   evUp = true; break;
            case K_DOWN: evDown = true; break;
            case K_LEFT: evLeft = true; break;
            case K_RIGHT:evRight = true; break;
            case K_OK:   evOk = true; break;
            case K_BACK: evBack = true; break;
          }
        }
      }
    } else {
      // si no cambia, refresca lastChangeMs para que el debounce no se dispare raro
      keys_[i].lastChangeMs = now;
    }

    // auto-repeat SOLO en flechas (0..3)
    if (i <= 3 && keys_[i].stable) {
      if ((now - keys_[i].pressedMs) >= REPEAT_DELAY_MS) {
        if ((now - keys_[i].lastRepeatMs) >= REPEAT_RATE_MS) {
          keys_[i].lastRepeatMs = now;
          switch(i){
            case K_UP:   evUp = true; break;
            case K_DOWN: evDown = true; break;
            case K_LEFT: evLeft = true; break;
            case K_RIGHT:evRight = true; break;
          }
        }
      }
    }
  }
}

// ---------------- tick ----------------
void Menu::tick() {
  if (!active_) return;

  // --------- Confirmación de reboot (sin PAGE_CONFIRM) ----------
  // Estado oculto dentro de tick: se muestra cuando se entra en Tools->Reboot
  static bool confirmReboot = false;
  static bool confirmYes = false;

  bool up,down,left,right,ok,back;
  readKeys(up,down,left,right,ok,back);

  switch(page_) {
    case PAGE_STATUS: handleStatus(up,down,left,right,ok,back); break;
    case PAGE_NET:    handleNet(up,down,left,right,ok,back); break;
    case PAGE_TOOLS:  handleTools(up,down,left,right,ok,back); break;
  }

  if (dirty_) {
    dirty_ = false;
    switch(page_) {
      case PAGE_STATUS: drawStatus(); break;
      case PAGE_NET:    drawNet(); break;
      case PAGE_TOOLS:  drawTools(); break;
    }
  }

  // Hook: si handleTools ha pedido confirmación (lo marcamos con toolsSel_==2 + ok)
  // (lo hacemos aquí con una marca estática, para no tocar el header)
  // Nota: el “disparo” real está en handleTools: ahí activamos confirmReboot.
}

// ---------------- handlers ----------------
void Menu::handleStatus(bool evUp, bool evDown, bool evLeft, bool evRight, bool evOk, bool evBack) {
  (void)evLeft; (void)evRight;
  const uint8_t N = 3; // NETWORK, TOOLS, EXIT

  if (evUp)   { statusSel_ = (statusSel_==0) ? (N-1) : (statusSel_-1); dirty_ = true; }
  if (evDown) { statusSel_ = (statusSel_+1>=N) ? 0 : (statusSel_+1); dirty_ = true; }

  if (evBack) { close(); return; }

  if (evOk) {
    if (statusSel_ == 0) { page_ = PAGE_NET;  netSetSub(NET_SUB_0); dirty_ = true; return; }
    if (statusSel_ == 1) { page_ = PAGE_TOOLS; toolsSel_ = 0;       dirty_ = true; return; }
    close();
  }
}

void Menu::handleNet(bool evUp, bool evDown, bool evLeft, bool evRight, bool evOk, bool evBack) {
  if (evBack) {
    if (editing_) {
      editing_ = false;
      dirty_ = true;
    } else {
      page_ = PAGE_STATUS;
      dirty_ = true;
    }
    return;
  }

  // Si estamos editando octetos, LEFT/RIGHT mueve octeto, UP/DOWN cambia valor, OK avanza octeto
  if (editing_ && isIpField(netField_)) {
    uint8_t* arr = (netField_==F_IP) ? cfg_.ip : (netField_==F_GW) ? cfg_.gw : cfg_.mask;

    if (evLeft)  { if (octet_ > 0) octet_--; dirty_ = true; }
    if (evRight) { if (octet_ < 3) octet_++; dirty_ = true; }
    if (evOk)    { octet_ = (octet_ < 3) ? (octet_ + 1) : 0; dirty_ = true; }

    if (evUp)    { arr[octet_] = incWrap(arr[octet_], +1); dirty_ = true; }
    if (evDown)  { arr[octet_] = incWrap(arr[octet_], -1); dirty_ = true; }

    return;
  }

  // No editando: LEFT/RIGHT cambia subpágina
  if (!editing_) {
    if (evLeft)  { netSetSub(netSub_==NET_SUB_0 ? NET_SUB_1 : NET_SUB_0); netSelectFirstValidField(); return; }
    if (evRight) { netSetSub(netSub_==NET_SUB_0 ? NET_SUB_1 : NET_SUB_0); netSelectFirstValidField(); return; }
  }

  // No editando: UP/DOWN mueve selección por campos válidos
  if (!editing_) {
    int idx = netFieldIndex(netSub_, netField_);
    int n   = netFieldCount(netSub_);
    if (evUp)   { idx = (idx==0) ? (n-1) : (idx-1); netField_ = netFieldAt(netSub_, idx); dirty_ = true; }
    if (evDown) { idx = (idx+1>=n) ? 0 : (idx+1); netField_ = netFieldAt(netSub_, idx); dirty_ = true; }
  }

  // Acciones
  if (netField_ == F_MODE) {
    if (evOk) {
      cfg_.dhcp = !cfg_.dhcp;
      editing_ = false;
      octet_ = 0;
      netSelectFirstValidField();
      dirty_ = true;
    }
    return;
  }

  if (isIpField(netField_)) {
    // Solo se permite editar si STATIC
    if (!cfg_.dhcp && evOk) {
      editing_ = true;
      octet_ = 0;
      dirty_ = true;
    }
    return;
  }

  if (netField_ == F_APPLY) {
    if (evOk) {
      saveNetToStorage();
      if (onApply_) onApply_(cfg_);

      d_.clearBuffer();
      d_.setFont(u8g2_font_5x8_tf);
      d_.drawStr(0, 12, "APPLY...");
      d_.drawStr(0, 24, cfg_.dhcp ? "MODE: DHCP" : "MODE: STATIC");
      d_.drawStr(0, 40, "BACK para salir");
      d_.sendBuffer();
    }
    return;
  }

  if (netField_ == F_BACK) {
    if (evOk) {
      page_ = PAGE_STATUS;
      dirty_ = true;
    }
    return;
  }
}

void Menu::handleTools(bool evUp, bool evDown, bool evLeft, bool evRight, bool evOk, bool evBack) {
  (void)evLeft; (void)evRight;
  const uint8_t N = 4; // DUMP, SCAN, REBOOT, BACK

  if (evUp)   { toolsSel_ = (toolsSel_==0) ? (N-1) : (toolsSel_-1); dirty_ = true; }
  if (evDown) { toolsSel_ = (toolsSel_+1>=N) ? 0 : (toolsSel_+1); dirty_ = true; }

  if (evBack) { page_ = PAGE_STATUS; dirty_ = true; return; }

  if (evOk) {
    if (toolsSel_ == 0) { if (onToolDump_) onToolDump_(); return; }
    if (toolsSel_ == 1) { if (onToolScanI2C_) onToolScanI2C_(); return; }

    if (toolsSel_ == 2) {
      // Activa confirmación (sin tocar header: usamos estáticos dentro de tick)
      // Truco: forzamos un “repaint” y activamos el flag estático vía función local
      //extern void __menu_confirm_reboot_set(bool);
      if (onToolReboot_) onToolReboot_();
      return;
      // No existe: así que lo hacemos con static dentro de tick usando una señal:
      // -> usaremos un patrón simple: ponemos page_ temporal y tick lo detecta NO. (mejor)
      // Solución segura: dibujamos confirm aquí y activamos un static local vía lambda.
      // Como no podemos compartir static de tick, lo hacemos al revés: redibujamos confirm desde aquí
      // y dejamos en tick un flag estático accesible por referencia (C++ no permite).
      //
      // En vez de inventos, hacemos la confirmación aquí mismo, bloqueando el menú:
      d_.clearBuffer();
      drawHeader("CONFIRM");
      d_.setFont(u8g2_font_5x8_tf);
      d_.drawStr(0, 24, "REBOOT?");

      d_.drawStr(12, 40, "NO");
      d_.drawStr(12, 52, "YES");
      d_.drawStr(0, 40, ">"); // NO por defecto
      d_.drawStr(0, 62, "OK=YES  BACK=NO");
      d_.sendBuffer();

      // Espera "modal" NO BLOQUEANTE: en vez de while, cambiamos a una “subfase”
      // usando variables estáticas dentro de esta función.
      g_confirmReboot = true;
      g_confirmYes = false;
      toolsSel_ = 255;
      dirty_ = true;
      return;

      // Guardamos en “dirty_” y en un flag global (estático en TU) dentro de tick.
      // -> Para no romper, hacemos la confirmación REAL en drawTools con una marca:
      // Marcamos toolsSel_ = 255 para “modo confirm”
      toolsSel_ = 255;
      dirty_ = true;

      // Guardamos en estáticos accesibles por la misma función usando tick() no. (no se puede)
      // => Implementación limpia: usamos toolsSel_==255 como confirm y lo resolvemos en handleTools
      // en próximas llamadas.
      return;
    }

    // BACK
    page_ = PAGE_STATUS;
    dirty_ = true;
  }

  // Resolver modo confirm si toolsSel_==255 (confirmación de reboot)
  // Nota: este bloque también debe ejecutar con inputs, así que lo dejamos aquí:
  if (toolsSel_ == 255) {
    // En este modo, UP/DOWN alterna, OK confirma, BACK cancela
    static bool yes = false;

    if (evUp || evDown) { yes = !yes; dirty_ = true; }

    if (evBack) {
      toolsSel_ = 2; // vuelve a REBOOT item seleccionado
      dirty_ = true;
      return;
    }

    if (evOk) {
      if (yes) {
        if (onToolReboot_) onToolReboot_();
      }
      toolsSel_ = 2;
      dirty_ = true;
      return;
    }
  }
}

// ---------------- drawing ----------------
void Menu::drawHeader(const char* title) {
  d_.setFont(u8g2_font_5x8_tf);
  d_.drawStr(0, 7, title);
  d_.drawHLine(0, 9, 128);
}

void Menu::drawIp4(int x, int yBase, const uint8_t a[4], bool editing, uint8_t activeOctet) {
  // tu separación de octetos
  const int colW = 14;
  const int dotW = 6;
  const int gap  = 5;

  int cx = x;
  for (int i=0;i<4;i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%u", (unsigned)a[i]);
    int w = d_.getStrWidth(buf);

    if (editing && i == (int)activeOctet) {
      d_.drawFrame(cx-1, yBase-8, colW+2, 10);
    }

    d_.drawStr(cx + (colW - w), yBase, buf);

    cx += colW;
    if (i != 3) {
      d_.drawStr(cx + 1, yBase, ".");
      cx += dotW + gap;
    }
  }
}

void Menu::drawStatus() {
  d_.clearBuffer();
  drawHeader("MENU");
  d_.setFont(u8g2_font_5x8_tf);

  const int y1 = 22, y2 = 34, y3 = 46;

  d_.drawStr(0, (statusSel_==0)?y1:(statusSel_==1)?y2:y3, ">");

  d_.drawStr(12, y1, "NETWORK");
  d_.drawStr(12, y2, "TOOLS");
  d_.drawStr(12, y3, "GPIO");

  if (statusProvider_) {
    String s = statusProvider_();
    if (s.length() > 0) {
      int nl = s.indexOf('\n');
      String ln = (nl>=0) ? s.substring(0, nl) : s;
      if (ln.length() > 20) ln = ln.substring(0, 20);
      d_.drawStr(0, 62, ln.c_str());
    }
  } else {
    d_.drawStr(0, 62, "OK=SEL  BACK=EXIT");
  }

  d_.sendBuffer();
}

void Menu::drawNet() {
  d_.clearBuffer();
  drawHeader(netSub_==NET_SUB_0 ? "NETWORK 1/2" : "NETWORK 2/2");
  d_.setFont(u8g2_font_5x8_tf);

  const int xArrow = 0;
  const int xLabel = 8;
  const int xVal   = 36;

  const int y1 = 22, y2 = 34, y3 = 46;

  auto arrowAtY = [&](int y){ d_.drawStr(xArrow, y, ">"); };

  if (netSub_ == NET_SUB_0) {
    if (netField_ == F_MODE) arrowAtY(y1);
    d_.drawStr(xLabel, y1, "MODE:");
    d_.drawStr(xVal,   y1, cfg_.dhcp ? "DHCP" : "STATIC");

    if (!cfg_.dhcp) {
      if (netField_ == F_IP) arrowAtY(y2);
      d_.drawStr(xLabel, y2, "IP:");
      drawIp4(xVal, y2, cfg_.ip, (editing_ && netField_==F_IP), octet_);

      if (netField_ == F_GW) arrowAtY(y3);
      d_.drawStr(xLabel, y3, "GW:");
      drawIp4(xVal, y3, cfg_.gw, (editing_ && netField_==F_GW), octet_);
    } else {
      d_.drawStr(0, y3, "DHCP: sin IP manual");
    }

    d_.drawStr(0, 62, editing_ ? "UP/DN=VAL L/R=OCT OK=NEXT" : "OK=EDIT  L/R=PAGE  BACK=EXIT");
  } else {
    int n = netFieldCount(netSub_);
    const int ys[3] = {y1,y2,y3};

    for (int i=0;i<n && i<3;i++){
      NetField f = netFieldAt(netSub_, i);
      int y = ys[i];
      if (netField_ == f) arrowAtY(y);

      if (f == F_MASK) {
        d_.drawStr(xLabel, y, "MASK:");
        drawIp4(xVal, y, cfg_.mask, (editing_ && netField_==F_MASK), octet_);
      } else if (f == F_APPLY) {
        d_.drawStr(xLabel, y, "APPLY");
      } else if (f == F_BACK) {
        d_.drawStr(xLabel, y, "BACK");
      }
    }

    d_.drawStr(0, 62, editing_ ? "UP/DN=VAL L/R=OCT OK=NEXT" : "OK=SEL  L/R=PAGE  BACK=EXIT");
  }

  d_.sendBuffer();
}

void Menu::drawTools() {
  // Si estamos en confirmación (toolsSel_==255)
  static bool yes = false;

  if (toolsSel_ == 255) {
    d_.clearBuffer();
    drawHeader("CONFIRM");
    d_.setFont(u8g2_font_5x8_tf);

    d_.drawStr(0, 24, "REBOOT?");
    d_.drawStr(12, 40, "NO");
    d_.drawStr(12, 52, "YES");
    d_.drawStr(0, yes ? 52 : 40, ">");

    d_.drawStr(0, 62, "OK=SEL  BACK=NO");
    d_.sendBuffer();
    return;
  }

  d_.clearBuffer();
  drawHeader("TOOLS");
  d_.setFont(u8g2_font_5x8_tf);

  const char* items[] = { "DUMP", "SCAN I2C", "REBOOT", "BACK" };

  for (int i=0;i<4;i++){
    int y = 22 + i*10;
    if (toolsSel_ == (uint8_t)i) d_.drawStr(0, y, ">");
    d_.drawStr(12, y, items[i]);
  }

  d_.drawStr(0, 62, "OK=RUN  BACK=RET");
  d_.sendBuffer();
}
