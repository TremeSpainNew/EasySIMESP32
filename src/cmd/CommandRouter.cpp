/*#include <cmd/CommandRouter.h>
#include <core/AppState.h>
#include <io/AdsCache.h>

// Declara aquí lo que uses (enviar(), etc.)
extern void enviar(const String& msg);

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
        if (!g_ads_ok) {
          enviar(F("ERR NOTCH RAW ADS no listo"));
          bloqueado=false;
          return;
        }
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
        if (!g_ads_ok) {
          enviar(F("ERR NOTCH ADDHERE ADS no listo"));
          bloqueado=false;
          return;
        }
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
        if (!g_ads_ok) {
          enviar(F("ERR NOTCH CAP ADS no listo"));
          bloqueado=false;
          return;
        }
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
*/