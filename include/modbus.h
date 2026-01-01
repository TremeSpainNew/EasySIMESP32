#pragma once
#include <Arduino.h>
#include <IPAddress.h>
#include <ModbusIP_ESP8266.h>   // Librería emelianov - maestro Modbus TCP
#include <ModbusRTU.h>          // Librería emelianov - maestro Modbus RTU
#include "EE.h"                 // Para persistencia en EEPROM/NVS

// -----------------------------------------------------------------------------
// Integra con tu proyecto: esta función ya existe en tu sketch principal.
extern void enviar(const String& msg);

// ========================= CONFIG RTU/RS485 =========================
// Puedes overridear estos defines desde main.cpp antes de incluir este header
#ifndef MB_RTU_SERIAL
  #define MB_RTU_SERIAL Serial2
#endif

#ifndef MB_RTU_TX_PIN
  #define MB_RTU_TX_PIN 17
#endif

#ifndef MB_RTU_RX_PIN
  #define MB_RTU_RX_PIN 18
#endif

// Pin DE/RE del transceptor RS485 (MAX485, SN75176, etc.)
#ifndef MB_RTU_DE_RE_PIN
  #define MB_RTU_DE_RE_PIN 6
#endif

#ifndef MB_RTU_BAUD
  #define MB_RTU_BAUD 19200
#endif

// -----------------------------------------------------------------------------
// Modbus masters globales
static ModbusIP  mb;     // Maestro TCP
static ModbusRTU mbRtu;  // Maestro RTU (RS485)

// ===== Modbus config (RAM) =====
enum MbFunc : uint8_t { MB_HREG=0, MB_IREG=1, MB_COIL=2, MB_ISTS=3 };
enum MbBus  : uint8_t { MB_BUS_TCP=0, MB_BUS_RTU=1 };

struct MbDevice {
  uint8_t   id;          // 0..N-1 (identificador lógico que tú asignas)
  IPAddress ip;          // IP esclavo Modbus TCP (solo si bus==TCP)
  uint16_t  port;        // Puerto TCP (guardado, pero modbusIP usa 502 fijo)
  uint8_t   unit;        // UnitId (slave id)
  MbBus     bus;         // TCP o RTU
  uint16_t  periodMs;    // Periodo "global" del device (si lo quieres usar)
  char      name[20];    // Nombre para UI
  bool      used;
};

struct MbTag {
  bool      isOutput;   // false=entrada, true=salida
  uint8_t   devId;      // referencia a MbDevice.id
  MbFunc    func;       // HREG, IREG, COIL, ISTS
  uint16_t  addr;       // dirección base (0-based)
  uint16_t  qty;        // cantidad (1 para simple)
  uint16_t  periodMs;   // periodo de sondeo (no usado aún)
  float     scale;      // y = raw*scale + offset
  float     offset;
  char      name[20];   // clave/tag
  bool      used;
};

static const int MAX_MB_DEV  = 16;
static const int MAX_MB_TAGS = 64;

static MbDevice g_mbDevices[MAX_MB_DEV];
static MbTag    g_mbTags[MAX_MB_TAGS];

// -----------------------------------------------------------------------------
// Utilidades tabla
static inline int findFreeDev() {
  for (int i=0;i<MAX_MB_DEV;i++) if (!g_mbDevices[i].used) return i;
  return -1;
}
static inline int findDevById(uint8_t id) {
  for (int i=0;i<MAX_MB_DEV;i++) if (g_mbDevices[i].used && g_mbDevices[i].id==id) return i;
  return -1;
}
static inline int findFreeTag() {
  for (int i=0;i<MAX_MB_TAGS;i++) if (!g_mbTags[i].used) return i;
  return -1;
}
static inline int findTagByName(const char* name) {
  for (int i=0;i<MAX_MB_TAGS;i++) if (g_mbTags[i].used && strcasecmp(g_mbTags[i].name,name)==0) return i;
  return -1;
}
static inline bool parseFunc(const char* s, MbFunc& f) {
  if (!strcasecmp(s,"HREG")) { f=MB_HREG; return true; }
  if (!strcasecmp(s,"IREG")) { f=MB_IREG; return true; }
  if (!strcasecmp(s,"COIL")) { f=MB_COIL; return true; }
  if (!strcasecmp(s,"ISTS")) { f=MB_ISTS; return true; }
  return false;
}
static inline bool funcIsWritable(MbFunc f) { return (f==MB_HREG || f==MB_COIL); }

// -----------------------------------------------------------------------------
// Conversión ingeniería <-> raw (uint16). Ajusta si necesitas signed o 32-bit.
static inline uint16_t engToRawU16(float eng, float scale, float off) {
  float rawf = (eng - off) / scale;
  long r = lroundf(rawf);
  if (r < 0) r = 0;
  if (r > 65535) r = 65535;
  return (uint16_t)r;
}
static inline float rawU16ToEng(uint16_t raw, float scale, float off) {
  return raw * scale + off;
}

// ========================= WRAPPERS RTU BLOQUEANTES =========================

static volatile bool               g_rtuDone   = false;
static volatile Modbus::ResultCode g_rtuResult = Modbus::EX_SUCCESS;
static uint16_t                    g_rtuTid    = 1;

static bool rtuCb(Modbus::ResultCode event, uint16_t /*transactionId*/, void* /*data*/) {
  g_rtuResult = event;
  g_rtuDone   = true;
  return true;
}

static bool rtuWait(uint16_t timeoutMs = 200) {
  uint32_t t0 = millis();
  while (!g_rtuDone && (millis() - t0 < timeoutMs)) {
    mbRtu.task();
    yield();
  }
  if (!g_rtuDone) return false;
  return g_rtuResult == Modbus::EX_SUCCESS;
}

static inline uint16_t rtuNextTid() {
  if (++g_rtuTid == 0) g_rtuTid = 1;
  return g_rtuTid;
}

static bool rtuReadCoil(uint8_t unit, uint16_t addr, bool* buf, uint16_t qty) {
  g_rtuDone = false;
  if (!mbRtu.readCoil(unit, addr, buf, qty, rtuCb)) return false;
  return rtuWait();
}
static bool rtuReadIsts(uint8_t unit, uint16_t addr, bool* buf, uint16_t qty) {
  g_rtuDone = false;
  if (!mbRtu.readIsts(unit, addr, buf, qty, rtuCb)) return false;
  return rtuWait();
}
static bool rtuReadHreg(uint8_t unit, uint16_t addr, uint16_t* buf, uint16_t qty) {
  g_rtuDone = false;
  if (!mbRtu.readHreg(unit, addr, buf, qty, rtuCb)) return false;
  return rtuWait();
}
static bool rtuReadIreg(uint8_t unit, uint16_t addr, uint16_t* buf, uint16_t qty) {
  g_rtuDone = false;
  if (!mbRtu.readIreg(unit, addr, buf, qty, rtuCb)) return false;
  return rtuWait();
}

static bool rtuWriteCoilSingle(uint8_t unit, uint16_t addr, bool val) {
  g_rtuDone = false;
  if (!mbRtu.writeCoil(unit, addr, val, rtuCb)) return false;
  return rtuWait();
}
static bool rtuWriteCoils(uint8_t unit, uint16_t addr, bool* buf, uint16_t qty) {
  g_rtuDone = false;
  if (!mbRtu.writeCoil(unit, addr, buf, qty, rtuCb)) return false;
  return rtuWait();
}
static bool rtuWriteHregSingle(uint8_t unit, uint16_t addr, uint16_t val) {
  g_rtuDone = false;
  if (!mbRtu.writeHreg(unit, addr, val, rtuCb)) return false;
  return rtuWait();
}
static bool rtuWriteHregs(uint8_t unit, uint16_t addr, uint16_t* buf, uint16_t qty) {
  g_rtuDone = false;
  if (!mbRtu.writeHreg(unit, addr, buf, qty, rtuCb)) return false;
  return rtuWait();
}

// -----------------------------------------------------------------------------
// Inicialización Modbus (maestro TCP + maestro RTU). No arrancamos servidor TCP.
static inline void mbInit() {
  memset(g_mbDevices, 0, sizeof(g_mbDevices));
  memset(g_mbTags,     0, sizeof(g_mbTags));

  // ---- RTU / RS485 ----
  MB_RTU_SERIAL.begin(MB_RTU_BAUD, SERIAL_8N1, MB_RTU_RX_PIN, MB_RTU_TX_PIN);
  mbRtu.begin(&MB_RTU_SERIAL, MB_RTU_DE_RE_PIN);
  mbRtu.master();

  // TCP: nada especial aquí, solo asegurarnos de NO llamar a mb.server()
}

// Debe llamarse en loop()
static inline void mbTask() {
  mb.task();     // Maestro TCP
  mbRtu.task();  // Maestro RTU
}

// -----------------------------------------------------------------------------
// Escritura genérica con fallback (coils y holding registers)
static uint16_t mbWriteTag(const MbTag& t, const float* engVals, int nVals) {
  int idx = findDevById(t.devId);
  if (idx < 0) { enviar("MB: devId no encontrado"); return 0; }

  const MbDevice& d = g_mbDevices[idx];
  const uint8_t unit = d.unit;

  if (nVals <= 0 || nVals != (int)t.qty) {
    enviar("MB: qty/valores no coincide");
    return 0;
  }

  // ========================= COILS =========================
  if (t.func == MB_COIL) {

    // --- TCP ---
    if (d.bus == MB_BUS_TCP) {
      const IPAddress ip = d.ip;

      if (t.qty == 1) {
        const bool v = (engVals[0] != 0.0f);
        return mb.writeCoil(ip, t.addr, v, nullptr, unit);
      } else {
        if (t.qty > 64) { enviar("MB: qty coils > 64"); return 0; }
        bool buf[64];
        for (uint16_t i = 0; i < t.qty; i++) buf[i] = (engVals[i] != 0.0f);

        uint16_t tid = mb.writeCoil(ip, t.addr, buf, t.qty, nullptr, unit); // FC15
        if (tid != 0) return tid;

        enviar("MB: FC15 falló, probando FC5 por coil (TCP)...");
        uint16_t lastTid = 0;
        for (uint16_t i = 0; i < t.qty; i++) {
          lastTid = mb.writeCoil(ip, t.addr + i, buf[i], nullptr, unit);
          if (lastTid == 0) {
            enviar(String("MB: FC5 tid=0 en coil idx=") + i);
            return 0;
          }
        }
        return lastTid;
      }
    }

    // --- RTU ---
    if (d.bus == MB_BUS_RTU) {
      if (t.qty == 1) {
        const bool v = (engVals[0] != 0.0f);
        if (!rtuWriteCoilSingle(unit, t.addr, v)) {
          enviar("MB RTU: write single coil falló");
          return 0;
        }
        return rtuNextTid();
      } else {
        if (t.qty > 64) { enviar("MB RTU: qty coils > 64"); return 0; }
        bool buf[64];
        for (uint16_t i = 0; i < t.qty; i++) buf[i] = (engVals[i] != 0.0f);

        if (!rtuWriteCoils(unit, t.addr, buf, t.qty)) {
          enviar("MB RTU: FC15 falló, probando FC5 por coil...");
          for (uint16_t i = 0; i < t.qty; i++) {
            if (!rtuWriteCoilSingle(unit, t.addr + i, buf[i])) {
              enviar(String("MB RTU: FC5 falló en coil idx=") + i);
              return 0;
            }
          }
        }
        return rtuNextTid();
      }
    }
  }

  // ========================= HREG =========================
  if (t.func == MB_HREG) {

    // --- TCP ---
    if (d.bus == MB_BUS_TCP) {
      const IPAddress ip = d.ip;

      if (t.qty == 1) {
        const uint16_t raw = engToRawU16(engVals[0], t.scale, t.offset);
        return mb.writeHreg(ip, t.addr, raw, nullptr, unit);
      } else {
        if (t.qty > 64) { enviar("MB: qty hregs > 64"); return 0; }
        uint16_t buf[64];
        for (uint16_t i = 0; i < t.qty; i++)
          buf[i] = engToRawU16(engVals[i], t.scale, t.offset);

        uint16_t tid = mb.writeHreg(ip, t.addr, buf, t.qty, nullptr, unit); // FC16
        if (tid != 0) return tid;

        enviar("MB: FC16 falló, probando FC6 por registro (TCP)...");
        uint16_t lastTid = 0;
        for (uint16_t i = 0; i < t.qty; i++) {
          lastTid = mb.writeHreg(ip, t.addr + i, buf[i], nullptr, unit);
          if (lastTid == 0) {
            enviar(String("MB: FC6 tid=0 en hreg idx=") + i);
            return 0;
          }
        }
        return lastTid;
      }
    }

    // --- RTU ---
    if (d.bus == MB_BUS_RTU) {
      if (t.qty == 1) {
        const uint16_t raw = engToRawU16(engVals[0], t.scale, t.offset);
        if (!rtuWriteHregSingle(unit, t.addr, raw)) {
          enviar("MB RTU: write single HREG falló");
          return 0;
        }
        return rtuNextTid();
      } else {
        if (t.qty > 64) { enviar("MB RTU: qty hregs > 64"); return 0; }
        uint16_t buf[64];
        for (uint16_t i = 0; i < t.qty; i++)
          buf[i] = engToRawU16(engVals[i], t.scale, t.offset);

        if (!rtuWriteHregs(unit, t.addr, buf, t.qty)) {
          enviar("MB RTU: FC16 falló, probando FC6 por registro...");
          for (uint16_t i = 0; i < t.qty; i++) {
            if (!rtuWriteHregSingle(unit, t.addr + i, buf[i])) {
              enviar(String("MB RTU: FC6 falló en hreg idx=") + i);
              return 0;
            }
          }
        }
        return rtuNextTid();
      }
    }
  }

  enviar("MB: intento de escritura en func no soportada");
  return 0;
}

// -----------------------------------------------------------------------------
// Lectura puntual (para MB.GET). Devuelve true si OK y llena 'out' con qty valores.
static bool mbReadOnce(const MbTag& t, String& out) {
  int idx = findDevById(t.devId);
  if (idx < 0) { out = "MB.GET: devId no encontrado"; return false; }
  const MbDevice& d = g_mbDevices[idx];
  const IPAddress ip = d.ip;
  const uint8_t unit = d.unit;

  out.reserve(32);

  // ========================= COILS =========================
  if (t.func == MB_COIL) {
    if (t.qty > 128) { out = "MB.GET: qty coils>128"; return false; }
    bool buf[128];

    if (d.bus == MB_BUS_TCP) {
      uint16_t tid = mb.readCoil(ip, t.addr, buf, t.qty, nullptr, unit);
      if (!tid) { out = "MB.GET: readCoil tid=0 (TCP)"; return false; }
      uint32_t t0 = millis();
      while (millis()-t0 < 200) { mb.task(); }
    } else {
      if (!rtuReadCoil(unit, t.addr, buf, t.qty)) {
        out = "MB.GET RTU: readCoil falló";
        return false;
      }
    }

    out = "";
    for (uint16_t i=0;i<t.qty;i++) {
      if (i) out += ' ';
      out += (buf[i] ? "1":"0");
    }
    return true;
  }

  // ========================= ISTS =========================
  if (t.func == MB_ISTS) {
    if (t.qty > 128) { out = "MB.GET: qty ists>128"; return false; }
    bool buf[128];

    if (d.bus == MB_BUS_TCP) {
      uint16_t tid = mb.readIsts(ip, t.addr, buf, t.qty, nullptr, unit);
      if (!tid) { out = "MB.GET: readIsts tid=0 (TCP)"; return false; }
      uint32_t t0 = millis();
      while (millis()-t0 < 200) { mb.task(); }
    } else {
      if (!rtuReadIsts(unit, t.addr, buf, t.qty)) {
        out = "MB.GET RTU: readIsts falló";
        return false;
      }
    }

    out = "";
    for (uint16_t i=0;i<t.qty;i++) {
      if (i) out += ' ';
      out += (buf[i] ? "1":"0");
    }
    return true;
  }

  // ========================= HREG =========================
  if (t.func == MB_HREG) {
    if (t.qty > 64) { out = "MB.GET: qty hreg>64"; return false; }
    uint16_t buf[64];

    if (d.bus == MB_BUS_TCP) {
      uint16_t tid = mb.readHreg(ip, t.addr, buf, t.qty, nullptr, unit);
      if (!tid) { out = "MB.GET: readHreg tid=0 (TCP)"; return false; }
      uint32_t t0 = millis();
      while (millis()-t0 < 200) { mb.task(); }
    } else {
      if (!rtuReadHreg(unit, t.addr, buf, t.qty)) {
        out = "MB.GET RTU: readHreg falló";
        return false;
      }
    }

    out = "";
    for (uint16_t i=0;i<t.qty;i++) {
      if (i) out += ' ';
      float eng = rawU16ToEng(buf[i], t.scale, t.offset);
      out += String(eng, 3);
    }
    return true;
  }

  // ========================= IREG =========================
  if (t.func == MB_IREG) {
    if (t.qty > 64) { out = "MB.GET: qty ireg>64"; return false; }
    uint16_t buf[64];

    if (d.bus == MB_BUS_TCP) {
      uint16_t tid = mb.readIreg(ip, t.addr, buf, t.qty, nullptr, unit);
      if (!tid) { out = "MB.GET: readIreg tid=0 (TCP)"; return false; }
      uint32_t t0 = millis();
      while (millis()-t0 < 200) { mb.task(); }
    } else {
      if (!rtuReadIreg(unit, t.addr, buf, t.qty)) {
        out = "MB.GET RTU: readIreg falló";
        return false;
      }
    }

    out = "";
    for (uint16_t i=0;i<t.qty;i++) {
      if (i) out += ' ';
      float eng = rawU16ToEng(buf[i], t.scale, t.offset);
      out += String(eng, 3);
    }
    return true;
  }

  out = "MB.GET: func no soportada";
  return false;
}

// -----------------------------------------------------------------------------
// Parser/handler de comandos MB.*  (llámalo desde tu handleLine)

// ===== Prototipos de persistencia MB =====
bool mbLoadFromEEPROM(uint16_t &nDev, uint16_t &nTag, uint32_t &crcOut);
void mbSaveToEEPROM(uint16_t nDev, uint16_t nTag, uint32_t crc);

static inline void handleMbCommand(const String& cmd) {
  // ---- MB.ADDDEV <id> <name> <ip> <port> <unit> <periodMs> <TCP|RTU> ----
  {
    int id, port, unit;
    unsigned long period;
    char name[24], ipStr[32], typeStr[8];

    int n = sscanf(
      cmd.c_str(),
      "MB.ADDDEV %d %23s %31s %d %d %lu %7s",
      &id, name, ipStr, &port, &unit, &period, typeStr
    );

    if (n == 7) {
      if (id < 0 || id > 250) { enviar("❌ ID inválido (0..250)"); return; }

      MbBus bus;
      if (!strcasecmp(typeStr, "TCP")) {
        bus = MB_BUS_TCP;
      } else if (!strcasecmp(typeStr, "RTU")) {
        bus = MB_BUS_RTU;
      } else {
        enviar("❌ Tipo inválido (TCP/RTU)");
        return;
      }

      if (findDevById((uint8_t)id) != -1) {
        enviar("❌ ID duplicado");
        return;
      }

      int slot = findFreeDev();
      if (slot < 0) {
        enviar("❌ Sin espacio para más dispositivos");
        return;
      }

      MbDevice& d = g_mbDevices[slot];
      d.id       = (uint8_t)id;
      d.port     = (uint16_t)port;
      d.unit     = (uint8_t)unit;
      d.bus      = bus;
      d.periodMs = (uint16_t)period;
      strncpy(d.name, name, sizeof(d.name)-1);
      d.name[sizeof(d.name)-1] = '\0';

      if (bus == MB_BUS_TCP) {
        IPAddress ip;
        if (!ip.fromString(ipStr)) {
          enviar("❌ IP inválida");
          return;
        }
        d.ip = ip;
      } else {
        d.ip = IPAddress(0,0,0,0); // simbólico en RTU
      }

      d.used = true;

      if (bus == MB_BUS_TCP) enviar("✅ MB.ADDDEV TCP OK");
      else                  enviar("✅ MB.ADDDEV RTU OK");

      return;
    }
  }

  // ---- MB.DELDEV <id> ----
  {
    int id;
    if (sscanf(cmd.c_str(), "MB.DELDEV %d", &id) == 1) {
      int i = findDevById((uint8_t)id);
      if (i < 0) { enviar("❌ Dispositivo no encontrado"); return; }
      // Borra tags de ese device
      for (int t=0;t<MAX_MB_TAGS;t++)
        if (g_mbTags[t].used && g_mbTags[t].devId == (uint8_t)id) g_mbTags[t].used = false;
      g_mbDevices[i].used = false;
      enviar("✅ MB.DELDEV OK");
      return;
    }
  }

  // ---- MB.LSDEV ----
  if (cmd.startsWith("MB.LSDEV")) {
    enviar("📋 Dispositivos:");
    for (int i=0;i<MAX_MB_DEV;i++) if (g_mbDevices[i].used) {
      const MbDevice& d = g_mbDevices[i];
      char buf[160];
      if (d.bus == MB_BUS_TCP) {
        snprintf(buf, sizeof(buf),
          "  id=%d name=%s bus=TCP ip=%u.%u.%u.%u port=%u unit=%u period=%u",
          d.id, d.name,
          d.ip[0], d.ip[1], d.ip[2], d.ip[3],
          d.port, d.unit, d.periodMs);
      } else {
        snprintf(buf, sizeof(buf),
          "  id=%d name=%s bus=RTU unit=%u period=%u",
          d.id, d.name,
          d.unit, d.periodMs);
      }
      enviar(String(buf));
    }
    return;
  }

  // ---- MB.ADDIN  <devId> <HREG|IREG|COIL|ISTS> <addr> <qty> <name> [periodMs] [scale] [offset] ----
  {
    int devId, addr, qty; char fstr[8], name[24];
    unsigned long period=200; float sc=1.0f, off=0.0f;
    int n = sscanf(cmd.c_str(), "MB.ADDIN %d %7s %d %d %23s %lu %f %f",
                   &devId, fstr, &addr, &qty, &name[0], &period, &sc, &off);
    if (n >= 5) {
      MbFunc f; if (!parseFunc(fstr,f))     { enviar("❌ Func inválida"); return; }
      if (findDevById((uint8_t)devId) < 0)  { enviar("❌ devId desconocido"); return; }
      if (qty <= 0 || qty > 32)             { enviar("❌ qty fuera de rango (1..32)"); return; }
      if (findTagByName(name) >= 0)         { enviar("❌ name duplicado"); return; }
      int slot = findFreeTag(); if (slot < 0) { enviar("❌ Sin espacio para tags"); return; }
      MbTag& t = g_mbTags[slot];
      t.used     = true;
      t.isOutput = false;
      t.devId    = (uint8_t)devId;
      t.func     = f;
      t.addr     = (uint16_t)addr;
      t.qty      = (uint16_t)qty;
      t.periodMs = (uint16_t)period;
      t.scale    = sc;
      t.offset   = off;
      strncpy(t.name, name, sizeof(t.name)-1);
      t.name[sizeof(t.name)-1] = '\0';
      enviar("✅ MB.ADDIN OK");
      return;
    }
  }

  // ---- MB.ADDOUT <devId> <HREG|COIL> <addr> <qty> <name> [periodMs] [scale] [offset] ----
  {
    int devId, addr, qty; char fstr[8], name[24];
    unsigned long period=0; float sc=1.0f, off=0.0f;
    int n = sscanf(cmd.c_str(), "MB.ADDOUT %d %7s %d %d %23s %lu %f %f",
                  &devId, fstr, &addr, &qty, &name[0], &period, &sc, &off);
    if (n >= 5) {
      MbFunc f; if (!parseFunc(fstr,f) || !funcIsWritable(f)) { enviar("❌ Func inválida para salida"); return; }
      if (findDevById((uint8_t)devId) < 0)  { enviar("❌ devId desconocido"); return; }
      if (qty <= 0 || qty > 64)             { enviar("❌ qty fuera de rango (1..64)"); return; }
      if (findTagByName(name) >= 0)         { enviar("❌ name duplicado"); return; }
      int slot = findFreeTag(); if (slot < 0) { enviar("❌ Sin espacio para tags"); return; }
      MbTag& t = g_mbTags[slot];
      t.used     = true;
      t.isOutput = true;
      t.devId    = (uint8_t)devId;
      t.func     = f;
      t.addr     = (uint16_t)addr;
      t.qty      = (uint16_t)qty;
      t.periodMs = (uint16_t)period; // no crítico en salidas
      t.scale    = sc;
      t.offset   = off;
      strncpy(t.name, name, sizeof(t.name)-1);
      t.name[sizeof(t.name)-1] = '\0';
      enviar("✅ MB.ADDOUT OK");
      return;
    }
  }

  // ---- MB.DELTAG <name> ----
  {
    char name[24];
    if (sscanf(cmd.c_str(), "MB.DELTAG %23s", name) == 1) {
      int i = findTagByName(name);
      if (i < 0) { enviar("❌ Tag no encontrado"); return; }
      g_mbTags[i].used = false;
      enviar("✅ MB.DELTAG OK");
      return;
    }
  }

  // ---- MB.LSTAG ----
  if (cmd.startsWith("MB.LSTAG")) {
    enviar("📋 Tags:");
    for (int i=0;i<MAX_MB_TAGS;i++) if (g_mbTags[i].used) {
      const MbTag& t = g_mbTags[i];
      const char* f =
        (t.func==MB_HREG?"HREG":
        (t.func==MB_IREG?"IREG":
        (t.func==MB_COIL?"COIL":"ISTS")));
      char buf[160];
      snprintf(buf, sizeof(buf),
        "  %s name=%s dev=%d func=%s addr=%u qty=%u period=%u scale=%.3f off=%.3f",
        t.isOutput?"OUT":"IN", t.name, t.devId, f, t.addr, t.qty, t.periodMs, t.scale, t.offset);
      enviar(String(buf));
    }
    return;
  }

  // ---- MB.SET <name> <v1> [v2] [v3] ... ----
  if (cmd.startsWith("MB.SET ")) {
    int sp = cmd.indexOf(' ');
    String rest = cmd.substring(sp+1);
    rest.trim();
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) { enviar("❌ MB.SET formato: MB.SET <name> <v1> [v2].."); return; }
    String name = rest.substring(0, sp2);
    String values = rest.substring(sp2+1);
    values.trim();

    int iTag = findTagByName(name.c_str());
    if (iTag < 0) { enviar("❌ MB.SET: tag no encontrado"); return; }
    MbTag* tag = &g_mbTags[iTag];
    if (!tag->isOutput) { enviar("❌ MB.SET: tag no es salida"); return; }
    if (!funcIsWritable(tag->func)) { enviar("❌ MB.SET: func no escribible"); return; }

    float v[64]; int nv=0;
    int from=0;
    while (nv < (int)tag->qty) {
      int p = values.indexOf(' ', from);
      String tok = (p<0)? values.substring(from) : values.substring(from, p);
      tok.trim();
      if (tok.length()==0) break;
      v[nv++] = tok.toFloat();
      if (p<0) break;
      from = p+1;
    }
    if (nv != (int)tag->qty) {
      enviar("❌ MB.SET: nº de valores != qty");
      return;
    }

    {
      int didx = findDevById(tag->devId);
      if (didx>=0) {
        const MbDevice& d = g_mbDevices[didx];
        char s[200];
        if (d.bus == MB_BUS_TCP) {
          snprintf(s, sizeof(s),
            "MB.SET -> name=%s dev=%d (%s) bus=TCP ip=%u.%u.%u.%u port=%u unit=%u func=%d addr=%u qty=%u",
            tag->name, tag->devId, d.name,
            d.ip[0], d.ip[1], d.ip[2], d.ip[3],
            d.port, d.unit, (int)tag->func, tag->addr, tag->qty);
        } else {
          snprintf(s, sizeof(s),
            "MB.SET -> name=%s dev=%d (%s) bus=RTU unit=%u func=%d addr=%u qty=%u",
            tag->name, tag->devId, d.name,
            d.unit, (int)tag->func, tag->addr, tag->qty);
        }
        enviar(String(s));
      }
    }

    uint16_t tid = mbWriteTag(*tag, v, nv);
    if (tid == 0) {
      enviar("MB.SET: error al iniciar transacción");
    } else {
      enviar(String("MB.SET: tid=") + tid);
    }
    return;
  }

  // ---- MB.GET <name> ----
  if (cmd.startsWith("MB.GET ")) {
    char name[24];
    if (sscanf(cmd.c_str(), "MB.GET %23s", name) == 1) {
      int iTag = findTagByName(name);
      if (iTag < 0) { enviar("❌ MB.GET: tag no encontrado"); return; }
      String out;
      if (mbReadOnce(g_mbTags[iTag], out)) {
        enviar(String("MB.GET ") + name + " = " + out);
      } else {
        enviar(out);
      }
      return;
    }
  }

  // ---- MB.SAVE ----
  if (cmd.startsWith("MB.SAVE")) {
    // Los parámetros se ignoran; la función interna recuenta dev/tag y calcula CRC
    mbSaveToEEPROM(0, 0, 0);
    return;
  }

  // ---- MB.DUMP ----
  if (cmd.startsWith("MB.DUMP")) {
    enviar("BEGIN MB");
    // Devices
    for (int i=0;i<MAX_MB_DEV;i++) if (g_mbDevices[i].used) {
      const MbDevice& d = g_mbDevices[i];
      const char* type = (d.bus==MB_BUS_TCP) ? "TCP" : "RTU";
      char buf[200];
      snprintf(buf, sizeof(buf),
        "MB.DEV %d %s %u.%u.%u.%u %u %u %u %s",
        d.id,
        d.name,
        d.ip[0], d.ip[1], d.ip[2], d.ip[3],
        d.port, d.unit, d.periodMs,
        type);
      enviar(String(buf));
    }
    // Tags
    for (int i=0;i<MAX_MB_TAGS;i++) if (g_mbTags[i].used) {
      const MbTag& t = g_mbTags[i];
      const char* f =
        (t.func==MB_HREG?"HREG":
        (t.func==MB_IREG?"IREG":
        (t.func==MB_COIL?"COIL":"ISTS")));
      char buf[200];
      snprintf(buf, sizeof(buf),
        "MB.TAG %s %d %s %u %u %s %u %.3f %.3f",
        t.isOutput?"OUT":"IN",
        t.devId, f, t.addr, t.qty, t.name, t.periodMs, t.scale, t.offset);
      enviar(String(buf));
    }
    enviar("END MB");
    return;
  }

  enviar("❓ Comando MB.* no reconocido");
}

// ====================== COMPATIBILIDAD CON CÓDIGO ANTIGUO ======================

// Ventana de EEPROM para Modbus
static uint32_t g_mbEepromBase = 0;
static uint32_t g_mbEepromSize = 0;

void modbusSetEepromWindow(uint32_t base, uint32_t size) {
  g_mbEepromBase = base;
  g_mbEepromSize = size;
}

// ====== Persistencia en EEPROM (devices + tags) ======

struct __attribute__((packed)) MbDevicePersist {
  uint8_t   id;
  uint8_t   bus;      // MbBus
  uint8_t   unit;
  uint16_t  port;
  uint16_t  periodMs;
  uint8_t   ip[4];
  char      name[20];
};

struct __attribute__((packed)) MbTagPersist {
  uint8_t   isOutput;
  uint8_t   devId;
  uint8_t   func;      // MbFunc
  uint16_t  addr;
  uint16_t  qty;
  uint16_t  periodMs;
  float     scale;
  float     offset;
  char      name[20];
};

// CRC32 (polinomio estándar 0xEDB88320)
static uint32_t mbCrc32Update(uint32_t crc, const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  while (len--) {
    crc ^= *p++;
    for (uint8_t k=0; k<8; ++k) {
      if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320UL;
      else         crc >>= 1;
    }
  }
  return crc;
}

// Firma EXACTA que usa tu main.cpp: (uint16_t&, uint16_t&, uint32_t&)
bool mbLoadFromEEPROM(uint16_t &nDev, uint16_t &nTag, uint32_t &crcOut) {
  nDev = 0;
  nTag = 0;
  crcOut = 0;

  if (g_mbEepromBase == 0 || g_mbEepromSize < 64) {
    return false;
  }

  uint32_t off = g_mbEepromBase;

  uint32_t magic = 0;
  EE::get(off, &magic, sizeof(magic));
  if (magic != 0x4D424445UL) { // 'M''B''D''E'
    return false;
  }
  off += sizeof(magic);

  uint8_t ver = EE::read(off++);
  if (ver != 1) {
    return false;
  }
  off += 3; // reservados

  uint16_t storedDev = 0, storedTag = 0;
  EE::get(off, &storedDev, sizeof(storedDev)); off += sizeof(storedDev);
  EE::get(off, &storedTag, sizeof(storedTag)); off += sizeof(storedTag);

  uint32_t storedCrc = 0;
  EE::get(off, &storedCrc, sizeof(storedCrc)); off += sizeof(storedCrc);

  if (storedDev > MAX_MB_DEV || storedTag > MAX_MB_TAGS) {
    return false;
  }

  size_t headerSize = 4 + 1 + 3 + 2 + 2 + 4;
  size_t needed = headerSize +
                  (size_t)storedDev * sizeof(MbDevicePersist) +
                  (size_t)storedTag * sizeof(MbTagPersist);
  if (needed > g_mbEepromSize) {
    return false;
  }

  // Limpiar tablas actuales
  memset(g_mbDevices, 0, sizeof(g_mbDevices));
  memset(g_mbTags,     0, sizeof(g_mbTags));

  uint32_t crc = 0xFFFFFFFFUL;

  // Cargar devices
  for (uint16_t i=0; i<storedDev; ++i) {
    MbDevicePersist pd;
    EE::get(off, &pd, sizeof(pd)); off += sizeof(pd);
    crc = mbCrc32Update(crc, &pd, sizeof(pd));

    int slot = findFreeDev();
    if (slot < 0) continue;
    MbDevice &d = g_mbDevices[slot];
    d.id       = pd.id;
    d.bus      = (MbBus)pd.bus;
    d.unit     = pd.unit;
    d.port     = pd.port;
    d.periodMs = pd.periodMs;
    d.ip       = IPAddress(pd.ip[0], pd.ip[1], pd.ip[2], pd.ip[3]);
    strncpy(d.name, pd.name, sizeof(d.name)-1);
    d.name[sizeof(d.name)-1] = '\0';
    d.used = true;
  }

  // Cargar tags
  for (uint16_t i=0; i<storedTag; ++i) {
    MbTagPersist pt;
    EE::get(off, &pt, sizeof(pt)); off += sizeof(pt);
    crc = mbCrc32Update(crc, &pt, sizeof(pt));

    int slot = findFreeTag();
    if (slot < 0) continue;
    MbTag &t = g_mbTags[slot];
    t.used     = true;
    t.isOutput = (pt.isOutput != 0);
    t.devId    = pt.devId;
    t.func     = (MbFunc)pt.func;
    t.addr     = pt.addr;
    t.qty      = pt.qty;
    t.periodMs = pt.periodMs;
    t.scale    = pt.scale;
    t.offset   = pt.offset;
    strncpy(t.name, pt.name, sizeof(t.name)-1);
    t.name[sizeof(t.name)-1] = '\0';
  }

  crc ^= 0xFFFFFFFFUL;

  if (crc != storedCrc) {
    // CRC incorrecto → consideramos inválido y limpiamos tablas
    memset(g_mbDevices, 0, sizeof(g_mbDevices));
    memset(g_mbTags,     0, sizeof(g_mbTags));
    return false;
  }

  nDev   = storedDev;
  nTag   = storedTag;
  crcOut = storedCrc;
  return true;
}

// Si en algún sitio llamas a MB.SAVE (mbSaveToEEPROM), ahora guarda de verdad:
void mbSaveToEEPROM(uint16_t /*nDev*/, uint16_t /*nTag*/, uint32_t /*crc*/) {
  if (g_mbEepromBase == 0 || g_mbEepromSize < 64) {
    enviar("MB.SAVE: ventana EEPROM Modbus no inicializada");
    return;
  }

  // Contar entries usados
  uint16_t usedDev = 0, usedTag = 0;
  for (int i=0;i<MAX_MB_DEV;i++)  if (g_mbDevices[i].used) usedDev++;
  for (int i=0;i<MAX_MB_TAGS;i++) if (g_mbTags[i].used)    usedTag++;

  size_t headerSize = 4 + 1 + 3 + 2 + 2 + 4;
  size_t needed = headerSize +
                  (size_t)usedDev * sizeof(MbDevicePersist) +
                  (size_t)usedTag * sizeof(MbTagPersist);

  if (needed > g_mbEepromSize) {
    enviar("MB.SAVE: sin espacio en ventana EEPROM");
    return;
  }

  uint32_t base = g_mbEepromBase;
  uint32_t off  = base;

  const uint32_t magic   = 0x4D424445UL; // 'MBDE'
  const uint8_t  version = 1;

  // Header preliminar (CRC se escribe más tarde)
  EE::put(off, &magic, sizeof(magic)); off += sizeof(magic);
  EE::write(off++, version);
  EE::write(off++, 0);
  EE::write(off++, 0);
  EE::write(off++, 0);

  EE::put(off, &usedDev, sizeof(usedDev)); off += sizeof(usedDev);
  EE::put(off, &usedTag, sizeof(usedTag)); off += sizeof(usedTag);

  uint32_t crcPos = off;
  uint32_t zeroCrc = 0;
  EE::put(off, &zeroCrc, sizeof(zeroCrc)); off += sizeof(zeroCrc);

  uint32_t crc = 0xFFFFFFFFUL;

  // Guardar devices
  for (int i=0;i<MAX_MB_DEV;i++) if (g_mbDevices[i].used) {
    const MbDevice &d = g_mbDevices[i];
    MbDevicePersist pd{};
    pd.id       = d.id;
    pd.bus      = (uint8_t)d.bus;
    pd.unit     = d.unit;
    pd.port     = d.port;
    pd.periodMs = d.periodMs;
    pd.ip[0]    = d.ip[0];
    pd.ip[1]    = d.ip[1];
    pd.ip[2]    = d.ip[2];
    pd.ip[3]    = d.ip[3];
    strncpy(pd.name, d.name, sizeof(pd.name)-1);
    pd.name[sizeof(pd.name)-1] = '\0';

    EE::put(off, &pd, sizeof(pd)); off += sizeof(pd);
    crc = mbCrc32Update(crc, &pd, sizeof(pd));
  }

  // Guardar tags
  for (int i=0;i<MAX_MB_TAGS;i++) if (g_mbTags[i].used) {
    const MbTag &t = g_mbTags[i];
    MbTagPersist pt{};
    pt.isOutput = t.isOutput ? 1 : 0;
    pt.devId    = t.devId;
    pt.func     = (uint8_t)t.func;
    pt.addr     = t.addr;
    pt.qty      = t.qty;
    pt.periodMs = t.periodMs;
    pt.scale    = t.scale;
    pt.offset   = t.offset;
    strncpy(pt.name, t.name, sizeof(pt.name)-1);
    pt.name[sizeof(pt.name)-1] = '\0';

    EE::put(off, &pt, sizeof(pt)); off += sizeof(pt);
    crc = mbCrc32Update(crc, &pt, sizeof(pt));
  }

  crc ^= 0xFFFFFFFFUL;

  // Escribir CRC definitivo en header
  EE::put(crcPos, &crc, sizeof(crc));

  EE::commit();

  enviar("✅ MB.SAVE dev=" + String(usedDev) +
         " tag=" + String(usedTag) +
         " crc=" + String((unsigned long)crc, 16));
}

// Inicialización antigua: ahora simplemente llama a mbInit()
void initModbusTCP() {
  mbInit();   // inicializa tablas y Modbus TCP+RTU
}

// Tick antiguo: ahora llama a mbTask()
void tickModbus() {
  mbTask();   // procesa Modbus TCP+RTU en cada loop
}
