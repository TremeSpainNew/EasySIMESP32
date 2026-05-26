#pragma once
#include <Arduino.h>
#include <WiFi.h>

// Usa la librería del repo emelianov para cliente TCP
// (si tu instalación usa <ModbusIP.h>, cámbialo aquí)
#include <ModbusIP_ESP8266.h>

// === Persistencia sobre NVS (EE.h) ===
#include "EE.h"  // reemplaza EEPROM.* por EE::*

// ======================= Tipos y estructuras =======================
enum MbFunc : uint8_t { MB_HREG=0, MB_IREG=1, MB_COIL=2, MB_ISTS=3 };

struct MbDevice {
  uint8_t   id;       // identificador lógico interno (0..250 recomendable)
  IPAddress ip;       // IP Modbus TCP
  uint8_t   unit;     // UnitId (slaveId)
  bool      used;
};

struct MbTag {
  bool      isOutput;   // false=entrada (lectura), true=salida (escritura)
  uint8_t   devId;      // referencia a MbDevice.id
  MbFunc    func;       // HREG, IREG, COIL, ISTS
  uint16_t  addr;       // dirección base (0-based)
  uint16_t  qty;        // cantidad de registros/bits
  uint16_t  periodMs;   // periodo de lectura (para IN). (no usado aún)
  float     scale;      // y = raw*scale + offset
  float     offset;
  char      name[20];   // nombre clave (null-terminated)
  bool      used;
};

// ---- IO lógico Modbus para la app/pantalla (no usa pines) ----
struct MbIO {
  char name[20];    // nombre visible para UI
  char tag[20];     // tag MB asociado (igual por defecto)
  bool used;
  bool isOutput;    // cache (derivado del tag)
};

// ======================= Parámetros en RAM =======================
static const int MAX_MB_DEV  = 16;
static const int MAX_MB_TAGS = 64;
static const int MAX_MB_IO   = 64;

static MbDevice g_mbDevices[MAX_MB_DEV];
static MbTag    g_mbTags[MAX_MB_TAGS];
static MbIO     g_mbIOs[MAX_MB_IO];

// Cliente Modbus/TCP global
static ModbusIP md;

// ======================= IO hacia fuera (Serial/Eth) =======================
extern void enviar(const String& msg); // implementada en tu main

// ======================= Helpers de tabla =======================
static int findFreeDev() {
  for (int i=0;i<MAX_MB_DEV;i++) if (!g_mbDevices[i].used) return i;
  return -1;
}
static int findDevById(uint8_t id) {
  for (int i=0;i<MAX_MB_DEV;i++)
    if (g_mbDevices[i].used && g_mbDevices[i].id==id) return i;
  return -1;
}
static int findFreeTag() {
  for (int i=0;i<MAX_MB_TAGS;i++) if (!g_mbTags[i].used) return i;
  return -1;
}
static int findTagByName(const char* name) {
  for (int i=0;i<MAX_MB_TAGS;i++)
    if (g_mbTags[i].used && strcasecmp(g_mbTags[i].name,name)==0) return i;
  return -1;
}
static bool parseFunc(const char* s, MbFunc& f) {
  if (!strcasecmp(s,"HREG")) { f=MB_HREG; return true; }
  if (!strcasecmp(s,"IREG")) { f=MB_IREG; return true; }
  if (!strcasecmp(s,"COIL")) { f=MB_COIL; return true; }
  if (!strcasecmp(s,"ISTS")) { f=MB_ISTS; return true; }
  return false;
}
static bool funcIsWritable(MbFunc f) { return (f==MB_HREG || f==MB_COIL); }

// ---- IO helpers ----
static int findFreeMbIO() {
  for (int i=0;i<MAX_MB_IO;i++) if (!g_mbIOs[i].used) return i;
  return -1;
}
static int findMbIOByName(const char* name) {
  for (int i=0;i<MAX_MB_IO;i++)
    if (g_mbIOs[i].used && strcasecmp(g_mbIOs[i].name,name)==0) return i;
  return -1;
}
static void deriveIsOutput(MbIO &io) {
  int ti = findTagByName(io.tag);
  io.isOutput = (ti>=0) ? g_mbTags[ti].isOutput : false;
}

// ======================= Conexión / transacciones =======================
static bool mbEnsureConn(ModbusIP &cli, IPAddress ip, uint16_t port = MODBUSTCP_PORT, uint32_t ms = 350) {
  unsigned long t0 = millis();
  if (!cli.isConnected(ip)) {
    cli.connect(ip, port);
    while (!cli.isConnected(ip) && (millis() - t0) < ms) cli.task();
  }
  return cli.isConnected(ip);
}

// Esperar hasta que la transacción termine (o timeout)
static bool mbWaitTrans(ModbusIP &cli, uint16_t tid, uint32_t ms = 800) {
  if (!tid) return false;
  unsigned long t0 = millis();
  while (cli.isTransaction(tid) && (millis()-t0) < ms) cli.task();
  return !cli.isTransaction(tid); // true si terminó
}

// ======================= “EEPROM” (persistencia con EE.h) =======================

// ---- Ventana para MODBUS: configurables en runtime desde el main ----
static uint32_t MB_EE_BASE = 0;     // offset inicial (lo fija tu app)
static uint32_t MB_EE_SIZE = 4096;  // tamaño ventana (lo fija tu app)

// Llamar en el setup ANTES de usar MB.SAVE/LOAD/DUMP
inline void modbusSetEepromWindow(uint32_t base, uint32_t size) {
  MB_EE_BASE = base;
  MB_EE_SIZE = size;
}

static const uint32_t MB_MAGIC     = 0x4D42504B; // 'MBPK'
static const uint16_t MB_VERSION   = 0x0002;     // v2: incluye IOs Modbus

// FNV-1a 32
static uint32_t fnv1a32(const uint8_t* data, size_t len) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < len; ++i) { h ^= data[i]; h *= 16777619u; }
  return h;
}
static uint32_t fnv1a32_update(uint32_t h, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) { h ^= data[i]; h *= 16777619u; }
  return h;
}

// Acceso relativo a la ventana usando EE.h
template<typename T>
static void eeWrite(uint32_t &pos, const T &v) {
  EE::put(MB_EE_BASE + pos, &v, sizeof(T));
  pos += sizeof(T);
}
template<typename T>
static void eeRead(uint32_t &pos, T &v) {
  EE::get(MB_EE_BASE + pos, &v, sizeof(T));
  pos += sizeof(T);
}

// Empaquetado estable (sin padding)
static inline size_t packedDeviceSize()   { return 7;  }   // id(1)+ip(4)+unit(1)+used(1)
static inline size_t packedTagSize_v1()   { return 36; }   // compat v1
static inline size_t packedTagSize_v2()   { return 38; }   // la actual (packTag abajo)
static inline size_t packedMbIOSize()     { return 41; }   // name[20]+tag[20]+used(1)
static inline size_t packedHeaderSizeV2() { return 4+2+2+2+2; } // magic+ver+nDev+nTag+nIO
static inline size_t packedCrcSize()      { return 4; }

// Pack/unpack DEV
static void packDevice(uint8_t* out, size_t &ofs, const MbDevice& d) {
  out[ofs++] = d.id;
  out[ofs++] = d.ip[0]; out[ofs++] = d.ip[1]; out[ofs++] = d.ip[2]; out[ofs++] = d.ip[3];
  out[ofs++] = d.unit;
  out[ofs++] = d.used ? 1 : 0;
}
static void unpackDevice(const uint8_t* in, size_t &ofs, MbDevice& d) {
  d.id = in[ofs++];
  d.ip = IPAddress(in[ofs], in[ofs+1], in[ofs+2], in[ofs+3]); ofs += 4;
  d.unit = in[ofs++];
  d.used = in[ofs++] != 0;
}

// Pack/unpack TAG
static void packTag(uint8_t* out, size_t &ofs, const MbTag& t) {
  out[ofs++] = t.isOutput ? 1 : 0;
  out[ofs++] = t.devId;
  out[ofs++] = (uint8_t)t.func;
  out[ofs++] = (uint8_t)(t.addr & 0xFF); out[ofs++] = (uint8_t)(t.addr >> 8);
  out[ofs++] = (uint8_t)(t.qty & 0xFF);  out[ofs++] = (uint8_t)(t.qty  >> 8);
  out[ofs++] = (uint8_t)(t.periodMs & 0xFF); out[ofs++] = (uint8_t)(t.periodMs >> 8);
  union { float f; uint8_t b[4]; } u;
  u.f = t.scale;  for (int i=0;i<4;i++) out[ofs++] = u.b[i];
  u.f = t.offset; for (int i=0;i<4;i++) out[ofs++] = u.b[i];
  for (int i=0;i<20;i++) out[ofs++] = (uint8_t)t.name[i];
  out[ofs++] = t.used ? 1 : 0;
}
static void unpackTag(const uint8_t* in, size_t &ofs, MbTag& t) {
  t.isOutput = in[ofs++] != 0;
  t.devId    = in[ofs++];
  t.func     = (MbFunc)in[ofs++];
  t.addr     = (uint16_t)in[ofs++] | ((uint16_t)in[ofs++]<<8);
  t.qty      = (uint16_t)in[ofs++] | ((uint16_t)in[ofs++]<<8);
  t.periodMs = (uint16_t)in[ofs++] | ((uint16_t)in[ofs++]<<8);
  union { float f; uint8_t b[4]; } u;
  for (int i=0;i<4;i++) u.b[i] = in[ofs++]; t.scale  = u.f;
  for (int i=0;i<4;i++) u.b[i] = in[ofs++]; t.offset = u.f;
  for (int i=0;i<20;i++) t.name[i] = (char)in[ofs++];
  t.name[19] = '\0';
  t.used = in[ofs++] != 0;
}

// Pack/unpack IO
static void packIO(uint8_t* out, size_t &ofs, const MbIO& io) {
  for (int i=0;i<20;i++) out[ofs++] = (uint8_t)io.name[i];
  for (int i=0;i<20;i++) out[ofs++] = (uint8_t)io.tag[i];
  out[ofs++] = io.used ? 1 : 0;
}
static void unpackIO(const uint8_t* in, size_t &ofs, MbIO& io) {
  for (int i=0;i<20;i++) io.name[i] = (char)in[ofs++]; io.name[19]='\0';
  for (int i=0;i<20;i++) io.tag[i]  = (char)in[ofs++]; io.tag[19]='\0';
  io.used = in[ofs++] != 0;
  deriveIsOutput(io);
}

// ====== Estimación de tamaño y guardas de capacidad ======
static void mbCountAll(uint16_t &nDev, uint16_t &nTag, uint16_t &nIO) {
  nDev=nTag=nIO=0;
  for (int i=0;i<MAX_MB_DEV;i++)  if (g_mbDevices[i].used) nDev++;
  for (int i=0;i<MAX_MB_TAGS;i++) if (g_mbTags[i].used)   nTag++;
  for (int i=0;i<MAX_MB_IO;i++)   if (g_mbIOs[i].used)    nIO++;
}
static size_t mbEstimateBytes(uint16_t addDev, uint16_t addTag, uint16_t addIO) {
  uint16_t nDev, nTag, nIO; mbCountAll(nDev,nTag,nIO);
  size_t payload = (size_t)(nDev+addDev) * packedDeviceSize()
                 + (size_t)(nTag+addTag) * packedTagSize_v2()
                 + (size_t)(nIO +addIO ) * packedMbIOSize();
  return packedHeaderSizeV2() + payload + packedCrcSize();
}
static bool mbHasRoomFor(uint16_t addDev, uint16_t addTag, uint16_t addIO) {
  size_t need = mbEstimateBytes(addDev, addTag, addIO);
  return need <= MB_EE_SIZE;
}

// Guardar / Cargar / Borrar
// ====== GUARDAR troceando por piezas ======
static bool mbSaveToEEPROM(uint16_t &outDev, uint16_t &outTag, uint32_t &outCrc) {
  if (!mbHasRoomFor(0,0,0)) return false;

  uint16_t nDev=0, nTag=0, nIO=0;
  for (int i=0;i<MAX_MB_DEV;i++)  if (g_mbDevices[i].used) nDev++;
  for (int i=0;i<MAX_MB_TAGS;i++) if (g_mbTags[i].used)   nTag++;
  for (int i=0;i<MAX_MB_IO;i++)   if (g_mbIOs[i].used)    nIO++;

  uint32_t pos = 0;

  // Cabecera (mismos bytes/orden que lee LOAD)
  const uint32_t magic = MB_MAGIC;
  const uint16_t ver   = MB_VERSION;

  // Para CRC sobre header+payload
  uint8_t head[4+2+2+2+2];
  size_t  hOfs = 0;
  memcpy(&head[hOfs], &magic, 4); hOfs += 4;
  memcpy(&head[hOfs], &ver,   2); hOfs += 2;
  memcpy(&head[hOfs], &nDev,  2); hOfs += 2;
  memcpy(&head[hOfs], &nTag,  2); hOfs += 2;
  memcpy(&head[hOfs], &nIO,   2); hOfs += 2;

  // Escribe cabecera troceada y prepara CRC
  EE::put(MB_EE_BASE + pos, &magic, 4); pos += 4;
  EE::put(MB_EE_BASE + pos, &ver,   2); pos += 2;
  EE::put(MB_EE_BASE + pos, &nDev,  2); pos += 2;
  EE::put(MB_EE_BASE + pos, &nTag,  2); pos += 2;
  EE::put(MB_EE_BASE + pos, &nIO,   2); pos += 2;

  uint32_t crc = fnv1a32(head, hOfs);

  // Devices
  for (int i=0;i<MAX_MB_DEV;i++) if (g_mbDevices[i].used) {
    uint8_t buf[64]; size_t ofs=0;
    packDevice(buf, ofs, g_mbDevices[i]);                // 7 bytes
    EE::put(MB_EE_BASE + pos, buf, ofs); pos += ofs;
    crc = fnv1a32_update(crc, buf, ofs);
  }

  // Tags
  for (int i=0;i<MAX_MB_TAGS;i++) if (g_mbTags[i].used) {
    uint8_t buf[64]; size_t ofs=0;
    packTag(buf, ofs, g_mbTags[i]);                      // 38 bytes
    EE::put(MB_EE_BASE + pos, buf, ofs); pos += ofs;
    crc = fnv1a32_update(crc, buf, ofs);
  }

  // IOs
  for (int i=0;i<MAX_MB_IO;i++) if (g_mbIOs[i].used) {
    uint8_t buf[64]; size_t ofs=0;
    packIO(buf, ofs, g_mbIOs[i]);                        // 41 bytes
    EE::put(MB_EE_BASE + pos, buf, ofs); pos += ofs;
    crc = fnv1a32_update(crc, buf, ofs);
  }

  // CRC final (4 bytes)
  EE::put(MB_EE_BASE + pos, &crc, 4); pos += 4;

  // Commit
  EE::commit();

  outDev = nDev; outTag = nTag; outCrc = crc;
  return true;
}

// ====== CARGAR leyendo troceado (sin get grande) ======
static bool mbLoadFromEEPROM(uint16_t &outDev, uint16_t &outTag, uint32_t &outCrc) {
  if (MB_EE_SIZE < packedHeaderSizeV2()+packedCrcSize()) return false;

  uint32_t pos = 0;

  // Lee cabecera (troceada)
  uint32_t magic=0; EE::get(MB_EE_BASE + pos, &magic, 4); pos += 4;
  if (magic != MB_MAGIC) return false;

  uint16_t ver=0, nDev=0, nTag=0, nIO=0;
  EE::get(MB_EE_BASE + pos, &ver,  2); pos += 2;
  EE::get(MB_EE_BASE + pos, &nDev, 2); pos += 2;
  EE::get(MB_EE_BASE + pos, &nTag, 2); pos += 2;
  if (ver >= 2) { EE::get(MB_EE_BASE + pos, &nIO, 2); pos += 2; }

  // Recalcula CRC sobre header (igual que SAVE)
  uint8_t head[4+2+2+2+2];
  size_t  hOfs = 0;
  memcpy(&head[hOfs], &magic, 4); hOfs += 4;
  memcpy(&head[hOfs], &ver,   2); hOfs += 2;
  memcpy(&head[hOfs], &nDev,  2); hOfs += 2;
  memcpy(&head[hOfs], &nTag,  2); hOfs += 2;
  if (ver >= 2) { memcpy(&head[hOfs], &nIO, 2); hOfs += 2; }

  uint32_t crcCalc = fnv1a32(head, hOfs);

  // Limpia RAM
  for (int i=0;i<MAX_MB_DEV;i++)  g_mbDevices[i].used = false;
  for (int i=0;i<MAX_MB_TAGS;i++) g_mbTags[i].used   = false;
  for (int i=0;i<MAX_MB_IO;i++)   g_mbIOs[i].used    = false;

  // Devices
  for (uint16_t i=0;i<nDev && i<MAX_MB_DEV; i++) {
    uint8_t buf[64]; size_t ofs=packedDeviceSize();
    EE::get(MB_EE_BASE + pos, buf, ofs); pos += ofs;
    MbDevice d; size_t inOfs=0; unpackDevice(buf, inOfs, d);
    int idx = findFreeDev(); if (idx>=0) g_mbDevices[idx] = d;
    crcCalc = fnv1a32_update(crcCalc, buf, ofs);
  }

  // Tags
  for (uint16_t i=0;i<nTag && i<MAX_MB_TAGS; i++) {
    uint8_t buf[64]; size_t ofs=(ver>=2)?packedTagSize_v2():packedTagSize_v1();
    EE::get(MB_EE_BASE + pos, buf, ofs); pos += ofs;
    MbTag t; size_t inOfs=0; unpackTag(buf, inOfs, t);
    int idx = findFreeTag(); if (idx>=0) g_mbTags[idx] = t;
    crcCalc = fnv1a32_update(crcCalc, buf, ofs);
  }

  // IOs (solo v2+)
  if (ver >= 2) {
    for (uint16_t i=0;i<nIO && i<MAX_MB_IO; i++) {
      uint8_t buf[64]; size_t ofs=packedMbIOSize();
      EE::get(MB_EE_BASE + pos, buf, ofs); pos += ofs;
      MbIO io; size_t inOfs=0; unpackIO(buf, inOfs, io);
      int idx = findFreeMbIO(); if (idx>=0) g_mbIOs[idx] = io;
      crcCalc = fnv1a32_update(crcCalc, buf, ofs);
    }
  }

  // Lee CRC almacenado y compara
  uint32_t crcStored=0; EE::get(MB_EE_BASE + pos, &crcStored, 4); pos += 4;
  if (crcCalc != crcStored) return false;

  outDev = nDev; outTag = nTag; outCrc = crcStored;
  return true;
}

static void mbClearEEPROM() {
  // Borra la ventana escribiendo 0xFF (opcional)
  // Aquí basta con invalidar magic
  uint32_t pos = 0;
  uint32_t magic = 0xFFFFFFFF;
  eeWrite(pos, magic);
  EE::commit();
}

// ======================= API público para el main =======================
inline void initModbusTCP() {
  // Si además quieres servidor Modbus, aquí iría md.server();
}
inline void tickModbus() {
  md.task(); // bombear estado de conexiones/transacciones
}

// ======================= Comandos MB.* y MODBUS.* =======================
inline void handleMbCommand(const String& cmd) {

  // =================== MODBUS.* (IO lógico para UI) ===================

  // MODBUS.ADD <name> <devId> <HREG|IREG|COIL|ISTS> <addr> <qty> [scale] [offset]
  {
    char name[24], fstr[8]; int devId, addr, qty; float sc=1.0f, off=0.0f;
    int n = sscanf(cmd.c_str(), "MODBUS.ADD %23s %d %7s %d %d %f %f",
                   name, &devId, fstr, &addr, &qty, &sc, &off);
    if (n >= 5) {
      bool willCreateTag = (findTagByName(name) < 0);
      if (!mbHasRoomFor(0, willCreateTag?1:0, 1)) { enviar("⚠️ MODBUS.ADD: sin espacio para IO/TAG en EEPROM Modbus"); return; }

      if (findMbIOByName(name) >= 0) { enviar("❌ MODBUS.ADD: name duplicado"); return; }
      MbFunc f; if (!parseFunc(fstr,f)) { enviar("❌ MODBUS.ADD: func inválida"); return; }
      if (findDevById((uint8_t)devId) < 0) { enviar("❌ MODBUS.ADD: devId desconocido"); return; }
      if (qty <= 0 || qty > 64) { enviar("❌ MODBUS.ADD: qty fuera de rango"); return; }

      // Crea tag si no existe (usa el mismo name para simplificar)
      if (willCreateTag) {
        int slot = findFreeTag(); if (slot < 0) { enviar("❌ MODBUS.ADD: sin espacio para tags"); return; }
        MbTag& t = g_mbTags[slot];
        t.used     = true;
        t.isOutput = (f==MB_COIL || f==MB_HREG);
        t.devId    = (uint8_t)devId;
        t.func     = f;
        t.addr     = (uint16_t)addr;
        t.qty      = (uint16_t)qty;
        t.periodMs = 0;
        t.scale    = sc;
        t.offset   = off;
        strncpy(t.name, name, sizeof(t.name)-1);
        t.name[sizeof(t.name)-1] = '\0';
      }

      int ioSlot = findFreeMbIO(); if (ioSlot < 0) { enviar("❌ MODBUS.ADD: sin espacio para IOs"); return; }
      MbIO &io = g_mbIOs[ioSlot];
      memset(&io, 0, sizeof(io));
      strncpy(io.name, name, sizeof(io.name)-1);
      strncpy(io.tag,  name, sizeof(io.tag)-1);
      io.used = true;
      deriveIsOutput(io);

      enviar("✅ MODBUS.ADD OK");
      return;
    }
  }

  // MODBUS.DEL <name>
  {
    char name[24];
    if (sscanf(cmd.c_str(), "MODBUS.DEL %23s", name) == 1) {
      int i = findMbIOByName(name);
      if (i < 0) { enviar("❌ MODBUS.DEL: no encontrado"); return; }
      g_mbIOs[i].used = false;
      enviar("✅ MODBUS.DEL OK");
      return;
    }
  }

  // MODBUS.LS
  if (cmd.startsWith("MODBUS.LS")) {
    enviar("📋 IO MODBUS:");
    for (int i=0;i<MAX_MB_IO;i++) if (g_mbIOs[i].used) {
      int ti = findTagByName(g_mbIOs[i].tag);
      if (ti>=0) {
        const MbTag& t = g_mbTags[ti];
        const char* f =
          (t.func==MB_HREG?"HREG":
          (t.func==MB_IREG?"IREG":
          (t.func==MB_COIL?"COIL":"ISTS")));
        char buf[200];
        snprintf(buf, sizeof(buf),
          "  %s io=%s tag=%s dev=%d func=%s addr=%u qty=%u",
          t.isOutput?"OUT":"IN", g_mbIOs[i].name, t.name, t.devId, f, t.addr, t.qty);
        enviar(String(buf));
      } else {
        enviar(String("  ? io=") + g_mbIOs[i].name + " (tag perdido)");
      }
    }
    return;
  }

  // MODBUS.READ <name>
  {
    char name[24];
    if (sscanf(cmd.c_str(), "MODBUS.READ %23s", name) == 1) {
      int i = findMbIOByName(name);
      if (i < 0) { enviar("❌ MODBUS.READ: io no encontrado"); return; }
      String s = "MB.READ ";
      s += g_mbIOs[i].tag;
      handleMbCommand(s);
      return;
    }
  }

  // MODBUS.SET <name> <vals...>
  {
    char name[24];
    if (sscanf(cmd.c_str(), "MODBUS.SET %23s", name) == 1) {
      int i = findMbIOByName(name);
      if (i < 0) { enviar("❌ MODBUS.SET: io no encontrado"); return; }
      deriveIsOutput(g_mbIOs[i]);
      if (!g_mbIOs[i].isOutput) { enviar("❌ MODBUS.SET: io no es salida"); return; }

      // Construye "MB.SET <tag> <vals...>"
      const char* p = strchr(cmd.c_str(), ' ');
      p = p ? p+1 : nullptr; // salta "MODBUS.SET"
      p = p ? strchr(p, ' ') : nullptr; // salta <name>
      if (!p) { enviar("❌ MODBUS.SET: faltan valores"); return; }
      p++;

      String s = "MB.SET ";
      s += g_mbIOs[i].tag; s += " ";
      s += String(p);
      handleMbCommand(s);
      return;
    }
  }

  // =================== MB.* ===================

  // ---------- MB.PINGIP <ip> [port] ----------
  {
    char ipStr[32]; int port = MODBUSTCP_PORT;
    if (sscanf(cmd.c_str(), "MB.PINGIP %31s %d", ipStr, &port) >= 1) {
      IPAddress ip;
      if (!ip.fromString(ipStr)) { enviar("❌ MB.PINGIP: IP inválida"); return; }
      unsigned long t0 = millis();
      bool ok = mbEnsureConn(md, ip, (uint16_t)port, 500);
      if (!ok) { char b[96]; snprintf(b,sizeof(b), "⚠️ MB.PINGIP FAIL ip=%s port=%d", ipStr, port); enviar(b); return; }
      unsigned long rtt = millis() - t0;
      char b[96]; snprintf(b,sizeof(b), "✅ MB.PINGIP OK ip=%s port=%d rtt=%lums", ipStr, port, rtt);
      enviar(b);
      return;
    }
  }

  // ---------- MB.ADDDEV <id> <ip> <unit> ----------
  {
    int id, unit; char ipStr[32];
    if (sscanf(cmd.c_str(), "MB.ADDDEV %d %31s %d", &id, ipStr, &unit) == 3) {
      if (!mbHasRoomFor(1,0,0)) { enviar("⚠️ MB.ADDDEV: sin espacio en EEPROM Modbus"); return; }
      if (id < 0 || id > 250) { enviar("❌ ID inválido (0..250)"); return; }
      IPAddress ip;
      if (!ip.fromString(ipStr)) { enviar("❌ IP inválida"); return; }
      if (findDevById((uint8_t)id) != -1) { enviar("❌ ID duplicado"); return; }
      int slot = findFreeDev();
      if (slot < 0) { enviar("❌ Sin espacio para más dispositivos"); return; }
      g_mbDevices[slot].id   = (uint8_t)id;
      g_mbDevices[slot].ip   = ip;
      g_mbDevices[slot].unit = (uint8_t)unit;
      g_mbDevices[slot].used = true;
      enviar("✅ MB.ADDDEV OK");
      return;
    }
  }

  // ---------- MB.DELDEV <id> ----------
  {
    int id;
    if (sscanf(cmd.c_str(), "MB.DELDEV %d", &id) == 1) {
      int i = findDevById((uint8_t)id);
      if (i < 0) { enviar("❌ Dispositivo no encontrado"); return; }
      // limpia tags dependientes
      for (int t=0;t<MAX_MB_TAGS;t++)
        if (g_mbTags[t].used && g_mbTags[t].devId == (uint8_t)id) g_mbTags[t].used = false;
      g_mbDevices[i].used = false;
      enviar("✅ MB.DELDEV OK");
      return;
    }
  }

  // ---------- MB.LSDEV ----------
  if (cmd.startsWith("MB.LSDEV")) {
    enviar("📋 Dispositivos:");
    for (int i=0;i<MAX_MB_DEV;i++) if (g_mbDevices[i].used) {
      char buf[96];
      snprintf(buf, sizeof(buf), "  id=%d ip=%u.%u.%u.%u unit=%d",
        g_mbDevices[i].id,
        g_mbDevices[i].ip[0], g_mbDevices[i].ip[1], g_mbDevices[i].ip[2], g_mbDevices[i].ip[3],
        g_mbDevices[i].unit);
      enviar(String(buf));
    }
    return;
  }

  // ---------- MB.PING <devId> ----------
  {
    int devId;
    if (sscanf(cmd.c_str(), "MB.PING %d", &devId) == 1) {
      int di = findDevById((uint8_t)devId);
      if (di < 0) { enviar("❌ MB.PING: devId no encontrado"); return; }
      MbDevice &d = g_mbDevices[di];
      unsigned long t0 = millis();
      bool ok = mbEnsureConn(md, d.ip, MODBUSTCP_PORT, 500);
      if (!ok) { char b[120]; snprintf(b,sizeof(b), "⚠️ MB.PING FAIL dev=%d ip=%u.%u.%u.%u port=%d",
                                      d.id, d.ip[0],d.ip[1],d.ip[2],d.ip[3], MODBUSTCP_PORT); enviar(b); return; }
      unsigned long rtt = millis()-t0;
      char b[120]; snprintf(b,sizeof(b), "✅ MB.PING OK dev=%d ip=%u.%u.%u.%u port=%d rtt=%lums",
                            d.id, d.ip[0],d.ip[1],d.ip[2],d.ip[3], MODBUSTCP_PORT, rtt);
      enviar(b);
      return;
    }
  }

  // ---------- MB.ADDIN  <devId> <HREG|IREG|COIL|ISTS> <addr> <qty> <name> [periodMs] [scale] [offset] ----------
  {
    int devId, addr, qty; char fstr[8], name[24];
    unsigned long period=200; float sc=1.0f, off=0.0f;
    int n = sscanf(cmd.c_str(), "MB.ADDIN %d %7s %d %d %23s %lu %f %f",
                  &devId, fstr, &addr, &qty, name, &period, &sc, &off);
    if (n >= 5) {
      if (!mbHasRoomFor(0,1,0)) { enviar("⚠️ MB.ADDIN: sin espacio para TAG"); return; }
      MbFunc f; if (!parseFunc(fstr,f)) { enviar("❌ Func inválida"); return; }
      if (findDevById((uint8_t)devId) < 0) { enviar("❌ devId desconocido"); return; }
      if (qty <= 0 || qty > 32) { enviar("❌ qty fuera de rango (1..32)"); return; }
      if (findTagByName(name) >= 0) { enviar("❌ name duplicado"); return; }
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

  // ---------- MB.ADDOUT <devId> <HREG|COIL> <addr> <qty> <name> [periodMs] [scale] [offset] ----------
  {
    int devId, addr, qty; char fstr[8], name[24];
    unsigned long period=0; float sc=1.0f, off=0.0f;
    int n = sscanf(cmd.c_str(), "MB.ADDOUT %d %7s %d %d %23s %lu %f %f",
                  &devId, fstr, &addr, &qty, name, &period, &sc, &off);
    if (n >= 5) {
      if (!mbHasRoomFor(0,1,0)) { enviar("⚠️ MB.ADDOUT: sin espacio para TAG"); return; }
      MbFunc f; if (!parseFunc(fstr,f) || !funcIsWritable(f)) { enviar("❌ Func inválida para salida"); return; }
      if (findDevById((uint8_t)devId) < 0) { enviar("❌ devId desconocido"); return; }
      if (qty <= 0 || qty > 64) { enviar("❌ qty fuera de rango (1..64)"); return; }
      if (findTagByName(name) >= 0) { enviar("❌ name duplicado"); return; }
      int slot = findFreeTag(); if (slot < 0) { enviar("❌ Sin espacio para tags"); return; }
      MbTag& t = g_mbTags[slot];
      t.used     = true;
      t.isOutput = true;
      t.devId    = (uint8_t)devId;
      t.func     = f;
      t.addr     = (uint16_t)addr;
      t.qty      = (uint16_t)qty;
      t.periodMs = (uint16_t)period;
      t.scale    = sc;
      t.offset   = off;
      strncpy(t.name, name, sizeof(t.name)-1);
      t.name[sizeof(t.name)-1] = '\0';
      enviar("✅ MB.ADDOUT OK");
      return;
    }
  }

  // ---------- MB.DELTAG <name> ----------
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

  // ---------- MB.LSTAG ----------
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

  // ---------- MB.DUMP ----------
  if (cmd.startsWith("MB.DUMP")) {
    enviar("BEGIN MB");
    for (int i=0;i<MAX_MB_DEV;i++) if (g_mbDevices[i].used) {
      char buf[96];
      snprintf(buf, sizeof(buf), "MB.DEV %d %u.%u.%u.%u %d",
        g_mbDevices[i].id,
        g_mbDevices[i].ip[0], g_mbDevices[i].ip[1], g_mbDevices[i].ip[2], g_mbDevices[i].ip[3],
        g_mbDevices[i].unit);
      enviar(String(buf));
    }
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
    // IOs
    enviar("MB.IOS:");
    for (int i=0;i<MAX_MB_IO;i++) if (g_mbIOs[i].used) {
      char b[80];
      snprintf(b, sizeof(b), "MB.IO %s -> %s", g_mbIOs[i].name, g_mbIOs[i].tag);
      enviar(String(b));
    }
    enviar("END MB");
    return;
  }

  // ---------- MB.READ <name> ----------
  {
    char name[24];
    if (sscanf(cmd.c_str(), "MB.READ %23s", name) == 1) {
      int idx = findTagByName(name);
      if (idx < 0) { enviar("❌ MB.READ: tag no encontrado"); return; }
      MbTag &t = g_mbTags[idx];
      int di = findDevById(t.devId);
      if (di < 0) { enviar("❌ MB.READ: devId inválido"); return; }
      MbDevice &d = g_mbDevices[di];

      if (!mbEnsureConn(md, d.ip)) { enviar("⚠️ MB.READ: no conectado"); return; }

      uint16_t tid = 0;
      if (t.func == MB_COIL || t.func == MB_ISTS) {
        bool buf[64]; if (t.qty > 64) { enviar("❌ MB.READ: qty>64"); return; }
        if (t.func == MB_COIL)
          tid = md.readCoil(d.ip, t.addr, buf, t.qty, nullptr, d.unit);
        else
          tid = md.readIsts(d.ip, t.addr, buf, t.qty, nullptr, d.unit);

        if (tid == 0) { enviar("⚠️ MB.READ: transacción no pudo iniciarse"); return; }
        if (!mbWaitTrans(md, tid)) { enviar("⚠️ MB.READ sin confirmación"); return; }

        String out = "MB.VAL " + String(t.name) + " ";
        for (uint16_t i=0;i<t.qty;i++) out += (buf[i] ? "1 " : "0 ");
        enviar(out);
        return;
      } else {
        uint16_t buf[64]; if (t.qty > 64) { enviar("❌ MB.READ: qty>64"); return; }
        if (t.func == MB_HREG)
          tid = md.readHreg(d.ip, t.addr, buf, t.qty, nullptr, d.unit);
        else
          tid = md.readIreg(d.ip, t.addr, buf, t.qty, nullptr, d.unit);

        if (tid == 0) { enviar("⚠️ MB.READ: transacción no pudo iniciarse"); return; }
        if (!mbWaitTrans(md, tid)) { enviar("⚠️ MB.READ sin confirmación"); return; }

        String out = "MB.VAL " + String(t.name) + " ";
        for (uint16_t i=0;i*t.qty;i++) { /* placeholder to keep structure */ }
        for (uint16_t i=0;i<t.qty;i++) {
          float v = (float)buf[i] * t.scale + t.offset;
          char f[24]; dtostrf(v, 0, 3, f);
          out += f; out += " ";
        }
        enviar(out);
        return;
      }
    }
  }

  // ---------- MB.SET <name> <v0> [v1 ...] ----------
  {
    char name[24];
    if (sscanf(cmd.c_str(), "MB.SET %23s", name) == 1) {
      int idx = findTagByName(name);
      if (idx < 0) { enviar("❌ MB.SET: tag no encontrado"); return; }
      MbTag &t = g_mbTags[idx];
      if (!t.isOutput || !(t.func == MB_COIL || t.func == MB_HREG)) { enviar("❌ MB.SET: tag no es salida COIL/HREG"); return; }

      int devIdx = findDevById(t.devId);
      if (devIdx < 0) { enviar("❌ MB.SET: devId inválido"); return; }
      MbDevice &d = g_mbDevices[devIdx];

      // prepara buffers
      bool      bufCoil[64];
      uint16_t  bufHreg[64];
      if (t.qty > 64) { enviar("❌ MB.SET: qty>64"); return; }

      // Parsear valores después del nombre
      const char* p = strchr(cmd.c_str(), ' ');
      p = p ? p+1 : nullptr; // salta "MB.SET"
      p = p ? strchr(p, ' ') : nullptr; // salta <name>
      if (!p) { enviar("❌ MB.SET: faltan valores"); return; }
      p++;

      int got = 0;
      if (t.func == MB_COIL) {
        while (*p && got < t.qty) {
          int v; int n = sscanf(p, "%d", &v);
          if (n != 1) break;
          bufCoil[got++] = (v != 0);
          while (*p && *p != ' ') p++; // avanza
          while (*p == ' ') p++;
        }
        if (got != t.qty) { enviar("❌ MB.SET: num valores != qty"); return; }
      } else { // HREG
        while (*p && got < t.qty) {
          int v; int n = sscanf(p, "%d", &v);
          if (n != 1) break;
          bufHreg[got++] = (uint16_t)v;
          while (*p && *p != ' ') p++;
          while (*p == ' ') p++;
        }
        if (got != t.qty) { enviar("❌ MB.SET: num valores != qty"); return; }
      }

      if (!mbEnsureConn(md, d.ip)) { enviar("⚠️ MB: no conectado al esclavo"); return; }

      uint16_t tid = 0;

      if (t.func == MB_COIL) {
        // ⚠️ Forzar siempre FC5 (una a una) para evitar packing de FC15
        if (t.qty == 1) {
          tid = md.writeCoil(d.ip, t.addr, (bool)(bufCoil[0]), nullptr, d.unit); // FC5
          if (tid == 0) { enviar("⚠️ MB: no se pudo iniciar la transacción (FC5)"); return; }
          if (!mbWaitTrans(md, tid)) { enviar("⚠️ MB.SET enviado, sin confirmación (FC5)"); return; }
          enviar("✅ MB.SET OK (FC5)");
          return;
        } else {
          bool okAll = true;
          for (uint16_t i = 0; i < t.qty; i++) {
            uint16_t tid1 = md.writeCoil(d.ip, t.addr + i, bufCoil[i], nullptr, d.unit); // FC5 x N
            if (tid1 == 0) { okAll = false; break; }
            if (!mbWaitTrans(md, tid1)) { okAll = false; break; }
          }
          if (!okAll) { enviar("⚠️ MB.SET: fallo en escritura FC5 xN"); return; }
          enviar("✅ MB.SET OK (FC5 xN)");
          return;
        }
      } else {
        // HREG: FC6 / FC16 con fallback
        if (t.qty == 1) {
          tid = md.writeHreg(d.ip, t.addr, bufHreg[0], nullptr, d.unit);            // FC6
        } else {
          tid = md.writeHreg(d.ip, t.addr, bufHreg, t.qty, nullptr, d.unit);        // FC16
          if (tid == 0) {
            // Fallback una por una (FC6)
            bool okAll = true;
            for (uint16_t i=0; i<t.qty; i++) {
              uint16_t tid1 = md.writeHreg(d.ip, t.addr + i, bufHreg[i], nullptr, d.unit);
              if (tid1 == 0 || !mbWaitTrans(md, tid1)) { okAll = false; break; }
            }
            if (!okAll) { enviar("⚠️ MB: transacción no pudo iniciarse (fallback HREG)"); return; }
            enviar("✅ MB.SET OK (fallback HREG)");
            return;
          }
        }
        if (tid == 0) { enviar("⚠️ MB: no se pudo iniciar la transacción (write HREG)"); return; }
        if (!mbWaitTrans(md, tid)) { enviar("⚠️ MB.SET enviado, sin confirmación (HREG)"); return; }
        enviar("✅ MB.SET OK (HREG)");
        return;
      }
    }
  }

  // ---------- MB.SAVE ----------
  if (cmd.startsWith("MB.SAVE")) {
    if (!mbHasRoomFor(0,0,0)) { enviar("⚠️ MB.SAVE: el paquete Modbus excede la ventana asignada"); return; }
    uint16_t nDev=0, nTag=0; uint32_t crc=0;
    bool ok = mbSaveToEEPROM(nDev, nTag, crc);
    char b[120];
    int nIO=0; for (int i=0;i<MAX_MB_IO;i++) if (g_mbIOs[i].used) nIO++;
    snprintf(b, sizeof(b), "%s MB.SAVE dev=%u tag=%u io=%d crc=%08lx",
            ok?"✅":"⚠️", nDev, nTag, nIO, (unsigned long)crc);
    enviar(b);
    return;
  }

  // ---------- MB.LOAD ----------
  if (cmd.startsWith("MB.LOAD")) {
    uint16_t nDev=0, nTag=0; uint32_t crc=0;
    bool ok = mbLoadFromEEPROM(nDev, nTag, crc);
    char b[120];
    int nIO=0; for (int i=0;i<MAX_MB_IO;i++) if (g_mbIOs[i].used) nIO++;
    snprintf(b, sizeof(b), "%s MB.LOAD dev=%u tag=%u io=%d crc=%08lx",
            ok?"✅":"⚠️", nDev, nTag, nIO, (unsigned long)crc);
    enviar(b);
    return;
  }

  // ---------- MB.CLEAR ----------
  if (cmd.startsWith("MB.CLEAR")) {
    mbClearEEPROM();
    enviar("✅ MB.CLEAR OK");
    return;
  }

  enviar("❓ Comando no reconocido");
}
