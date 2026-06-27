#include "EE.h"
#include <Arduino.h>

static Preferences s_prefs;
static bool s_ok = false;
static size_t s_len = 0;

#define EE_ADDR_ETH_DHCP   24000
#define EE_ADDR_ETH_IP     24001
#define EE_ADDR_ETH_GW     24005
#define EE_ADDR_ETH_MASK   24009

static void makeKey(uint32_t addr, char* out, size_t outlen) {
  // clave estable a partir del offset
  // k + 8 dígitos (cubre hasta 99.999.999)
  snprintf(out, outlen, "k%08lu", (unsigned long)addr);
}

bool EE::begin(size_t sizeBytes) {
  // Abre un namespace fijo para toda la “EEPROM”
  // OJO: no llames NADA de EE antes de setup()
  s_ok = s_prefs.begin("EE", false);
  s_len = sizeBytes;
  return s_ok;
}

void EE::end() {
  if (s_ok) { s_prefs.end(); s_ok = false; }
}

uint8_t EE::read(uint32_t addr) {
  if (!s_ok) return 0xFF;
  char key[16]; makeKey(addr, key, sizeof(key));
  uint8_t b = 0xFF;
  size_t n = s_prefs.getBytes(key, &b, sizeof(b));
  return (n == 1) ? b : 0xFF;
}

void EE::write(uint32_t addr, uint8_t val) {
  if (!s_ok) return;
  char key[16]; makeKey(addr, key, sizeof(key));
  s_prefs.putBytes(key, &val, sizeof(val));
}

void EE::get(uint32_t addr, void* dst, size_t len) {
  if (!s_ok || !dst || !len) return;
  char key[16]; makeKey(addr, key, sizeof(key));
  // si no existe, deja dst sin tocar → inicialízala tú antes de llamar
  s_prefs.getBytes(key, dst, len);
}

void EE::put(uint32_t addr, const void* src, size_t len) {
  if (!s_ok || !src || !len) return;
  char key[16]; makeKey(addr, key, sizeof(key));
  s_prefs.putBytes(key, src, len);
}

void EE::setDhcpEnabled(bool enabled) {
  write(EE_ADDR_ETH_DHCP, enabled ? 1 : 0);
}

bool EE::getDhcpEnabled() {
  uint8_t v = read(EE_ADDR_ETH_DHCP);
  if (v == 0xFF) return true;   // por defecto DHCP
  return v != 0;
}

void EE::setStaticIP(IPAddress ip) {
  uint8_t raw[4] = { ip[0], ip[1], ip[2], ip[3] };
  put(EE_ADDR_ETH_IP, raw, 4);
}

IPAddress EE::getStaticIP() {
  uint8_t raw[4] = { 0, 0, 0, 0 };
  get(EE_ADDR_ETH_IP, raw, 4);

  if (raw[0] == 0xFF || raw[0] == 0) {
    return IPAddress(0, 0, 0, 0);
  }

  return IPAddress(raw[0], raw[1], raw[2], raw[3]);
}

void EE::setStaticGateway(IPAddress ip) {
  uint8_t raw[4] = { ip[0], ip[1], ip[2], ip[3] };
  put(EE_ADDR_ETH_GW, raw, 4);
}

IPAddress EE::getStaticGateway() {
  uint8_t raw[4] = { 0, 0, 0, 0 };
  get(EE_ADDR_ETH_GW, raw, 4);

  if (raw[0] == 0xFF) {
    return IPAddress(0, 0, 0, 0);
  }

  return IPAddress(raw[0], raw[1], raw[2], raw[3]);
}

void EE::setStaticMask(IPAddress ip) {
  uint8_t raw[4] = { ip[0], ip[1], ip[2], ip[3] };
  put(EE_ADDR_ETH_MASK, raw, 4);
}

IPAddress EE::getStaticMask() {
  uint8_t raw[4] = { 0, 0, 0, 0 };
  get(EE_ADDR_ETH_MASK, raw, 4);

  if (raw[0] == 0xFF || raw[0] == 0) {
    return IPAddress(0, 0, 0, 0);
  }

  return IPAddress(raw[0], raw[1], raw[2], raw[3]);
}

void EE::commit() {
  // Preferences no requiere commit; lo dejamos por compatibilidad
}

size_t EE::length() {
  return s_len ? s_len : 16384; // o el valor que uses
}
