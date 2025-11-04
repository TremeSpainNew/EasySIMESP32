#pragma once
#include <Preferences.h>

namespace EE {
  bool begin(size_t sizeBytes = 0);   // size opcional, solo informativo
  void end();

  // API tipo “EEPROM”
  uint8_t read(uint32_t addr);
  void    write(uint32_t addr, uint8_t val);
  void    get(uint32_t addr, void* dst, size_t len);
  void    put(uint32_t addr, const void* src, size_t len);

  // utilidades
  void    commit();       // NVS no lo necesita, pero lo dejas por compat
  size_t  length();       // si lo usas, devuelve el size de begin() o fijo
}
