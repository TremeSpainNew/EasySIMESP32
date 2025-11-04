#ifndef EEPROMUTILS_H
#define EEPROMUTILS_H

#include <EEPROM.h>

#if defined(PLACA_MEGA)
  #define EEPROM_MAX_SIZE 2048
#elif defined(PLACA_NANO) || defined(PLACA_UNO)
  #define EEPROM_MAX_SIZE 512
#else
  #define EEPROM_MAX_SIZE 2048
#endif

#define EEPROM_START 0

void saveConfig(const String& config) {
  int len = config.length();
  if (len > EEPROM_MAX_SIZE - 2) return;

  EEPROM.write(EEPROM_START, (len >> 8) & 0xFF);
  EEPROM.write(EEPROM_START + 1, len & 0xFF);

  for (int i = 0; i < len; i++) {
    EEPROM.write(EEPROM_START + 2 + i, config[i]);
  }
}

String loadConfig() {
  int len = (EEPROM.read(EEPROM_START) << 8) | EEPROM.read(EEPROM_START + 1);
  if (len <= 0 || len > EEPROM_MAX_SIZE - 2) return "";

  String config = "";
  for (int i = 0; i < len; i++) {
    config += (char)EEPROM.read(EEPROM_START + 2 + i);
  }
  return config;
}

#endif