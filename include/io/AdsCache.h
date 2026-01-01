#pragma once
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

// ADS global (solo lo usa este módulo)
extern Adafruit_ADS1115 g_ads;
extern bool g_ads_ok;

void adsInit();         // begin + gain
void adsPollCache();    // refresca cache con mutex no bloqueante

bool adsGetCached(uint8_t ch, int &outRaw);      // lee cache
bool adsReadNowSafe(uint8_t ch, int &outRaw);    // lectura puntual con mutex (para comandos)
