#pragma once
#include <Arduino.h>

void oledInit();       // begin + mensaje inicial
void oledMaybeDraw();  // refresco no bloqueante (usa mutex)
bool oledIsOk();
