#pragma once
#include <Arduino.h>

// Entrada única para comandos desde Serial/TCP/AP/5090
void handleLine(const char* command, const char* value);
