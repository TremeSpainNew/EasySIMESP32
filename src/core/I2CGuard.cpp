/*#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <core/I2CGuard.h>

static SemaphoreHandle_t g_i2cMutex = nullptr;

void i2cInitMutex() {
  if (!g_i2cMutex) g_i2cMutex = xSemaphoreCreateMutex();
}

bool i2cTryLock(uint32_t timeoutMs) {
  if (!g_i2cMutex) return true; // por si aún no está creado
  return xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void i2cUnlock() {
  if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
}*/
