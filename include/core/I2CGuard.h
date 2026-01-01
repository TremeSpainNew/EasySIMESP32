#pragma once
#include <Arduino.h>

void i2cInitMutex();
bool i2cTryLock(uint32_t timeoutMs = 0);
void i2cUnlock();
