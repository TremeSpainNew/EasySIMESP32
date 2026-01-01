/*#include <io/AdsCache.h>
#include <core/I2CGuard.h>

Adafruit_ADS1115 g_ads;
bool g_ads_ok = false;

static unsigned long lastAdsReadMs = 0;
static const unsigned long ADS_READ_PERIOD_MS = 30;

static int  adsRawCache[4] = {0,0,0,0};
static bool adsCacheOk[4]  = {false,false,false,false};

void adsInit() {
  g_ads_ok = g_ads.begin(0x48);
  if (g_ads_ok) {
    g_ads.setGain(GAIN_ONE);
  }
}

void adsPollCache() {
  if (!g_ads_ok) return;
  unsigned long now = millis();
  if (now - lastAdsReadMs < ADS_READ_PERIOD_MS) return;
  lastAdsReadMs = now;

  if (!i2cTryLock(0)) return; // no bloquees el loop

  for (uint8_t ch=0; ch<4; ch++) {
    int16_t r = g_ads.readADC_SingleEnded(ch);
    if (r >= 0) { adsRawCache[ch] = (int)r; adsCacheOk[ch] = true; }
    else        { adsCacheOk[ch] = false; }
  }
  i2cUnlock();
}

bool adsGetCached(uint8_t ch, int &outRaw) {
  if (ch > 3) return false;
  if (!adsCacheOk[ch]) return false;
  outRaw = adsRawCache[ch];
  if (outRaw < 0) outRaw = 0;
  return true;
}

bool adsReadNowSafe(uint8_t ch, int &outRaw) {
  if (!g_ads_ok || ch > 3) return false;
  if (!i2cTryLock(20)) return false; // puntual: aquí sí puedes esperar un poco
  int16_t r = g_ads.readADC_SingleEnded(ch);
  i2cUnlock();
  if (r < 0) return false;
  outRaw = (int)r;
  if (outRaw < 0) outRaw = 0;
  return true;
}
*/