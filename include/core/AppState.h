#pragma once
#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>

// Estado global mínimo (evita extern por todas partes)
struct AppState {
  // modos
  bool modoConfig = false;
  bool bloqueado  = false;
  bool apActive   = false;

  // ethernet
  bool eth_connected = false;
  bool ethOutEnabled = true;
  bool ethStaticFallbackUsed = false;

  // info board
  const char* boardType = "BigBoardSep";
};

extern AppState gApp;
