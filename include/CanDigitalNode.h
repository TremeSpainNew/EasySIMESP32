#pragma once

#include <Arduino.h>
#include "CanManager.h"

class CanDigitalNode {
public:
  CanDigitalNode(uint8_t nodeId);

  void addInput(uint8_t channel, uint8_t pin, bool pullup = true);
  void addOutput(uint8_t channel, uint8_t pin);

  void begin();
  void loop();

  void handleOutputSet(uint8_t node, uint8_t channel, uint8_t value);

private:
  struct InputItem {
    uint8_t channel;
    uint8_t pin;
    bool pullup;
    int lastValue;
  };

  struct OutputItem {
    uint8_t channel;
    uint8_t pin;
    uint8_t value;
  };

  static const uint8_t MAX_INPUTS = 16;
  static const uint8_t MAX_OUTPUTS = 16;

  uint8_t nodeId;

  InputItem inputs[MAX_INPUTS];
  OutputItem outputs[MAX_OUTPUTS];

  uint8_t inputCount = 0;
  uint8_t outputCount = 0;

  unsigned long lastHeartbeat = 0;
  unsigned long lastHello = 0;

  void sendInput(uint8_t channel, uint8_t value);
  void sendOutputAck(uint8_t channel, uint8_t value);
};
