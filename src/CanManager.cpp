#include "CanManager.h"
#include <driver/twai.h>

CanManager canManager;
static CanManager* g_canManagerInstance = nullptr;

bool CanManager::begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t speed) {
  g_canManagerInstance = this;

  for (int i = 0; i < 128; ++i) {
    knownNodes[i] = false;
    knownLastSeen[i] = 0;
  }
  for (int i = 0; i < MAX_REGISTERED_PROFILES; ++i) {
    profiles[i] = nullptr;
  }
  profileCount = 0;

  can.onInput(&CanManager::bridgeInput);

  if (!can.beginMaster((long)speed, (int)rxPin, (int)txPin)) {
    Serial.println("❌ CAN: error arrancando EasySIM_CAN master");
    started = false;
    return false;
  }

  Serial.println("✅ CAN iniciado correctamente");
  can.scan();
  started = true;
  lastScanMs = millis();
  return true;
}

void CanManager::loop() {
  if (!started) return;

  can.loop();
  pollKnownNodes();

  if ((millis() - lastScanMs) >= 5000) {
    can.scan();
    lastScanMs = millis();
  }
}

bool CanManager::sendLegacyFrame(uint32_t id, const uint8_t* data, uint8_t len) {
  if (len > 8) return false;

  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = len;

  for (uint8_t i = 0; i < len; i++) {
    msg.data[i] = data[i];
  }

  return twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK;
}

bool CanManager::sendOutputSet(uint8_t node, uint8_t channel, uint8_t value) {
  if (!started) return false;

  can.sendDigitalOutput(node, channel, value ? 1 : 0);

  if (outputAckCb) {
    outputAckCb(node, channel, value ? 1 : 0);
  }

  return true;
}

bool CanManager::sendOutputByCommand(const char* command, uint8_t value, bool* usedProfileInvert) {
  if (usedProfileInvert) {
    *usedProfileInvert = false;
  }

  if (!started || !command || !command[0]) return false;

  for (uint8_t node = 0; node < 128; ++node) {
    const EasySimCANNodeInfo* info = can.nodeInfo(node);
    if (!info || !info->online) continue;

    const EasySimCANProfile* profile = registeredProfileByType(info->profile);
    if (!profile) continue;

    const EasySimCANPinConfig* pinCfg = profile->pinByCommand(command);
    if (!pinCfg) continue;
    if (pinCfg->mode != EasySimCANPinMode::OUTPUT_MODE) continue;

    uint8_t finalValue = value ? 1 : 0;
    if (pinCfg->inverted) {
      finalValue = finalValue ? 0 : 1;
      if (usedProfileInvert) {
        *usedProfileInvert = true;
      }
    }

    can.sendDigitalOutput(node, pinCfg->globalPin, finalValue);

    if (outputAckCb) {
      outputAckCb(node, pinCfg->globalPin, finalValue);
    }

    return true;
  }

  return false;
}

bool CanManager::sendOutputByProfilePin(const char* profileName, uint8_t pin, uint8_t value, bool* usedProfileInvert) {
  if (usedProfileInvert) {
    *usedProfileInvert = false;
  }

  if (!started || !profileName || !profileName[0]) return false;

  for (uint8_t node = 0; node < 128; ++node) {
    const EasySimCANNodeInfo* info = can.nodeInfo(node);
    if (!info || !info->online) continue;

    const EasySimCANProfile* profile = registeredProfileByType(info->profile);
    if (!profile) continue;
    if (strcasecmp(profile->name(), profileName) != 0) continue;

    const EasySimCANPinConfig* pinCfg = profile->pinByGlobalPin(pin);
    if (!pinCfg) continue;
    if (pinCfg->mode != EasySimCANPinMode::OUTPUT_MODE) continue;

    uint8_t finalValue = value ? 1 : 0;
    if (pinCfg->inverted) {
      finalValue = finalValue ? 0 : 1;
      if (usedProfileInvert) {
        *usedProfileInvert = true;
      }
    }

    can.sendDigitalOutput(node, pinCfg->globalPin, finalValue);

    if (outputAckCb) {
      outputAckCb(node, pinCfg->globalPin, finalValue);
    }

    return true;
  }

  return false;
}

bool CanManager::sendHeartbeat(uint8_t node) {
  uint8_t data[1] = { node };
  return sendLegacyFrame(EASY_CAN_ID_HEARTBEAT, data, 1);
}

bool CanManager::sendHello(uint8_t node, uint8_t inputs, uint8_t outputs) {
  uint8_t data[3] = { node, inputs, outputs };
  return sendLegacyFrame(EASY_CAN_ID_HELLO, data, 3);
}

void CanManager::bridgeInput(uint8_t node, uint8_t channel, int value, const char* command) {
  if (!g_canManagerInstance) return;
  g_canManagerInstance->handleInput(node, channel, (uint8_t)(value ? 1 : 0), command);
}

void CanManager::handleInput(uint8_t node, uint8_t channel, uint8_t value, const char* command) {
  if (inputCb) {
    inputCb(node, channel, value, command);
  }
}

void CanManager::pollKnownNodes() {
  for (uint8_t node = 0; node < 128; ++node) {
    const EasySimCANNodeInfo* info = can.nodeInfo(node);
    if (!info || !info->online) continue;

    if (!knownNodes[node]) {
      knownNodes[node] = true;
      if (helloCb) {
        helloCb(node);
      }
    }

    if (knownLastSeen[node] != info->lastSeenMs) {
      knownLastSeen[node] = info->lastSeenMs;
      if (heartbeatCb) {
        heartbeatCb(node);
      }
    }
  }
}

void CanManager::registerProfile(EasySimCANProfile* profile) {
  if (!profile) return;

  can.registerProfile(profile);

  for (uint8_t i = 0; i < profileCount; ++i) {
    if (profiles[i] == profile || (profiles[i] && profiles[i]->type() == profile->type())) {
      profiles[i] = profile;
      return;
    }
  }

  if (profileCount < MAX_REGISTERED_PROFILES) {
    profiles[profileCount++] = profile;
  }
}

const EasySimCANProfile* CanManager::registeredProfileByType(EasySimCANProfileType type) const {
  for (uint8_t i = 0; i < profileCount; ++i) {
    if (profiles[i] && profiles[i]->type() == type) {
      return profiles[i];
    }
  }

  return nullptr;
}

void CanManager::onInput(InputCallback cb) {
  inputCb = cb;
}

void CanManager::onOutputAck(OutputAckCallback cb) {
  outputAckCb = cb;
}

void CanManager::onHello(NodeCallback cb) {
  helloCb = cb;
}

void CanManager::onHeartbeat(NodeCallback cb) {
  heartbeatCb = cb;
}
