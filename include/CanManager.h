#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include <EasySimCAN.h>
#include <Profiles/EasySimCANProfile.h>

#define EASY_CAN_ID_HELLO        0x001
#define EASY_CAN_ID_HEARTBEAT    0x002
#define EASY_CAN_ID_INPUT_CHANGE 0x100
#define EASY_CAN_ID_OUTPUT_SET   0x200
#define EASY_CAN_ID_OUTPUT_ACK   0x201

class CanManager {
public:
  typedef void (*InputCallback)(uint8_t node, uint8_t channel, uint8_t value, const char* command);
  typedef void (*OutputAckCallback)(uint8_t node, uint8_t channel, uint8_t value);
  typedef void (*NodeCallback)(uint8_t node);

  bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t speed = 500000);
  void loop();

  bool sendOutputSet(uint8_t node, uint8_t channel, uint8_t value);
  bool sendOutputByCommand(const char* command, uint8_t value, bool* usedProfileInvert = nullptr);
  bool sendOutputByProfilePin(const char* profileName, uint8_t pin, uint8_t value, bool* usedProfileInvert = nullptr);
  bool sendHeartbeat(uint8_t node);
  bool sendHello(uint8_t node, uint8_t inputs, uint8_t outputs);
  void registerProfile(EasySimCANProfile* profile);

  void onInput(InputCallback cb);
  void onOutputAck(OutputAckCallback cb);
  void onHello(NodeCallback cb);
  void onHeartbeat(NodeCallback cb);

private:
  static void bridgeInput(uint8_t node, uint8_t channel, int value, const char* command);
  void handleInput(uint8_t node, uint8_t channel, uint8_t value, const char* command);
  bool sendLegacyFrame(uint32_t id, const uint8_t* data, uint8_t len);
  void pollKnownNodes();
  const EasySimCANProfile* registeredProfileByType(EasySimCANProfileType type) const;

  EasySimCAN can;
  bool started = false;
  unsigned long lastScanMs = 0;
  bool knownNodes[128] = {false};
  uint32_t knownLastSeen[128] = {0};
  static constexpr uint8_t MAX_REGISTERED_PROFILES = 8;
  EasySimCANProfile* profiles[MAX_REGISTERED_PROFILES] = {nullptr};
  uint8_t profileCount = 0;

  InputCallback inputCb = nullptr;
  OutputAckCallback outputAckCb = nullptr;
  NodeCallback helloCb = nullptr;
  NodeCallback heartbeatCb = nullptr;
};

extern CanManager canManager;
