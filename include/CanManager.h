#pragma once

#include <Arduino.h>
#include <driver/twai.h>

#define EASY_CAN_ID_HELLO        0x001
#define EASY_CAN_ID_HEARTBEAT    0x002
#define EASY_CAN_ID_INPUT_CHANGE 0x100
#define EASY_CAN_ID_OUTPUT_SET   0x200
#define EASY_CAN_ID_OUTPUT_ACK   0x201

class CanManager {
public:
  typedef void (*InputCallback)(uint8_t node, uint8_t channel, uint8_t value);
  typedef void (*OutputAckCallback)(uint8_t node, uint8_t channel, uint8_t value);
  typedef void (*NodeCallback)(uint8_t node);

  bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t speed = 500000);
  void loop();

  bool sendOutputSet(uint8_t node, uint8_t channel, uint8_t value);
  bool sendHeartbeat(uint8_t node);
  bool sendHello(uint8_t node, uint8_t inputs, uint8_t outputs);

  void onInput(InputCallback cb);
  void onOutputAck(OutputAckCallback cb);
  void onHello(NodeCallback cb);
  void onHeartbeat(NodeCallback cb);

private:
  bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len);
  void handleFrame(const twai_message_t& msg);

  InputCallback inputCb = nullptr;
  OutputAckCallback outputAckCb = nullptr;
  NodeCallback helloCb = nullptr;
  NodeCallback heartbeatCb = nullptr;
};

extern CanManager canManager;