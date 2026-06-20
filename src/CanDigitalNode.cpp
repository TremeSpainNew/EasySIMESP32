#include "CanDigitalNode.h"

CanDigitalNode::CanDigitalNode(uint8_t nodeId)
  : nodeId(nodeId)
{
}

void CanDigitalNode::addInput(uint8_t channel, uint8_t pin, bool pullup) {
  if (inputCount >= MAX_INPUTS) return;

  inputs[inputCount].channel = channel;
  inputs[inputCount].pin = pin;
  inputs[inputCount].pullup = pullup;
  inputs[inputCount].lastValue = -1;

  inputCount++;
}

void CanDigitalNode::addOutput(uint8_t channel, uint8_t pin) {
  if (outputCount >= MAX_OUTPUTS) return;

  outputs[outputCount].channel = channel;
  outputs[outputCount].pin = pin;
  outputs[outputCount].value = 0;

  outputCount++;
}

void CanDigitalNode::begin() {
  for (uint8_t i = 0; i < inputCount; i++) {
    pinMode(inputs[i].pin, inputs[i].pullup ? INPUT_PULLUP : INPUT);
    inputs[i].lastValue = -1;
  }

  for (uint8_t i = 0; i < outputCount; i++) {
    pinMode(outputs[i].pin, OUTPUT);
    digitalWrite(outputs[i].pin, LOW);
    outputs[i].value = 0;
  }

  canManager.sendHello(nodeId, inputCount, outputCount);
}

void CanDigitalNode::loop() {
  unsigned long now = millis();

  if (now - lastHello > 5000) {
    lastHello = now;
    canManager.sendHello(nodeId, inputCount, outputCount);
  }

  if (now - lastHeartbeat > 1000) {
    lastHeartbeat = now;
    canManager.sendHeartbeat(nodeId);
  }

  for (uint8_t i = 0; i < inputCount; i++) {
    int raw = digitalRead(inputs[i].pin);

    uint8_t value;

    if (inputs[i].pullup) {
      value = (raw == LOW) ? 1 : 0;
    } else {
      value = (raw == HIGH) ? 1 : 0;
    }

    if (value != inputs[i].lastValue) {
      inputs[i].lastValue = value;
      sendInput(inputs[i].channel, value);
    }
  }
}

void CanDigitalNode::handleOutputSet(uint8_t node, uint8_t channel, uint8_t value) {
  if (node != nodeId) return;

  for (uint8_t i = 0; i < outputCount; i++) {
    if (outputs[i].channel == channel) {
      outputs[i].value = value ? 1 : 0;
      digitalWrite(outputs[i].pin, outputs[i].value ? HIGH : LOW);
      sendOutputAck(channel, outputs[i].value);
      return;
    }
  }
}

void CanDigitalNode::sendInput(uint8_t channel, uint8_t value) {
  uint8_t data[3] = { nodeId, channel, value };
  twai_message_t msg = {};

  msg.identifier = EASY_CAN_ID_INPUT_CHANGE;
  msg.data_length_code = 3;
  msg.data[0] = data[0];
  msg.data[1] = data[1];
  msg.data[2] = data[2];

  twai_transmit(&msg, pdMS_TO_TICKS(50));
}

void CanDigitalNode::sendOutputAck(uint8_t channel, uint8_t value) {
  uint8_t data[3] = { nodeId, channel, value };
  twai_message_t msg = {};

  msg.identifier = EASY_CAN_ID_OUTPUT_ACK;
  msg.data_length_code = 3;
  msg.data[0] = data[0];
  msg.data[1] = data[1];
  msg.data[2] = data[2];

  twai_transmit(&msg, pdMS_TO_TICKS(50));
}