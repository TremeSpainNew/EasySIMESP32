#include "CanManager.h"

CanManager canManager;

bool CanManager::begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t speed) {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config;

  switch (speed) {
    case 125000:
      t_config = TWAI_TIMING_CONFIG_125KBITS();
      break;
    case 250000:
      t_config = TWAI_TIMING_CONFIG_250KBITS();
      break;
    case 500000:
    default:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      break;
  }

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("❌ CAN: error instalando driver TWAI");
    return false;
  }

  if (twai_start() != ESP_OK) {
    Serial.println("❌ CAN: error arrancando TWAI");
    return false;
  }

  Serial.println("✅ CAN iniciado correctamente");
  return true;
}

void CanManager::loop() {
  twai_message_t msg;

  while (twai_receive(&msg, 0) == ESP_OK) {
    handleFrame(msg);
  }
}

bool CanManager::sendFrame(uint32_t id, const uint8_t* data, uint8_t len) {
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
  uint8_t data[3] = { node, channel, value };
  return sendFrame(EASY_CAN_ID_OUTPUT_SET, data, 3);
}

bool CanManager::sendHeartbeat(uint8_t node) {
  uint8_t data[1] = { node };
  return sendFrame(EASY_CAN_ID_HEARTBEAT, data, 1);
}

bool CanManager::sendHello(uint8_t node, uint8_t inputs, uint8_t outputs) {
  uint8_t data[3] = { node, inputs, outputs };
  return sendFrame(EASY_CAN_ID_HELLO, data, 3);
}

void CanManager::handleFrame(const twai_message_t& msg) {
  if (msg.data_length_code == 0) return;

  switch (msg.identifier) {
    case EASY_CAN_ID_HELLO:
      if (msg.data_length_code >= 1 && helloCb) {
        helloCb(msg.data[0]);
      }
      break;

    case EASY_CAN_ID_HEARTBEAT:
      if (msg.data_length_code >= 1 && heartbeatCb) {
        heartbeatCb(msg.data[0]);
      }
      break;

    case EASY_CAN_ID_INPUT_CHANGE:
      if (msg.data_length_code >= 3 && inputCb) {
        inputCb(msg.data[0], msg.data[1], msg.data[2]);
      }
      break;

    case EASY_CAN_ID_OUTPUT_ACK:
      if (msg.data_length_code >= 3 && outputAckCb) {
        outputAckCb(msg.data[0], msg.data[1], msg.data[2]);
      }
      break;

    default:
      break;
  }
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