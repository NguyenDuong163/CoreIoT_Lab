#include "PumpControl.h"

volatile bool pumpState = false; // Trạng thái của bơm
volatile bool pumpChanged = false; // Trạng thái tự động của bơm

void pumpTask(void *pvParameters) {
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  while (true) {
    if (pumpState) {
      digitalWrite(PUMP_PIN, HIGH);
    } else {
      digitalWrite(PUMP_PIN, LOW);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Kiểm tra mỗi 100ms
  }
}

RPC_Response setPump(const RPC_Data &data) {
  bool newState = data;

  pumpState = newState;
  if (newState) {
    tb.sendTelemetryData("pumpStatus", "ON");
  } else {
    tb.sendTelemetryData("pumpStatus", "OFF");
  }
  pumpChanged = true;
  return RPC_Response("setValue", newState);
}

const std::array<RPC_Callback, 1U> pump_callbacks = {
  RPC_Callback{ "setValue", setPump }
};