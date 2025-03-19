#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include <Arduino.h>
#include <ThingsBoard.h>

#define PUMP_PIN 10
extern volatile bool pumpState;
extern volatile bool pumpChanged;

extern ThingsBoard tb;

void pumpTask(void *pvParameters);
RPC_Response setPump(const RPC_Data &data);

extern const std::array<RPC_Callback, 1U> pump_callbacks;

#endif // PUMP_CONTROL_H