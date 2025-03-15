#ifndef EXTERNAL_LED_CONTROL_H
#define EXTERNAL_LED_CONTROL_H

#include <Arduino.h>
#include <ThingsBoard.h>
#include <Adafruit_NeoPixel.h>

extern volatile bool neoPixelState;
extern volatile bool externalLedState;

extern ThingsBoard tb; 

void initNeoPixel();
void neoPixelTask(void *pvParameters);
RPC_Response setExternLed(const RPC_Data &data);

extern const std::array<RPC_Callback, 1U> exLed_callbacks;

#endif // EXTERNAL_LED_CONTROL_H