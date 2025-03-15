#include "ExternalLedControl.h"

Adafruit_NeoPixel pixels(4, 8, NEO_GRB + NEO_KHZ800); // D5-D6

volatile bool neoPixelState = false; // Trạng thái của LED NeoPixel
volatile bool externalLedState = false;

void initNeoPixel() {
    pixels.begin();
    pixels.clear(); // Đảm bảo LED tắt khi khởi động
    pixels.show();
}

void neoPixelTask(void *pvParameters) {
    initNeoPixel();

    while (true) {
        if (neoPixelState) {
            pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Xanh lá khi bật
        } else {
            pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Đỏ khi tắt
        }
        pixels.show();

        vTaskDelay(100 / portTICK_PERIOD_MS); // Kiểm tra mỗi 100ms
    }
}

RPC_Response setExternLed(const RPC_Data &data) {
    bool newState = data;

    neoPixelState = newState;
    if (newState) {
        tb.sendTelemetryData("ex_LED_Status", "ON");
      } else {
        tb.sendTelemetryData("ex_LED_Status", "OFF");
      }
    externalLedState = true;
    return RPC_Response("setValue", newState);
}

const std::array<RPC_Callback, 1U> exLed_callbacks = {
    RPC_Callback{ "setValue", setExternLed }
};