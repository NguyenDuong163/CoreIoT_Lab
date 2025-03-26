#include "Led_scheduler.h"

void ledSchedulerTask(void *pvParameters) {
    const int ledPin = LED_PIN; // Assuming LED_PIN is defined globally
    pinMode(ledPin, OUTPUT);
    bool ledState = false;
    int blinkCount = 0;

    while (true) {
        if (sched_blink_led) {
            if (blinkCount < (sched_blink_led_duration / sched_blink_led_interval)) {
                ledState = !ledState;
                tb.sendTelemetryData("sched_led_state", ledState ? "ON" : "OFF");
                digitalWrite(ledPin, ledState);
                blinkCount++;
                vTaskDelay(sched_blink_led_interval / portTICK_PERIOD_MS);
            } else {
                Serial.println("Blinking finished");
                sched_blink_led = false;
                blinkCount = 0;
                digitalWrite(ledPin, LOW); // Turn off LED after blinking
            }
        } else {
            vTaskDelay(100 / portTICK_PERIOD_MS); // Check every 100ms if blinking is enabled
        }
    }
}