#ifndef LED_SCHEDULER_H
#define LED_SCHEDULER_H

#include <Arduino.h>
#include <ThingsBoard.h>

#define LED_PIN 48

extern ThingsBoard tb; 

extern bool sched_blink_led;
extern int sched_blink_led_interval;
extern int sched_blink_led_duration;

void ledSchedulerTask(void *pvParameters);

#endif // LED_SCHEDULER_H