#ifndef SWITCH_HPP
#define SWITCH_HPP

#include <Arduino.h>

#include "packets_vars_functions.hpp"

const int LIMIT_SWITCH_PIN = 7;
const int DEBOUNCE_TIME = 25; // [ms]

static long last_time = 0;

bool trigger = false; // latch variable for software debouncing

void IRAM_ATTR limitSwitchISR() { trigger = true; }

void handleSwitch(void *param) {
    while(1) {
        if (trigger) {
            long now = millis();

            if (now - last_time > DEBOUNCE_TIME) {
                if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
                    clicks += 1;
                    if (heading_state % 2 == 0) { clicks_on_straight++; }
                }

                last_time = now;
            }

            else { trigger = false; }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // executes at 50 Hz
    }
}

#endif