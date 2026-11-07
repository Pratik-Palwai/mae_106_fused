#ifndef SWITCH_HPP
#define SWITCH_HPP

#include <Arduino.h>

#include "robot_variables.hpp"

const int BOOT_PIN = 8;
bool boot_trigger = false;

const int LIMIT_SWITCH_PIN = 7;
const int DEBOUNCE_TIME = 25; // [ms]
bool limit_switch_trigger = false; // latch variable for software debouncing

void IRAM_ATTR bootPinISR() { boot_trigger = true; }

void bootRobot(void *param) {
    while(1) {
        if (boot_trigger) {
            booted = true;
            boot_time = millis();
            vTaskDelete(NULL);
        }

        vTaskDelay(20);
    }
}

void IRAM_ATTR limitSwitchISR() { limit_switch_trigger = true; }

void handleSwitch(void *param) {
    static long last_time = 0;

    while(1) {
        if (limit_switch_trigger) {
            long now = millis();

            if (now - last_time > DEBOUNCE_TIME) {
                if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
                    clicks += 1;
                    if (heading_state % 2 == 0) { clicks_on_straight++; }
                }

                last_time = now;
            }

            else { limit_switch_trigger = false; }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // executes at 50 Hz
    }
}

#endif