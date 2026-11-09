#include <Arduino.h>
#include <Wire.h>

#include "heading.hpp"
#include "actuation.hpp"
#include "sensors.hpp"
#include "robot_variables.hpp"
#include "switch.hpp"

void setup() {
    Serial.begin(115200);
    delay(2000);

    compass_main.manual_calibration = false;
    
    pinMode(BOOT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootPinISR, RISING);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, RISING);

    xTaskCreate(updateHeading, "YAW", 4096, NULL, 7, NULL);
    xTaskCreate(handleSwitch, "SWITCH", 4096, NULL, 6, NULL);
    xTaskCreate(bootRobot, "BOOT", 4096, NULL, 5, NULL);
    xTaskCreate(steerRobot, "STEER", 4096, NULL, 4, NULL);
    xTaskCreate(firePiston, "FIRE", 4096, NULL, 3, NULL);
    xTaskCreate(serialOutput, "SERIAL", 4096, NULL, 2, NULL);
    xTaskCreate(blinkLED, "LED", 4096, NULL, 1, NULL);
}

void loop() { } // nothing needs to be in loop() because FreeRTOS handles all