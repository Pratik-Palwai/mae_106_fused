#ifndef ROBOT_VARIABLES_HPP
#define ROBOT_VARIABLES_HPP

#include <Arduino.h>

bool booted = false;
unsigned long boot_time = 0;

float yaw = 0.0; // main robot heading variable (-180, 180] deg

const int CLICKS_BEFORE_TURN = 20; // tune this
const int CLICKS_AFTER_TURN = 20; // tune this

int clicks = 0; // will be incremented by the handleSwitch task and the interrupt
int clicks_on_straight = 0; // for knowing when to stop at the end of the trench

int heading_state = 0; // 0:initial heading, 1:initial turning, 2:final heading, 3: stop

double pid_input = 0.0, target_heading = 0.0, pid_output = 0.0; // pid input and output variables, used in actuation.hpp

// returns difference between angles, accounting for angle wrapping
float angleDiff(float a, float b) {
    float c = a - b;
    while (c > 180.0) { c -= 360.0; }
    while (c < -180.0) { c += 360.0; }

    return c;
}

void serialOutput(void *param) {
    while(1)
    {
        Serial.print(">timestamp:" + String(micros()));
        Serial.print(",yaw:" + String(yaw));
        Serial.print(",clicks:" + String(clicks));
        Serial.print(",heading_state:" + String(heading_state));
        Serial.print(",booted:" + String(int(booted)));
        Serial.print("\r\n");

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif