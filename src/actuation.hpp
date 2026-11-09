#ifndef ACTUATION_HPP
#define ACTUATION_HPP

#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

#include "robot_variables.hpp"

const int SOLENOID_PIN = 5;
const int OPEN_TIME = 750, CLOSE_TIME = 250;

const int LED_BUILTIN_PIN = 8;
int led_status = 0;

volatile bool solenoid_state = false;
volatile bool actuation_allowed = true;

Servo steering_servo;
const int SERVO_PIN = 2;

// tune these depending on the track length
const int CLICKS_BEFORE_TURN = 20, CLICKS_AFTER_TURN = 20;

// pid input and output variables, used in actuation.hpp
double pid_input = 0.0, target_heading = 0.0, pid_output = 0.0;

// pid mode can be DIRECT or REVERSE depending on how the servo and magnetometer are mounted
const float K_P = 0.50, K_I = 0.0, K_D = 0.0;
PID steering_correction(&pid_input, &pid_output, &target_heading, K_P, K_I, K_D, REVERSE);

void steerRobot(void *param) {
    float initial_heading = 180.0;
    float final_heading = 270.0;
    float current_target = initial_heading;

    while(1) {
        if (heading_state == 0) { current_target = initial_heading; }
        else { current_target = final_heading; }

        pid_input = angleDiff(yaw, current_target);

        if ((heading_state == 0) && (clicks_on_straight > CLICKS_BEFORE_TURN)) { heading_state = 1; }
        else if (heading_state == 1) {
            if (abs(pid_input) < 5.0) { heading_state = 2; } // transition from turning state to corridor state
            
            clicks_on_straight = 0; // don't increment this value while turning
        }

        // transition between corridor state to stopped state
        else if ((heading_state == 2) && (clicks_on_straight > CLICKS_AFTER_TURN)) { heading_state = 3; }

        steering_correction.Compute();
        steering_servo.write(90.0 + pid_output); // convert the +/- heading from the PID to a 0-180 command for the servo

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void firePiston(void *param) {
    while (1) {
        if (millis() - boot_time < 15000000) { actuation_allowed = false; } // race hasn't started yet
        if (millis() - boot_time > 15000000) { actuation_allowed = true; } // race ongoing 
        if (millis() - boot_time > 75000000) { actuation_allowed = false; } // race is over (time)
        if (heading_state == 3) { actuation_allowed = false; } // race is over (distance)

        if (actuation_allowed) {
            digitalWrite(SOLENOID_PIN, solenoid_state);

            if (solenoid_state) {
                alpha = 1.0; // trust the gyro completely because the compass will experience noise
                vTaskDelay(pdMS_TO_TICKS(OPEN_TIME));
            }
            else {
                alpha = ALPHA_DEFAULT; // return the gyro to its default state
                vTaskDelay(pdMS_TO_TICKS(CLOSE_TIME));
            }

            solenoid_state = !solenoid_state;
        }

        else {
            digitalWrite(SOLENOID_PIN, LOW);
            vTaskDelay(250);
        }
    }
}

void blinkLED(void *param) {
    while (1) {
        digitalWrite(LED_BUILTIN_PIN, led_status);
        led_status = 1 - led_status;

        if (booted) { vTaskDelay(pdMS_TO_TICKS(63)); } // fast blinking: "armed"
        else { vTaskDelay(pdMS_TO_TICKS(125)); } // slow blinking: "disarmed"
    }
}

#endif