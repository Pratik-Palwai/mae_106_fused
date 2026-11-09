#ifndef AHRS_HPP
#define AHRS_HPP

#include <Arduino.h>

#include "sensors.hpp"
#include "robot_variables.hpp"
#include "fusion.hpp"

Filter106 filter_main; // create an instance of the filter, defined in "fusion.hpp", which fuses the gyro and compass data for smoother and drift-free data

Gyro106 gyro_main; // create an instance of the IMU, defined in "imu.hpp"
Magnetometer106 compass_main; // create an instance of the compass, defined in "magnetometer.hpp"

// rtos task to update the robot's heading, executed at 0.10 kHz to prevent sensor packets from being used more than once
void updateHeading(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    while(1) {
        filter_main.update(gyro_main.getRate(), compass_main.calculateHeadingUnfiltered());
        yaw = filter_main.getHeading();        

        xTaskDelayUntil(&last_wake, period); // again, xTaskDelayUntil() gives better accuracy than a simple vTaskDelay()
    }
}

#endif