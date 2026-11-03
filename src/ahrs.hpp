#ifndef AHRS_HPP
#define AHRS_HPP

#include <Arduino.h>

#include "imu.hpp"
#include "magnetometer.hpp"
#include "robot_variables.hpp"
#include "fusion.hpp"

// create an instance of the Madgwick filter, which is a common sensor fusion algorithm for IMUs
Filter106 filter_main; // it uses the accelerometer, gyroscope, and magnetometer to create a full 3-axis AHRS, but for 106 we only need to use the yaw

InertialMeasurementUnit106 imu_main; // create an instance of the IMU, defined in "imu.hpp"
Magnetometer106 compass_main; // create an instance of the compass, defined in "magnetometer.hpp"

// rtos task to update Madgwick AHRS, executed at 0.5 kHz to prevent sensor packets from being used more than once
void updateAHRS(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    while(1) {
        filter_main.update(imu_main.getRate(), compass_main.calculateHeading());
        yaw = filter_main.getHeading();        

        xTaskDelayUntil(&last_wake, period); // again, xTaskDelayUntil() gives better accuracy than a simple vTaskDelay()
    }
}

#endif