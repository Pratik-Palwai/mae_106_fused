#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <LSM6.h>

#include "robot_variables.hpp"

const int GYRO_CAL_SAMPLES = 3000; // number of samples to be averaged out, more samples increases accuracy but takes longer
const float GYRO_SCALING = 0.00875; // convert betweeen whatever units the gyro is in to degrees per second

// only one instance of this class ever needs to be created because there is only one IMU on the robot
class InertialMeasurementUnit106 { // placing all the methods into a class makes it easier to split code between files
    LSM6 sensor; // this is the core sensor, the STMicroelectronics LSM6DS33

    // gyroscopes drift because the rates even when resting aren't zero
    float rate_bias_z; // by subtracting the rate biases we can try to minimize the drift (resting rate becomes closer to zero)

public:
    void initialize() { // initializes the I2C communcation between the ESP32 and the sensor
        if (!sensor.init()) { 
            Serial.println("Failed to detect/initialize LSM6");
            while(1);
        }
        else { Serial.println("Successfully detected and initialized LSM6"); }
        
        sensor.enableDefault();        
    }

    // take samples from the gyro and average them out over a few seconds to calculate the rate bias offsets
    void calibrate() {
        double rates_sum_x = 0.0, rates_sum_y = 0.0, rates_sum_z = 0.0;

        Serial.print("Starting gyro calibration loop ... ");

        for (int i = 0; i < GYRO_CAL_SAMPLES; ++i) {
            delay(1); sensor.read();
            rates_sum_z += sensor.g.z * GYRO_SCALING;
        }

        Serial.println("finished");

        rate_bias_z = rates_sum_z / GYRO_CAL_SAMPLES;

        Serial.println("    gyro_rate_bias_z: " + String(rate_bias_z));
    }

    // return the corrected rate of the gyro in degrees/second
    float getRate() {
        sensor.read();
        return (sensor.g.z * GYRO_SCALING) - rate_bias_z;
    }
};

#endif