#ifndef FUSION_HPP
#define FUSION_HPP

#include <Arduino.h>
#include <cmath>

#include <robot_variables.hpp>

class Filter106 {
    float alpha = 0.0;
    bool initialized = false;
    float fused_heading = 0.0;
    float gyro_heading = 0.0;
    float mag_heading = 0.0;
    unsigned long t_previous = micros();

public:
    Filter106(float alpha_ = 0.90) { this->alpha = alpha_; }

    void setAlpha(float alpha_) { this->alpha = alpha_;}
    
    void reset(float initial_heading) {
        fused_heading = initial_heading;
        t_previous = micros();
        initialized = true;
    }

    void update(float gyro_rate, float mag_heading) {
        unsigned long t_now = micros();
        float delta_t = (t_now - t_previous) / 1000000.0;

        if (initialized == false) {
            this->reset(mag_heading);
            t_previous = t_now;
            initialized = true;
        }

        else {
            float predicted_heading = fused_heading + (gyro_rate * delta_t);
            float mag_error = angleDiff(mag_heading, predicted_heading);
            fused_heading = predicted_heading + ((1 - alpha) * mag_error);
            
            fused_heading = fmod(fused_heading + 360.0, 360.0);
            if (fused_heading < 0.0) { fused_heading += 360.0; }

            t_previous = t_now;
        }
    }

    float getHeading() { return this->fused_heading; }
};

#endif