#pragma once

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"


class PWM
{
    private:
        int pin_number;             // pin to be driven
        
    public:
        float clockDiv = 64;
        float wrap = 39062;

        // usec calcalcation variables
        float pwm_usec_val;       // Current pwm value
        int pwm_usec_max;         // Maximum pwm value
        int pwm_usec_min;         // Minimum pwm value
        int pwm_usec_percentage;  // Current % pwm is being driven at given min/max

        // angle calculation variables
        int pwm_angle;              // servo pwm angle
        int pwm_angle_min;          // servo max angle
        int pwm_angle_max;          // servo min angle

        // math variables
        float value_scaled;

        PWM(int pin, float start_usec, int pwmMinimum, int pwmMaximum);
        void update_pwm_percentage();
        void write_pwm(float usec);
};