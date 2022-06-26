
#include "pwm_driver.h"

PWM::PWM(int pin, float start_usec, int pwmMinimum, int pwmMaximum)
{
    // assign pin & initialize member variables
    pin_number = pin;
    pwm_usec_val = start_usec;
    pwm_usec_max = pwmMaximum;
    pwm_usec_min = pwmMinimum;
    
    // assign gpio pwm function
    gpio_set_function(pin_number, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin_number);

    pwm_config config = pwm_get_default_config();
    
    uint64_t clockspeed = clock_get_hz(clock_index::clk_sys); ///< Processors, bus fabric, memory, memory mapped registers

    while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
    wrap = clockspeed/clockDiv/50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    // initialize pwm with min/max pwm value
    // write_pwm(pwm_usec_val);
}

void PWM::update_pwm_percentage()
{
    pwm_usec_percentage = ((pwm_usec_val - pwm_usec_min) / (pwm_usec_max - pwm_usec_min)) * 100;
}

void PWM::write_pwm(float usec)
{
    pwm_usec_val = usec;

    // pwm_usec_val = (usec >= pwm_usec_min && usec <= pwm_usec_max) ? usec : ((usec > pwm_usec_max) ? pwm_usec_max : pwm_usec_min);

    pwm_set_gpio_level(pin_number, (pwm_usec_val / 20000.f) * wrap);

    // update percentage
    update_pwm_percentage();
}