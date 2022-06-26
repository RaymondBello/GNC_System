#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "../lib/pwm_driver/pwm_driver.h"
#include "config.h"

// PPM Timing constants
uint32_t ppm_delta;
uint32_t ppm_prev;
uint32_t MIN_PPM_DELAY_BETWEEN_PACKETS = 5000; //usec
uint32_t MAX_PPM_DELAY_BETWEEN_CHANNELS = 500; //usec
int PPM_CHANNEL_INDEX = 0;
int prev_idx;
bool PPM_START = false;

// 8 Channels for FS-iA10B PPM-Output Setting. check 'config.h'
uint32_t PPM_PACKET[NUM_PPM_CHANNELS] = {0};


void ppm_callback(uint gpio, uint32_t events){

    ppm_delta = time_us_32() - ppm_prev;
    ppm_prev = time_us_32();

    // Decode PPM Signal
    if (ppm_delta > MIN_PPM_DELAY_BETWEEN_PACKETS) 
    {
        PPM_START = true;
    }
    else if (ppm_delta < MAX_PPM_DELAY_BETWEEN_CHANNELS)
    {
        if (PPM_START)
        {
            PPM_CHANNEL_INDEX = 0;
            PPM_START = false;
        }
        else
        {
            if (ppm_delta) //sanity check
            { 
                PPM_CHANNEL_INDEX++;
                if (PPM_CHANNEL_INDEX > NUM_PPM_CHANNELS - 1){
                    PPM_CHANNEL_INDEX = 0; // Reset index to zero 
                }
            }
        }    
    }
    else
    {
        PPM_PACKET[PPM_CHANNEL_INDEX] = ppm_delta;
    }

}

void init_radio(){

    gpio_set_irq_enabled_with_callback(PPM_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &ppm_callback);

}