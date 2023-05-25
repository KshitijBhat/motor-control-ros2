/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Output PWM signals on pins 0 and 1

#include "pico/stdlib.h"
#include "hardware/pwm.h"

const uint ENA = 15;
const uint IN1 = 14;
const uint IN2 = 13;
const uint LED_PIN = 25;

int main() {
    /// \tag::setup_pwm[]
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_put(IN1, 1);

    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_put(IN2, 0);

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(ENA, GPIO_FUNC_PWM);
    // gpio_set_function(1, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(ENA);

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(slice_num, 255);
    // Set channel A output high for one cycle before dropping
    // pwm_set_chan_level(slice_num, PWM_CHAN_A, 100);
    // Set initial B output high for three cycles before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 100);
    // Set the PWM running
    // pwm_set_enabled(slice_num, true);
    /// \end::setup_pwm[]
    pwm_set_enabled(slice_num, true);

    // Note we could also use pwm_set_gpio_level(gpio, x) which looks up the
    // correct slice and channel for a given GPIO.
    for(int i=0; i<5; i++){
        gpio_put(LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
        // pwm_set_enabled(slice_num, true);
    }
}