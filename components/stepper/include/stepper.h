#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"

typedef struct
{
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t en_pin;
    float gear_ratio;
    uint16_t steps_per_rev;
    uint8_t microsteps;
    gptimer_handle_t timer;
    volatile bool step_level;
    bool running;
    float velocity;
} stepper_t;

void stepper_init(stepper_t *stepper,
                  gpio_num_t step_pin,
                  gpio_num_t dir_pin,
                  gpio_num_t en_pin,
                  float gear_ratio,
                  uint16_t steps_per_rev,
                  uint8_t microsteps);
void stepper_enable(const stepper_t *stepper);
void stepper_disable(const stepper_t *stepper);
void stepper_set_velocity(stepper_t *stepper, float rad_per_sec);

#endif // STEPPER_H