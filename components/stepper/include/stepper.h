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

    bool step_level;
    bool running;
    float velocity;
    float max_velocity;
    float max_acceleration;
    int64_t last_update_us;

    gptimer_handle_t timer;
} stepper_t;

void stepper_init(stepper_t *stepper,
                  gpio_num_t step_pin,
                  gpio_num_t dir_pin,
                  gpio_num_t en_pin,
                  float gear_ratio,
                  uint16_t steps_per_rev,
                  uint8_t microsteps,
                  float max_velocity,
                  float max_acceleration);
void stepper_enable(const stepper_t *stepper);
void stepper_disable(const stepper_t *stepper);
void stepper_set_velocity(stepper_t *stepper, float rad_per_sec);

#endif // STEPPER_H