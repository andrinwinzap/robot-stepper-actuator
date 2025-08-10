#include "stepper.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <math.h>
#include <stdlib.h>
#include "esp_timer.h"

#define STEPPER_SPEED_CHANGE_THRESHOLD 1e-3

static const char *TAG = "stepper";

static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    stepper_t *stepper = (stepper_t *)user_data;
    gpio_set_level(stepper->step_pin, stepper->step_level);
    stepper->step_level = !stepper->step_level;
    return true;
}

void stepper_init(stepper_t *stepper,
                  gpio_num_t step_pin,
                  gpio_num_t dir_pin,
                  gpio_num_t en_pin,
                  float gear_ratio,
                  uint16_t steps_per_rev,
                  uint8_t microsteps,
                  float max_velocity,
                  float max_acceleration,
                  bool invert_direction)
{
    stepper->step_pin = step_pin;
    stepper->dir_pin = dir_pin;
    stepper->en_pin = en_pin;
    stepper->gear_ratio = gear_ratio;
    stepper->steps_per_rev = steps_per_rev;
    stepper->microsteps = microsteps;
    stepper->step_level = false;
    stepper->running = false;
    stepper->velocity = 0.0f;
    stepper->max_velocity = max_velocity;
    stepper->max_acceleration = max_acceleration;
    stepper->invert_direction = invert_direction;
    stepper->last_update_us = esp_timer_get_time();

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << step_pin) | (1ULL << dir_pin) | (1ULL << en_pin),
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    gpio_set_level(step_pin, 0);
    gpio_set_level(dir_pin, 0);
    gpio_set_level(en_pin, 1);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &stepper->timer));

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(stepper->timer, &callbacks, stepper));
    ESP_ERROR_CHECK(gptimer_enable(stepper->timer));

    ESP_LOGI(TAG, "Stepper initialized: step_pin=%d dir_pin=%d en_pin=%d", step_pin, dir_pin, en_pin);
}

void stepper_enable(const stepper_t *stepper)
{
    gpio_set_level(stepper->en_pin, 0);
    ESP_LOGD(TAG, "Stepper enabled");
}

void stepper_disable(const stepper_t *stepper)
{
    gpio_set_level(stepper->en_pin, 1);
    ESP_LOGD(TAG, "Stepper disabled");
}

void stepper_set_velocity(stepper_t *stepper, float target_velocity)
{
    if (!stepper->timer)
        return;

    int64_t now_us = esp_timer_get_time();
    float delta_time = (now_us - stepper->last_update_us) / 1e6f;
    stepper->last_update_us = now_us;

    if (fabsf(target_velocity) > stepper->max_velocity)
    {
        target_velocity = (target_velocity > 0) ? stepper->max_velocity : -stepper->max_velocity;
    }

    float delta_v = target_velocity - stepper->velocity;
    float max_delta_v = stepper->max_acceleration * delta_time;

    if (fabsf(delta_v) > max_delta_v)
    {
        target_velocity = stepper->velocity + copysignf(max_delta_v, delta_v);
    }

    if (fabs(target_velocity - stepper->velocity) < STEPPER_SPEED_CHANGE_THRESHOLD)
        return;

    stepper->velocity = target_velocity;

    if (stepper->running)
    {
        gptimer_stop(stepper->timer);
        stepper->running = false;
        ESP_LOGD(TAG, "Stopped timer");
    }

    if (target_velocity == 0.0f)
    {
        gpio_set_level(stepper->step_pin, 0);
        ESP_LOGD(TAG, "Velocity is zero, motor idle");
        return;
    }

    int64_t steps_per_sec = target_velocity * (stepper->gear_ratio * stepper->steps_per_rev * stepper->microsteps) / (2 * M_PI);
    if (steps_per_sec == 0)
        steps_per_sec = 1;

    uint64_t us_per_step = 500000ULL / llabs(steps_per_sec);
    bool dir_level = (target_velocity > 0) ? 0 : 1;
    if (stepper->invert_direction)
    {
        dir_level = !dir_level;
    }
    gpio_set_level(stepper->dir_pin, dir_level);

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = us_per_step,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(stepper->timer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(stepper->timer));
    stepper->running = true;

    ESP_LOGD(TAG, "Set velocity: %.2f rad/s -> %lld steps/s (us/step: %llu) dir: %s",
             target_velocity, steps_per_sec, us_per_step, target_velocity > 0 ? "CW" : "CCW");
}
