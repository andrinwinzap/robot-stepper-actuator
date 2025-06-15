#include "stepper.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <math.h>
#include <stdlib.h>

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
                  uint8_t microsteps)
{
    stepper->step_pin = step_pin;
    stepper->dir_pin = dir_pin;
    stepper->en_pin = en_pin;
    stepper->gear_ratio = gear_ratio;
    stepper->steps_per_rev = steps_per_rev;
    stepper->microsteps = microsteps;
    stepper->step_level = false;
    stepper->running = false;

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

    ESP_LOGD(TAG, "Stepper initialized: step_pin=%d dir_pin=%d en_pin=%d", step_pin, dir_pin, en_pin);
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

void stepper_set_velocity(stepper_t *stepper, float rad_per_sec)
{
    if (!stepper->timer)
        return;

    if (stepper->running)
    {
        gptimer_stop(stepper->timer);
        stepper->running = false;
        ESP_LOGD(TAG, "Stopped previous motion");
    }

    if (rad_per_sec == 0)
    {
        gpio_set_level(stepper->step_pin, 0);
        ESP_LOGD(TAG, "Velocity is zero, motor idle");
        return;
    }

    int64_t steps_per_sec = rad_per_sec * (stepper->gear_ratio * stepper->steps_per_rev * stepper->microsteps) / (2 * M_PI);
    if (steps_per_sec == 0)
        steps_per_sec = 1;

    uint64_t us_per_step = 500000ULL / llabs(steps_per_sec);
    gpio_set_level(stepper->dir_pin, rad_per_sec > 0 ? 0 : 1);

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = us_per_step,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(stepper->timer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(stepper->timer));
    stepper->running = true;

    ESP_LOGD(TAG, "Set velocity: %.2f rad/s -> %lld steps/s (us/step: %llu) dir: %s",
             rad_per_sec, steps_per_sec, us_per_step, rad_per_sec > 0 ? "CW" : "CCW");
}
