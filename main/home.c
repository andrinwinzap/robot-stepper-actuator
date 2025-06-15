#include "home.h"
#include "esp_log.h"

static const char *TAG = "home";

void home(stepper_t *stepper, as5600_t *as5600)
{
    enum
    {
        HOME_WAIT_FOR_SENSOR,
        HOME_MOVE_OFF_SENSOR,
        HOME_FIND_CW_EDGE,
        HOME_FIND_CCW_EDGE,
        HOME_RETURN_TO_CENTER
    } state = HOME_WAIT_FOR_SENSOR;

    unsigned long now;

    bool hall_triggered;
    unsigned long hall_trigger_timestamp = 0;

    float cw_edge;
    float ccw_edge;

    while (1)
    {
        as5600_update(as5600);
        now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        hall_triggered = (gpio_get_level(GPIO_NUM_23) == 0);

        switch (state)
        {
        case HOME_WAIT_FOR_SENSOR:
        {
            if (hall_triggered)
            {
                if (hall_trigger_timestamp == 0)
                {
                    hall_trigger_timestamp = now;
                }
                else if (now - hall_trigger_timestamp >= 50 + 500)
                {
                    ESP_LOGI(TAG, "Initial home position found");
                    as5600_set_position(as5600, 0.0f);
                    stepper_set_velocity(stepper, -0.1f);
                    state = HOME_MOVE_OFF_SENSOR;
                }
                else if (now - hall_trigger_timestamp >= 50)
                {
                    stepper_enable(stepper);
                }
            }
            else
            {
                stepper_disable(stepper);
            }
            break;
        }
        case HOME_MOVE_OFF_SENSOR:
        {
            if (fabs(as5600_get_position(as5600)) >= 0.1f)
            {
                stepper_set_velocity(stepper, 0.1f);
                state = HOME_FIND_CW_EDGE;
            }
            break;
        }

        case HOME_FIND_CW_EDGE:
        {
            if (hall_triggered)
            {
                cw_edge = as5600_get_position(as5600);
                state = HOME_FIND_CCW_EDGE;
            }
            break;
        }

        case HOME_FIND_CCW_EDGE:
        {
            if (!hall_triggered)
            {
                ccw_edge = as5600_get_position(as5600);
                as5600_set_position(as5600, ((ccw_edge - cw_edge) / 2.0f));
                stepper_set_velocity(stepper, -0.1f);
                state = HOME_RETURN_TO_CENTER;
            }
            break;
        }

        case HOME_RETURN_TO_CENTER:
        {
            if (fabs(as5600_get_position(as5600)) < 0.01f)
            {
                stepper_set_velocity(stepper, 0);
                ESP_LOGI(TAG, "Homing finished");
                return;
            }
            break;
        }

        default:
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}