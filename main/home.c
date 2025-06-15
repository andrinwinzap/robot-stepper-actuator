#include "home.h"
#include "esp_log.h"

static const char *TAG = "homing";

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

    float cw_edge, ccw_edge;

    ESP_LOGI(TAG, "Starting homing procedure");

    while (1)
    {
        as5600_update(as5600);
        now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        hall_triggered = (gpio_get_level(GPIO_NUM_23) == 0);

        float pos = as5600_get_position(as5600);
        ESP_LOGV(TAG, "State: %d, Position: %.4f, Hall: %s", state, pos, hall_triggered ? "TRIGGERED" : "not triggered");

        switch (state)
        {
        case HOME_WAIT_FOR_SENSOR:
        {
            if (hall_triggered)
            {
                if (hall_trigger_timestamp == 0)
                {
                    hall_trigger_timestamp = now;
                    ESP_LOGD(TAG, "Hall sensor triggered, waiting for stable signal...");
                }
                else if (now - hall_trigger_timestamp >= 50 + 500)
                {
                    ESP_LOGD(TAG, "Stable sensor signal detected. Setting initial position to 0.");
                    as5600_set_position(as5600, 0.0f);
                    stepper_set_velocity(stepper, -0.1f);
                    state = HOME_MOVE_OFF_SENSOR;
                }
                else if (now - hall_trigger_timestamp >= 50)
                {
                    if (!stepper->running)
                        stepper_enable(stepper);
                }
            }
            else if (hall_trigger_timestamp != 0)
            {

                ESP_LOGD(TAG, "Hall sensor unstable. Resetting timestamp.");
                hall_trigger_timestamp = 0;
                stepper_disable(stepper);
            }
            break;
        }

        case HOME_MOVE_OFF_SENSOR:
        {
            if (fabs(pos) >= 0.1f)
            {
                ESP_LOGI(TAG, "Moved off hall sensor. Searching for edges.");
                stepper_set_velocity(stepper, 0.1f);
                state = HOME_FIND_CW_EDGE;
            }
            break;
        }

        case HOME_FIND_CW_EDGE:
        {
            if (hall_triggered)
            {
                cw_edge = pos;
                ESP_LOGI(TAG, "CW edge detected at %.4f", cw_edge);
                state = HOME_FIND_CCW_EDGE;
            }
            break;
        }

        case HOME_FIND_CCW_EDGE:
        {
            if (!hall_triggered)
            {
                ccw_edge = pos;
                float center = (ccw_edge - cw_edge) / 2.0f;
                ESP_LOGI(TAG, "CCW edge detected at %.4f", ccw_edge);
                ESP_LOGI(TAG, "Center calculated at %.4f. Returning to center...", center);
                as5600_set_position(as5600, center);
                stepper_set_velocity(stepper, -0.1f);
                state = HOME_RETURN_TO_CENTER;
            }
            break;
        }

        case HOME_RETURN_TO_CENTER:
        {
            if (fabs(pos) < 0.01f)
            {
                stepper_set_velocity(stepper, 0);
                ESP_LOGI(TAG, "Homing complete. Final position: %.4f", pos);
                return;
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown state: %d", state);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
