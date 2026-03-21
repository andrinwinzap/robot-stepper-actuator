#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"
#include "nvs_flash.h"
#include "as5600.h"
#include "stepper.h"
#include "pid.h"
#include "macros.h"
#include "config.h"
#include "actuator.h"
#include "home.h"

#define LED_ERROR_PIN    GPIO_NUM_15
#define LED_HOMED_PIN    GPIO_NUM_16

#define TOPIC_BUFFER_SIZE 64
#define COMMAND_BUFFER_LEN 2
#define STATE_BUFFER_LEN 2
#define UROS_RECONNECT_DELAY_MS 500
#define UROS_PING_TIMEOUT_MS 100
#define UROS_PING_ATTEMPTS 1
#define UROS_PING_INTERVAL_MS 1000
#define UROS_EXECUTOR_WAIT_MS 10

rcl_publisher_t state_publisher;
std_msgs__msg__Float32MultiArray state_publisher_msg;
char state_publisher_topic[TOPIC_BUFFER_SIZE];
static float state_buffer[STATE_BUFFER_LEN];

rcl_subscription_t command_subscriber;
std_msgs__msg__Float32MultiArray command_subscriber_msg;
char command_subscriber_topic[TOPIC_BUFFER_SIZE];
static float command_buffer[COMMAND_BUFFER_LEN];

SemaphoreHandle_t pid_semaphore;

static gptimer_handle_t control_timer = NULL;

static size_t uart_port = UART_NUM_1;

actuator_t actuator;

static const char *TAG = "Actuator";

typedef enum
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} uros_state_t;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void command_subscriber_callback(const void *msgin);

static bool create_micro_ros_entities(
    rcl_allocator_t *allocator,
    rclc_support_t *support,
    rcl_node_t *node,
    rcl_timer_t *timer,
    rclc_executor_t *executor)
{
    rcl_ret_t rc;

    rc = rclc_support_init(support, 0, NULL, allocator);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_node_init_default(node, "actuator", "", support);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_publisher_init_best_effort(
        &state_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        state_publisher_topic);
    if (rc != RCL_RET_OK) return false;

    std_msgs__msg__Float32MultiArray__init(&state_publisher_msg);
    state_publisher_msg.data.data = state_buffer;
    state_publisher_msg.data.capacity = STATE_BUFFER_LEN;
    state_publisher_msg.data.size = STATE_BUFFER_LEN;

    rc = rclc_subscription_init_best_effort(
        &command_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        command_subscriber_topic);
    if (rc != RCL_RET_OK) return false;

    std_msgs__msg__Float32MultiArray__init(&command_subscriber_msg);
    command_subscriber_msg.data.data = command_buffer;
    command_subscriber_msg.data.capacity = COMMAND_BUFFER_LEN;
    command_subscriber_msg.data.size = 0;

    rc = rclc_timer_init_default2(
        timer,
        support,
        RCL_MS_TO_NS(20),
        timer_callback,
        true);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_executor_init(executor, &support->context, 2, allocator);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_executor_add_timer(executor, timer);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_executor_add_subscription(
        executor,
        &command_subscriber,
        &command_subscriber_msg,
        command_subscriber_callback,
        ON_NEW_DATA);
    if (rc != RCL_RET_OK) return false;

    state_publisher_msg.data.data[0] = 0.0f;
    state_publisher_msg.data.data[1] = 0.0f;
    return true;
}

static void destroy_micro_ros_entities(
    rclc_support_t *support,
    rcl_node_t *node,
    rcl_timer_t *timer,
    rclc_executor_t *executor)
{
    rclc_executor_fini(executor);
    rcl_timer_fini(timer);
    rcl_subscription_fini(&command_subscriber, node);
    rcl_publisher_fini(&state_publisher, node);
    rcl_node_fini(node);
    rclc_support_fini(support);
}

bool IRAM_ATTR gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    xSemaphoreGiveFromISR(pid_semaphore, &high_task_wakeup);

    return high_task_wakeup == pdTRUE;
}

void init_control_timer()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &control_timer));

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = gptimer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(control_timer, &callbacks, NULL));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000 / PID_LOOP_FREQUENCY_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(control_timer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_enable(control_timer));
    ESP_ERROR_CHECK(gptimer_start(control_timer));
}

void i2c_bus_init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));
}

void led_init(void)
{
    gpio_num_t leds[] = {LED_OK_PIN, LED_ERROR_PIN, LED_HOMED_PIN};
    for (int i = 0; i < 3; i++) {
        gpio_set_level(leds[i], 1); // drive high before enabling output to avoid glitch
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << leds[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }
}

#define ERROR_FLAG_ENCODER  (1u << 0)
#define ERROR_FLAG_AGENT    (1u << 1)

static volatile uint32_t error_flags = 0;

static void set_error_flag(uint32_t flag)
{
    error_flags |= flag;
    gpio_set_level(LED_OK_PIN, 1);    // ok off
    gpio_set_level(LED_ERROR_PIN, 0); // error on
}

static void clear_error_flag(uint32_t flag)
{
    error_flags &= ~flag;
    if (error_flags == 0) {
        gpio_set_level(LED_ERROR_PIN, 1); // error off
        gpio_set_level(LED_OK_PIN, 0);    // ok on
    }
}

void gpio_input_init(gpio_num_t pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

void pid_loop_task(void *param)
{
    float dt_s = 1.0f / PID_LOOP_FREQUENCY_HZ;
    const int64_t PID_LOG_INTERVAL_US = (int64_t)(1000000.0f / PID_LOG_FREQUENCY_HZ);

    double pid_freq = 0.0;
    double loop_time_us = 0.0;
    int64_t last_us = 0;
    int64_t last_log_us = 0;

    float pos_feedback;
    float pos_delta;
    float vel_feedback = 0;
    float vel_sig;
    float vel_sig_delta;
    float max_vel_sig_delta = ACTUATOR_MAX_ACCELERATION * dt_s;

    as5600_update(&actuator.encoder);
    actuator.pos = as5600_get_position(&actuator.encoder);
    for (;;)
    {
        if (xSemaphoreTake(pid_semaphore, portMAX_DELAY) == pdTRUE)
        {
            int64_t loop_start_us = esp_timer_get_time();

            if (as5600_update(&actuator.encoder)) {
                clear_error_flag(ERROR_FLAG_ENCODER);
            } else {
                set_error_flag(ERROR_FLAG_ENCODER);
            }
            pos_feedback = as5600_get_position(&actuator.encoder);
            pos_delta = pos_feedback - actuator.pos;
            vel_feedback = AS5600_VELOCITY_FILTER_ALPHA * (pos_delta / dt_s) + (1.0f - AS5600_VELOCITY_FILTER_ALPHA) * vel_feedback;
            actuator.pos = pos_feedback;
            actuator.vel = vel_feedback;

            actuator.pos_ctrl += actuator.vel_ctrl * dt_s;
            if (actuator.pos_ctrl > ACTUATOR_MAX) {
                actuator.pos_ctrl = ACTUATOR_MAX;
                actuator.vel_ctrl = 0.0f;
            }
            else if (actuator.pos_ctrl < ACTUATOR_MIN) {
                actuator.pos_ctrl = ACTUATOR_MIN;
                actuator.vel_ctrl = 0.0f;
            }
            
            vel_sig = pid_update(&actuator.pos_pid,
                                 actuator.pos_ctrl,
                                 pos_feedback,
                                 actuator.vel_ctrl);

            vel_sig_delta = vel_sig - vel_feedback;

            if (vel_sig_delta > max_vel_sig_delta)
            {
                vel_sig = vel_feedback + max_vel_sig_delta;
            }
            else if (vel_sig_delta < -max_vel_sig_delta)
            {
                vel_sig = vel_feedback - max_vel_sig_delta;
            }

            stepper_set_velocity(&actuator.stepper, vel_sig);
            
            int64_t now_us = esp_timer_get_time();
            loop_time_us = PID_LOOP_TIME_ALPHA * (float)(now_us - loop_start_us) + (1.0f - PID_LOOP_TIME_ALPHA) * loop_time_us;

            if (last_us > 0)
            {
                float time_between_loops_ms = (now_us - last_us) / 1000.0f;
                float current_frequency = 1000.0f / time_between_loops_ms;
                pid_freq = PID_FREQ_ALPHA * current_frequency + (1.0f - PID_FREQ_ALPHA) * pid_freq;
            }
            last_us = now_us;

            if ((now_us - last_log_us) >= PID_LOG_INTERVAL_US)
            {
                ESP_LOGD(TAG,
                         "PID Freq: %.2f Hz | Loop: %.0f us | "
                         "vel_meas=%.4f vel_ctrl=%.4f",
                         pid_freq, loop_time_us,
                         vel_feedback, vel_sig);
                last_log_us = now_us;
            }
        }

        taskYIELD();
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        state_publisher_msg.data.data[0] = actuator.pos;
        state_publisher_msg.data.data[1] = actuator.vel;
        RCSOFTCHECK(rcl_publish(&state_publisher, &state_publisher_msg, NULL));
    }
}

void command_subscriber_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size < 2)
    {
        ESP_LOGW(TAG, "Received command with insufficient data size: %zu", msg->data.size);
        return;
    }

    float pos = msg->data.data[0];
    if (pos > ACTUATOR_MAX)
    {
        pos = ACTUATOR_MAX;
    }
    else if (pos < ACTUATOR_MIN)
    {
        pos = ACTUATOR_MIN;
    }
    actuator.pos_ctrl = pos;

    float vel = msg->data.data[1];
    if (vel > ACTUATOR_MAX_VELOCITY)
        vel = ACTUATOR_MAX_VELOCITY;
    else if (vel < -ACTUATOR_MAX_VELOCITY)
        vel = -ACTUATOR_MAX_VELOCITY;
    actuator.vel_ctrl = vel;
}

void micro_ros_task(void *arg)
{
    (void)arg;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    uros_state_t state = WAITING_AGENT;
    int64_t last_ping_check_us = 0;

    rclc_support_t support = {0};
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    state_publisher = rcl_get_zero_initialized_publisher();
    command_subscriber = rcl_get_zero_initialized_subscription();

    for (;;)
    {
        switch (state)
        {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(UROS_PING_TIMEOUT_MS, UROS_PING_ATTEMPTS) == RMW_RET_OK)
            {
                state = AGENT_AVAILABLE;
            }
            else
            {
                set_error_flag(ERROR_FLAG_AGENT);
                vTaskDelay(pdMS_TO_TICKS(UROS_RECONNECT_DELAY_MS));
            }
            break;

        case AGENT_AVAILABLE:
            support = (rclc_support_t){0};
            node = rcl_get_zero_initialized_node();
            timer = rcl_get_zero_initialized_timer();
            executor = rclc_executor_get_zero_initialized_executor();
            state_publisher = rcl_get_zero_initialized_publisher();
            command_subscriber = rcl_get_zero_initialized_subscription();

            if (create_micro_ros_entities(&allocator, &support, &node, &timer, &executor))
            {
                state = AGENT_CONNECTED;
                last_ping_check_us = esp_timer_get_time();
                clear_error_flag(ERROR_FLAG_AGENT);
                ESP_LOGI(TAG, "Agent connected");
            }
            else
            {
                ESP_LOGW(TAG, "Entity creation failed, retrying...");
                destroy_micro_ros_entities(&support, &node, &timer, &executor);
                set_error_flag(ERROR_FLAG_AGENT);
                state = WAITING_AGENT;
                vTaskDelay(pdMS_TO_TICKS(UROS_RECONNECT_DELAY_MS));
            }
            break;

        case AGENT_CONNECTED:
            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(UROS_EXECUTOR_WAIT_MS)) != RCL_RET_OK)
            {
                state = AGENT_DISCONNECTED;
                break;
            }

            if ((esp_timer_get_time() - last_ping_check_us) >= (UROS_PING_INTERVAL_MS * 1000LL))
            {
                if (rmw_uros_ping_agent(UROS_PING_TIMEOUT_MS, UROS_PING_ATTEMPTS) != RMW_RET_OK)
                {
                    state = AGENT_DISCONNECTED;
                    break;
                }
                last_ping_check_us = esp_timer_get_time();
            }

            vTaskDelay(pdMS_TO_TICKS(1));
            break;

        case AGENT_DISCONNECTED:
            ESP_LOGW(TAG, "Agent disconnected, destroying entities...");
            set_error_flag(ERROR_FLAG_AGENT);
            rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
            if (rmw_context != NULL)
            {
                (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
            }
            destroy_micro_ros_entities(&support, &node, &timer, &executor);
            state = WAITING_AGENT;
            vTaskDelay(pdMS_TO_TICKS(UROS_RECONNECT_DELAY_MS));
            break;

        default:
            state = WAITING_AGENT;
            break;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Setup...");

    led_init();
    clear_error_flag(0); // all clear → ok LED on

    snprintf(state_publisher_topic, TOPIC_BUFFER_SIZE, "/%s/%s/get_state", ROBOT_NAME, JOINT_NAME);
    snprintf(command_subscriber_topic, TOPIC_BUFFER_SIZE, "/%s/%s/send_command", ROBOT_NAME, JOINT_NAME);

    pid_semaphore = xSemaphoreCreateBinary();

    init_control_timer();

    i2c_bus_init(AS5600_I2C_PORT,
                 AS5600_I2C_SDA,
                 AS5600_I2C_SCL);

    bool encoder_ready = as5600_init(
        &actuator.encoder,
        AS5600_I2C_PORT,
        AS5600_DEFAULT_ADDR,
        GEAR_RATIO,
        INVERT_AS5600,
        false);

    if (!encoder_ready) {
        ESP_LOGE(TAG, "Setup Failed! Restarting in 5 seconds...");
        set_error_flag(ERROR_FLAG_ENCODER);
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }

    gpio_input_init(ENDSTOP_PIN);

    stepper_init(
        &actuator.stepper,
        STEPPER_STEP_PIN,
        STEPPER_DIR_PIN,
        STEPPER_EN_PIN,
        GEAR_RATIO,
        STEPPER_STEPS_PER_REVOLUTION,
        STEPPER_MICROSTEPS,
        INVERT_STEPPER);

    pid_init(
        &actuator.pos_pid,
        POSITION_KP,
        POSITION_KI,
        POSITION_KD,
        POSITION_KF,
        ACTUATOR_MAX_VELOCITY,
        1.0f / PID_LOOP_FREQUENCY_HZ);

    actuator.pos_ctrl = 0.0f;
    actuator.vel_ctrl = 0.0f;

    if (!home(&actuator.stepper, &actuator.encoder)) {
        ESP_LOGE(TAG, "Homing failed! Restarting in 5 seconds...");
        set_error_flag(ERROR_FLAG_ENCODER);
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    gpio_set_level(LED_HOMED_PIN, 0); // homed LED on

    xTaskCreatePinnedToCore(
        pid_loop_task,
        "pid_loop",
        16384,
        NULL,
        10,
        NULL,
        1);

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        16384,
        NULL,
        5,
        NULL,
        0);

    ESP_LOGI(TAG, "Setup Complete");
}
