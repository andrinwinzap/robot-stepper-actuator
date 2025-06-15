#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "as5600.h"
#include "stepper.h"
#include "pid.h"
#include "home.h"
#include "config.h"

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;

rcl_subscription_t velocity_subscriber;
std_msgs__msg__Float32 velocity_msg;

rcl_publisher_t homed_publisher;
std_msgs__msg__Bool homed_msg;

volatile bool is_homed = false;

static const char *TAG = "Actuator";

typedef float (*target_provider_t)(void);

typedef struct
{
    stepper_t stepper;
    as5600_t as5600;
    pid_controller_t pid;
    volatile float velocity_target;
} actuator_t;

actuator_t actuator;

void velocity_subscriber_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    actuator.velocity_target = msg->data;
    ESP_LOGI(TAG, "Received new velocity target: %f", msg->data);
}

void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = 0,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

void hall_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_23),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

float get_velocity_target(void)
{
    return actuator.velocity_target;
}

float get_joint_position(void)
{
    return as5600_get_position(&actuator.as5600);
}

void control_loop_task(void *param)
{
    target_provider_t get_target = (target_provider_t)param;
    float dt_s = 1.0f / CONTROL_LOOP_FREQUENCY;
    float dt_ms = 1000 / CONTROL_LOOP_FREQUENCY;

    for (;;)
    {
        as5600_update(&actuator.as5600);

        float measured = as5600_get_velocity(&actuator.as5600);
        float target = get_target();
        float control_signal = pid_update(&actuator.pid, target, measured, dt_s);

        stepper_set_velocity(&actuator.stepper, control_signal);

        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        // Publish homed status
        homed_msg.data = is_homed;
        RCSOFTCHECK(rcl_publish(&homed_publisher, &homed_msg, NULL));

        // Publish joint position
        msg.data = get_joint_position();
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_float32_publisher", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &homed_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "homed"));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "freertos_float32_publisher"));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &velocity_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "velocity_target"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 20;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &velocity_subscriber,
        &velocity_msg,
        velocity_subscriber_callback,
        ON_NEW_DATA));

    msg.data = 0.0f;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_1;

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Setup...");

    esp_log_level_set("pid", ESP_LOG_INFO);
    esp_log_level_set("stepper", ESP_LOG_INFO);
    esp_log_level_set("as5600", ESP_LOG_INFO);
    esp_log_level_set("homing", ESP_LOG_INFO);

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
        16000,
        NULL,
        5,
        NULL,
        0);

    actuator.velocity_target = 0.0f;

    stepper_init(
        &actuator.stepper,
        STEP_PIN,
        DIR_PIN,
        EN_PIN,
        GEAR_RATIO,
        STEPS_PER_REV,
        MICROSTEPS);

    i2c_init();
    as5600_init(
        &actuator.as5600,
        I2C_PORT,
        AS5600_DEFAULT_ADDR,
        SPEED_FILTER_ALPHA,
        SPEED_DEADBAND,
        GEAR_RATIO);

    pid_init(
        &actuator.pid,
        KP,
        KI,
        KD,
        KF);

    hall_init();

    home(&actuator.stepper, &actuator.as5600);
    is_homed = true;

    xTaskCreatePinnedToCore(
        control_loop_task,
        "control_loop",
        8000,
        (void *)get_velocity_target,
        10,
        NULL,
        1);

    ESP_LOGI(TAG, "Setup Complete");
}