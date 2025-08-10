#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "as5600.h"
#include "stepper.h"
#include "pid.h"
#include "home.h"

#include "macros.h"
#include "config.h"

#define TOPIC_BUFFER_SIZE 64

rcl_publisher_t position_publisher;
std_msgs__msg__Float32 position_publisher_msg;
char position_publisher_topic[TOPIC_BUFFER_SIZE];

rcl_publisher_t velocity_publisher;
std_msgs__msg__Float32 velocity_publisher_msg;
char velocity_publisher_topic[TOPIC_BUFFER_SIZE];

rcl_subscription_t position_subscriber;
std_msgs__msg__Float32 position_subscriber_msg;
char position_subscriber_topic[TOPIC_BUFFER_SIZE];

static const char *TAG = "Actuator";

typedef struct
{
    stepper_t stepper;
    as5600_t as5600;
    pid_controller_t pid;
    volatile float position_ctrl;
} actuator_t;

actuator_t actuator;

void position_subscriber_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    float pos = msg->data;
    if (pos > ACTUATOR_MAX)
    {
        pos = ACTUATOR_MAX;
    }
    else if (pos < ACTUATOR_MIN)
    {
        pos = ACTUATOR_MIN;
    }
    actuator.position_ctrl = pos;
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

float actuator_get_position(void)
{
    return as5600_get_position(&actuator.as5600);
}

void pid_loop_task(void *param)
{
    float dt_ms = 1000 / CONTROL_LOOP_FREQUENCY;

    for (;;)
    {
        as5600_update(&actuator.as5600);

        float feedback = as5600_get_position(&actuator.as5600);
        float control_signal = pid_update(&actuator.pid, actuator.position_ctrl, feedback);

        stepper_set_velocity(&actuator.stepper, control_signal);

        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        position_publisher_msg.data = actuator_get_position();
        RCSOFTCHECK(rcl_publish(&position_publisher, &position_publisher_msg, NULL));
        
        velocity_publisher_msg.data = as5600_get_velocity(&actuator.as5600);
        RCSOFTCHECK(rcl_publish(&velocity_publisher, &velocity_publisher_msg, NULL));
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
    RCCHECK(rclc_node_init_default(&node, "actuator", "", &support));

    // create position publisher
    RCCHECK(rclc_publisher_init_default(
        &position_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        position_publisher_topic));

    // create velocity publisher
    RCCHECK(rclc_publisher_init_default(
        &velocity_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        velocity_publisher_topic));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &position_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        position_subscriber_topic));

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
        &position_subscriber,
        &position_subscriber_msg,
        position_subscriber_callback,
        ON_NEW_DATA));

    position_publisher_msg.data = 0.0f;
    velocity_publisher_msg.data = 0.0f;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&position_publisher, &node));
    RCCHECK(rcl_publisher_fini(&velocity_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Setup...");

    snprintf(position_publisher_topic, TOPIC_BUFFER_SIZE, "/%s/%s/get_position", robot_name, joint_name);
    snprintf(velocity_publisher_topic, TOPIC_BUFFER_SIZE, "/%s/%s/get_velocity", robot_name, joint_name);
    snprintf(position_subscriber_topic, TOPIC_BUFFER_SIZE, "/%s/%s/set_position", robot_name, joint_name);

    i2c_bus_init(I2C_PORT,
                 I2C_SDA_GPIO,
                 I2C_SCL_GPIO);

    as5600_init(
        &actuator.as5600,
        I2C_PORT,
        AS5600_DEFAULT_ADDR,
        SPEED_FILTER_ALPHA,
        SPEED_DEADBAND,
        GEAR_RATIO,
        INVERT_AS5600,
        false);

    gpio_input_init(HALL_EFFECT_SENSOR_PIN);

    stepper_init(
        &actuator.stepper,
        STEP_PIN,
        DIR_PIN,
        EN_PIN,
        GEAR_RATIO,
        STEPS_PER_REV,
        MICROSTEPS,
        MAX_SPEED,
        MAX_ACCELERATION,
        INVERT_STEPPER);

    pid_init(
        &actuator.pid,
        KP,
        KI,
        KD,
        1.0f / CONTROL_LOOP_FREQUENCY);

    actuator.position_ctrl = 0.0f;

    home(&actuator.stepper, &actuator.as5600);

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
        pid_loop_task,
        "control_loop",
        16384,
        NULL,
        10,
        NULL,
        1);

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