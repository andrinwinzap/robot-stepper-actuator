#include "pid.h"
#include "esp_log.h"

static const char *TAG = "pid";

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float kf)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(pid_controller_t *pid, float target, float measured, float dt)
{
    float error = target - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->kf * target +
                   pid->kp * error +
                   pid->ki * pid->integral +
                   pid->kd * derivative;

    ESP_LOGD(TAG, "target: %f, measured: %f, error: %f, output: %f", target, measured, error, output);

    return output;
}