#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf;

    float integral;
    float prev_error;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float kf);
float pid_update(pid_controller_t *pid, float target, float measured, float dt);

#endif // PID_H