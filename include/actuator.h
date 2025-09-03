#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "as5600.h"
#include "pid.h"
#include "stepper.h"

typedef struct
{
    stepper_t stepper;
    as5600_t encoder;
    pid_controller_t pos_pid;
    volatile float pos_ctrl;
    volatile float vel_ctrl;
    volatile float pos;
    volatile float vel;
} actuator_t;

#endif // ACTUATOR_H
