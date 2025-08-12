#ifndef JOINT_4_H
#define JOINT_4_H

const char *robot_name = "robot";
const char *joint_name = "joint_4";

#define CONTROL_LOOP_FREQUENCY 100

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO 18
#define I2C_SCL_GPIO 17
#define I2C_FREQ_HZ 400000
#define INVERT_AS5600 true

#define STEP_PIN GPIO_NUM_4
#define DIR_PIN GPIO_NUM_5
#define EN_PIN GPIO_NUM_6
#define STEPS_PER_REVOLUTION 200
#define MICROSTEPS 8
#define GEAR_RATIO 13.0f
#define INVERT_STEPPER false

#define HALL_EFFECT_SENSOR_PIN GPIO_NUM_8

#define ACTUATOR_MAX M_PI
#define ACTUATOR_MIN -M_PI
#define MAX_VELOCITY M_PI * 5
#define MAX_ACCELERATION M_PI * 5

#define VELOCITY_FILTER_ALPHA 0.1f
#define ACCELERATION_FILTER_ALPHA 0.1f

#define KP 2.0f
#define KI 0.0f
#define KD 0.0f
#define KF 1.0f

#endif // JOINT_4_H